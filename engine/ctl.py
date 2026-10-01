#!/usr/bin/env python3
"""ctl.py - operator console for a running sim (`./sim ctl <cmd> ...`).

Generalized from Phase-4A-Tasks/tools/fleet_ctl.py: units come from the
sim manifest (any block mix), pilot commands use each unit's flight
controller type, and GCS commands go to the sim's control port - served by
the GCS bridge (real droneengage_server) or by the fake server.

<unit> = a unit name (copter1), its 1-based position in the sim (2),
block:<block> (every unit of that block), a comma list, or all.

GCS:
  deploy <pkg.json> [binding.json]  1085 upload; blocks until "package READY".
                                    No binding: every unit with de_mavlink.
  start [mission] | stop | resume | clear | pause [hold] | status-req
  activate <unit> <mission>         manual mission activation
  fire <event> [json] [unit]        inject an event (1086 a:"fire")
  event <name>                      legacy 1061 (fake server: from OTHER_UNIT_1)
  send <json>                       raw wire message ({RUN}, {WM_TID:<unit>})
  journal <unit> [from]             P2-12 download (1088 pages)
  roster <seconds> <min_peers>      listen for 1092 rosters as a GCS
  commline <unit> <s>               in-band cut: 6509 {ws:false, wsd:s}

Pilot (units with an fc):
  arm <unit>    disarm <unit>    mode <unit> <MODE>    fc <unit>
  land <unit>   copter LAND / rover HOLD, wait disarmed; plane -> reset
  reset <unit>  power-cycle: SITL at home + de_comm + modules, wait ready

Link faults (server type fake only):
  link_cut <unit> <s> | kick <unit> | loss <unit> <%> | delay <unit> <ms> |
  link_clear <unit> | links | clock offset <ms>

Processes:
  kill <unit> <proc> | restart <unit> <proc> | start_proc <unit> <proc>
      proc = sitl | de_comm | de_mavlink | <module> ; unit "server" with
      proc = de_server | de_auth | gcs_bridge | fake_server | all
  server_up                          relaunch every server process
  status                             units, ports, live pids
  wait <s> | wait_online <timeout> | wait_ready <unit> <timeout>
  wait_journal <unit> <regex> <timeout>   (lines since the last `sim up`)
"""

import asyncio
import json
import os
import re
import signal
import socket
import ssl
import subprocess
import sys
import time
import uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simlib

DEPLOYED = os.path.join(simlib.RUN_DIR, "deployed.json")


def manifest():
    return simlib.load_manifest()


def server():
    return manifest()["server"]


def resolve(tok, manifest_=None):
    """'all' | name | 1-based index | block:<b> | comma list -> [unit]."""
    m = manifest_ or manifest()
    units = m["units"]
    tok = "all" if tok is None else str(tok)
    if tok == "all":
        return units
    out = []
    for t in tok.split(","):
        if t.startswith("block:"):
            hit = [u for u in units if u["block"] == t[6:]]
        elif t.isdigit() and 1 <= int(t) <= len(units):
            hit = [units[int(t) - 1]]
        else:
            hit = [u for u in units if u["name"] == t]
        if not hit:
            sys.exit(f"unknown unit {t!r}; sim units: "
                     + ", ".join(u["name"] for u in units))
        out += [u for u in hit if u not in out]
    return out


def with_fc(units):
    return [u for u in units if u["fc"]]


# ------------------------------------------------------------- control port

FAULT_CMDS = ("link", "kick", "clock", "stats")


def ctrl(line, read_s=0.4):
    """Send one control line; print the reply. Returns the reply text.
    Exits non-zero when the backend reports ERR."""
    if line.split()[0] in FAULT_CMDS and server()["type"] != "fake":
        sys.exit(f"'{line.split()[0]}' is a link fault - it needs "
                 "`server: {type: fake}` in the sim (this sim runs the real "
                 "droneengage_server)")
    port = server()["control_port"]
    try:
        s = socket.create_connection(("127.0.0.1", port), 3)
    except OSError as e:
        sys.exit(f"cannot reach the sim control port {port}: {e}")
    s.sendall((line + "\n").encode())
    s.settimeout(read_s)
    data = b""
    try:
        while True:
            chunk = s.recv(65536)
            if not chunk:
                break
            data += chunk
    except socket.timeout:
        pass
    s.close()
    text = data.decode(errors="replace")
    out = "\n".join(ln for ln in text.splitlines() if ln != "OK")
    if out:
        print(out)
    if any(ln.startswith("ERR") for ln in text.splitlines()):
        sys.exit(1)
    return text


def ctrl_stream(line, secs):
    """Control line whose reply is JSON frames (bridge listen/req)."""
    port = server()["control_port"]
    s = socket.create_connection(("127.0.0.1", port), 3)
    s.sendall((line + "\n").encode())
    s.settimeout(secs + 5)
    buf = b""
    try:
        while True:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
    except socket.timeout:
        pass
    s.close()
    frames = []
    for ln in buf.decode(errors="replace").splitlines():
        if ln.startswith("ERR"):
            sys.exit(ln)
        try:
            frames.append(json.loads(ln))
        except ValueError:
            pass
    return frames


def action(obj, tg="_AGN_"):
    ctrl("action " + tg + " " + json.dumps(obj))


# ------------------------------------------------------------- mavlink

FC_COMP = 1      # MAV_COMP_ID_AUTOPILOT1


def fc_heartbeat(m, u, timeout=3):
    """Next HEARTBEAT from this unit's autopilot only (sysid = slot,
    component 1). ArduPilot also routes de_mavlink's GCS heartbeat (sysid
    255) and the SITL camera (component 100) onto serial0."""
    t0 = time.time()
    while True:
        left = timeout - (time.time() - t0)
        if left <= 0:
            return None
        hb = m.recv_match(type="HEARTBEAT", timeout=left, blocking=True)
        if hb is not None and hb.get_srcSystem() == u["fc"]["sysid"] \
                and hb.get_srcComponent() == FC_COMP:
            return hb


def mode_table(u):
    from pymavlink import mavutil
    return getattr(mavutil, simlib.FC_KINDS[u["fc"]["vehicle"]]["modes"])


def fc_mode(u, hb):
    return mode_table(u).get(hb.custom_mode, f"MODE{hb.custom_mode}")


def set_fc_mode(m, u, name):
    """SET_MODE from this vehicle's table (pymavlink's set_mode() picks the
    table from the LAST heartbeat seen, which may be de_mavlink's)."""
    from pymavlink import mavutil
    table = {v: k for k, v in mode_table(u).items()}
    if name not in table:
        sys.exit(f"{u['name']}: no mode {name!r} for a {u['fc']['vehicle']} "
                 f"(have {sorted(table)})")
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        table[name])


def mav(u, timeout=90):
    """Connect to the unit's SITL serial0 (one TCP client at a time)."""
    from pymavlink import mavutil
    uri = f"tcp:127.0.0.1:{u['fc']['pilot_port']}"
    t0 = time.time()
    while True:
        try:
            m = mavutil.mavlink_connection(uri, autoreconnect=False,
                                           retries=2, source_system=254)
            if fc_heartbeat(m, u, timeout=8) is not None:
                m.target_system = u["fc"]["sysid"]
                m.target_component = FC_COMP
                return m
        except (OSError, ConnectionError):
            pass
        if time.time() - t0 > timeout:
            sys.exit(f"no heartbeat from {u['name']} on {uri} after {timeout}s")
        time.sleep(2)


def wait_position(m, timeout=180):
    """Block until the EKF has an absolute horizontal position."""
    m.mav.request_data_stream_send(m.target_system, m.target_component, 0, 4, 1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        e = m.recv_match(type="EKF_STATUS_REPORT", timeout=2, blocking=True)
        if e is not None and (e.flags & 0x10):       # EKF_POS_HORIZ_ABS
            return True
        time.sleep(1)
    return False


def arm(u, deadline=300):
    m = mav(u)
    try:
        return _arm(u, m, deadline)
    finally:
        m.close()


def _arm(u, m, deadline):
    from pymavlink import mavutil
    want = u["fc"]["arm_mode"]
    t0 = time.time()
    while time.time() - t0 < deadline:
        if not wait_position(m, timeout=60):
            print(f"{u['name']}: still no EKF position", flush=True)
            continue
        hb = fc_heartbeat(m, u)
        if hb is None or (fc_mode(u, hb) != want and fc_mode(u, hb) != "AUTO"):
            set_fc_mode(m, u, want)
            time.sleep(0.5)
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        t1 = time.time()
        while time.time() - t1 < 3:
            hb = fc_heartbeat(m, u, timeout=1)
            if hb and hb.base_mode & 128:
                print(f"{u['name']}: ARMED ({fc_mode(u, hb)})")
                return True
        print(f"{u['name']}: arm retry...", flush=True)
    print(f"{u['name']}: not armed after {deadline}s")
    return False


def is_armed(u):
    m = mav(u)
    try:
        hb = fc_heartbeat(m, u)
        if hb is None:
            print(f"{u['name']}: no FC heartbeat - armed state unknown", flush=True)
            return None
        return bool(hb.base_mode & 128)
    finally:
        m.close()


def parallel(fn, units):
    """fn(u) on every unit in its own thread -> {name: result}."""
    import threading
    res = {}

    def one(u):
        try:
            res[u["name"]] = fn(u)
        except SystemExit as e:
            print(e, flush=True)
            res[u["name"]] = False

    ths = [threading.Thread(target=one, args=(u,)) for u in units]
    for t in ths:
        t.start()
    for t in ths:
        t.join()
    return res


def arm_all(units, deadline=300):
    """Arm in parallel, then re-check all are STILL armed (DISARM_DELAY can
    drop the first one while the last still waits for EKF)."""
    t0 = time.time()
    todo = list(units)
    while todo:
        res = parallel(lambda u: arm(u, deadline), todo)
        if not all(res.values()):
            return False
        todo = []
        for u in units:
            st = is_armed(u)
            if st is None:
                return False
            if not st:
                print(f"{u['name']}: auto-disarmed while waiting for the "
                      "others - re-arming", flush=True)
                todo.append(u)
        if todo and time.time() - t0 > deadline:
            print(f"not all armed together within {deadline}s: "
                  f"{[u['name'] for u in todo]}")
            return False
    return True


def land(u, timeout=180):
    """Bring the vehicle down and disarmed, the way its type allows:
    copter LAND, rover HOLD + disarm, plane (can't land in place) reset."""
    how = u["fc"]["land"]
    if how == "reset":
        return reset(u)
    from pymavlink import mavutil
    m = mav(u)
    try:
        hb = fc_heartbeat(m, u)
        if hb is None:
            print(f"{u['name']}: no FC heartbeat - cannot land", flush=True)
            return False
        if not hb.base_mode & 128:
            print(f"{u['name']}: already disarmed", flush=True)
            return True
        t0 = time.time()
        while time.time() - t0 < timeout:
            hb = fc_heartbeat(m, u, timeout=2)
            if hb is None:
                continue
            if not hb.base_mode & 128:
                print(f"{u['name']}: {how} + disarmed ({time.time() - t0:.0f}s)",
                      flush=True)
                return True
            if fc_mode(u, hb) != how:
                set_fc_mode(m, u, how)
            elif how == "HOLD":
                m.mav.command_long_send(
                    m.target_system, m.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0)
        print(f"{u['name']}: still armed after {timeout}s in {how}", flush=True)
        return False
    finally:
        m.close()


def fc_state(u):
    m = mav(u)
    try:
        m.mav.request_data_stream_send(m.target_system, m.target_component, 0, 4, 1)
        hb = fc_heartbeat(m, u)
        cur = m.recv_match(type="MISSION_CURRENT", timeout=3, blocking=True)
        bat = m.recv_match(type="SYS_STATUS", timeout=3, blocking=True)
        print(f"{u['name']}: {u['fc']['vehicle']} mode={fc_mode(u, hb) if hb else '?'} "
              f"armed={bool(hb and hb.base_mode & 128)} "
              f"seq={cur.seq if cur else '?'} "
              f"bat={bat.battery_remaining if bat else '?'}%")
    finally:
        m.close()


def mission_current(u, timeout=5):
    m = mav(u)
    try:
        m.mav.request_data_stream_send(m.target_system, m.target_component, 0, 4, 1)
        t0 = time.time()
        while time.time() - t0 < timeout:
            cur = m.recv_match(type="MISSION_CURRENT", timeout=2, blocking=True)
            if cur:
                return cur.seq
        return None
    finally:
        m.close()


# ------------------------------------------------------------- processes

def owner_dir(owner):
    return os.path.join(simlib.RUN_DIR, "server") if owner == "server" \
        else simlib.unit_dir({"name": owner})


def kill_proc(owner, proc):
    pf = simlib.pid_path(owner, proc)
    if not os.path.isfile(pf):
        print(f"{owner}.{proc}: no pidfile")
        return False
    pid = int(open(pf).read().strip())
    if os.path.isdir(f"/proc/{pid}") and not simlib.pid_is_ours(pid):
        print(f"{owner}.{proc}: pid {pid} is not a sim process (recycled) - not killed")
        os.unlink(pf)
        return True
    try:
        os.kill(pid, signal.SIGKILL)
        print(f"{owner}.{proc}: killed pid {pid}")
    except OSError:
        print(f"{owner}.{proc}: pid {pid} already gone")
    os.unlink(pf)
    return True


def start_proc(owner, proc):
    d = owner_dir(owner)
    script = os.path.join(d, f"launch.{proc}.sh")
    if not os.path.isfile(script):
        sys.exit(f"no launch script {script}")
    os.makedirs(os.path.join(d, "logs"), exist_ok=True)
    log = os.path.join(d, "logs", proc + ".log")
    subprocess.Popen(["setsid", "nohup", "bash", script],
                     stdout=open(log, "a"), stderr=subprocess.STDOUT,
                     stdin=subprocess.DEVNULL)
    # the launch script writes the real pidfile itself
    pf = simlib.pid_path(owner, proc)
    t0 = time.time()
    while time.time() - t0 < 3 and not os.path.isfile(pf):
        time.sleep(0.1)
    pid = open(pf).read().strip() if os.path.isfile(pf) else "?"
    print(f"{owner}.{proc}: started pid {pid}")


def server_procs(arg):
    procs = manifest()["server_procs"]
    if arg in (None, "all"):
        return procs
    if arg not in procs:
        sys.exit(f"server has no process {arg!r} (have {procs})")
    return [arg]


def reset(u, timeout=180):
    """Power-cycle the vehicle: restart SITL (-w: fresh eeprom, at home,
    disarmed), de_comm and every module, then wait until ready again.
    de_comm itself is restarted because it journals "nav adapter hello"
    only once per process - a de_mavlink-only restart can't show ready.
    Package / run state reloads from de_comm's state files, as on a real
    power cycle."""
    marks = simlib.journal_mark(u)
    procs = simlib.procs_of(u)
    for p in procs:
        kill_proc(u["name"], p)
    time.sleep(1)
    for p in procs:
        start_proc(u["name"], p)
    return wait_ready([u], timeout, marks={u["name"]: marks})


def wait_ready(units, timeout, marks=None):
    """Every catalog `ready_re` of each unit, in journal lines since
    `marks` (default: since the last `sim up`)."""
    cat = simlib.load_catalog()
    pending = {(u["name"], mod): (u, rx) for u in units
               for mod, rx in simlib.ready_checks(u, cat)}
    t0 = time.time()
    while pending and time.time() - t0 < timeout:
        for key, (u, rx) in list(pending.items()):
            since = (marks or {}).get(u["name"]) or simlib.boot_mark(u)
            if simlib.journal_match(u, rx, since):
                print(f"{u['name']}: {key[1]} ready ({time.time() - t0:.0f}s)")
                del pending[key]
        if pending:
            time.sleep(1)
    if pending:
        print(f"TIMEOUT: not ready after {timeout}s: "
              + ", ".join(f"{n}.{m}" for n, m in sorted(pending)))
        return False
    return True


def online_parties():
    port = server()["control_port"]
    try:
        s = socket.create_connection(("127.0.0.1", port), 3)
        s.sendall(b"list\n")
        s.settimeout(2)
        data = b""
        while True:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            data += chunk
        s.close()
    except OSError:
        return set()
    return {ln[2:] for ln in data.decode().splitlines() if ln.startswith("d:")}


# ------------------------------------------------------------- GCS reads

def _ssl_ctx():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


async def _fake_ws_collect(first, mts, secs, stop_on_last):
    """Fake server: open a GCS ws of our own, optionally send `first`,
    collect frames of `mts`."""
    import websockets
    out = []
    async with websockets.connect(f"wss://127.0.0.1:{server()['port']}",
                                  ssl=_ssl_ctx(), compression=None) as ws:
        await ws.send(json.dumps({"ty": "s", "mt": "de_auth", "f": "ctl",
                                  "s": "gcs_sim_ctl", "at": "g"}))
        t0 = time.time()
        while time.time() - t0 < 10:
            if json.loads(await asyncio.wait_for(ws.recv(), 10)).get("mt") == "de_auth_ack":
                break
        if first:
            await ws.send(json.dumps(first))
        t0 = time.time()
        while time.time() - t0 < secs:
            try:
                msg = json.loads(await asyncio.wait_for(ws.recv(), secs - (time.time() - t0)))
            except asyncio.TimeoutError:
                break
            if msg.get("mt") in mts:
                out.append(msg)
                if stop_on_last and not msg.get("ms", {}).get("more"):
                    break
    return out


def gcs_collect(first, mts, secs, stop_on_last=False):
    if server()["type"] == "fake":
        return asyncio.run(_fake_ws_collect(first, mts, secs, stop_on_last))
    if first:
        return ctrl_stream(f"req {secs} {list(mts)[0]} {json.dumps(first)}", secs)
    return ctrl_stream(f"listen {secs} {','.join(map(str, mts))}", secs)


def journal_download(u, frm=0):
    frames = gcs_collect({"ty": "i", "mt": 1086, "tg": u["party_id"],
                          "ms": {"a": "journal", "from": frm}}, {1088}, 15, True)
    return [ln for f in frames for ln in f.get("ms", {}).get("lines", [])]


def roster(secs):
    heard, last = {}, {}
    for msg in gcs_collect(None, {1092}, secs):
        ms = msg.get("ms", {})
        src = ms.get("src", msg.get("sd", "?"))
        heard[src] = heard.get(src, 0) + 1
        last[src] = ms
    for p, e in last.items():
        print(f"  {p}: st={e.get('st')} mis={e.get('mis') or '-'} "
              f"bat={e.get('bat')} tq={e.get('tq')}")
    return heard


# ------------------------------------------------------------- commands

def current_run_id():
    best = (0, None)
    for u in manifest()["units"]:
        for ln in simlib.journal_lines(u):
            m = re.search(r"run (\S+) started", ln[5])
            if m and ln[0] >= best[0]:
                best = (ln[0], m.group(1))
    return best[1]


def deployed_units():
    """Units that got a package (the last deploy), else every unit that
    runs a nav adapter."""
    try:
        names = set(json.load(open(DEPLOYED)))
        us = [u for u in manifest()["units"] if u["name"] in names]
        if us:
            return us
    except (OSError, ValueError):
        pass
    return [u for u in manifest()["units"] if "de_mavlink" in u["modules"]]


def cmd_status():
    m = manifest()
    srv = m["server"]
    pids = simlib.read_pids()
    print(f"sim {m.get('sim_file') or '-'} ({m['sim_hash']})")
    print(f"server {srv['type']}{' +auth' if srv['auth'] else ''}: "
          f"wss {srv['port']}  control {srv['control_port']}"
          + (f"  auth {srv['auth_port']}" if srv["auth"] else ""))
    for p in m["server_procs"]:
        pid = pids.get(f"server.{p}")
        print(f"  {'server':10s} {p}={'run' if pid else 'DOWN'}"
              + (f"({pid})" if pid else ""))
    for u in m["units"]:
        bits = []
        for p in simlib.procs_of(u):
            pid = pids.get(f"{u['name']}.{p}")
            bits.append(f"{p}={'run' if pid else 'DOWN'}" + (f"({pid})" if pid else ""))
        fc = u["fc"]
        extra = (f" {fc['vehicle']} sysid:{fc['sysid']} pilot:{fc['pilot_port']}"
                 if fc else " no-fc")
        print(f"  {u['name']:10s} [{u['block']}]{extra} udp:{u['ports']['de_comm']}  "
              + " ".join(bits))
    print("online: " + (", ".join(sorted(online_parties())) or "-"))


def main(argv):
    if not argv:
        print(__doc__)
        return
    c, a = argv[0], argv[1:]

    if c == "status":
        cmd_status()
    elif c == "deploy":
        f = os.path.abspath(a[0])
        targets = [u for u in manifest()["units"] if "de_mavlink" in u["modules"]]
        if len(a) > 1:
            binding = json.load(open(a[1]))
            if not binding:
                sys.exit("deploy: empty binding")
            targets = resolve(",".join(str(k) for k in binding))
            print("deploy: binding roles are not applied yet - same package "
                  f"to {[u['name'] for u in targets]}")
        if not targets:
            sys.exit("deploy: no target units")
        marks = {u["name"]: simlib.journal_mark(u) for u in targets}
        for u in targets:
            ctrl(f"upload {f} {u['party_id']}")
        res = []
        for u in targets:
            t0 = time.time()
            ok = False
            while time.time() - t0 < 30:
                if simlib.journal_match(u, "package READY", marks[u["name"]]):
                    ok = True
                    break
                time.sleep(0.5)
            print(f"{u['name']}: package {'READY' if ok else 'NOT READY (timeout)'}")
            res.append(ok)
        try:
            prev = set(json.load(open(DEPLOYED)))
        except (OSError, ValueError):
            prev = set()
        json.dump(sorted(prev | {u["name"] for u in targets}), open(DEPLOYED, "w"))
        if not all(res):
            sys.exit(1)
    elif c == "start":
        obj = {"a": "start", "mission": a[0]} if a else {"a": "start"}
        obj["run"] = str(uuid.uuid4())       # one run id for the whole fleet
        units = deployed_units()
        marks = {u["name"]: simlib.journal_mark(u) for u in units}
        action(obj)
        pending = {u["name"] for u in units}
        t0 = time.time()
        while pending and time.time() - t0 < 45:
            for u in units:
                if u["name"] in pending and simlib.journal_match(
                        u, r"RUN run .* started", marks[u["name"]]):
                    pending.discard(u["name"])
                    print(f"{u['name']}: run started")
            if pending and time.time() - t0 > 10:
                for u in units:              # retry the stragglers
                    if u["name"] in pending:
                        action(obj, u["party_id"])
                time.sleep(3)
            elif pending:
                time.sleep(0.5)
        if pending:
            sys.exit(f"start not confirmed on {sorted(pending)}")
    elif c == "clear":
        action({"a": "clear"})
        if os.path.isfile(DEPLOYED):
            os.unlink(DEPLOYED)
    elif c in ("stop", "resume"):
        action({"a": c})
    elif c == "status-req":
        action({"a": "status"})
    elif c == "pause":
        action({"a": "pause", "hold": bool(a and a[0] == "hold")})
    elif c == "activate":
        u = resolve(a[0])[0]
        action({"a": "activate", "mission": a[1]}, u["party_id"])
    elif c == "fire":
        o = {"a": "fire", "event": a[0]}
        rest = a[1:]
        if rest and rest[0].startswith("{"):
            o["payload"] = json.loads(rest[0])
            rest = rest[1:]
        if not rest or rest[0] == "all":
            action(o)
        else:
            for u in resolve(rest[0]):
                action(o, u["party_id"])
    elif c == "event":
        ctrl("event " + a[0])
    elif c == "send":
        text = a[0]
        if "{RUN}" in text:
            run = current_run_id()
            if not run:
                sys.exit("send: no 'run <id> started' journaled yet")
            text = text.replace("{RUN}", run)
        for m in re.finditer(r"\{WM_TID:([^}]+)\}", text):
            u = resolve(m.group(1))[0]
            hits = simlib.journal_match(u, r"WM_NEW (\S+)")
            if not hits:
                sys.exit(f"send: {u['name']} journaled no WM_NEW yet")
            text = text.replace(m.group(0), re.search(r"WM_NEW (\S+)", hits[-1][5]).group(1))
        ctrl("send " + text)
    elif c == "journal":
        u = resolve(a[0])[0]
        for ln in journal_download(u, int(a[1]) if len(a) > 1 else 0):
            print(ln)
    elif c == "roster":
        secs = float(a[0]) if a else 5
        want = int(a[1]) if len(a) > 1 else 0
        heard = roster(secs)
        for p, n in sorted(heard.items()):
            print(f"  {p}: {n} roster msg(s)")
        print(f"{len(heard)} distinct peer(s) in {secs:.0f}s")
        if len(heard) < want:
            sys.exit(f"FAIL: saw {len(heard)} < {want} roster senders")
    elif c == "arm":
        us = with_fc(resolve(a[0] if a else "all"))
        sys.exit(0 if us and arm_all(us) else 1)
    elif c == "land":
        us = with_fc(resolve(a[0] if a else "all"))
        res = parallel(land, us)
        sys.exit(0 if all(res.values()) else 1)
    elif c == "reset":
        us = with_fc(resolve(a[0] if a else "all"))
        res = parallel(reset, us)
        sys.exit(0 if us and all(res.values()) else 1)
    elif c == "disarm":
        from pymavlink import mavutil
        for u in with_fc(resolve(a[0] if a else "all")):
            m = mav(u)
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                    0, 0, 21196, 0, 0, 0, 0, 0)
            m.close()
            print(f"{u['name']}: disarm sent")
    elif c == "mode":
        res = []
        for u in with_fc(resolve(a[0])):
            m = mav(u)
            t0 = time.time()
            cur = None
            while time.time() - t0 < 30:
                hb = fc_heartbeat(m, u, timeout=2)
                cur = fc_mode(u, hb) if hb else cur
                if cur == a[1]:
                    break
                set_fc_mode(m, u, a[1])
            m.close()
            ok = cur == a[1]
            print(f"{u['name']}: mode={cur} {'OK' if ok else 'FAILED'}")
            res.append(ok)
        if res and not all(res):
            sys.exit(1)
    elif c == "fc":
        for u in with_fc(resolve(a[0] if a else "all")):
            fc_state(u)
    elif c == "link_cut":
        for u in resolve(a[0]):
            ctrl(f"link {u['party_id']} cut {a[1]}")
    elif c == "commline":
        for u in resolve(a[0]):
            ctrl(f"commline {u['party_id']} {a[1]}")
    elif c == "kick":
        for u in resolve(a[0]):
            ctrl(f"kick {u['party_id']}")
    elif c == "loss":
        for u in resolve(a[0]):
            ctrl(f"link {u['party_id']} loss {a[1]}")
    elif c == "delay":
        for u in resolve(a[0]):
            ctrl(f"link {u['party_id']} delay {a[1]}")
    elif c == "link_clear":
        for u in resolve(a[0]):
            ctrl(f"link {u['party_id']} clear")
    elif c == "links":
        ctrl("stats")
    elif c == "clock":
        ctrl(f"clock offset {a[1]}")
    elif c == "server_up":
        for p in server_procs("all"):
            if p not in {k.split(".", 1)[1] for k in simlib.read_pids()
                         if k.startswith("server.")}:
                start_proc("server", p)
    elif c in ("kill", "restart", "start_proc"):
        if a[0] == "server":
            owners = [("server", p) for p in server_procs(a[1] if len(a) > 1 else "all")]
        else:
            owners = [(u["name"], a[1]) for u in resolve(a[0])]
        for owner, p in owners:
            if c in ("kill", "restart"):
                kill_proc(owner, p)
        if c == "restart":
            time.sleep(1)
        for owner, p in owners:
            if c in ("restart", "start_proc"):
                start_proc(owner, p)
    elif c == "wait_online":
        want = {u["party_id"] for u in manifest()["units"]}
        timeout = float(a[0]) if a else 90
        t0 = time.time()
        got = set()
        while time.time() - t0 < timeout:
            got = online_parties() & want
            if got == want:
                print(f"all {len(want)} units online ({time.time()-t0:.0f}s)")
                return
            time.sleep(1)
        sys.exit(f"TIMEOUT: {len(got)}/{len(want)} units online after "
                 f"{timeout}s; missing {sorted(want - got)}")
    elif c == "wait_ready":
        us = resolve(a[0] if a else "all")
        if not wait_ready(us, float(a[1]) if len(a) > 1 else 120):
            sys.exit(1)
    elif c == "wait":
        time.sleep(float(a[0]))
    elif c == "wait_journal":
        u = resolve(a[0])[0]
        rx, timeout = a[1], float(a[2]) if len(a) > 2 else 60
        t0 = time.time()
        since = simlib.boot_mark(u)
        while time.time() - t0 < timeout:
            hits = simlib.journal_match(u, rx, since)
            if hits:
                print(f"{u['name']}: journal match after {time.time()-t0:.1f}s:")
                for h in hits:
                    print("   " + " ".join(map(str, h[:6])))
                return
            time.sleep(1)
        sys.exit(f"TIMEOUT: {rx!r} not in {u['name']} journal after {timeout}s")
    else:
        print(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    main(sys.argv[1:])
