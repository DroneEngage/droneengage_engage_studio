#!/usr/bin/env python3
"""ctl.py - operator console for a running sim (`./sim ctl <cmd> ...`).

Generalized from Phase-4A-Tasks/tools/fleet_ctl.py: units come from the
sim manifest (any block mix), pilot commands use each unit's flight
controller type, and GCS commands go to the sim's control port - served by
the GCS bridge (real droneengage_server) or by the fake server.

<unit> = a unit name (copter1), its 1-based position in the sim (2),
block:<block> (every unit of that block), a comma list, or all.

Several sims can be up at once (each owns run/<name>/). Unit names are
unique across sims, so unit commands just work; sim-scoped commands
(server procs, GCS traffic with no target unit) use `-s <name>` - with a
single sim up nothing extra is needed:  ./sim ctl -s quadcopter status

GCS:
  deploy <pkg.json> [binding.json]  1085 upload; blocks until "package READY".
                                    No binding: every unit with de_mavlink.
  start [mission] | stop | resume | clear | pause [hold] | status-req
  activate <unit> <mission>         manual mission activation
  fire <event> [json] [unit]        inject an event (1086 a:"fire")
  event <name>                      legacy 1061 (fake server: from OTHER_UNIT_1)
  send <json>                       raw wire message ({RUN}, {OP_RUN},
                                    {PKG:<file>}, {HASH:<file>},
                                    {WM_TID:<unit>})

Fleet ops (P4A-09, DE_OP_CONTROL 1094):
  deploy <spec.json>                a spec with a top-level "roles" map runs
                                    the two-phase deploy: one PREPARE per
                                    bound unit carrying pkg+hash+run+bind;
                                    a bare package falls back to 1085
  opdeploy <spec.json> [timeout]    the same, spelled out
  opstart [json]                    fleet START on the deploy's run:
                                    {"run","lead_ms","in_ms","t_start",
                                     "start":{role:mis},"targets":[party]}
  opabort [run|-] [why]             ABORT every bound unit (idempotent)
  opstatus [seconds]                poll each bound unit's op status
  opmsg <tg|all> <ms> [--mt N] [--expect RE] [--secs N]
                                    one frame as a real GCS; --expect is a
                                    regex a reply line must match
  journal <unit> [from]             P2-12 download (1088 pages)
  roster <seconds> <min_peers>      listen for 1092 rosters as a GCS
  commline <unit> <s>               in-band cut: 6509 {ws:false, wsd:s}

Pilot (units with an fc):
  arm <unit>    disarm <unit>    mode <unit> <MODE>    fc <unit>
  land <unit>   copter LAND / rover HOLD, wait disarmed; plane -> reset
  reset <unit>  power-cycle: SITL at home + de_comm + modules, wait ready

Link faults (server type fake only):
  link_cut <unit> <s> | kick <unit> | loss <unit> <%> | delay <unit> <ms> |
  link_clear <unit> | links | clock offset <ms> |
  noclock <unit> <0|1>                drop/restore the unit's clock stamps
                                      (P5-07 no_time_sync: it stays tq=0)

Vehicle faults:
  param <unit> <NAME> <value>       SITL param set (battery SIM_BATT_VOLTAGE,
                                    gps SIM_GPS_DISABLE)
  rc <unit> <chan> <pwm>            RC_CHANNELS_OVERRIDE; pwm 65535 releases

Processes:
  kill <unit> <proc> | restart <unit> <proc> | start_proc <unit> <proc>
  bootid <unit>                     new boot id (DE_BOOT_ID_FILE) - the next
                                    de_comm start counts as a reboot
      proc = sitl | de_comm | de_mavlink | <module> ; unit "server" with
      proc = de_server | de_auth | gcs_bridge | fake_server | all
  server_up                          relaunch every server process
  status                             units, ports, live pids
  wait <s> | wait_online <timeout> | wait_ready <unit> <timeout>
  wait_journal <unit> <regex> <timeout>   (lines since the last `sim up`)
"""

import asyncio
import hashlib
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

CTX = None            # pinned sim name: `ctl -s <name>` or launcher/runner


def _ctx_name():
    """The sim a server/GCS/proc command applies to: the -s pin, else the
    only LIVE sim (manifests of downed sims stay on disk - they don't
    count)."""
    if CTX:
        return CTX
    names = simlib.live_sim_names() or simlib.sim_names()
    if len(names) == 1:
        return names[0]
    if not names:
        sys.exit("no sim generated - run `./sim up <sim.yaml>` first")
    sys.exit("several sims live (" + ", ".join(names) +
             ") - pick one with `./sim ctl -s <name> ...`")


def manifest(name=None):
    return simlib.load_manifest(name or CTX)


def manifests():
    return simlib.all_manifests()


def deployed_file():
    return simlib.deployed_path(_ctx_name())


def server():
    return manifest()["server"]


def resolve(tok, manifest_=None):
    """'all' | name | 1-based index | block:<b> | comma list -> [unit].
    Units come from the pinned sim, or every LIVE sim merged - dead sims'
    units are skipped (unit names are unique across live sims anyway -
    check_conflicts enforces it)."""
    if manifest_:
        units = manifest_["units"]
    elif CTX:
        units = manifest()["units"]
    else:
        units = [u for n in simlib.live_sim_names()
                 for u in simlib.load_manifest(n)["units"]]
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

FAULT_CMDS = ("link", "kick", "clock", "stats", "noclock")


def ctrl(line, read_s=0.4, sim=None):
    """Send one control line; print the reply. Returns the reply text.
    Exits non-zero when the backend reports ERR."""
    srv = manifest(sim)["server"]
    if line.split()[0] in FAULT_CMDS and srv["type"] != "fake":
        sys.exit(f"'{line.split()[0]}' is a link fault - it needs "
                 "`server: {type: fake}` in the sim (this sim runs "
                 f"{srv['type']} server)")
    port = srv["control_port"]
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


def ctrl_stream(line, secs, sim=None):
    """Control line whose reply is JSON frames (bridge listen/req)."""
    port = manifest(sim)["server"]["control_port"]
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


def mav(u, timeout=90, src=254):
    """Connect to the unit's SITL serial0 (one TCP client at a time).
    src=255 poses as the GCS (SYSID_MYGCS) - required for
    RC_CHANNELS_OVERRIDE which SITL ignores from other sysids."""
    from pymavlink import mavutil
    uri = f"tcp:127.0.0.1:{u['fc']['pilot_port']}"
    t0 = time.time()
    m = None
    while True:
        try:
            if m is not None:
                m.close()          # SITL serial0 chokes on abandoned clients
            m = mavutil.mavlink_connection(uri, autoreconnect=False,
                                           retries=2, source_system=src)
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

def owner_sim(owner):
    """Sim owning a process owner: a unit's own sim, or the context sim
    for the synthetic 'server' owner."""
    if owner == "server":
        return _ctx_name()
    u = simlib.find_unit(owner)
    if not u:
        sys.exit(f"unknown unit {owner!r} (no generated sim has it)")
    return u["sim"]


def owner_dir(owner):
    return simlib.server_dir(owner_sim(owner)) if owner == "server" \
        else simlib.unit_dir(simlib.find_unit(owner))


def kill_proc(owner, proc):
    pf = simlib.pid_path(owner_sim(owner), owner, proc)
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
    pf = simlib.pid_path(owner_sim(owner), owner, proc)
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


def online_parties(sim=None):
    """Parties recently heard by one sim's GCS backend (its control port)."""
    port = manifest(sim)["server"]["control_port"]
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


# ------------------------------------------------------------- P4A-09
# DE_OP_CONTROL (1094) - the two-phase fleet deploy over a real GCS
# session. On the fake server a dedicated ws session carries the frames
# (sd is stamped by the server; the 9007 push gives the fleet clock for
# t_start). On the real server the GCS bridge relays a `reqm` batch and a
# 9005 ping supplies the fleet clock; without either, units fall back to
# the relative start_in_ms.

def _op_state_file():
    return simlib.op_state_path(_ctx_name())


def _op_state():
    """The last opdeploy's {run, op, bind:{party:role}} - {} before the
    first deploy. Lets opstart/opabort/opstatus find the fleet run id."""
    try:
        return json.load(open(_op_state_file()))
    except (OSError, ValueError):
        return {}


def _all_parties():
    return [u["party_id"] for u in manifest()["units"]]


def _op_parties():
    """The bound parties of the last deploy, else every unit."""
    return sorted(_op_state().get("bind") or {}) or _all_parties()


def _pkg_path(p):
    """A {PKG:}/{HASH:} token path: as given, relative to the sim root,
    or bare inside packages/."""
    for cand in (p, os.path.join(simlib.SIM_ROOT, p),
                 os.path.join(simlib.SIM_ROOT, "packages", p)):
        if os.path.isfile(cand):
            return cand
    sys.exit(f"no such package file: {p!r}")


def _expand_pkg_tokens(text):
    """"{PKG:<file>}" -> the file's text as a JSON string; "{HASH:<file>}"
    -> its "sha256:<hex>". Lets a scenario build a real PREPARE without
    literal package text."""
    for m in re.finditer(r'"\{PKG:([^}]+)\}"', text):
        text = text.replace(
            m.group(0),
            json.dumps(open(_pkg_path(m.group(1))).read()))
    for m in re.finditer(r'"\{HASH:([^}]+)\}"', text):
        h = "sha256:" + hashlib.sha256(
            open(_pkg_path(m.group(1)), "rb").read()).hexdigest()
        text = text.replace(m.group(0), json.dumps(h))
    return text


async def _fake_gcs_roundtrip(build, secs):
    """Fake server: open a GCS ws, send the wire messages
    build(fleet_ms|None) returns as [(tg, msg)], and collect every
    non-roster reply for `secs` more seconds. The fake server stamps sd
    and pushes a 9007 fleet-clock stamp after auth - fleet_ms is that
    stamp plus elapsed wall time (the same estimate the WebClient makes).
    Returns {sd_party: [wire_msg, ...]}."""
    import websockets
    fleet = [None, 0.0]      # (9007 "ts" stamp, local receipt time)
    got_ack = False
    replies = {}
    async with websockets.connect(f"wss://127.0.0.1:{server()['port']}",
                                  ssl=_ssl_ctx(), compression=None) as ws:
        await ws.send(json.dumps({"ty": "s", "mt": "de_auth", "f": "ctl",
                                  "s": "gcs_sim_ctl", "at": "g"}))
        t0 = time.time()
        while time.time() - t0 < 10:
            # done once the ack is in; the 9007 stamp is only waited on
            # briefly (the fake server always sends it right after auth)
            if got_ack and (fleet[0] is not None or time.time() - t0 > 3):
                break
            try:
                msg = json.loads(await asyncio.wait_for(
                    ws.recv(), 10 - (time.time() - t0)))
            except asyncio.TimeoutError:
                break
            if msg.get("mt") == "de_auth_ack":
                got_ack = True
            elif msg.get("mt") == 9007 and isinstance(msg.get("ms"), dict):
                fleet[0], fleet[1] = msg["ms"].get("ts"), time.time()
        if not got_ack:
            sys.exit("GCS session: no de_auth_ack from the server")
        fleet_now = (fleet[0] + (time.time() - fleet[1]) * 1000.0) \
            if fleet[0] is not None else None
        for tg, msg in build(fleet_now):
            msg["tg"] = tg
            await ws.send(json.dumps(msg))
        t0 = time.time()
        while time.time() - t0 < secs:
            try:
                msg = json.loads(await asyncio.wait_for(
                    ws.recv(), secs - (time.time() - t0)))
            except asyncio.TimeoutError:
                break
            if msg.get("mt") == 1092:
                continue        # roster churn - not a reply
            replies.setdefault(msg.get("sd", "?"), []).append(msg)
    return replies


def _bridge_gcs_roundtrip(build, secs):
    """Real server: the frames go through the GCS bridge's control port
    as one `reqm` batch (sd = the bridge's GCS party); matching replies
    stream back. A 9005 ping supplies the fleet clock - with no pong the
    start falls back to the relative start_in_ms."""
    fleet_now = None
    t0 = time.time()
    for g in ctrl_stream("req 3 9005 " + json.dumps(
            {"ty": "s", "mt": 9005, "ms": {}}), 3):
        ts = (g.get("ms") or {}).get("ts")
        if ts:
            # the pong's server-side stamp, aged by the reply's delay
            fleet_now = ts + (time.time() - t0) * 500.0
    frames = []
    for tg, msg in build(fleet_now):
        msg["tg"] = tg
        frames.append(msg)
    mts = {f.get("mt") for f in frames} | {1094}
    replies = {}
    for m in ctrl_stream("reqm %g %s %s" % (secs, ",".join(map(str, mts)),
                                          json.dumps(frames)), secs):
        if m.get("mt") == 1092:
            continue
        replies.setdefault(m.get("sd", "?"), []).append(m)
    return replies


def gcs_roundtrip(build, secs):
    """One request/reply exchange as a GCS on whichever server the sim
    runs; see _fake_gcs_roundtrip/_bridge_gcs_roundtrip."""
    if server()["type"] == "fake":
        return asyncio.run(_fake_gcs_roundtrip(build, secs))
    return _bridge_gcs_roundtrip(build, secs)


def _last_op_reply(frames, run, actions):
    """The newest 1094 reply for `run` with a in `actions`, else None."""
    for g in reversed(frames):
        ms = g.get("ms") or {}
        if g.get("mt") == 1094 and ms.get("run") == run \
                and ms.get("a") in actions:
            return ms
    return None


def _is_deploy_spec(path):
    """A P4A-09 deploy spec has a top-level "roles" map; a bare package
    does not - deploy dispatches on this."""
    try:
        head = simlib.load_config(path)
    except Exception:
        return False
    return isinstance(head, dict) and isinstance(head.get("roles"), dict)


def cmd_opdeploy(spec_path, timeout=60.0):
    """Two-phase deploy (P4A-09). The spec file holds
      {"op": name, "run"?: id,
       "roles": {"<role>": {"pkg": "<file>", "units": ["copter1", ...]}}}
    - one package per role, one PREPARE per bound unit carrying the full
    bind map and the shared run id. Every bound unit must answer `ready`
    or the deploy fails."""
    spec_path = os.path.abspath(spec_path)
    spec = simlib.load_config(spec_path)
    if not isinstance(spec.get("roles"), dict):
        sys.exit(f"opdeploy: {spec_path} is not a deploy spec "
                 "(no top-level \"roles\" map)")
    base = os.path.dirname(spec_path)
    run = spec.get("run") or str(uuid.uuid4())
    jobs, ohs, op = [], [], spec.get("op", "")
    for role, r in spec["roles"].items():
        text = open(os.path.join(base, r["pkg"])).read()
        try:
            pkg = json.loads(simlib.strip_comments(text))
        except ValueError as e:
            sys.exit(f"opdeploy: {r['pkg']} is not plain JSON ({e})")
        h = "sha256:" + hashlib.sha256(text.encode()).hexdigest()
        ohs.append(pkg.get("op_hash", ""))
        if not op:
            op = pkg.get("op", "")
        for tok in r.get("units", []):
            jobs.append((resolve(str(tok))[0], role, text, h))
    if not jobs:
        sys.exit("opdeploy: the spec binds no units")
    oh = spec.get("oh") or next((h for h in ohs if h), "")
    if not oh:
        sys.exit("opdeploy: no op hash - set 'oh' or use 2.2 packages")
    if any(h not in ("", oh) for h in ohs):
        sys.exit("opdeploy: the role packages disagree on op_hash")
    bind = {u["party_id"]: role for u, role, _, _ in jobs}
    if len(bind) != len(jobs):
        sys.exit("opdeploy: a unit is bound twice")

    def build(_fleet_now):
        return [(u["party_id"],
                 {"ty": "i", "mt": 1094,
                  "ms": {"a": "prepare", "op": op, "oh": oh, "run": run,
                         "bind": bind, "pkg": text, "hash": h}})
                for u, _role, text, h in jobs]

    print(f"op {op} run {run}: PREPARE -> "
          + ", ".join(f"{u['name']}={r}" for u, r, _, _ in jobs))
    replies = gcs_roundtrip(build, timeout)
    ok = True
    for u, role, _t, _h in jobs:
        ans = _last_op_reply(replies.get(u["party_id"], []),
                             run, ("ready", "fail"))
        if ans is None:
            print(f"{u['name']}: no PREPARE reply after {timeout:.0f}s")
            ok = False
        elif ans.get("a") == "ready":
            print(f"{u['name']}: READY as {role}")
        else:
            print(f"{u['name']}: FAIL ({ans.get('why', '?')})")
            ok = False
    # even a partial deploy leaves state on the units - record it so
    # opstart/opabort/opstatus can reach it
    json.dump({"run": run, "op": op, "bind": bind},
              open(_op_state_file(), "w"), indent=1)
    if not ok:
        sys.exit(1)
    print(f"op run {run}: all {len(jobs)} bound units READY")


def cmd_opstart(overrides):
    """Fleet START: one {a:"start"} per bound unit with the deploy's run
    id and a t_start lead_ms ahead on the fleet clock (with start_in_ms
    as the tq=0 fallback). Waits for every unit's `started`/`fail`."""
    ov = json.loads(overrides) if overrides else {}
    st = _op_state()
    run = ov.get("run") or st.get("run")
    if not run:
        sys.exit("opstart: no op deploy - run 'deploy <spec.json>' first")
    parties = ov.get("targets") or sorted(st.get("bind") or _all_parties())
    lead = int(ov.get("lead_ms", 5000))
    in_ms = int(ov.get("in_ms", lead))

    def build(fleet_now):
        ms = {"a": "start", "run": run, "start_in_ms": in_ms}
        if isinstance(ov.get("t_start"), (int, float)):
            ms["t_start"] = int(ov["t_start"])
        elif fleet_now is not None:
            ms["t_start"] = int(fleet_now + lead)
        if isinstance(ov.get("start"), dict):
            ms["start"] = ov["start"]
        return [(p, {"ty": "i", "mt": 1094, "ms": dict(ms)})
                for p in parties]

    # replies arrive when the schedule fires (lead ahead), possibly plus
    # the pre-start FC check on FC roles
    secs = max(15.0, (lead + in_ms) / 1000.0 + 30.0)
    replies = gcs_roundtrip(build, secs)
    ok = True
    for p in parties:
        ans = _last_op_reply(replies.get(p, []), run, ("started", "fail"))
        if ans is None:
            print(f"{p}: no START reply")
            ok = False
        elif ans.get("a") == "started":
            print(f"{p}: STARTED")
        else:
            print(f"{p}: FAIL ({ans.get('why', '?')})")
            ok = False
    sys.exit(0 if ok else 1)


def cmd_opabort(run_arg, why):
    """Fleet ABORT: {a:"abort", run, why} to every bound unit; each
    answers {a:"aborted"} (idempotent - a unit with no run still does)."""
    st = _op_state()
    run = st.get("run", "") if run_arg in (None, "-") else run_arg
    parties = sorted(st.get("bind") or _all_parties())

    def build(_):
        ms = {"a": "abort", "run": run, "why": why}
        return [(p, {"ty": "i", "mt": 1094, "ms": dict(ms)})
                for p in parties]

    replies = gcs_roundtrip(build, 8)
    ok = True
    for p in parties:
        got = any(g.get("mt") == 1094 and g.get("ms", {}).get("a") == "aborted"
                  for g in replies.get(p, []))
        print(f"{p}: {'ABORTED' if got else 'no reply'}")
        ok &= got
    sys.exit(0 if ok else 1)


def cmd_opstatus(secs):
    """1094 {a:"status"} to every bound unit; prints each unit's
    {a:"status"} reply (st/role/run/armed/hash)."""
    st = _op_state()
    run = st.get("run", "")
    parties = sorted(st.get("bind") or _all_parties())

    def build(_):
        ms = {"a": "status", "run": run}
        return [(p, {"ty": "i", "mt": 1094, "ms": dict(ms)})
                for p in parties]

    replies = gcs_roundtrip(build, secs)
    ok = True
    for p in parties:
        ans = _last_op_reply(replies.get(p, []), run, ("status",)) or \
            next((g.get("ms") for g in reversed(replies.get(p, []))
                  if g.get("mt") == 1094 and g["ms"].get("a") == "status"),
                 None)
        if ans is None:
            print(f"{p}: no status reply")
            ok = False
        else:
            print(f"{p}: st={ans.get('st')} run={ans.get('run') or '-'} "
                  f"role={ans.get('role') or '-'} armed={ans.get('armed')} "
                  f"hash={(ans.get('hash') or '-')[:16]}")
    sys.exit(0 if ok else 1)


def cmd_opmsg(args):
    """opmsg <tg|all> <ms-json> [--mt N] [--expect RE] [--secs N] -
    inject one frame as a real GCS and print the replies. mt defaults to
    1094; --expect makes a silent/mismatching reply a failure."""
    tg, body = args[0], args[1]
    mt, expect, secs = 1094, None, 5.0
    i = 2
    while i < len(args):
        if args[i] == "--mt":
            mt = int(args[i + 1]); i += 2
        elif args[i] == "--expect":
            expect = args[i + 1]; i += 2
        elif args[i] == "--secs":
            secs = float(args[i + 1]); i += 2
        else:
            sys.exit(f"opmsg: unknown flag {args[i]!r}")
    if "{OP_RUN}" in body:
        body = body.replace("{OP_RUN}", _op_run())
    body = _expand_pkg_tokens(body)
    ms = json.loads(body)
    if tg == "all":
        tg = "_AGN_"
    elif not tg.startswith("_"):
        tg = resolve(tg)[0]["party_id"]

    def build(_):
        return [(tg, {"ty": "i", "mt": mt, "ms": ms})]

    replies = gcs_roundtrip(build, secs)
    rx = re.compile(expect) if expect else None
    hit = False
    for sd, frames in sorted(replies.items()):
        for g in frames:
            line = f"{sd}: mt={g.get('mt')} {json.dumps(g.get('ms'))}"
            print("  " + line)
            if rx and rx.search(line):
                hit = True
    if not replies:
        print("  (no replies)")
    if expect and not hit:
        sys.exit(f"opmsg: no reply matched {expect!r}")


def _op_run():
    run = _op_state().get("run", "")
    if not run:
        sys.exit("no op deploy yet - {OP_RUN} has no value")
    return run


def deployed_units():
    try:
        names = set(json.load(open(deployed_file())))
        us = [u for u in manifest()["units"] if u["name"] in names]
        if us:
            return us
    except (OSError, ValueError):
        pass
    return [u for u in manifest()["units"] if "de_mavlink" in u["modules"]]


def cmd_status():
    mans = manifests()
    if not mans:
        print("no sim generated - ./sim up <sim.yaml>")
        return
    for name, m in mans.items():
        srv = m["server"]
        pids = simlib.read_pids(name)
        print(f"sim {name} ({m.get('sim_file') or '-'}, {m['sim_hash']})")
        if srv["type"] == "external":
            print(f"  server external {srv['ip']}:{srv['port']}"
                  + (f" via auth {srv['auth_ip']}:{srv['auth_port']}"
                     if srv["auth"] else "")
                  + f"  control {srv['control_port']}")
        else:
            print(f"  server {srv['type']}{' +auth' if srv['auth'] else ''}: "
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
                bits.append(f"{p}={'run' if pid else 'DOWN'}"
                            + (f"({pid})" if pid else ""))
            fc = u["fc"]
            extra = (f" {fc['vehicle']} sysid:{fc['sysid']} pilot:{fc['pilot_port']}"
                     if fc else " no-fc")
            print(f"  {u['name']:10s} [{u['block']}]{extra} "
                  f"udp:{u['ports']['de_comm']}  " + " ".join(bits))
        print("  online: "
              + (", ".join(sorted(online_parties(name))) or "-"))


def main(argv):
    global CTX
    argv = list(argv)
    while argv and argv[0] in ("-s", "--sim"):
        CTX, argv = argv[1], argv[2:]
    if not argv:
        print(__doc__)
        return
    c, a = argv[0], argv[1:]

    if c == "status":
        cmd_status()
    elif c == "deploy":
        f = os.path.abspath(a[0])
        if _is_deploy_spec(f):
            # P4A-09: a spec with a "roles" map runs the two-phase
            # 1094 deploy instead of the flat 1085 upload
            cmd_opdeploy(f, float(a[1]) if len(a) > 1 and
                         re.fullmatch(r"[\d.]+", a[1] or "") else 60.0)
            return
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
            prev = set(json.load(open(deployed_file())))
        except (OSError, ValueError):
            prev = set()
        json.dump(sorted(prev | {u["name"] for u in targets}), open(deployed_file(), "w"))
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
        if os.path.isfile(deployed_file()):
            os.unlink(deployed_file())
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
        if "{OP_RUN}" in text:
            text = text.replace("{OP_RUN}", _op_run())
        text = _expand_pkg_tokens(text)
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
    elif c == "noclock":
        # P5-07 no_time_sync fault: the fake server stops stamping "ts"
        # for the unit (it stays tq=0) until `noclock <unit> 0`
        for u in resolve(a[0]):
            ctrl(f"noclock {u['party_id']} {a[1]}")
    elif c == "param":
        # P5-07 battery/gps faults: runtime SITL parameter set
        for u in with_fc(resolve(a[0])):
            m = mav(u)
            try:
                m.mav.param_set_send(m.target_system, m.target_component,
                                     a[1].encode(), float(a[2]), 9)
                got = m.recv_match(type="PARAM_VALUE", blocking=True,
                                   timeout=5)
                ok = got is not None
                print(f"{u['name']}: {a[1]} <- {a[2]} "
                      f"({'set' if ok else 'no PARAM_VALUE echo'})")
            finally:
                m.close()
    elif c == "rc":
        # P5-07 rc_override fault: RC_CHANNELS_OVERRIDE, 65535 releases
        for u in with_fc(resolve(a[0])):
            m = mav(u, src=255)
            try:
                chans = [65535] * 18
                chans[int(a[1]) - 1] = int(a[2])
                m.mav.rc_channels_override_send(m.target_system,
                                                m.target_component, *chans)
                print(f"{u['name']}: rc{a[1]} <- {a[2]}")
            finally:
                m.close()
    elif c == "server_up":
        for p in server_procs("all"):
            if p not in {k.split(".", 1)[1]
                         for k in simlib.read_pids(_ctx_name())
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
    elif c == "bootid":
        # P4F-03: rewrite the unit's boot-id file so its next de_comm
        # start counts as a reboot (DE_BOOT_ID_FILE hook)
        for u in resolve(a[0]):
            p = os.path.join(simlib.unit_dir(u), "boot_id")
            simlib.write_text(p, f"boot-{uuid.uuid4()}\n")
            # a launch script generated before the hook existed needs
            # the export patched in so a reused sim still honors it
            sh = os.path.join(simlib.unit_dir(u), "launch.de_comm.sh")
            txt = open(sh).read()
            if "DE_BOOT_ID_FILE" not in txt:
                open(sh, "w").write(txt.replace(
                    "\nexec ", f"\nexport DE_BOOT_ID_FILE='{p}'\nexec ", 1))
    elif c in ("opdeploy",):
        cmd_opdeploy(a[0], float(a[1]) if len(a) > 1 else 60.0)
    elif c == "opstart":
        cmd_opstart(a[0] if a else "")
    elif c == "opabort":
        cmd_opabort(a[0] if a else None, a[1] if len(a) > 1 else "")
    elif c == "opstatus":
        cmd_opstatus(float(a[0]) if a else 6)
    elif c == "opmsg":
        cmd_opmsg(a)
    elif c == "wait_online":
        # the pinned sim's units, or every live sim's merged
        sims = [CTX] if CTX else simlib.live_sim_names()
        us = [u for n in sims for u in simlib.load_manifest(n)["units"]]
        want = {u["party_id"] for u in us}
        timeout = float(a[0]) if a else 90
        t0 = time.time()
        got = set()
        while time.time() - t0 < timeout:
            seen = set()
            for n in sims:
                seen |= online_parties(n)
            got = seen & want
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
