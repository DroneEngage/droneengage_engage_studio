#!/usr/bin/env python3
"""ctl.py - operator console for a running sim (`./sim ctl <cmd> ...`).

Generalized from Phase-4A-Tasks/tools/fleet_ctl.py: units come from the
sim manifest (any block mix), pilot commands use each unit's flight
controller type, and GCS commands go to the sim's control port - served by
the GCS bridge (real droneengage_server) or by the fake server.

<unit> = a unit name (copter1), its 1-based position in the sim (2),
block:<block> (every unit of that block), a comma list, or all.

Several sims can be up at once (each owns var/run/<name>/). Unit names may
repeat across sims that spawn their own server - a bare name matching
more than one live fleet is refused as ambiguous, pin it with -s;
sim-scoped commands (server procs, GCS traffic with no target unit) use
`-s <name>` - with a single sim up nothing extra is needed:
./sim ctl -s quadcopter status

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
  beacon <unit> <seconds> [min]     count the unit's mt=1004 presence
                                    beacons on a fresh GCS listen - bounded
                                    "GCS still hears it" evidence inside an
                                    asymmetric link-fault window (SL-0007)
  commline <unit> <s>               in-band cut: 6509 {ws:false, wsd:s}
  write_file <unit> <module> <path> <json> [<path> <json> ...]
                                    write JSON file(s) into
                                    var/run/<unit>/<module>/ (drives seams like
                                    de_rpi_gpio's gpio_inputs.json); a +N
                                    token sleeps N ms before the next write

Pilot (units with an fc):
  arm <unit>    disarm <unit>    mode <unit> <MODE>    fc <unit>
  land <unit>   copter LAND / rover HOLD, wait disarmed; plane -> reset
  reset <unit>  power-cycle: SITL at home + de_comm + modules, wait ready

Link faults (server type fake, or real with `link_proxy: true`):
  link_cut <unit> <s> | kick <unit> | delay <unit> <ms> |
  link_clear <unit> | links
  loss <unit> <%> | clock offset <ms> |
  noclock <unit> <0|1>                message-level / server-stamp faults -
                                      server type fake only (the byte
                                      proxy cannot see inside TLS)
  link <unit> <sub> [args...]         raw link_model control (SL-0007):
                                      stall <s> | ge <p_gb> <p_bg> <lg>
                                      <lb> [drop|stall] | bw <B/s> |
                                      jitter <ms> | loss_tx <%> |
                                      loss_rx <%> | profile <name> |
                                      cut <s> | clear
                                      (loss_tx/loss_rx and profiles with
                                      loss knobs are fake-only)
  relay stats                       the standalone udp_relay's counters
                                    ({pairs, ports_free, rejected}) - sims
                                    whose server runs one
                                    (`server: {udp_relay: ...}`, SL-0089)

Vehicle faults:
  param <unit> <NAME> <value>       SITL param set, verified: the
                                    PARAM_VALUE echo must match name AND
                                    value, else exit 1
  param_get <unit> <NAME>           print the live param value
                                    (PARAM_REQUEST_READ)
  rc <unit> <chan> <pwm>            RC_CHANNELS_OVERRIDE; pwm 65535 releases

FCB-link faults (units/sims with `fcb_proxy: true`, SL-0008; any
transport - the per-unit proxy is ours, not the server's):
  fcb_cut <unit> <s>              close both ends; reconnects are accepted
                                  then dropped until <s> expires
  fcb_stall <unit> <ms>           hold the link silent for ms, then resume
  fcb_rate <unit> <baud>          UART pacing: cap each direction at
                                  baud/10 bytes/s (SL-0101); 0 removes
  fcb_corrupt <unit> <drop%> <flip%> [seed]
                                  seeded frame drop / bit flips
  fcb_drop <unit> <msgid|all> <%> drop only frames of one msgid (all clears)
  fcb_inject <unit> <hex>         raw MAVLink bytes towards de_mavlink
  fcb_clear <unit>                remove rules, end any cut/stall
  fcb_stats <unit>                proxy counters + active rules
  fcb_unplug <unit> [s]           pull the serial cable a `fcb_link`
                                  (kind pty/host_serial/bt_spp/udp)
                                  unit's
                                  de_mavlink sits on: the unit's serial
                                  bridge (fcb_tty / bt_spp) is killed,
                                  so the device node vanishes (pty link
                                  + /dev/pts/N, adapter socket, RFCOMM
                                  channel). [s] blocks and replugs
                                  after it (SL-0103)
  fcb_plug <unit>                 plug the device back: restart the
                                  unit's serial bridge (same link path)
  bt_squatters <unit>             list foreign (non-sim, non-root)
                                  processes holding RFCOMM sockets - a
                                  squatter (kdeconnectd is the known
                                  thief) on a bt_spp unit's channel
                                  accepts the phone's dial instead of
                                  our bridge (SL-0105)

Pilot (units with an fc, cont'd):
  takeoff <unit> [alt_m]            MAV_CMD_NAV_TAKEOFF (arm in GUIDED first)

Module s2s faults (the unit's udp_proxy: [<module>] opt-in - a scenario
`fault: udp_chunks` injects it itself; works on any server type):
  udp_chunks <unit> <module> <drop%> <reorder> <dup%> [seed]
                                    seeded chunk faults on the module<->
                                    de_comm proxy: % datagrams dropped /
                                    % duplicated / shuffle window
  udp_chunks <unit> <module> clear  back to pass-through
  udp_chunks <unit> <module> stats  the proxy's counters

Processes:
  add_module <unit> <mod> [--cam dev]  hot-add a module to a live unit
                                    (de_camera allocates a free DE-CAM
                                    label + feed); remove_module reverses
  remove_module <unit> <proc>         kill a hot-added module + its feed
  kill|restart|start_proc <unit> <proc> [proc...]
  freeze <unit> <proc> [s]          SIGSTOP the process - it stays alive
                                    (sockets open) but silent: a hang, not
                                    a crash. With <s> it self-resumes.
  unfreeze <unit> <proc>            SIGCONT a frozen process
  cpu_starve <unit> <proc> <duty_pct> [dur_s] [cycle_ms]
                                    SIGSTOP/SIGCONT duty cycle (SL-0014):
                                    the proc keeps its pid and sockets but
                                    runs only (100-duty_pct)% of each
                                    cycle_ms window. dur_s 0/absent = until
                                    cpu_unstarve or `sim down`
  cpu_unstarve <unit> <proc>        stop the duty cycle, SIGCONT the proc
  journal_unwritable <unit> [s]     chmod the unit's journal dir + files
                                    read-only - open journal fds still
                                    write, but every new journal sink open
                                    fails (de_comm stays alive). With [s]
                                    it blocks and restores after it.
  journal_writable <unit>           restore the journal dir writable
  clock_step <unit> <offset_s>      step a `faketime: true` unit's de_comm
                                    clock by offset_s (signed seconds) via
                                    its FAKETIME_TIMESTAMP_FILE - per-process,
                                    no relaunch, no adjtimex
  bootid <unit>                     new boot id (DE_BOOT_ID_FILE) - the next
                                    de_comm start counts as a reboot
      proc = sitl | de_comm | de_mavlink | <module> ; unit "server" with
      proc = de_server | de_auth | gcs_bridge | fake_server | all
  server_up                          relaunch every server process
  status [live|all] [--open-web]     units, ports, live pids, webclient
                                     links; `all` also lists downed sims;
                                     --open-web xdg-opens the sim's link
  wait <s> | wait_online <timeout> | wait_ready <unit> <timeout>
  wait_journal <unit> <regex> <timeout>   (lines since the last `sim up`)
"""

import asyncio
import base64
import copy
import hashlib
import hmac
import json
import os
import re
import shutil
import signal
import socket
import ssl
import subprocess
import sys
import time
import uuid

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

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
    units are skipped. Unit names may repeat across live sims on
    disjoint server namespaces (check_conflicts allows it now), so a
    bare name that matches in several live sims is ambiguous - pin the
    sim with -s."""
    merged = manifest_ is None and not CTX
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
            if merged and len({u["sim"] for u in hit}) > 1:
                sys.exit(f"ambiguous unit {t!r} - live in sims "
                         + ", ".join(sorted({u["sim"] for u in hit}))
                         + "; pin one with -s <sim>")
        if not hit:
            sys.exit(f"unknown unit {t!r}; sim units: "
                     + ", ".join(u["name"] for u in units))
        out += [u for u in hit if u not in out]
    return out


def with_fc(units):
    return [u for u in units if u["fc"]]


# ------------------------------------------------------------- control port

FAULT_CMDS = ("link", "kick", "clock", "stats", "noclock")

LOSS_MSG = ("loss is a message-level fault - it needs "
            "`server: {type: fake}` (the byte-level link proxy relays "
            "TLS and cannot drop a frame inside it)")


def _ctrl_exchange(port, line, read_s=0.4):
    """One control line to a control port -> raw reply text. The write
    side is half-closed so the backend ends its read loop and closes -
    recv returns on EOF instead of always burning read_s."""
    try:
        s = socket.create_connection(("127.0.0.1", port), 3)
    except OSError as e:
        sys.exit(f"cannot reach the sim control port {port}: {e}")
    s.sendall((line + "\n").encode())
    try:
        s.shutdown(socket.SHUT_WR)
    except OSError:
        pass
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
    return data.decode(errors="replace")


def _link_fault(line, srv, units, read_s):
    """A FAULT_CMDS line on a non-fake server (SL-0009): routed to the
    units' link_proxy control ports when `link_proxy` is on. Byte-level
    faults only - message-level loss and the clock stamps stay fake-only
    (a TLS relay cannot see inside the stream)."""
    head = line.split()[0]
    if head in ("clock", "noclock"):
        sys.exit(f"'{head}' rewrites server-stamped fields - it needs "
                 "`server: {type: fake}`")
    proxies = {u["party_id"]: u["link_proxy"]["ctrl"]
               for u in units if u.get("link_proxy")}
    if not proxies:
        sys.exit(f"'{head}' is a link fault - it needs "
                 "`server: {type: fake}` in the sim (this sim runs "
                 f"{srv['type']} server), or `server: {{type: real, "
                 "link_proxy: true}}` for the byte-level faults")
    if head == "stats":
        forwards = {p: "stats" for p in sorted(proxies)}
    else:
        parts = line.split(None, 2)
        if len(parts) < 2:
            sys.exit(f"'{line}' needs <party> <fault> [args]")
        party, rest = parts[1], (parts[2] if len(parts) > 2 else "")
        if head == "link":
            if not rest:
                sys.exit(f"'{line}' needs a link fault (stall|ge|bw|"
                         "jitter|delay|profile|cut|clear)")
            sub = rest.split()[0]
            if sub in ("loss", "loss_tx", "loss_rx"):
                sys.exit(LOSS_MSG)
            if sub == "profile":
                import link_model
                name = rest.split()[1] if len(rest.split()) > 1 else ""
                prof = link_model.PROFILES.get(name)
                if prof and any(k in prof
                                for k in ("loss", "loss_tx", "loss_rx")):
                    sys.exit(f"profile {name}: {LOSS_MSG}")
        targets = sorted(proxies) if party == "all" else [party]
        forwards = {}
        for p in targets:
            if p not in proxies:
                if head == "link" and rest.split()[0] == "clear":
                    print(f"{p}: no link proxy - nothing to clear")
                    continue
                sys.exit(f"{p}: no link proxy (not a unit of this sim, "
                         "or it runs without `link_proxy`)")
            forwards[p] = f"{head} {p} {rest}".rstrip()
    failed = False
    for p, fwd in forwards.items():
        text = _ctrl_exchange(proxies[p], fwd, read_s)
        out = "\n".join(ln for ln in text.splitlines() if ln != "OK")
        if out:
            print(out)
        failed = failed or any(ln.startswith("ERR")
                               for ln in text.splitlines())
    if failed:
        sys.exit(1)


def ctrl(line, read_s=0.4, sim=None):
    """Send one control line; print the reply. Returns the reply text.
    Exits non-zero when the backend reports ERR."""
    m = manifest(sim)
    srv = m["server"]
    if line.split()[0] in FAULT_CMDS and srv["type"] != "fake":
        return _link_fault(line, srv, m["units"], read_s)
    text = _ctrl_exchange(srv["control_port"], line, read_s)
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


# ------------------------------------------------------------- fcb proxy

def fcb_ctrl(u, line, read_s=0.5):
    """One control line to the unit's fcb proxy (SL-0008). The proxy is
    per-unit on fc.fcb_proxy.ctrl, spawned by us - the server type does
    not matter (these faults work on real and fake transports alike)."""
    pr = (u.get("fc") or {}).get("fcb_proxy")
    if not pr:
        sys.exit(f"{u['name']}: no fcb proxy - set `fcb_proxy: true` "
                 "on the unit or the sim")
    try:
        s = socket.create_connection(("127.0.0.1", int(pr["ctrl"])), 3)
    except OSError as e:
        sys.exit(f"{u['name']}: fcb proxy ctrl port {pr['ctrl']} "
                 f"unreachable: {e}")
    s.sendall((line + "\n").encode())
    s.settimeout(read_s)
    data = b""
    try:
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            data += chunk
    except socket.timeout:
        pass
    s.close()
    text = data.decode(errors="replace")
    out = "\n".join(ln for ln in text.splitlines() if ln != "OK")
    if out:
        print(f"{u['name']}: {out}")
    if any(ln.startswith("ERR") for ln in text.splitlines()):
        sys.exit(1)
    return text


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

    # daemon: on SIGINT the main thread's join raises first and interpreter
    # shutdown must not park on a still-landing worker (land waits out a
    # real SITL descent)
    ths = [threading.Thread(target=one, args=(u,), daemon=True)
           for u in units]
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


def _param_id(msg):
    """PARAM_VALUE.param_id comes back as str or NUL-padded bytes."""
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode(errors="replace")
    return str(pid).rstrip("\x00")


def fc_param_get(m, name, timeout=8):
    """PARAM_REQUEST_READ -> the live value of `name`, or None when the
    FC never echoes it (unknown param)."""
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    t0 = time.time()
    while time.time() - t0 < timeout:
        got = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if got is not None and _param_id(got) == name:
            return got.param_value
    return None


def fc_param_set(m, u, name, value, timeout=5):
    """PARAM_SET verified by the PARAM_VALUE echo: the echo's param_id
    must be `name` and its value `value` (float32 round-trip), else this
    reports False. Stale PARAM_VALUEs (a queued answer to an earlier
    PARAM_REQUEST_READ) carry the right name with the old value, so a
    name-matching echo only ends the wait when the VALUE matches;
    otherwise a param_get read-back decides, retried once."""
    want = float(value)
    for attempt in range(2):
        m.mav.param_set_send(m.target_system, m.target_component,
                             name.encode(), want, 9)
        t0 = time.time()
        while time.time() - t0 < timeout:
            got = m.recv_match(
                type="PARAM_VALUE", blocking=True,
                timeout=max(0.2, timeout - (time.time() - t0)))
            if got is None:
                break
            if _param_id(got) != name:
                continue
            if abs(got.param_value - want) <= max(1e-4, abs(want) * 1e-5):
                print(f"{u['name']}: {name} <- {want} set", flush=True)
                return True
        back = fc_param_get(m, name)
        if back is not None and \
                abs(back - want) <= max(1e-4, abs(want) * 1e-5):
            print(f"{u['name']}: {name} <- {want} set", flush=True)
            return True
    print(f"{u['name']}: {name} <- {want} "
          f"echo mismatch: {back}", flush=True)
    return False


# ------------------------------------------------------------- processes

def owner_sim(owner):
    """Sim owning a process owner: a unit's own sim, or the context sim
    for the synthetic 'server'/'render'/'feed' owners."""
    if owner in ("server", "render", "feed"):
        return _ctx_name()
    u = simlib.find_unit(owner, CTX)
    if not u:
        sys.exit(f"unknown unit {owner!r} (no generated sim has it)")
    return u["sim"]


def owner_dir(owner):
    if owner == "server":
        return simlib.server_dir(owner_sim(owner))
    if owner == "render":
        return simlib.render_dir(owner_sim(owner))
    if owner == "feed":
        return simlib.feed_dir(owner_sim(owner))
    u = simlib.find_unit(owner, CTX)
    if not u:
        sys.exit(f"unknown unit {owner!r} (no generated sim has it)")
    return simlib.unit_dir(u)


def add_module(uname, mod, cam_dev=None):
    """SL-0165: hot-add a module to a live unit without restarting
    SITL. Picks the next free instance name + port, generates its
    config/launch artifacts; de_camera also allocates a free DE-CAM
    label (or --cam dev), starts the feed and waits CAPTURE before the
    proc (exclusive_caps needs a producer first). The manifest records
    it under units[].hot_added - the spec hash stays unchanged, so a
    later `sim up` reuses the live sim."""
    sim_name = _ctx_name()
    m = simlib.load_manifest(sim_name)
    u = next((x for x in m["units"] if x["name"] == uname), None)
    if not u:
        sys.exit(f"no live unit {uname!r} in {sim_name}")
    catalog = simlib.load_catalog()
    base = simlib.module_base(mod, catalog)
    if mod in ("de_comm", "sitl") or base not in catalog:
        sys.exit(f"add_module: cannot hot-add {mod!r}")
    if mod in u["modules"] and mod != base:
        sys.exit(f"{uname} already runs {mod}")
    if base in ((u.get("android") or {}).get("remote_modules") or []):
        sys.exit(f"add_module: {mod} is an android remote module - "
                 "hot-add refused")
    insts = [x for x in u["modules"]
             if simlib.module_base(x, catalog) == base]
    n = 1 + max((simlib.module_instance(x, catalog) for x in insts),
                default=0)
    proc = base if n == 1 else f"{base}{n}"
    slot = (u["ports"]["de_comm"] - simlib.DE_COMM_PORT_BASE) // 10
    port = simlib.instance_port(catalog[base]["port_base"], slot, n)
    if port in set(u["ports"].values()):
        sys.exit(f"add_module: port {port} is already claimed on {uname}")
    binp = simlib.resolve_bin(proc, catalog[base])
    if not os.path.isfile(binp):
        sys.exit(f"add_module: binary {binp} does not exist")
    # the instance inherits the base module's overlay; a config.<proc>
    # entry would still win on top
    u["config"][proc] = simlib.deep_merge(
        copy.deepcopy(u["config"].get(base) or {}),
        u["config"].get(proc) or {})
    cfg = u["config"][proc]
    if base == "de_camera":
        dev = cam_dev or simlib.free_cam_label()
        if not dev:
            sys.exit("add_module: no free DE-CAM label in the pool")
        camcfg = cfg.setdefault("camera", {})
        for k in ("camera_start_index", "camera_end_index"):
            camcfg.pop(k, None)
            cfg.pop(k, None)
        e = {"name": f"CAM{n - 1}", "key": f"cam_{n - 1}"}
        if str(dev).isdigit():
            e["device_num"] = int(dev)
        else:
            e["device_name"] = str(dev)
        camcfg["camera_list"] = [e]
        if n > 1:
            cfg["module_id"] = \
                f"{cfg.get('module_id') or 'de_camera'}{n}"
    u["modules"].append(proc)
    u["ports"][proc] = port
    simlib.gen_module(u, m, proc, catalog)
    fed = []
    if base == "de_camera" and m.get("camera_feed"):
        idxs = simlib.camera_feed_indices(u)
        for idx in idxs:
            fp = f"{u['name']}_{idx}"
            simlib.gen_feed_script(m, fp, idx)
            start_proc("feed", fp)
            fed.append(fp)
        # exclusive_caps: the producer must advertise CAPTURE before
        # de_camera enumerates devices
        t0 = time.time()
        for idx in idxs:
            while not simlib.video_capture_capable(idx):
                if time.time() - t0 > 20:
                    sys.exit(f"feed never opened /dev/video{idx}")
                time.sleep(1)
    start_proc(u["name"], proc)
    mark = simlib.journal_mark(u)
    rx = (catalog.get(base) or {}).get("ready_log_re")
    if rx:
        t0 = time.time()
        while not simlib.log_match(u, proc, rx, mark):
            if time.time() - t0 > 30:
                print(f"{uname}.{proc}: no ready line - continuing")
                break
            time.sleep(1)
    u.setdefault("hot_added", []).append(proc)
    m["feed_procs"] = sorted(set(m.get("feed_procs") or []) | set(fed))
    simlib.write_json(simlib.manifest_path(sim_name), m, indent=2)
    print(f"{uname}.{proc}: added"
          + (f" ({dev})" if base == "de_camera" else ""))


def remove_module(uname, proc):
    """reverse add_module: kill the proc and its feed(s), strip the
    manifest entries (modules/ports/config/hot_added/feed_procs) and
    leave the run-dir artifacts for post-mortem."""
    sim_name = _ctx_name()
    m = simlib.load_manifest(sim_name)
    u = next((x for x in m["units"] if x["name"] == uname), None)
    if not u or proc not in u["modules"]:
        sys.exit(f"{uname} does not run {proc}")
    if proc in ("de_comm", "sitl"):
        sys.exit(f"remove_module: cannot remove {proc}")
    # feed indices this proc drives - computed before stripping the
    # module so the binding still resolves
    idxs = simlib.camera_feed_indices(u) \
        if simlib.module_base(proc) == "de_camera" else []
    kill_proc(u["name"], proc)
    killed_feeds = []
    for idx in idxs:
        fp = f"{u['name']}_{idx}"
        if fp in (m.get("feed_procs") or []):
            kill_proc("feed", fp)
            killed_feeds.append(fp)
    u["modules"].remove(proc)
    u["ports"].pop(proc, None)
    u["config"].pop(proc, None)
    if proc in (u.get("hot_added") or []):
        u["hot_added"].remove(proc)
    m["feed_procs"] = sorted(set(m.get("feed_procs") or [])
                             - set(killed_feeds))
    simlib.write_json(simlib.manifest_path(sim_name), m, indent=2)
    print(f"{uname}.{proc}: removed")


def kill_proc(owner, proc):
    u = _android_unit(owner)
    if u and proc in u["android"]["remote_modules"]:
        import android as _android
        _android.stop_module(u, proc)
        _android.wait_module_down(u, proc)
        pf = simlib.pid_path(owner_sim(owner), owner, proc)
        if os.path.isfile(pf):
            os.unlink(pf)
        print(f"{owner}.{proc}: stopped on device")
        return True
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


def kill_proc_group(owner, proc):
    """kill_proc's whole-process-group variant: the pidfile names a
    setsid'd launch wrapper whose children (e.g. fcb_tty's socat) would
    otherwise outlive it - and for fcb_tty an orphaned socat keeps the
    pty master open, so the device never actually 'unplugs'."""
    pf = simlib.pid_path(owner_sim(owner), owner, proc)
    if not os.path.isfile(pf):
        print(f"{owner}.{proc}: no pidfile")
        return False
    pid = int(open(pf).read().strip())
    if os.path.isdir(f"/proc/{pid}") and not simlib.pid_is_ours(pid):
        print(f"{owner}.{proc}: pid {pid} is not a sim process "
              "(recycled) - not killed")
        os.unlink(pf)
        return True
    try:
        os.killpg(os.getpgid(pid), signal.SIGKILL)
        print(f"{owner}.{proc}: killed process group {pid}")
    except OSError:
        print(f"{owner}.{proc}: pid {pid} already gone")
    os.unlink(pf)
    return True


def _signal_proc(owner, proc, sig):
    """SIGSTOP/SIGCONT one pidfile-tracked process. Like kill_proc it
    refuses a pid pid_is_ours rejects (recycled or foreign) - a freeze
    that silently no-ops would fake a hang that never happened. The
    pidfile stays: the process still exists, just stopped."""
    pf = simlib.pid_path(owner_sim(owner), owner, proc)
    if not os.path.isfile(pf):
        print(f"{owner}.{proc}: no pidfile")
        return False
    pid = int(open(pf).read().strip())
    if not os.path.isdir(f"/proc/{pid}"):
        print(f"{owner}.{proc}: pid {pid} already gone")
        return False
    if not simlib.pid_is_ours(pid):
        print(f"{owner}.{proc}: pid {pid} is not a sim process - "
              "refusing to signal it")
        return False
    try:
        os.kill(pid, sig)
    except OSError as e:
        print(f"{owner}.{proc}: signal to pid {pid} failed: {e}")
        return False
    what = "stopped" if sig == signal.SIGSTOP else "resumed"
    print(f"{owner}.{proc}: {what} pid {pid}")
    return True


def freeze_proc(owner, proc, secs=None):
    """SIGSTOP now; with `secs` block and SIGCONT after it (the bare
    command leaves the resume to `unfreeze` - the scenario's dur_s
    schedules that call so the timeline is not held)."""
    if not _signal_proc(owner, proc, signal.SIGSTOP):
        return False
    if secs is not None:
        time.sleep(float(secs))
        return _signal_proc(owner, proc, signal.SIGCONT)
    return True


def _pidfile_pid(owner, proc):
    """pid from the unit's pidfile, but only while it names a live sim
    process - the same pid_is_ours gate _signal_proc applies."""
    pf = simlib.pid_path(owner_sim(owner), owner, proc)
    if not os.path.isfile(pf):
        return None
    pid = int(open(pf).read().strip())
    if not os.path.isdir(f"/proc/{pid}") or not simlib.pid_is_ours(pid):
        return None
    return pid


def _starve_worker(pf, spf, duty_pct, dur_s, cycle_ms):
    """SIGSTOP/SIGCONT duty cycle on the pid named by `pf` (SL-0014).
    Re-resolves the pidfile every half-cycle so a restarted target is
    left alone only until its pidfile points at a live ours process.
    Runs `dur_s` seconds (<=0 = until killed: `cpu_unstarve` or the
    pid-dir sweep on `sim up`), always SIGCONTs and drops its own
    pidfile `spf` on the way out."""
    cycle_s = float(cycle_ms) / 1000.0
    stop_s = cycle_s * float(duty_pct) / 100.0
    run_s = cycle_s - stop_s
    end = time.time() + float(dur_s) if float(dur_s) > 0 else None
    quit_ = [False]
    try:
        signal.signal(signal.SIGTERM, lambda *_: quit_.__setitem__(0, True))
    except ValueError:
        pass    # main-thread only; a test driving the worker in a thread
                # just can't SIGTERM it - dur_s still bounds the loop
    simlib.write_text(spf, str(os.getpid()))

    def our_pid():
        try:
            pid = int(open(pf).read().strip())
        except (OSError, ValueError):
            return None
        if not os.path.isdir(f"/proc/{pid}") or not simlib.pid_is_ours(pid):
            return None
        return pid

    def sig(sig_):
        pid = our_pid()
        if pid is not None:
            try:
                os.kill(pid, sig_)
            except OSError:
                pass

    def sleep_sliced(secs):
        t0 = time.time()
        while (not quit_[0] and (end is None or time.time() < end)
               and time.time() - t0 < secs):
            time.sleep(min(0.2, secs))

    try:
        while not quit_[0] and (end is None or time.time() < end):
            sig(signal.SIGSTOP)
            sleep_sliced(stop_s)
            sig(signal.SIGCONT)
            sleep_sliced(run_s)
    finally:
        sig(signal.SIGCONT)
        try:
            os.unlink(spf)
        except OSError:
            pass


def cpu_starve_proc(owner, proc, duty_pct, dur_s=0.0, cycle_ms=1000.0):
    """Spawn the detached `_starve` worker on a live ours process; the
    caller stays unblocked (the scenario's dur_s schedules nothing -
    the worker ends itself)."""
    duty_pct = float(duty_pct)
    if not 0.0 < duty_pct < 100.0:
        print(f"{owner}.{proc}: duty_pct {duty_pct:g} not in (0,100)")
        return False
    if _pidfile_pid(owner, proc) is None:
        print(f"{owner}.{proc}: no live sim pid to starve")
        return False
    sim = owner_sim(owner)
    pf = simlib.pid_path(sim, owner, proc)
    spf = simlib.pid_path(sim, owner, proc + ".starve")
    subprocess.Popen(
        ["setsid", "nohup", sys.executable, os.path.abspath(__file__),
         "_starve", pf, spf, str(duty_pct), str(float(dur_s)),
         str(float(cycle_ms))],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        stdin=subprocess.DEVNULL)
    t0 = time.time()
    while time.time() - t0 < 3 and not os.path.isfile(spf):
        time.sleep(0.05)
    if not os.path.isfile(spf):
        print(f"{owner}.{proc}: starve worker did not start")
        return False
    print(f"{owner}.{proc}: starved {duty_pct:g}% of every "
          f"{float(cycle_ms):g}ms cycle"
          + (f" for {float(dur_s):g}s" if float(dur_s) > 0 else ""))
    return True


def cpu_unstarve_proc(owner, proc):
    """End a duty cycle: SIGTERM the worker (it SIGCONTs the target on
    its way out) and SIGCONT the target anyway in case the worker died
    leaving it stopped."""
    sim = owner_sim(owner)
    spf = simlib.pid_path(sim, owner, proc + ".starve")
    if os.path.isfile(spf):
        try:
            wpid = int(open(spf).read().strip())
        except ValueError:
            wpid = None
        if wpid is not None and simlib.pid_is_ours(wpid):
            try:
                os.kill(wpid, signal.SIGTERM)
            except OSError:
                pass
        t0 = time.time()
        while time.time() - t0 < 3 and os.path.isfile(spf):
            time.sleep(0.05)
        if os.path.isfile(spf):
            try:
                os.unlink(spf)
            except OSError:
                pass
    else:
        # the worker already finished (dur_s) - SIGCONT is still the
        # right restore, so an absent worker is not an error
        print(f"{owner}.{proc}: no starve worker running")
    return _signal_proc(owner, proc, signal.SIGCONT)


def journal_set_writable(u, writable):
    """chmod the unit's journal dir and every file in it (SL-0014).
    Read-only keeps de_comm alive and its open journal fd writable, but
    every NEW journal sink open fails - the fault bites when the sink
    path changes (deploy/clear/run). Restore order matters: dir first,
    then files."""
    jdir = simlib.journal_dir(u)
    if not os.path.isdir(jdir):
        print(f"{u['name']}: no journal dir {jdir}")
        return False
    if writable:
        os.chmod(jdir, 0o755)
        for f in os.listdir(jdir):
            p = os.path.join(jdir, f)
            if os.path.isfile(p):
                os.chmod(p, 0o644)
    else:
        for f in os.listdir(jdir):
            p = os.path.join(jdir, f)
            if os.path.isfile(p):
                os.chmod(p, 0o444)
        os.chmod(jdir, 0o555)
    print(f"{u['name']}: journal dir {'writable' if writable else 'read-only'}")
    return True


def journal_unwritable(u, secs=None):
    """journal dir + files read-only; with `secs` block and restore
    (the bare command leaves the restore to `journal_writable` - the
    scenario's dur_s schedules that call so the timeline is not held)."""
    if _pidfile_pid(u["name"], "de_comm") is None:
        print(f"{u['name']}: no live sim de_comm - refusing to fault "
              "another process's journals")
        return False
    if not journal_set_writable(u, False):
        return False
    if secs is not None:
        time.sleep(float(secs))
        return journal_set_writable(u, True)
    return True


def clock_step(u, offset_s):
    """Runtime per-process clock step on a `faketime: true` unit's
    de_comm (SL-0014): rewrite the FAKETIME_TIMESTAMP_FILE. "+3600"
    shifts that process's clock an hour ahead; "+0" steps it back to
    real time. The unit must have booted under libfaketime - there is
    no retroactive preload."""
    if not u.get("faketime"):
        print(f"{u['name']}: clock_step needs `faketime: true` on the "
              "unit (de_comm must launch under libfaketime)")
        return False
    if _pidfile_pid(u["name"], "de_comm") is None:
        print(f"{u['name']}: no live sim de_comm - refusing to step "
              "another process's clock")
        return False
    rc = simlib.faketime_rc_path(u)
    spec = f"{float(offset_s):+g}"      # libfaketime relative offset
    tmp = rc + ".tmp"
    simlib.write_text(tmp, f"{spec}\n")
    os.replace(tmp, rc)
    print(f"{u['name']}: de_comm clock stepped {spec}s "
          f"(FAKETIME_TIMESTAMP_FILE {rc})")
    return True


def _android_unit(owner):
    """The manifest unit dict when <owner> is a unit with android
    remote modules (SL-0095), else None."""
    m = simlib.load_manifest(owner_sim(owner), required=False)
    if not m:
        return None
    u = next((x for x in m.get("units", []) if x["name"] == owner), None)
    if u and (u.get("android") or {}).get("remote_modules"):
        return u
    return None


def start_proc(owner, proc):
    u = _android_unit(owner)
    if u and proc in u["android"]["remote_modules"]:
        # runs on the phone inside the Andruav app: push the generated
        # config into de_override/<mod>/ then start the module service.
        # mode: app (SL-0112) - the app owns configuration; the sim
        # pushes nothing but ctl start/stop still routes to the service.
        import android as _android
        if _android.android_mode(u) != "app":
            _android.push_module_config(
                u, proc, os.path.join(owner_dir(owner), proc))
        _android.start_module(u, proc)
        print(f"{simlib.col_green(f'{owner}.{proc}')}: "
              f"started on device {_android.serial_of(u) or 'default'}")
        return
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
    print(f"{simlib.col_green(f'{owner}.{proc}')}: "
          f"started pid {simlib.col_bold(pid)}")


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
    pending = {(u["name"], mod): (u, rx, sink) for u in units
               for mod, rx, sink in simlib.ready_checks(u, cat)}
    t0 = time.time()
    while pending and time.time() - t0 < timeout:
        for key, (u, rx, sink) in list(pending.items()):
            # DE-0050: a proc that already exited can never report
            # ready - fail fast naming it instead of waiting out the
            # timeout (e.g. de_camera dying on a missing /dev/videoN)
            pf = simlib.pid_path(u["sim"], u["name"], key[1])
            if os.path.isfile(pf):
                try:
                    pid = int(open(pf).read().strip())
                except ValueError:
                    pid = None
                if pid and not os.path.isdir(f"/proc/{pid}"):
                    print(simlib.col_red(
                        f"{u['name']}: {key[1]} exited - "
                        f"see {simlib.unit_dir(u)}/logs/{key[1]}.log"))
                    return False
            # a mark of {} (no journal file yet) means "from the start",
            # not "fall back to the boot mark" - that would match the
            # ready line of the previous boot
            since = marks[u["name"]] if marks and u["name"] in marks \
                else simlib.boot_mark(u)
            if sink == "log":
                hit = simlib.log_match(u, key[1], rx, since)
            else:
                hit = simlib.journal_match(u, rx, since)
            if hit:
                print(simlib.col_green(
                    f"{u['name']}: {key[1]} ready "
                    f"({time.time() - t0:.0f}s)"))
                del pending[key]
        if pending:
            time.sleep(1)
    if pending:
        print(simlib.col_red(f"TIMEOUT: not ready after {timeout}s: "
              + ", ".join(f"{n}.{m}" for n, m in sorted(pending))))
        return False
    # AN-0058: a phone-side de_comm can satisfy the module ready_res while
    # its FCB registration (de_mavlink -> 'nav adapter hello') is still in
    # flight - the unit then classes static and the first deploy fails
    # 'vehicle type not known yet'. The mirror can also pull a STALE hello
    # from the device's persistent journal, so gate on a line timestamped
    # after this wait began (device clock is already shifted to host time).
    fcb_units = {u["name"]: u for u in units
                 if u.get("fc") and "de_comm" in
                 ((u.get("android") or {}).get("remote_modules") or [])}
    if fcb_units:
        t2 = time.time()
        while fcb_units and time.time() - t2 < min(timeout, 90):
            for uname, u in list(fcb_units.items()):
                # A hello counts only if de_comm journaled it during THIS
                # process's lifetime - a reused (still alive) de_comm keeps
                # its registration, so its older hello is already proof.
                import android as _android
                proc_ms = _android.module_started_ms(u, "de_comm")
                if proc_ms is None:
                    continue
                if [ln for ln in simlib.journal_match(u, r"nav adapter hello")
                        if ln[0] >= proc_ms - 5000]:
                    print(simlib.col_green(
                        f"{uname}: remote fcb registered "
                        f"({time.time() - t0:.0f}s)"))
                    del fcb_units[uname]
            if fcb_units:
                time.sleep(1)
        if fcb_units:
            print(simlib.col_red(
                "TIMEOUT: remote de_comm units with no fresh FCB "
                "registration: " + ", ".join(sorted(fcb_units))))
            return False
    return True


def online_parties(sim=None):
    """Parties recently heard by one sim's GCS backend (its control port)."""
    port = manifest(sim)["server"]["control_port"]
    try:
        s = socket.create_connection(("127.0.0.1", port), 3)
        s.sendall(b"list\n")
        try:
            s.shutdown(socket.SHUT_WR)
        except OSError:
            pass
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


def link_proxy_online(u):
    """SL-0009: the unit's link_proxy `list` - True when the session is
    live AND established (a re-dial still held by a stall reads offline,
    like an un-authed ws on the fake). None = proxy unreachable."""
    pr = u.get("link_proxy")
    if not pr:
        return None
    try:
        s = socket.create_connection(("127.0.0.1", pr["ctrl"]), 3)
        s.sendall(b"list\n")
        try:
            s.shutdown(socket.SHUT_WR)
        except OSError:
            pass
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
        return None
    return any(ln == "d:" + u["party_id"]
               for ln in data.decode().splitlines())


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
            raw = await asyncio.wait_for(ws.recv(), 10)
            if not isinstance(raw, str):
                continue            # binary intermodule frame (e.g. mt 6502)
            if json.loads(raw).get("mt") == "de_auth_ack":
                break
        if first:
            await ws.send(json.dumps(first))
        t0 = time.time()
        while time.time() - t0 < secs:
            try:
                raw = await asyncio.wait_for(ws.recv(), secs - (time.time() - t0))
            except asyncio.TimeoutError:
                break
            if not isinstance(raw, str):
                continue            # binary intermodule frame (e.g. mt 6502)
            msg = json.loads(raw)
            if msg.get("mt") in mts:
                out.append(msg)
                if stop_on_last and not msg.get("ms", {}).get("more"):
                    break
    return out


async def _relay_control(mt, ms):
    """One request on the udp_relay's S2S control socket (SL-0089):
    answer the nonce challenge with HMAC-SHA256(nonce, per-run shared
    secret), send {mt, ms, rid:1}, return the reply envelope."""
    import websockets
    ur = server().get("udp_relay")
    if not ur:
        sys.exit("this sim's server has no udp_relay")
    secret = open(os.path.join(simlib.server_dir(_ctx_name()),
                               "udp_relay_s2s_secret.key")).read().strip()
    async with websockets.connect(f"wss://127.0.0.1:{ur['port']}",
                                  ssl=_ssl_ctx(), compression=None) as ws:
        ch = json.loads(await asyncio.wait_for(ws.recv(), 10))
        sig = base64.b64encode(hmac.new(
            secret.encode(), str(ch["nonce"]).encode(),
            hashlib.sha256).digest()).decode()
        await ws.send(json.dumps({"s2s_auth": "response", "sig": sig,
                                  "id": "sim_ctl"}))
        await ws.send(json.dumps({"mt": mt, "ms": ms, "rid": 1}))
        while True:
            rep = json.loads(await asyncio.wait_for(ws.recv(), 10))
            if rep.get("rid") == 1:
                return rep


def relay_stats():
    """The udp_relay's {pairs, ports_free, rejected} counters."""
    rep = asyncio.run(_relay_control("stats", {}))
    return rep.get("ms") or rep


def gcs_collect(first, mts, secs, stop_on_last=False):
    if server()["type"] == "fake":
        return asyncio.run(_fake_ws_collect(first, mts, secs, stop_on_last))
    if first:
        return ctrl_stream(f"req {secs} {list(mts)[0]} {json.dumps(first)}", secs)
    return ctrl_stream(f"listen {secs} {','.join(map(str, mts))}", secs)


def write_module_file(u, module, relpath, doc):
    """GE-07: write one JSON document into var/run/<unit>/<module>/<relpath>,
    atomically (tmp + rename) so a file-polling module never reads a
    half-written file. The path may not escape the module directory."""
    mdir = os.path.realpath(os.path.join(simlib.unit_dir(u), module))
    p = os.path.realpath(os.path.join(mdir, relpath))
    if p != mdir and not p.startswith(mdir + os.sep):
        raise simlib.SimError(f"write_file: {relpath} escapes {mdir}")
    os.makedirs(os.path.dirname(p) or mdir, exist_ok=True)
    tmp = p + ".tmp"
    with open(tmp, "w") as f:
        json.dump(doc, f)
        f.write("\n")
    os.replace(tmp, p)
    return p


def write_module_files(u, module, writes):
    """GF-07: run (relpath, doc, delay_ms) triples in order, sleeping
    delay_ms before each write. Zero-delay writes stay back-to-back
    inside one file-poll window; a delayed write lands in a later
    window (a real bounce below a debounce threshold, not a burst
    collapse)."""
    out = []
    for relpath, doc, delay_ms in writes:
        if delay_ms > 0:
            time.sleep(delay_ms / 1000.0)
        out.append(write_module_file(u, module, relpath, doc))
    return out


def udp_proxy_call(u, module, obj, wait_s=2.0):
    """SL-0010: one control datagram to a unit's udp_proxy process
    ({set|clear|stats} JSON in, a JSON state line back). The proxy port
    is the unit's udp_proxy_<module> entry - absent when the sim did
    not opt that module in."""
    port = (u.get("ports") or {}).get("udp_proxy_" + module)
    if not port:
        sys.exit(f"{u['name']}.{module}: no udp_proxy - add "
                 f"`udp_proxy: [{module}]` to the unit (a scenario "
                 "`fault: udp_chunks` does it itself)")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(wait_s)
    try:
        s.sendto(json.dumps(obj).encode(), ("127.0.0.1", port))
        data, _ = s.recvfrom(65536)
    except socket.timeout:
        sys.exit(f"{u['name']}.{module}: proxy on :{port} did not "
                 "answer - process down? (./sim ctl status)")
    finally:
        s.close()
    rep = json.loads(data.decode(errors="replace"))
    print(f"{u['name']}.{module}: {json.dumps(rep)}")
    if not rep.get("ok"):
        sys.exit(1)
    return rep


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
    bare inside packages/, or bare inside var/build/packages/by-name/
    (on-demand .deop compiles, SL-0026)."""
    for cand in (p, os.path.join(simlib.SIM_ROOT, p),
                 os.path.join(simlib.SIM_ROOT, "packages", p),
                 os.path.join(simlib.VAR_DIR, "build", "packages",
                              "by-name", p)):
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
                raw = await asyncio.wait_for(
                    ws.recv(), 10 - (time.time() - t0))
            except asyncio.TimeoutError:
                break
            if not isinstance(raw, str):
                continue            # binary intermodule frame (e.g. mt 6502)
            msg = json.loads(raw)
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
                raw = await asyncio.wait_for(
                    ws.recv(), secs - (time.time() - t0))
            except asyncio.TimeoutError:
                break
            if not isinstance(raw, str):
                continue            # binary intermodule frame (e.g. mt 6502)
            msg = json.loads(raw)
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
            # the pong is stamped right after the ping goes out, but the
            # stream is read only after the whole window - age the stamp
            # by everything since the ping (over-estimates by <= one RTT)
            fleet_now = ts + (time.time() - t0) * 1000.0
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
      {"op": name, "run"?: id, "oh"?: op_hash, "opgraph"?: "<file>",
       "roles": {"<role>": {"pkg": "<file>", "units": ["copter1", ...]}}}
    - one package per role, one PREPARE per bound unit carrying the full
    bind map and the shared run id. Every bound unit must answer `ready`
    or the deploy fails. A spec "opgraph" (SG-04, compile_op.js's
    <prefix>.opgraph.json) is copied next to the op state so a debrief
    finds the static graph without a GCS cloud save."""
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
    # SG-04: a spec's "opgraph" names the de.opgraph/1 sidecar relative to
    # the spec (the same base the role pkgs resolve on); it must carry the
    # deploy's op hash and is copied into the run dir for debriefs
    opgraph = spec.get("opgraph")
    if opgraph:
        og_src = os.path.join(base, opgraph)
        if not os.path.isfile(og_src):
            sys.exit(f"opdeploy: spec 'opgraph' {opgraph!r} not found "
                     f"next to {os.path.basename(spec_path)}")
        try:
            og = json.loads(simlib.strip_comments(open(og_src).read()))
        except ValueError as e:
            sys.exit(f"opdeploy: {opgraph} is not plain JSON ({e})")
        if og.get("op_hash") not in (None, oh):
            sys.exit("opdeploy: the opgraph op_hash disagrees with the "
                     "deploy's op hash")
        opgraph = os.path.join(simlib.run_dir(_ctx_name()),
                               os.path.basename(og_src))
        os.makedirs(os.path.dirname(opgraph), exist_ok=True)
        shutil.copyfile(og_src, opgraph)
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
    # opstart/opabort/opstatus can reach it; "opgraph" is the run-dir copy
    # of the static graph (SG-04) for harness-run debriefs
    st = {"run": run, "op": op, "bind": bind}
    if opgraph:
        st["opgraph"] = opgraph
    json.dump(st, open(_op_state_file(), "w"), indent=1)
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


def xdg_open(url):
    """Best-effort browser open for a printed link - a missing opener
    leaves the printed URL standing."""
    try:
        subprocess.Popen(["xdg-open", url],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    except OSError as e:
        print(f"xdg-open failed: {e} - open the URL above by hand")


def cmd_web(name=None, open_browser=True, planning=False):
    """Print the sim's webclient deep links (main view + planner) and
    xdg-open the requested one - studio's session open. The URLs carry
    the sim's auth endpoint (or a direct comm-server target when no
    login exists) plus its GCS account - see simlib.webclient_url. Each
    page gets its own GCS senderID: same-party connections kick each
    other off the comm server."""
    name = name or _ctx_name()
    m = simlib.load_manifest(name)
    srv = m["server"]
    if srv["type"] != "external":
        ws_proc = "fake_server" if srv["type"] == "fake" else "de_server"
        if not simlib.read_pids(name).get(f"server.{ws_proc}"):
            print(simlib.col_red(f"{name}: server is down") +
                  " - the links below would open a dead page")
    web = simlib.webclient_url(m)
    plan = simlib.webclient_url(m, "/planning", "GCSPLN_" + name)
    print("webclient: " + simlib.col_cyan(web))
    print("planning:  " + simlib.col_cyan(plan))
    if open_browser:
        xdg_open(plan if planning else web)


def _sim_status_json(name, m, pids, srv_up):
    """One sim's status as a JSON-safe dict (the `status --json` shape -
    docs/MACHINE_API.md)."""
    srv = m["server"]
    out = {"name": name, "sim_file": m.get("sim_file"),
           "sim_hash": m["sim_hash"],
           "live": bool(pids),
           "server": {"type": srv["type"], "auth": srv["auth"],
                      "port": srv["port"],
                      "control_port": srv["control_port"],
                      "auth_port": srv.get("auth_port"),
                      "up": srv_up},
           "procs": {},
           "units": []}
    if srv["type"] == "external":
        out["server"]["conn"] = f"external {srv['ip']}:{srv['port']}" \
            + (f" via auth {srv['auth_ip']}:{srv['auth_port']}"
               if srv["auth"] else "")
    for p in m["server_procs"]:
        out["procs"][f"server.{p}"] = pids.get(f"server.{p}")
    for u in m["units"]:
        ud = {"name": u["name"], "block": u["block"],
              "fc": u["fc"], "udp": u["ports"]["de_comm"],
              "procs": {p: pids.get(f"{u['name']}.{p}")
                        for p in simlib.procs_of(u)}}
        out["units"].append(ud)
    if srv_up:
        out["online"] = sorted(online_parties(name))
        out["webclient"] = simlib.webclient_url(m)
        out["planning"] = simlib.webclient_url(
            m, "/planning", "GCSPLN_" + name)
    return out


def cmd_status(scope="live", json_out=False, open_web=False):
    mans = manifests()
    if json_out and not mans:
        print(json.dumps({"sims": []}, indent=1))
        return
    if not mans:
        print("no sim generated - ./sim up <sim.yaml>")
        return
    dead = set(mans) - set(simlib.live_sim_names())
    if scope != "all":
        mans = {n: m for n, m in mans.items() if n not in dead}
    if json_out and not mans:
        print(json.dumps({"sims": [], "downed": len(dead)}, indent=1))
        return
    if not mans:
        print(f"no live sim ({len(dead)} downed - `status all` shows them)")
        return
    sims_json = []
    srv_ups = {}
    for name, m in mans.items():
        srv = m["server"]
        pids = simlib.read_pids(name)
        # `online`/the webclient links only mean something while THIS
        # sim's server proc is alive - its ports may already serve a
        # different sim reusing the same spec. External sims own no local
        # server process; the remote one is taken as up.
        ws_proc = "fake_server" if srv["type"] == "fake" else "de_server"
        srv_up = srv["type"] == "external" \
            or bool(pids.get(f"server.{ws_proc}"))
        srv_ups[name] = srv_up
        if json_out:
            sims_json.append(_sim_status_json(name, m, pids, srv_up))
            continue
        print(f"sim {simlib.col_bold(name)}"
              + ("" if pids else " " + simlib.col_red("(down)"))
              + f" ({m.get('sim_file') or '-'}, {m['sim_hash']})")
        if srv["type"] == "external":
            conn = f"external {srv['ip']}:{srv['port']}" \
                   + (f" via auth {srv['auth_ip']}:{srv['auth_port']}"
                      if srv["auth"] else "")
            print(f"  server {simlib.col_cyan(conn)}"
                  f"  control {srv['control_port']}")
        else:
            print(f"  server {srv['type']}{' +auth' if srv['auth'] else ''}: "
                  + simlib.col_cyan(f"wss {srv['port']}  "
                                    f"control {srv['control_port']}"
                                    + (f"  auth {srv['auth_port']}"
                                       if srv["auth"] else "")))
        for p in m["server_procs"]:
            pid = pids.get(f"server.{p}")
            state = simlib.col_green(f"run({pid})") if pid \
                else simlib.col_red("DOWN")
            print(f"  {'server':10s} {p}={state}")
        for u in m["units"]:
            bits = []
            for p in simlib.procs_of(u):
                pid = pids.get(f"{u['name']}.{p}")
                bits.append(f"{p}=" + (simlib.col_green(f"run({pid})")
                                       if pid else simlib.col_red("DOWN")))
            fc = u["fc"]
            extra = (f" {simlib.col_yellow(fc['vehicle'])}"
                     f" sysid:{fc['sysid']}"
                     f" pilot:{simlib.col_cyan(fc['pilot_port'])}"
                     if fc else " no-fc")
            print(f"  {simlib.col_green(u['name'])}"
                  f"{' ' * max(0, 10 - len(u['name']))}"
                  f"[{u['block']}]{extra} "
                  f"udp:{simlib.col_cyan(u['ports']['de_comm'])}  "
                  + " ".join(bits))
        if srv_up:
            online = ", ".join(sorted(online_parties(name)))
            print("  online: "
                  + (simlib.col_green(online) if online else "-"))
            print("  webclient: "
                  + simlib.col_cyan(simlib.webclient_url(m)))
            # a second GCS needs its own senderID - same-party
            # connections kick each other off the comm server
            print("  planning:  "
                  + simlib.col_cyan(simlib.webclient_url(
                      m, "/planning", "GCSPLN_" + name)))
        else:
            print("  (server down - `sim down` frees nothing here; "
                  "`sim clean` removes the leftover run dir)")
    if json_out:
        print(json.dumps({"sims": sims_json}, indent=1))
    elif open_web:
        # `--open-web`: the browser-open half of the retired `web`
        # command - the links themselves are printed above. -s picks the
        # sim when several are listed.
        name = CTX or (next(iter(mans)) if len(mans) == 1 else None)
        if not name:
            sys.exit("several sims listed - pick one with "
                     "`./sim status -s <sim> --open-web`")
        m = mans.get(name) or simlib.load_manifest(name)
        srv = m["server"]
        if name in srv_ups:
            up = srv_ups[name]
        else:
            # a -s pin outside the printed set (e.g. downed under the
            # default `live` scope) - check its server proc directly
            ws_proc = ("fake_server" if srv["type"] == "fake"
                       else "de_server")
            up = srv["type"] == "external" or bool(
                simlib.read_pids(name).get(f"server.{ws_proc}"))
        if not up:
            sys.exit(simlib.col_red(f"{name}: server is down") +
                     " - the link would open a dead page")
        xdg_open(simlib.webclient_url(m))


def main(argv):
    global CTX
    argv = list(argv)
    while argv and argv[0] in ("-s", "--sim"):
        CTX, argv = argv[1], argv[2:]
    if not argv:
        simlib.print_usage(__doc__)
        return
    c, a = argv[0], argv[1:]

    if c == "status":
        json_out = "--json" in a
        open_web = "--open-web" in a
        a = [x for x in a if x not in ("--json", "--open-web")]
        scope = a[0] if a else "live"
        if scope not in ("live", "all") or len(a) > 1:
            sys.exit("usage: status [live|all] [--json] [--open-web]")
        cmd_status(scope, json_out=json_out, open_web=open_web)
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
    elif c == "write_file":
        u = resolve(a[0])[0]
        if len(a) < 4:
            sys.exit("write_file <unit> <module> <path> <json> "
                     "[<path> <json> ...] [+N <path> <json> ...]")
        # GF-07: a +N token sleeps N ms before the following write -
        # inside the same ctl process, so the gap is real time, not a
        # subprocess boundary
        writes = []
        delay_ms = 0
        i = 2
        while i < len(a):
            m = re.fullmatch(r"\+(\d+)", a[i])
            if m:
                delay_ms = int(m.group(1))
                i += 1
                continue
            if i + 1 >= len(a):
                sys.exit("write_file: <path> without <json>")
            writes.append((a[i], json.loads(a[i + 1]), delay_ms))
            delay_ms = 0
            i += 2
        for p in write_module_files(u, a[1], writes):
            print(p)
    elif c == "journal":
        u = resolve(a[0])[0]
        for ln in journal_download(u, int(a[1]) if len(a) > 1 else 0):
            print(ln)
    elif c == "beacon":
        # SL-0007: mt=1004 presence beacons are the unit's own periodic
        # broadcasts to _GCS_ - hearing them inside a link-fault window
        # proves the unit->server->GCS direction is alive without
        # needing a multi-unit fleet for 1092 rosters
        u = resolve(a[0])[0]
        secs = float(a[1]) if len(a) > 1 else 5
        want = int(a[2]) if len(a) > 2 else 1
        n = sum(1 for m in gcs_collect(None, {1004}, secs)
                if m.get("sd") == u["party_id"])
        print(f"  {u['name']}: {n} presence beacon(s) in {secs:g}s")
        if n < want:
            sys.exit(f"FAIL: heard {n} < {want} of {u['name']}'s "
                     "1004 beacons")
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
    elif c == "link":
        # SL-0007 raw link-model control passthrough:
        # `link <unit> stall 30` / `link 1 ge 0.05 0.2 0 90 stall` /
        # `link all profile lte_edge` - sub-command args after the unit
        # token go to the fake server verbatim
        if len(a) < 2:
            sys.exit("usage: link <unit> <sub> [args...]  "
                     "(stall|ge|bw|jitter|loss_tx|loss_rx|profile|"
                     "cut|clear)")
        for u in resolve(a[0]):
            ctrl(f"link {u['party_id']} "
                 + " ".join(str(x) for x in a[1:]))
    elif c == "links":
        ctrl("stats")
    elif c == "relay":
        # SL-0089: the standalone udp_relay's control API (per-run HMAC
        # secret). `relay stats` prints the {pairs, ports_free,
        # rejected} counters udp_proxy_rejects reads.
        if not a or a[0] != "stats":
            sys.exit("usage: relay stats")
        print(json.dumps(relay_stats(), indent=1))
    elif c == "clock":
        ctrl(f"clock offset {a[1]}")
    elif c == "noclock":
        # P5-07 no_time_sync fault: the fake server stops stamping "ts"
        # for the unit (it stays tq=0) until `noclock <unit> 0`
        for u in resolve(a[0]):
            ctrl(f"noclock {u['party_id']} {a[1]}")
    elif c == "udp_chunks":
        # SL-0010: knobs on the module<->de_comm chunk proxy
        # (udp_proxy.py). <drop%> <reorder> <dup%> [seed] | clear | stats
        mod = a[1]
        for u in resolve(a[0]):
            if len(a) < 3 or a[2] == "stats":
                udp_proxy_call(u, mod, {"stats": True})
            elif a[2] == "clear":
                udp_proxy_call(u, mod, {"clear": True})
            else:
                udp_proxy_call(u, mod, {"set": {
                    "drop_pct": float(a[2]),
                    "reorder": int(a[3]),
                    "dup_pct": float(a[4]),
                    "seed": int(a[5]) if len(a) > 5 else 0}})
    elif c == "param":
        # P5-07 battery/gps faults: runtime SITL parameter set; the
        # PARAM_VALUE echo must carry our name AND value
        ok = True
        for u in with_fc(resolve(a[0])):
            m = mav(u)
            try:
                ok &= fc_param_set(m, u, a[1], a[2])
            finally:
                m.close()
        if not ok:
            sys.exit(1)
    elif c == "param_get":
        ok = True
        for u in with_fc(resolve(a[0])):
            m = mav(u)
            try:
                v = fc_param_get(m, a[1])
            finally:
                m.close()
            if v is None:
                print(f"{u['name']}: {a[1]} not on the FC")
                ok = False
            else:
                print(f"{u['name']}: {a[1]} = {v}")
        if not ok:
            sys.exit(1)
    elif c == "takeoff":
        # NAV_TAKEOFF in the current mode (arm sets GUIDED); waits until
        # the EKF reports >= 80% of the target altitude
        from pymavlink import mavutil
        alt = float(a[1]) if len(a) > 1 else 10
        res = []
        for u in with_fc(resolve(a[0])):
            m = mav(u)
            try:
                m.mav.request_data_stream_send(
                    m.target_system, m.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 4, 1)
                m.mav.command_long_send(
                    m.target_system, m.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, alt)
                t0 = time.time()
                ok = False
                while time.time() - t0 < 60:
                    pos = m.recv_match(type="GLOBAL_POSITION_INT",
                                       timeout=3, blocking=True)
                    if pos is not None and \
                            pos.get_srcSystem() == u["fc"]["sysid"] and \
                            pos.relative_alt >= alt * 800:
                        ok = True
                        break
                print(f"{u['name']}: takeoff {alt}m "
                      f"{'reached' if ok else 'not confirmed'}",
                      flush=True)
                res.append(ok)
            finally:
                m.close()
        if res and not all(res):
            sys.exit(1)
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
    elif c in ("fcb_cut", "fcb_stall", "fcb_rate", "fcb_corrupt",
               "fcb_drop", "fcb_inject", "fcb_clear", "fcb_stats"):
        # SL-0008: the per-unit fcb proxy's control port - the fault is
        # ours, no server involved (any transport works)
        us = with_fc(resolve(a[0]))
        if not us:
            sys.exit(f"{a[0]}: no FC unit in the target set")
        for u in us:
            if c == "fcb_cut":
                fcb_ctrl(u, f"cut {a[1]}")
            elif c == "fcb_stall":
                fcb_ctrl(u, f"stall {a[1]}")
            elif c == "fcb_rate":
                fcb_ctrl(u, f"rate {a[1]}")
            elif c == "fcb_corrupt":
                fcb_ctrl(u, "corrupt %s %s %s"
                         % (a[1], a[2], a[3] if len(a) > 3 else 0))
            elif c == "fcb_drop":
                fcb_ctrl(u, "drop %s %s"
                         % (a[1], a[2] if len(a) > 2 else 100))
            elif c == "fcb_inject":
                fcb_ctrl(u, f"inject {a[1]}")
            elif c == "fcb_clear":
                fcb_ctrl(u, "clear")
            else:
                fcb_ctrl(u, "stats")
    elif c in ("fcb_unplug", "fcb_plug"):
        # SL-0103: the serial-device fault for `fcb_link` units - kill
        # the unit's fcb_tty bridge so the device node vanishes (pty:
        # the link file and /dev/pts/N go with the bridge), plug it back
        # with the same link path. `fcb_unplug <unit> <s>` blocks like
        # `freeze <s>`; the scenario's dur_s schedules `fcb_plug`
        # instead so the timeline is not held.
        us = with_fc(resolve(a[0]))
        if not us:
            sys.exit(f"{a[0]}: no FC unit in the target set")
        ok = True
        for u in us:
            # the serial bridge's proc name follows the link kind
            # (simlib.bridge_proc): socat fronts pty/host_serial,
            # bt_spp_bridge.py fronts BT; kind tcp has no bridge
            proc = simlib.bridge_proc(u)
            if proc is None:
                print(f"{u['name']}: {c} needs an `fcb_link` bridged "
                      "unit (kind pty/host_serial/bt_spp/udp) - kind "
                      "tcp dials the proxy, there is no device to unplug",
                      flush=True)
                ok = False
                continue
            if c == "fcb_unplug":
                ok &= kill_proc_group(u["name"], proc)
            else:
                start_proc(u["name"], proc)
        if c == "fcb_unplug" and len(a) > 1:
            time.sleep(float(a[1]))
            for u in us:
                proc = simlib.bridge_proc(u)
                if proc:
                    start_proc(u["name"], proc)
        if not ok:
            sys.exit(1)
    elif c == "bt_squatters":
        # SL-0105: name foreign (non-sim, non-root) processes holding
        # RFCOMM sockets - one bound before our bt_spp bridge on the
        # same channel keeps winning the client's dials (the kernel
        # does not expose the bound channel in /proc, so any foreign
        # holder is a candidate; the unit's channel is printed for
        # context)
        us = with_fc(resolve(a[0]))
        if not us:
            sys.exit(f"{a[0]}: no FC unit in the target set")
        squats = simlib.bt_rfcomm_squatters()
        for u in us:
            ch = ((u.get("fc") or {}).get("fcb_link") or {}).get(
                "channel", "-")
            if not squats:
                print(f"{u['name']}: ch{ch} clean - no foreign "
                      "RFCOMM sockets")
                continue
            print(f"{u['name']}: ch{ch} squatter candidate(s): "
                  + ", ".join(f"{s['name']}({s['pid']})"
                              f"x{s['sockets']}" for s in squats))
        if squats:
            sys.exit(1)
    elif c == "server_up":
        for p in server_procs("all"):
            if p not in {k.split(".", 1)[1]
                         for k in simlib.read_pids(_ctx_name())
                         if k.startswith("server.")}:
                start_proc("server", p)
    elif c == "add_module":
        a = list(a)
        cam = None
        if "--cam" in a:
            i = a.index("--cam")
            if i + 1 >= len(a):
                sys.exit("--cam needs a value")
            cam = a.pop(i + 1)
            a.pop(i)
        add_module(a[0], a[1], cam)
    elif c == "remove_module":
        remove_module(a[0], a[1])
    elif c in ("kill", "restart", "start_proc"):
        procs = a[1:]
        if a[0] == "server":
            owners = [("server", p) for t in (procs or ["all"])
                      for p in server_procs(t)]
        else:
            if not procs:
                sys.exit(f"usage: {c} <unit> <proc> [proc...]")
            owners = [(u["name"], p) for u in resolve(a[0])
                      for p in procs]
        for owner, p in owners:
            if c in ("kill", "restart"):
                kill_proc(owner, p)
        if c == "restart":
            time.sleep(1)
        for owner, p in owners:
            if c in ("restart", "start_proc"):
                start_proc(owner, p)
    elif c in ("freeze", "unfreeze"):
        if a[0] == "server":
            owners = [("server", p)
                      for p in server_procs(a[1] if len(a) > 1 else "all")]
        else:
            owners = [(u["name"], a[1]) for u in resolve(a[0])]
        ok = True
        for owner, p in owners:
            if c == "freeze":
                ok &= freeze_proc(owner, p,
                                  float(a[2]) if len(a) > 2 else None)
            else:
                ok &= _signal_proc(owner, p, signal.SIGCONT)
        if not ok:
            sys.exit(1)
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
        # SL-0104: an `fc_client: external` unit has no local de_comm -
        # its party never joins the server roster, so it is not awaited
        want = {u["party_id"] for u in us
                if u.get("fc_client") != "external"}
        if not want:
            print("no local units to wait for "
                  "(all fc_client: external)")
            return
        timeout = float(a[0]) if a else 90
        t0 = time.time()
        got = set()
        while time.time() - t0 < timeout:
            seen = set()
            for n in sims:
                seen |= online_parties(n)
            got = seen & want
            if got == want:
                print(simlib.col_green(
                    f"all {len(want)} units online ({time.time()-t0:.0f}s)"))
                return
            time.sleep(1)
        sys.exit(simlib.col_red(
            f"TIMEOUT: {len(got)}/{len(want)} units online after "
            f"{timeout}s; missing {sorted(want - got)}"))
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
    elif c == "_starve":
        # detached duty-cycle worker spawned by cpu_starve - argv is
        # raw pidfile paths, no unit resolution
        _starve_worker(a[0], a[1], float(a[2]), float(a[3]), float(a[4]))
    elif c == "cpu_starve":
        ok = True
        for u in resolve(a[0]):
            ok &= cpu_starve_proc(u["name"], a[1], float(a[2]),
                                  float(a[3]) if len(a) > 3 else 0.0,
                                  float(a[4]) if len(a) > 4 else 1000.0)
        if not ok:
            sys.exit(1)
    elif c == "cpu_unstarve":
        ok = True
        for u in resolve(a[0]):
            ok &= cpu_unstarve_proc(u["name"], a[1])
        if not ok:
            sys.exit(1)
    elif c == "journal_unwritable":
        ok = True
        for u in resolve(a[0]):
            ok &= journal_unwritable(u, float(a[1]) if len(a) > 1 else None)
        if not ok:
            sys.exit(1)
    elif c == "journal_writable":
        ok = True
        for u in resolve(a[0]):
            ok &= journal_set_writable(u, True)
        if not ok:
            sys.exit(1)
    elif c == "clock_step":
        ok = True
        for u in resolve(a[0]):
            ok &= clock_step(u, float(a[1]))
        if not ok:
            sys.exit(1)
    else:
        simlib.print_usage(__doc__)
        sys.exit(1)


if __name__ == "__main__":
    try:
        main(sys.argv[1:])
    except KeyboardInterrupt:
        sys.exit(130)
