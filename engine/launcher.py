#!/usr/bin/env python3
"""launcher.py - bring a sim up / down (used by ./sim up|down).

up: generate var/run/<name>/, start the server side (real/fake spawn their
own; external only runs this sim's GCS bridge), then every unit's
processes (SITL, de_comm, modules), and wait until each unit is online
at the server and every module reports ready. Sims run in parallel -
up restarts only the same-named sim and refuses ports/names that clash
with a live one; `sim run` passes exclusive=True so a test still owns
the machine (SIM_SHARED=1 keeps a test additive instead, for a machine
other sessions are using). down: stop one sim, or every sim.
Fully headless: logs in var/run/<sim>/<unit>/logs/ and var/run/<sim>/server/logs/,
pids in var/run/<sim>/pids/. Only pids whose command line points into this
repo are ever signalled.
"""

import os
import shutil
import signal
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simlib
import ctl


def _pids_dirs(name=None):
    """Pid dirs to tear down: one sim's, or every sim's plus the legacy
    flat var/run/pids/ dir left by the pre-parallel layout."""
    if name:
        return [simlib.pids_dir(name)]
    dirs = [os.path.join(simlib.RUN_DIR, "pids")]
    if os.path.isdir(simlib.RUN_DIR):
        dirs += [os.path.join(simlib.RUN_DIR, d, "pids")
                 for d in sorted(os.listdir(simlib.RUN_DIR))
                 if os.path.isdir(os.path.join(simlib.RUN_DIR, d, "pids"))]
    return dirs


def _proc_stopped(pid):
    """True when /proc shows the pid in state T (SIGSTOP/freeze fault)."""
    try:
        return open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1] \
            .split()[0] == "T"
    except (OSError, IndexError):
        return False


def _unstop(pid):
    """SIGCONT a stopped pid before the SIGTERM below: a frozen process
    never runs its TERM handler and would sit out the whole grace
    window (and leak in state T when the killer is kill -9 shy)."""
    if _proc_stopped(pid):
        try:
            os.kill(pid, signal.SIGCONT)
        except OSError:
            pass


def _journal_restore(sim_dir):
    """A journal_unwritable fault (SL-0014) may have left a unit's
    journal dir read-only - restore the write bits so a post-mortem
    `collect`, the next `sim up`, or a human reading the journals never
    inherits a dead journal."""
    try:
        entries = os.listdir(sim_dir)
    except OSError:
        return
    for ent in entries:
        jdir = os.path.join(sim_dir, ent, "journal")
        if not os.path.isdir(jdir):
            continue
        try:
            os.chmod(jdir, 0o755)
        except OSError:
            pass
        try:
            files = os.listdir(jdir)
        except OSError:
            continue
        for f in files:
            p = os.path.join(jdir, f)
            try:
                if os.path.isfile(p):
                    os.chmod(p, 0o644)
            except OSError:
                pass


def down(name=None, quiet=False):
    dirs = [d for d in _pids_dirs(name) if os.path.isdir(d)]
    strays = _stray_pids(name)
    foreign_kept = False
    if not _force():
        # SL-0053 - a live sim another session owns is never silently
        # downed: keep it, subtract its strays, and say so.
        kept = []
        for d in dirs:
            sname, o = _dir_owner(d)
            if o:
                foreign_kept = True
                print(simlib.col_yellow(
                    f"sim {sname!r} owned by another session "
                    f"(sid {o['sid']}) - kept; --force / SIM_FORCE=1 "
                    "to stop it"), flush=True)
                strays = [p for p in strays
                          if p not in set(_stray_pids(sname))]
            else:
                kept.append(d)
        dirs = kept
    for d in dirs:
        _journal_restore(os.path.dirname(d))
    if not dirs and not strays:
        if not quiet and not foreign_kept:
            print("nothing running" if name is None
                  else f"sim {name!r}: nothing running")
        return
    pids = []
    for d in dirs:
        for f in sorted(os.listdir(d)):
            if not f.endswith(".pid"):
                continue
            path = os.path.join(d, f)
            try:
                pid = int(open(path).read().strip())
            except ValueError:
                pid = None
            os.unlink(path)
            if pid and os.path.isdir(f"/proc/{pid}"):
                if simlib.pid_is_ours(pid):
                    _unstop(pid)
                    os.kill(pid, signal.SIGTERM)
                    pids.append(pid)
                else:
                    print(f"skip pid {pid} ({f[:-4]}): not a sim process")
    t0 = time.time()
    while time.time() - t0 < 3 and any(os.path.isdir(f"/proc/{p}") for p in pids):
        time.sleep(0.2)
    for p in pids:
        if os.path.isdir(f"/proc/{p}") and simlib.pid_is_ours(p):
            os.kill(p, signal.SIGKILL)
    strays = [p for p in strays if p not in pids]
    for p in strays:
        if os.path.isdir(f"/proc/{p}") and simlib.pid_is_ours(p):
            _unstop(p)
            os.kill(p, signal.SIGTERM)
    t0 = time.time()
    while time.time() - t0 < 3 and any(os.path.isdir(f"/proc/{p}") for p in strays):
        time.sleep(0.2)
    for p in strays:
        if os.path.isdir(f"/proc/{p}") and simlib.pid_is_ours(p):
            os.kill(p, signal.SIGKILL)
    # free the sim's machine-wide port leases (foreign-owned sims kept
    # above were already pruned from `dirs`, so their leases survive)
    freed = set()
    for d in dirs:
        sname, _o = _dir_owner(d)
        if sname and sname not in freed:
            simlib.lease_release(sname)
            freed.add(sname)
    if name and name not in freed:
        simlib.lease_release(name)
    if not quiet and (pids or strays or not foreign_kept):
        print(simlib.col_yellow(
            f"sim stopped ({len(pids) + len(strays)} processes)"))


def _stray_pids(name=None):
    """Live pids whose command line runs under var/run/<name>/ (or any var/run/<sim>/
    when name is None) but that no pidfile tracks - e.g. a restart that
    outlived a crashed supervisor.  pid_is_ours still gates every kill."""
    root = os.path.join(simlib.RUN_DIR, name) + "/" if name \
        else simlib.RUN_DIR + "/"
    out = []
    try:
        entries = os.listdir("/proc")
    except OSError:
        return out
    for e in entries:
        if not e.isdigit():
            continue
        try:
            cmd = open(f"/proc/{e}/cmdline", "rb").read() \
                .replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if root in cmd:
            out.append(int(e))
    return out


def wait_port(port, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            socket.create_connection(("127.0.0.1", port), 1).close()
            return True
        except OSError:
            time.sleep(0.5)
    return False


def port_busy(port, host="127.0.0.1", timeout=0.5):
    try:
        socket.create_connection((host, port), timeout).close()
        return True
    except OSError:
        return False


def make_cert(name):
    cert, key = simlib.cert_path(name), simlib.key_path(name)
    if os.path.isfile(cert) and os.path.isfile(key):
        return
    os.makedirs(os.path.dirname(cert), exist_ok=True)
    subprocess.check_call(
        ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "365",
         "-subj", "/CN=127.0.0.1", "-keyout", key, "-out", cert],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _label(sim_src):
    return os.path.abspath(sim_src) if isinstance(sim_src, str) else "inline"


def _shared():
    return os.environ.get("SIM_SHARED") == "1"


def _force():
    """SIM_FORCE=1 / --force: take over a sim another live session owns
    (SL-0053) - down it where the owner guard would refuse."""
    return os.environ.get("SIM_FORCE") == "1"


def _dir_owner(pids_dir):
    """(sim_name, owner) for one pids dir - owner only when the sim is
    live AND owned by a foreign live session (SL-0053). The legacy flat
    var/run/pids/ dir names no sim, so it is always (None, None)."""
    sname = os.path.basename(os.path.dirname(pids_dir))
    if sname == os.path.basename(simlib.RUN_DIR):
        return None, None
    if sname not in simlib.live_sim_names():
        return sname, None
    return sname, simlib.foreign_owner(
        simlib.load_manifest(sname, required=False))


def up(sim_src, fresh=False, timeout=180, exclusive=False,
       world_stream=False, open_web=False):
    """sim_src: a sim file path, or an inline sim dict (from a test).
    Restarts only the same-named sim; `exclusive` (test runs) downs every
    other sim first. `world_stream` says the caller will serve
    de.worldstream/1 on render.stream_port (a scenario with a world) -
    the printed viewer URL only advertises ?world= then. `open_web`
    xdg-opens the webclient deep link once the fleet is up."""
    sim = simlib.load_sim(sim_src,
                          exclusive=exclusive and not _shared())
    name = sim["name"]
    srv = sim["server"]
    simlib.check_binaries(sim)          # before stopping the running sim
    if exclusive and not _shared():
        if not _force():
            # SL-0053 - 'tests own the machine' must not silently down
            # a sim another session owns; refuse with the way out.
            foreign = [(n, simlib.foreign_owner(
                            simlib.load_manifest(n, required=False)))
                       for n in simlib.live_sim_names()]
            foreign = [(n, o) for n, o in foreign if o]
            if foreign:
                sys.exit(simlib.col_red(
                    "cannot own the machine: sim(s) "
                    + ", ".join(f"{n} (session {o['sid']})"
                                for n, o in foreign)
                    + " belong to another live session - "
                    "`--shared --slot-base any` runs beside them, "
                    "--force downs them"))
        down(quiet=True)                # tests own the machine
    else:
        # additive `sim up`: refuse ports of a live sim (and unit names
        # where they still must be unique), restart only the same-named
        # one
        simlib.check_conflicts(sim)
        # refuse ports another worktree leases or a foreign process
        # binds - before the same-name down() below frees ours
        simlib.lease_check(sim)
        if not _force() and name in simlib.live_sim_names():
            fo = simlib.foreign_owner(
                simlib.load_manifest(name, required=False))
            if fo:
                sys.exit(simlib.col_red(
                    f"sim {name!r} is owned by another session "
                    f"(sid {fo['sid']}) and still running - "
                    "--force / SIM_FORCE=1 to take it over"))
        down(name, quiet=True)
    if srv["type"] == "external":
        # nothing local to bind except this sim's control port; but the
        # user's server must be reachable or no unit ever comes online
        if port_busy(srv["control_port"]):
            sys.exit(simlib.col_red(
                f"control port {srv['control_port']} is in use - set "
                "`server: {control_port: ...}` in the sim"))
        host, port = (srv["auth_ip"], srv["auth_port"]) if srv["auth"] \
            else (srv["ip"], srv["port"])
        if not port_busy(port, host, timeout=5):
            sys.exit(simlib.col_red(
                f"cannot reach the external server at {host}:{port} - "
                "start your servers (servers/sh_start_servers.sh) or "
                "fix the sim's server: block"))
    else:
        busy = [p for p in ([srv["port"], srv["control_port"]]
                            + ([srv["auth_port"], srv["auth_s2s_port"]]
                               if srv["auth"] else [])
                            + ([srv["udp_relay"]["port"]]
                               if srv.get("udp_relay") else []))
                if port_busy(p)]
        if busy:
            sys.exit(simlib.col_red(
                f"ports {busy} are in use by something else - either "
                "down what holds them (./sim status), set "
                "`server: {port: ...}` in the sim, or rerun with "
                "--shared --slot-base any to park next to live sims "
                "on a free port block (the server port rides the "
                "slot for transport: scenarios)"))
    rdir = simlib.run_dir(name)
    if fresh and os.path.isdir(rdir):
        shutil.rmtree(rdir)
    os.makedirs(simlib.pids_dir(name), exist_ok=True)
    m = simlib.generate(sim, _label(sim_src))
    # lease the whole port set atomically - all of it or none of it,
    # then the procs below bind what the registry already names ours
    simlib.lease_commit(sim)
    conn = (f"external server {srv['ip']}:{srv['port']}"
            + (f" via auth {srv['auth_ip']}:{srv['auth_port']}"
               if srv["auth"] else "")
            if srv["type"] == "external" else
            f"server {srv['type']}{' +auth' if srv['auth'] else ''} "
            f"on {srv['port']}")
    print(f"sim {simlib.col_bold(name)}: {len(m['units'])} unit(s), "
          f"slot {sim['slot_base']}, " + simlib.col_cyan(conn))
    # the resolved fleet: unit, vehicle and ports - the initial settings
    # the run is about to launch with
    for u in m["units"]:
        fc = u["fc"]
        print("  " + simlib.col_green(f"{u['name']:12s}")
              + (f"{simlib.col_yellow(fc['vehicle'])} "
                 f"sysid={fc['sysid']} "
                 f"pilot={simlib.col_cyan(fc['pilot_port'])} "
                 f"fcb={simlib.col_cyan(fc['fcb_port'])} "
                 + (f"fcbpx={simlib.col_cyan(fc['fcb_proxy']['port'])}/"
                    f"{simlib.col_cyan(fc['fcb_proxy']['ctrl'])} "
                    if fc.get("fcb_proxy") else "")
                 if fc else "no-fc ")
              + " ".join(f"{mod}:{simlib.col_cyan(p)}"
                         for mod, p in u["ports"].items())
              + (f" linkpx={simlib.col_cyan(u['link_proxy']['port'])}"
                 f"/{simlib.col_cyan(u['link_proxy']['ctrl'])}"
                 if u.get("link_proxy") else "")
              + (f" acc={u['account']['user']}" if u.get("account") else ""))

    # ---- server side ----
    ctl.CTX = name
    if srv["type"] == "fake":
        make_cert(name)
    for p in m["server_procs"]:
        ctl.start_proc("server", p)
        if p == "de_auth" and not wait_port(srv["auth_port"], 30):
            sys.exit(simlib.col_red(
                "authenticator did not open its port - see "
                f"{rdir}/server/logs/de_auth.log"))
        if p == "udp_relay" and not wait_port(
                srv["udp_relay"]["port"], 30):
            sys.exit(simlib.col_red(
                "udp relay did not open its control port - see "
                f"{rdir}/server/logs/udp_relay.log"))
        if p in ("de_server", "fake_server") and not wait_port(srv["port"], 30):
            sys.exit(simlib.col_red(
                f"{p} did not open {srv['port']} - "
                f"see {rdir}/server/logs/{p}.log"))
    if not wait_port(srv["control_port"], 15):
        sys.exit(simlib.col_red(
            f"control port did not open - see {rdir}/server/logs/"))

    # ---- render viewer (P5-14): Mavlink3DMap bridge + vite dev ----
    render = m.get("render") or {}
    for p in m.get("render_procs", []):
        ctl.start_proc("render", p)
    if render and m.get("render_procs"):
        if not wait_port(render["mav_ws_port"], 15):
            sys.exit(simlib.col_red(
                f"mav_bridge did not open {render['mav_ws_port']} - "
                f"see {rdir}/render/logs/mav_bridge.log"))
        # vite cold-starts slowly on a big dep tree; warn, don't fail -
        # the URL still works once it is up
        if not wait_port(render["web_port"], 30):
            print(simlib.col_yellow(
                f"web viewer slow to open {render['web_port']} - "
                f"see {rdir}/render/logs/web.log"))
        print("render viewer: " + simlib.col("1;36",
              simlib.render_viewer_url(render, world_stream)))
        if render.get("mode") in ("cameras", "full") \
                and "render_sink" in m.get("render_procs", []):
            if not wait_port(render["sink_port"], 15):
                sys.exit(simlib.col_red(
                    f"render_sink did not open {render['sink_port']} "
                    f"- see {rdir}/render/logs/render_sink.log"))
            print("render farm: "
                  + simlib.col_cyan(f"ws://127.0.0.1:{render['sink_port']}")
                  + " + headless chromium")

    # ---- units ----
    for u in m["units"]:
        simlib.save_boot_mark(u)
    for u in m["units"]:
        for p in simlib.procs_of(u):
            ctl.start_proc(u["name"], p)

    print(simlib.col_dim("waiting for every unit to come online ..."),
          flush=True)
    try:
        ctl.main(["wait_online", str(timeout)])
    except SystemExit as e:
        if e.code:
            print(e.code)
            sys.exit(simlib.col_red(
                "sim up FAILED (units offline) - check "
                "var/run/<unit>/logs/de_comm.log"))
    if not ctl.wait_ready(m["units"], timeout):
        sys.exit(simlib.col_red(
            "sim up FAILED (modules not ready) - check var/run/<unit>/logs/"))
    simlib.lease_mark_bound(name)
    print(simlib.col_green("sim up.")
          + " operator console: ./sim ctl status"
          + (simlib.col_yellow(
             "\nSITL needs ~1-2 min for GPS/EKF before `arm` succeeds.")
             if any(u["fc"] for u in m["units"]) else ""))
    # the GCS deep links - always printed; --open-web only adds the
    # browser open
    print("webclient: " + simlib.col_cyan(simlib.webclient_url(m)))
    # a second GCS needs its own senderID - same-party connections kick
    # each other off the comm server
    print("planning:  " + simlib.col_cyan(
        simlib.webclient_url(m, "/planning", "GCSPLN_" + name)))
    if open_web:
        ctl.xdg_open(simlib.webclient_url(m))
    return m


def ensure(sim_src, timeout=180, world_stream=False, exclusive=True):
    """Reuse the running sim if it is the same sim (hash) and every
    process is alive; otherwise bring it up fresh. `exclusive` (the
    default: tests own the machine) downs every other sim on the reuse
    path and passes through to `up` on the fresh path; additive callers
    (a bare `run`/`view` of a sims/*.yaml) pass exclusive=False so other
    live sims stay up - the fresh up then does what `sim up` does
    (conflict check, same-name restart). `world_stream` is passed to
    `up` for the viewer URL's ?world= param."""
    sim = simlib.load_sim(sim_src,
                          exclusive=exclusive and not _shared())
    m = simlib.load_manifest(sim["name"], required=False)
    if m and not _force() and sim["name"] in simlib.live_sim_names():
        # SL-0053 - reusing, swapping or restarting a sim another live
        # session owns is the same silent takeover `up` refuses.
        fo = simlib.foreign_owner(m)
        if fo:
            sys.exit(simlib.col_red(
                f"sim {sim['name']!r} is owned by another session "
                f"(sid {fo['sid']}) and still running - "
                "--force / SIM_FORCE=1 to take it over"))
    if m and m.get("sim_hash") == sim["hash"]:
        alive = set(simlib.read_pids(sim["name"]))
        need = {f"server.{p}" for p in m["server_procs"]}
        need |= {f"render.{p}" for p in m.get("render_procs", [])}
        for u in m["units"]:
            need |= {f"{u['name']}.{p}" for p in simlib.procs_of(u)}
        if need <= alive:
            if exclusive:
                # a test owns the machine on the reuse path too
                for other in ([] if _shared()
                              else simlib.live_sim_names()):
                    if other != sim["name"]:
                        down(other, quiet=True)
            ctl.CTX = sim["name"]
            print(simlib.col_dim(
                f"sim {sim['name']} already up and unchanged - reusing"),
                flush=True)
            render = m.get("render") or {}
            if render and m.get("render_procs"):
                print("render viewer: " + simlib.col("1;36",
                      simlib.render_viewer_url(render, world_stream)),
                      flush=True)
            print("webclient: "
                  + simlib.col_cyan(simlib.webclient_url(m)))
            print("planning:  "
                  + simlib.col_cyan(simlib.webclient_url(
                      m, "/planning", "GCSPLN_" + sim["name"])))
            return m
        if alive:
            print(simlib.col_yellow(
                f"sim {sim['name']} incomplete "
                f"(down: {sorted(need - alive)}) - restarting"),
                flush=True)
        else:
            print(simlib.col_dim(f"sim {sim['name']} is down - starting"),
                  flush=True)
    elif m and _swap_candidate(m, sim) and _sim_alive(m):
        # identical fleet spec; only the --module binaries differ -
        # swap just those procs instead of a cold SITL restart
        try:
            if _swap_modules(sim, m, timeout, exclusive):
                return m
        except simlib.SimError as e:
            print(simlib.col_yellow(f"module swap failed ({e})"
                                    " - restarting"), flush=True)
    elif m:
        print(simlib.col_yellow(
            f"sim {sim['name']} spec changed - restarting"), flush=True)
    return up(sim_src, fresh=True, timeout=timeout, exclusive=exclusive,
              world_stream=world_stream)


def _swap_candidate(m, sim):
    """The live manifest is the same fleet under a different --module
    set: rehashing the new spec with the manifest's stored overrides
    reproduces its sim_hash exactly - so overrides alone differ."""
    if not (simlib.module_overrides() or m.get("module_overrides")):
        return False
    return m.get("sim_hash") == \
        simlib.sim_hash(sim, m.get("module_overrides"))


def _sim_alive(m):
    """Every proc the manifest expects is running (a downed manifest is
    post-mortem state, not a swap target)."""
    alive = set(simlib.read_pids(m["name"]))
    need = {f"server.{p}" for p in m["server_procs"]}
    need |= {f"render.{p}" for p in m.get("render_procs", [])}
    for u in m["units"]:
        need |= {f"{u['name']}.{p}" for p in simlib.procs_of(u)}
    return bool(need) and need <= alive


def _swap_modules(sim, m, timeout=60, exclusive=True):
    """Restart just the --module-overridden procs on the live same-fleet
    sim: rewrite each affected launch.<proc>.sh at the new binary,
    restart it, wait for its catalog ready line, adopt the new hash."""
    new_ov = simlib.module_overrides()
    old_ov = m.get("module_overrides") or {}
    changed = sorted(k for k in set(new_ov) | set(old_ov)
                     if new_ov.get(k) != old_ov.get(k))
    if not changed:
        return True
    if exclusive:
        # a test owns the machine on the swap path too
        for other in ([] if _shared() else simlib.live_sim_names()):
            if other != sim["name"]:
                down(other, quiet=True)
    cat = simlib.load_catalog()
    ctl.CTX = sim["name"]
    marks = {u["name"]: simlib.journal_mark(u)
             for u in m["units"]
             if set(simlib.procs_of(u)) & set(changed)}
    for u in m["units"]:
        for mod in set(simlib.procs_of(u)) & set(changed):
            old_bin = simlib.bin_with_overrides(
                mod, cat.get(mod), old_ov)
            new_bin = simlib.resolve_bin(mod, cat.get(mod))
            if old_bin == new_bin:
                continue
            simlib.swap_proc_bin(u, mod, old_bin, new_bin)
            print(simlib.col_yellow(
                f"{u['name']}.{mod}: {old_bin} -> {new_bin}"),
                flush=True)
            ctl.main(["restart", u["name"], mod])
    # a restarted module does not re-emit its journal ready_re (de_comm
    # journals e.g. 'nav adapter hello' once per de_comm process, not per
    # module connect) - so readiness here = the module's own proc log
    # showing fresh output since the mark (its catalog ready_log_re when
    # it has one)
    t0 = time.time()
    pending = {(u["name"], mod) for u in m["units"]
               for mod in set(simlib.procs_of(u)) & set(changed)}
    byname = {u["name"]: u for u in m["units"]}
    while pending and time.time() - t0 < 30:
        for uname, mod in sorted(pending):
            rx = (cat.get(mod) or {}).get("ready_log_re") or r"\S"
            if simlib.log_match(byname[uname], mod, rx, marks[uname]):
                pending.discard((uname, mod))
        if pending:
            time.sleep(1)
    if pending:
        print(simlib.col_yellow("module swap: no fresh output from "
              + ", ".join(f"{n}.{p}" for n, p in sorted(pending))
              + " - continuing anyway"), flush=True)
    # the s2s reconnect to de_comm (and de_mavlink's FC link) takes a
    # few seconds past the first log line
    time.sleep(3)
    m["sim_hash"] = sim["hash"]
    m["module_overrides"] = dict(new_ov)
    simlib.write_json(simlib.manifest_path(sim["name"]), m, indent=2)
    print(simlib.col_dim(
        f"sim {sim['name']} module swap done - reusing the warm sim"),
        flush=True)
    return True


# --------------------------------------------------------------- studio


def m3d_ready():
    """Mavlink3DMap usable on this machine: the udp2websocket bridge
    script plus a vite.js under the checkout - the same pair gen_render
    and doctor look for."""
    m3d = simlib.MAVLINK3DMAP_DIR
    bridge = os.path.join(m3d, "backend", "src", "udp2websocket.js")
    vite = next((p for p in (
            os.path.join(m3d, "node_modules", "vite", "bin", "vite.js"),
            os.path.join(m3d, "frontend", "node_modules", "vite",
                         "bin", "vite.js"))
            if os.path.isfile(p)), None)
    return os.path.isfile(bridge) and bool(vite)


def _studio_world(name_or_path):
    """--world's value: a path as given, else a worlds/<name>[.yaml]
    shorthand under the sim root."""
    if os.path.isfile(name_or_path):
        return os.path.abspath(name_or_path)
    for cand in (os.path.join(simlib.SIM_ROOT, "worlds", name_or_path),
                 os.path.join(simlib.SIM_ROOT, "worlds",
                              str(name_or_path) + ".yaml")):
        if os.path.isfile(cand):
            return cand
    raise simlib.SimError(
        f"--world {name_or_path!r}: not a file and no "
        f"worlds/{name_or_path}.yaml")


def _studio_poses(m, tele):
    """{name: {lat,lng,cls}} for the world's react authority - the
    scenario runner's _fleet_poses minus the fleet cls table (a sim
    file has no cls aliases; the block name is it)."""
    out = {}
    for u in m["units"]:
        if u.get("fc"):
            row = tele.latest(u["name"]) if tele else {}
            if row.get("lat") is not None:
                out[u["name"]] = {"lat": row["lat"], "lng": row["lng"],
                                  "cls": u["block"]}
        else:
            p = u.get("position") or {}
            out[u["name"]] = {"lat": p.get("lat"), "lng": p.get("lng"),
                              "cls": u["block"]}
    return out


def _studio_spec_name(raw, sim_src):
    """The name load_sim derives for a file spec, recomputed for the
    inline dict we hand ensure() (a dict has no filename to stem from):
    explicit name: wins, else the file stem plus one _overlay suffix
    per applied overlay - own with: + SIM_WITH, the same list
    _spec_overlays builds."""
    if raw.get("name"):
        return raw["name"]
    stem = (os.path.splitext(os.path.basename(sim_src))[0]
            if isinstance(sim_src, str) else "studio")
    own = raw.get("with") or []
    if isinstance(own, str):
        own = [x.strip() for x in own.split(",") if x.strip()]
    env = [x.strip() for x in
           str(simlib.setting("SIM_WITH", "")).split(",") if x.strip()]
    applied = [os.path.splitext(os.path.basename(x))[0]
               for x in list(own) + env]
    return stem + ("_" + "_".join(applied) if applied else "")


def studio(sim_src, world=None, record_out=None, teardown=False,
           timeout=180):
    """`./sim studio` - the human session in one command (SL-0029):
    bring the sim up (reuse if warm), open the webclient in planning
    mode, serve the de.worldstream/1 feed + Mavlink3DMap viewer when
    available (studio still runs without it: the sim file's own
    render: wins, a spec without one gets {mode: viewer} so the
    stream's port exists, and a missing Mavlink3DMap just drops the
    viewer procs - never the session), record the session through
    record.py's command_plugin seam when --record names a file, and
    park until Ctrl-C. On exit the recording is finalized and the sim
    stays up unless teardown (--down).

    `world` is a world file path or a worlds/<name> shorthand - the
    spec gets the same injection a scenario run does (stub config,
    geo_projection, SITL wind via world_mod.prepare_sim) and the
    epoch + truth tick + stream start the same way, minus the
    assertions.
    """
    import world as world_mod
    import telemetry as telemetry_mod

    seed = 1
    m3d = m3d_ready()
    # the raw spec (extends: applied) so world/render injection lands
    # before unit expansion; untouched specs take the file path
    # through ensure() unchanged
    raw, _src = simlib._sim_spec(sim_src, None)
    spec_edit = False
    render = raw.get("render")
    if render is None and (m3d or world):
        # studio is planner + 3D viewer: a spec that never asked for
        # one gets the viewer mode - its stream_port is also where a
        # --world feed is served
        raw["render"] = {"mode": "viewer"}
        spec_edit = True
    elif isinstance(render, dict) and not m3d \
            and not render.get("headless"):
        # Mavlink3DMap absent must not break the session: run the
        # stream without the viewer procs (headless keeps the render
        # spec's ports, so the sim hash is unchanged)
        raw["render"] = dict(render, headless=True)
        spec_edit = True
    wld = None
    if world:
        wpath = _studio_world(world)
        wld = world_mod.load(wpath, seed)
        # prepare_sim's pose_port wants the slot the sim will actually
        # resolve to - the same math load_sim applies ('any' cannot be
        # known this early, so it falls back to the spec's own)
        ov = str(os.environ.get("SIM_SLOT_BASE_OVERRIDE") or "")
        raw["slot_base"] = int(ov) if ov.isdigit() else \
            int(raw.get("slot_base", simlib.SLOT_BASE))
        # the same world injection a scenario run gets (stub config,
        # geo_projection, SITL wind) before the fleet resolves
        world_mod.prepare_sim({"sim": raw, "world": wpath,
                               "seed": seed}, wld)
        spec_edit = True

    if spec_edit:
        raw.setdefault("name", _studio_spec_name(raw, sim_src))
        m = ensure(raw, exclusive=False, timeout=timeout,
                   world_stream=wld is not None)
        if isinstance(sim_src, str):
            # the manifest's provenance is still the file, not
            # 'inline' (record writes it as the scenario's sim: base)
            src_file = os.path.abspath(sim_src)
            if m.get("sim_file") != src_file:
                m["sim_file"] = src_file
                simlib.write_json(simlib.manifest_path(m["name"]), m,
                                  indent=2)
    else:
        m = ensure(sim_src, exclusive=False, timeout=timeout,
                   world_stream=wld is not None)
    name = m["name"]
    ctl.CTX = name

    tele = recorder = stream = feeder = None
    if wld is not None:
        # the parked-scenario world wiring: epoch (the stubs' start
        # gun), the 1 Hz truth tick = react authority, the ADS-B
        # feeder, and the de.worldstream/1 feed on stream_port
        world_mod.clear_run_state(name)
        epoch_ms = world_mod.write_epoch(name, wpath, seed)
        tele = telemetry_mod.Telemetry(
            m["units"], os.path.join(simlib.run_dir(name),
                                     "telemetry.jsonl"))
        tele.start()
        recorder = world_mod.TruthRecorder(
            wld, epoch_ms, world_mod.truth_path(name),
            overrides_path=world_mod.overrides_path(name),
            fleet_poses=lambda: _studio_poses(m, tele))
        recorder.start()
        if wld.aircraft():
            adsb_units = [u for u in m["units"]
                          if u["fc"] and u["fc"].get("fcb_proxy")]
            if adsb_units:
                feeder = world_mod.AdsbFeeder(
                    wld, epoch_ms, adsb_units, m["home"][2])
                feeder.start()
            else:
                print("  world has cls:aircraft entities but no FC "
                      "unit runs fcb_proxy - no ADS-B injection",
                      flush=True)
        render = m.get("render") or {}
        if render:
            if port_busy(render["stream_port"]):
                print(simlib.col_yellow(
                    f"world stream port {render['stream_port']} is "
                    "already bound - the viewer's ?world= feed may be "
                    "stale"), flush=True)
            import world_stream
            stream = world_stream.WorldStream(
                wld, m, epoch_ms, render["stream_port"],
                world_mod.truth_path(name),
                world_stream.log_path(name))
            stream.start()

    rec_t0 = None
    record_mod = None
    if record_out:
        # arm before the session opens so the first click lands in the
        # file (arm() bounces de_server once on a real server)
        import record as record_mod
        rec_t0 = record_mod.arm(m)

    print(simlib.col_bold(f"\n== studio: {name} - Ctrl-C to finish "
                          "the session =="), flush=True)
    # prints the webclient + planning deep links and xdg-opens the
    # planning one
    ctl.cmd_web(name, open_browser=True, planning=True)
    render = m.get("render") or {}
    if not m3d:
        print(simlib.col_yellow(
            "viewer:    skipped - Mavlink3DMap not found (set "
            "MAVLINK3DMAP_DIR in sim.local.env)"), flush=True)
    elif render:
        print("viewer:    " + simlib.col("1;36",
              simlib.render_viewer_url(render, wld is not None)),
              flush=True)
    if wld is not None:
        print(f"world:     {wpath}", flush=True)
    online = ", ".join(sorted(ctl.online_parties(name)))
    print("online:    "
          + (simlib.col_green(online) if online else "-"), flush=True)
    if record_out:
        print("record:    " + simlib.col_cyan(record_out), flush=True)
    print(simlib.col_dim("--down: sim comes down on exit"
                         if teardown else "sim stays up on exit"),
          flush=True)

    def _term(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("studio released\n", flush=True)
    finally:
        if rec_t0 is not None:
            record_mod.disarm(m)
            try:
                n_steps, n_unmapped, n_noise = record_mod.finalize(
                    m, rec_t0, record_out)
                print(f"wrote {record_out}: {n_steps} step(s)"
                      + (f", {n_unmapped} unmapped frame(s) as comments"
                         if n_unmapped else "")
                      + f" ({n_noise} routine frames skipped)",
                      flush=True)
            except Exception as e:
                print(f"  record finalize failed: {e}", flush=True)
        if stream is not None:
            stream.stop()
        if feeder is not None:
            feeder.stop()
        if recorder is not None:
            recorder.stop()
        if tele is not None:
            tele.stop()
        if teardown:
            down(name)
        else:
            print(f"sim {simlib.col_bold(name)} is still up - "
                  f"`./sim down {name}` to stop", flush=True)
    return m
