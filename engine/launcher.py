#!/usr/bin/env python3
"""launcher.py - bring a sim up / down (used by ./sim up|down).

up: stop what a previous `sim up` started, generate run/, start the server
side, then every unit's processes (SITL, de_comm, modules), and wait until
each unit is online at the server and every module reports ready.
Fully headless: logs in run/<unit>/logs/ and run/server/logs/, pids in
run/pids/. Only pids whose command line points into this repo are ever
signalled.
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


def down(quiet=False):
    if not os.path.isdir(simlib.PIDS_DIR):
        if not quiet:
            print("nothing running")
        return
    pids = []
    for f in sorted(os.listdir(simlib.PIDS_DIR)):
        if not f.endswith(".pid"):
            continue
        path = os.path.join(simlib.PIDS_DIR, f)
        try:
            pid = int(open(path).read().strip())
        except ValueError:
            pid = None
        os.unlink(path)
        if pid and os.path.isdir(f"/proc/{pid}"):
            if simlib.pid_is_ours(pid):
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
    if not quiet:
        print(f"sim stopped ({len(pids)} processes)")


def wait_port(port, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            socket.create_connection(("127.0.0.1", port), 1).close()
            return True
        except OSError:
            time.sleep(0.5)
    return False


def port_busy(port):
    try:
        socket.create_connection(("127.0.0.1", port), 0.5).close()
        return True
    except OSError:
        return False


def make_cert():
    if os.path.isfile(simlib.CERT) and os.path.isfile(simlib.KEY):
        return
    os.makedirs(os.path.dirname(simlib.CERT), exist_ok=True)
    subprocess.check_call(
        ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "365",
         "-subj", "/CN=127.0.0.1", "-keyout", simlib.KEY, "-out", simlib.CERT],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _label(sim_src):
    return os.path.abspath(sim_src) if isinstance(sim_src, str) else "inline"


def up(sim_src, fresh=False, timeout=180):
    """sim_src: a sim file path, or an inline sim dict (from a test)."""
    sim = simlib.load_sim(sim_src)
    simlib.check_binaries(sim)          # before stopping the running sim
    down(quiet=True)
    srv = sim["server"]
    busy = [p for p in ([srv["port"], srv["control_port"]]
                        + ([srv["auth_port"], srv["auth_s2s_port"]] if srv["auth"] else []))
            if port_busy(p)]
    if busy:
        sys.exit(f"ports {busy} are in use by something else - set "
                 "`server: {port: ...}` in the sim")
    if fresh and os.path.isdir(simlib.RUN_DIR):
        shutil.rmtree(simlib.RUN_DIR)
    os.makedirs(simlib.PIDS_DIR, exist_ok=True)
    m = simlib.generate(sim, _label(sim_src))
    print(f"sim {os.path.basename(_label(sim_src))}: {len(m['units'])} unit(s), "
          f"server {srv['type']}{' +auth' if srv['auth'] else ''} on {srv['port']}")

    # ---- server side ----
    if srv["type"] == "fake":
        make_cert()
    for p in m["server_procs"]:
        ctl.start_proc("server", p)
        if p == "de_auth" and not wait_port(srv["auth_port"], 30):
            sys.exit("authenticator did not open its port - see run/server/logs/de_auth.log")
        if p in ("de_server", "fake_server") and not wait_port(srv["port"], 30):
            sys.exit(f"{p} did not open {srv['port']} - see run/server/logs/{p}.log")
    if not wait_port(srv["control_port"], 15):
        sys.exit("control port did not open - see run/server/logs/")

    # ---- units ----
    for u in m["units"]:
        simlib.save_boot_mark(u)
    for u in m["units"]:
        for p in simlib.procs_of(u):
            ctl.start_proc(u["name"], p)

    print("waiting for every unit to come online ...", flush=True)
    try:
        ctl.main(["wait_online", str(timeout)])
    except SystemExit as e:
        if e.code:
            print(e.code)
            sys.exit("sim up FAILED (units offline) - check run/<unit>/logs/de_comm.log")
    if not ctl.wait_ready(m["units"], timeout):
        sys.exit("sim up FAILED (modules not ready) - check run/<unit>/logs/")
    print("sim up. operator console: ./sim ctl status"
          + ("\nSITL needs ~1-2 min for GPS/EKF before `arm` succeeds."
             if any(u["fc"] for u in m["units"]) else ""))
    return m


def ensure(sim_src, timeout=180):
    """Reuse the running sim if it is the same sim (hash) and every
    process is alive; otherwise bring it up fresh."""
    sim = simlib.load_sim(sim_src)
    m = simlib.load_manifest(required=False)
    if m and m.get("sim_hash") == sim["hash"]:
        alive = set(simlib.read_pids())
        need = {f"server.{p}" for p in m["server_procs"]}
        for u in m["units"]:
            need |= {f"{u['name']}.{p}" for p in simlib.procs_of(u)}
        if need <= alive:
            return m
        print(f"sim incomplete (down: {sorted(need - alive)}) - restarting")
    return up(sim_src, fresh=True, timeout=timeout)
