"""`./sim up de_servers` / `./sim servers` - the developer's own server
stack: the headless equivalent of servers/sh_start_servers.sh.

Each server runs FROM ITS REPO with no --config override, so it loads
its own server.config + server.config.local on the real ports - the
stack `server: {type: external}` sims join (de_auth -> de_server ->
de_storage -> de_udp_proxy, in the script's order).

Deliberately NOT a sim: no manifest under var/run/, and the procs carry
the --de_servers marker instead of a var/run path, so simlib.pid_is_ours
stays False for them - `sim down`, `sim run`'s exclusive teardown and
`sim clean` can never sweep the stack mid-test (an external test needs
its server alive). Only `./sim down de_servers` stops it. State lives
under var/servers/: launch.<proc>.sh, pids/, logs/.
"""

import json
import os
import signal
import socket
import subprocess
import sys
import time

import simlib

STACK_DIR = os.path.join(simlib.VAR_DIR, "servers")
PIDS_DIR = os.path.join(STACK_DIR, "pids")
LOGS_DIR = os.path.join(STACK_DIR, "logs")

# proc -> (repo, env key for error hints, port env override if any).
# Launch order matters: auth first so the comm server's s2s link dials a
# live target (the old script's `sleep 1` chain).
PROCS = [
    ("de_auth",      simlib.AUTH_REPO,      "DE_AUTH_DIR",      None),
    ("de_server",    simlib.SERVER_REPO,    "DE_SERVER_DIR",    None),
    ("de_storage",   simlib.STORAGE_REPO,   "DE_STORAGE_DIR",
     "de_storage_server_port"),
    ("de_udp_proxy", simlib.UDP_RELAY_REPO, "DE_UDP_RELAY_DIR", None),
]

# argv marker identifying our procs. Must NOT be a var/run or engine
# path - that is what keeps pid_is_ours False and the stack out of every
# sim teardown.
MARK = "--de_servers"


def _pidfile(proc):
    return os.path.join(PIDS_DIR, proc + ".pid")


def _pid_is_stack(pid):
    """The pid is alive and its cmdline carries the stack marker."""
    try:
        cmd = open(f"/proc/{pid}/cmdline", "rb").read() \
            .replace(b"\0", b" ").decode(errors="replace")
    except OSError:
        return False
    return MARK in cmd


def pids():
    """{proc: pid} of stack procs alive and marked."""
    out = {}
    if not os.path.isdir(PIDS_DIR):
        return out
    for f in os.listdir(PIDS_DIR):
        if not f.endswith(".pid"):
            continue
        try:
            pid = int(open(os.path.join(PIDS_DIR, f)).read().strip())
            os.kill(pid, 0)
        except (ValueError, OSError):
            continue
        if _pid_is_stack(pid):
            out[f[:-4]] = pid
    return out


def _port(proc, repo, env_key):
    """The port the server will bind: its repo's server.config +
    server.config.local (the same shallow merge the common loader does),
    then the env override some servers honour."""
    cfg = simlib.load_config(os.path.join(repo, "server.config"))
    loc = os.path.join(repo, "server.config.local")
    if os.path.isfile(loc):
        cfg.update(simlib.load_config(loc))
    if env_key and os.environ.get(env_key):
        return int(os.environ[env_key])
    return int(cfg.get("server_port") or 0)


def _wait_port(port, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            socket.create_connection(("127.0.0.1", port), 1).close()
            return True
        except OSError:
            time.sleep(0.5)
    return False


def _launch_script(proc, repo):
    """Run in the repo (node_modules + its own server.config(.local)
    resolve there); the --de_servers marker is ignored by the server but
    identifies the pid as this stack's."""
    path = os.path.join(STACK_DIR, f"launch.{proc}.sh")
    simlib.write_text(path,
                      "#!/bin/bash\ncd '%s'\necho $$ > '%s'\n"
                      "exec '%s' ./src/server.js %s\n"
                      % (repo, _pidfile(proc), simlib.NODE, MARK), 0o755)
    return path


def up(fresh=False):
    """Start the four servers in order, waiting on each one's port."""
    if fresh:
        down(quiet=True)
    os.makedirs(PIDS_DIR, exist_ok=True)
    os.makedirs(LOGS_DIR, exist_ok=True)
    live = pids()
    if len(live) == len(PROCS):
        print("de_servers already up - `./sim servers` for status, "
              "--fresh to restart")
        return
    for proc, repo, env_key, port_env in PROCS:
        if proc in live:
            print(f"{simlib.col_dim(f'de_servers.{proc}')}: "
                  f"already running pid {live[proc]}")
            continue
        if not os.path.isfile(os.path.join(repo, "src", "server.js")):
            raise simlib.SimError(
                f"{proc}: no src/server.js under {repo} - set {env_key} "
                "in sim.local.env")
        script = _launch_script(proc, repo)
        log = os.path.join(LOGS_DIR, proc + ".log")
        subprocess.Popen(["setsid", "nohup", "bash", script],
                         stdout=open(log, "a"), stderr=subprocess.STDOUT,
                         stdin=subprocess.DEVNULL)
        t0 = time.time()
        while time.time() - t0 < 3 and not os.path.isfile(_pidfile(proc)):
            time.sleep(0.1)
        port = _port(proc, repo, port_env)
        if port and not _wait_port(port, 30):
            raise simlib.SimError(
                f"{proc} did not open :{port} - see {log}")
        pid = open(_pidfile(proc)).read().strip() \
            if os.path.isfile(_pidfile(proc)) else "?"
        print(f"{simlib.col_green(f'de_servers.{proc}')}: "
              f"pid {simlib.col_bold(pid)}  port {port}")
    print("de_servers up - external sims can join "
          "(./sim up sims/quadcopter.yaml)")


def down(quiet=False):
    """Stop the stack by its own pidfiles - marker-checked like
    pid_is_ours so a recycled pid is never killed."""
    live = pids()
    if not live:
        if not quiet:
            print("de_servers: nothing running")
        return
    for proc, pid in live.items():
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
    t0 = time.time()
    while time.time() - t0 < 3 and \
            any(os.path.isdir(f"/proc/{p}") for p in live.values()):
        time.sleep(0.2)
    for pid in live.values():
        if os.path.isdir(f"/proc/{pid}") and _pid_is_stack(pid):
            os.kill(pid, signal.SIGKILL)
    for f in os.listdir(PIDS_DIR):
        if f.endswith(".pid"):
            os.unlink(os.path.join(PIDS_DIR, f))
    if not quiet:
        print(simlib.col_yellow(
            f"de_servers stopped ({len(live)} processes)"))


def logs(proc=None):
    d = LOGS_DIR
    files = [os.path.join(d, f) for f in sorted(os.listdir(d))
             if proc is None or f == proc + ".log"]
    if not files:
        sys.exit(f"no logs under {d}")
    subprocess.call(["tail", "-n", "40"] + files)


def status(json_out=False):
    live = pids()
    rows = []
    for proc, repo, env_key, port_env in PROCS:
        try:
            port = _port(proc, repo, port_env)
        except (simlib.SimError, ValueError):
            port = 0
        try:
            socket.create_connection(("127.0.0.1", port), 0.3).close()
            bound = True
        except OSError:
            bound = False
        rows.append({"proc": proc, "repo": repo,
                     "pid": live.get(proc), "port": port,
                     "bound": bound})
    if json_out:
        print(json.dumps({"de_servers": rows}, indent=1))
        return
    print("de_servers "
          + simlib.col_dim(f"({len(live)}/{len(PROCS)} procs up)"))
    for r in rows:
        state = simlib.col_green("bound") if r["bound"] \
            else (simlib.col_yellow("not bound")
                  if r["pid"] else "down")
        print(f"  {r['proc']:14s} pid {r['pid'] or '-':>6}  "
              f"port {r['port']:>6}  {state}")


def main(a):
    """./sim servers [status] [--json]"""
    a = [x for x in a if x != "status"]
    json_out = "--json" in a
    bad = [x for x in a if x != "--json"]
    if bad:
        sys.exit("usage: ./sim servers [status] [--json]")
    status(json_out)
