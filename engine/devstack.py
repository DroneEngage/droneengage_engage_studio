"""`./sim up de_servers` / `./sim up de_webclient` / `./sim servers` -
the developer's own stack: the headless equivalent of
servers/sh_start_servers.sh plus the webclient's `npm start`.

de_servers: each server runs FROM ITS REPO with no --config override,
so it loads its own server.config + server.config.local on the real
ports - the stack `server: {type: external}` sims join (de_auth ->
de_server -> de_storage -> de_udp_proxy, in the script's order).

de_webclient: `npm start` in DE_WEBCLIENT_DIR - the react-scripts dev
server on SIM_WEBCLIENT_URL (https :3000 default). The webclient takes
its auth/server target from URL params, so the deep links printed by
`sim status` keep working unchanged.

Deliberately NOT a sim: no manifest under var/run/, and the procs carry
the --<stack> marker instead of a var/run path, so simlib.pid_is_ours
stays False for them - `sim down`, `sim run`'s exclusive teardown and
`sim clean` can never sweep the stack mid-test (an external test needs
its server alive). Only `./sim down <stack>` stops it. State lives
under var/<stack minus the de_ prefix>/: launch.<proc>.sh, pids/, logs/.
"""

import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.parse

import simlib

# stack name -> procs in launch order, each (proc, repo, env key for
# error hints, port env override if any). de_servers order matters:
# auth first so the comm server's s2s link dials a live target (the old
# script's `sleep 1` chain).
STACKS = {
    "de_servers": [
        ("de_auth",      simlib.AUTH_REPO,      "DE_AUTH_DIR",      None),
        ("de_server",    simlib.SERVER_REPO,    "DE_SERVER_DIR",    None),
        ("de_storage",   simlib.STORAGE_REPO,   "DE_STORAGE_DIR",
         "de_storage_server_port"),
        ("de_udp_proxy", simlib.UDP_RELAY_REPO, "DE_UDP_RELAY_DIR", None),
    ],
    "de_webclient": [
        ("de_webclient", simlib.WEBCLIENT_REPO, "DE_WEBCLIENT_DIR", None),
    ],
}

# per-stack port-wait seconds: webpack's dev server binds late compared
# to the node servers
UP_TIMEOUT = {"de_webclient": 90}


def _stack_dir(stack):
    """de_servers -> var/servers, de_webclient -> var/webclient."""
    return os.path.join(simlib.VAR_DIR, stack[3:])


def _pids_dir(stack):
    return os.path.join(_stack_dir(stack), "pids")


def _logs_dir(stack):
    return os.path.join(_stack_dir(stack), "logs")


def _pidfile(stack, proc):
    return os.path.join(_pids_dir(stack), proc + ".pid")


# argv marker identifying a stack's procs. Must NOT be a var/run or
# engine path - that is what keeps pid_is_ours False and the stack out
# of every sim teardown.
def _marker(stack):
    return "--" + stack


def _pid_is_stack(pid, stack):
    """The pid is alive and its cmdline carries the stack marker."""
    try:
        cmd = open(f"/proc/{pid}/cmdline", "rb").read() \
            .replace(b"\0", b" ").decode(errors="replace")
    except OSError:
        return False
    return _marker(stack) in cmd


def pids(stack="de_servers"):
    """{proc: pid} of stack procs alive and marked."""
    out = {}
    d = _pids_dir(stack)
    if not os.path.isdir(d):
        return out
    for f in os.listdir(d):
        if not f.endswith(".pid"):
            continue
        try:
            pid = int(open(os.path.join(d, f)).read().strip())
            os.kill(pid, 0)
        except (ValueError, OSError):
            continue
        if _pid_is_stack(pid, stack):
            out[f[:-4]] = pid
    return out


def _port(stack, proc, repo, env_key):
    """The port the proc will bind. Servers: their repo's server.config
    + server.config.local (the same shallow merge the common loader
    does), then the env override some servers honour. The webclient:
    $PORT, else the SIM_WEBCLIENT_URL port."""
    if stack == "de_webclient":
        if os.environ.get("PORT"):
            return int(os.environ["PORT"])
        u = urllib.parse.urlparse(simlib.WEBCLIENT_URL)
        return u.port or (443 if u.scheme == "https" else 80)
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


def _npm():
    """npm next to simlib.NODE (an nvm NODE= path), else PATH's."""
    return os.path.join(os.path.dirname(simlib.NODE), "npm") \
        if os.path.dirname(simlib.NODE) else "npm"


def _launch_script(stack, proc, repo):
    """Run in the repo (node_modules + its own server.config(.local)
    resolve there); the --<stack> marker is ignored by the launched
    program but identifies the pid as this stack's. The webclient rides
    `npm start -- <marker>` so the npm pid carries it (and react-scripts
    ignores it)."""
    if stack == "de_webclient":
        # BROWSER=none: CRA opens the browser on first compile - the sim
        # stays headless, `--open-web` does it on demand
        cmd = "export BROWSER=none\nexec '%s' start -- %s" \
            % (_npm(), _marker(stack))
    else:
        cmd = "exec '%s' ./src/server.js %s" % (simlib.NODE,
                                                _marker(stack))
    path = os.path.join(_stack_dir(stack), f"launch.{proc}.sh")
    simlib.write_text(path,
                      "#!/bin/bash\ncd '%s'\necho $$ > '%s'\n%s\n"
                      % (repo, _pidfile(stack, proc), cmd), 0o755)
    return path


def _check_repo(stack, proc, repo, env_key):
    if stack == "de_webclient":
        if not os.path.isfile(os.path.join(repo, "package.json")):
            raise simlib.SimError(
                f"{proc}: no package.json under {repo} - set {env_key} "
                "in sim.local.env")
        if not os.path.isdir(os.path.join(repo, "node_modules",
                                          "react-scripts")):
            raise simlib.SimError(
                f"{proc}: no react-scripts under {repo}/node_modules - "
                "run npm install there")
    elif not os.path.isfile(os.path.join(repo, "src", "server.js")):
        raise simlib.SimError(
            f"{proc}: no src/server.js under {repo} - set {env_key} "
            "in sim.local.env")


def up(stack="de_servers", fresh=False):
    """Start the stack's procs in order, waiting on each one's port."""
    procs = STACKS[stack]
    if fresh:
        down(stack, quiet=True)
    os.makedirs(_pids_dir(stack), exist_ok=True)
    os.makedirs(_logs_dir(stack), exist_ok=True)
    live = pids(stack)
    if len(live) == len(procs):
        print(f"{stack} already up - `./sim servers` for status, "
              "--fresh to restart")
        return
    for proc, repo, env_key, port_env in procs:
        if proc in live:
            print(f"{simlib.col_dim(f'{stack}.{proc}')}: "
                  f"already running pid {live[proc]}")
            continue
        _check_repo(stack, proc, repo, env_key)
        script = _launch_script(stack, proc, repo)
        log = os.path.join(_logs_dir(stack), proc + ".log")
        subprocess.Popen(["setsid", "nohup", "bash", script],
                         stdout=open(log, "a"), stderr=subprocess.STDOUT,
                         stdin=subprocess.DEVNULL)
        t0 = time.time()
        while time.time() - t0 < 3 \
                and not os.path.isfile(_pidfile(stack, proc)):
            time.sleep(0.1)
        port = _port(stack, proc, repo, port_env)
        if port and not _wait_port(port, UP_TIMEOUT.get(stack, 30)):
            raise simlib.SimError(
                f"{proc} did not open :{port} - see {log}")
        pid = open(_pidfile(stack, proc)).read().strip() \
            if os.path.isfile(_pidfile(stack, proc)) else "?"
        print(f"{simlib.col_green(f'{stack}.{proc}')}: "
              f"pid {simlib.col_bold(pid)}  port {port}")
    if stack == "de_webclient":
        print(f"de_webclient up - {simlib.WEBCLIENT_URL} (the first "
              "compile takes a while - `./sim logs de_webclient` "
              "to watch)")
    else:
        print("de_servers up - external sims can join "
              "(./sim up sims/quadcopter.yaml)")


def _kill_tree(pid, stack, sig):
    """Signal the stack proc's whole group when it leads one (npm ->
    sh -> react-scripts), else the pid alone. Re-checks the marker so a
    pid recycled between pids() and here is never signalled."""
    try:
        if not _pid_is_stack(pid, stack):
            return
        if os.getpgid(pid) == pid:
            os.killpg(pid, sig)
        else:
            os.kill(pid, sig)
    except OSError:
        pass


def down(stack="de_servers", quiet=False):
    """Stop the stack by its own pidfiles - marker-checked like
    pid_is_ours so a recycled pid is never killed."""
    live = pids(stack)
    if not live:
        if not quiet:
            print(f"{stack}: nothing running")
        return
    for proc, pid in live.items():
        _kill_tree(pid, stack, signal.SIGTERM)
    t0 = time.time()
    while time.time() - t0 < 3 and \
            any(os.path.isdir(f"/proc/{p}") for p in live.values()):
        time.sleep(0.2)
    for pid in live.values():
        if os.path.isdir(f"/proc/{pid}") and _pid_is_stack(pid, stack):
            _kill_tree(pid, stack, signal.SIGKILL)
    d = _pids_dir(stack)
    for f in os.listdir(d):
        if f.endswith(".pid"):
            os.unlink(os.path.join(d, f))
    if not quiet:
        print(simlib.col_yellow(
            f"{stack} stopped ({len(live)} processes)"))


def logs(stack="de_servers", proc=None):
    d = _logs_dir(stack)
    files = [os.path.join(d, f) for f in sorted(os.listdir(d))
             if proc is None or f == proc + ".log"]
    if not files:
        sys.exit(f"no logs under {d}")
    subprocess.call(["tail", "-n", "40"] + files)


def _rows(stack):
    live = pids(stack)
    rows = []
    for proc, repo, env_key, port_env in STACKS[stack]:
        try:
            port = _port(stack, proc, repo, port_env)
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
    return rows


def status(stack="de_servers"):
    print(stack + " " + simlib.col_dim(
        f"({len(pids(stack))}/{len(STACKS[stack])} procs up)"))
    for r in _rows(stack):
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
    if json_out:
        print(json.dumps({s: _rows(s) for s in STACKS}, indent=1))
        return
    for s in STACKS:
        status(s)
