"""`./sim up <stack>` / `./sim down <stack>` / `./sim servers` - the
developer's own stacks, declared in stacks/*.yaml (SL-0139; format in
stacks/README.md).

stacks/de_servers.yaml: the headless equivalent of
servers/sh_start_servers.sh - each server runs FROM ITS REPO with no
--config override, so it loads its own server.config +
server.config.local on the real ports - the stack `server: {type:
external}` sims join (de_auth -> de_server -> de_storage ->
de_udp_proxy, in the script's order).

stacks/de_webclient.yaml: `npm start` in DE_WEBCLIENT_DIR - the
react-scripts dev server on SIM_WEBCLIENT_URL (https :3000 default).
The webclient takes its auth/server target from URL params, so the deep
links printed by `sim status` keep working unchanged.

Deliberately NOT sims: no manifest under var/run/, and the procs carry
the --<stack> marker instead of a var/run path, so simlib.pid_is_ours
stays False for them - `sim down`, `sim run`'s exclusive teardown and
`sim clean` can never sweep the stack mid-test (an external test needs
its server alive). Only `./sim down <stack>` stops it. State lives
under var/<stack minus any de_ prefix>/: launch.<proc>.sh, pids/, logs/.
"""

import glob
import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.parse

import simlib

STACKS_DIR = os.path.join(simlib.SIM_ROOT, "stacks")

# proc kinds: "exec" runs a built binary at <repo>/<bin> (the Rust
# servers - they ignore unknown args, so the --<stack> marker still
# rides argv); "npm" rides `npm start` (the react-scripts dev server)
# and takes its port from $PORT / SIM_WEBCLIENT_URL
_KINDS = ("npm", "exec")


def _load_stacks():
    """stacks/*.yaml -> {name: {"procs": [{name, repo, repo_env, bin,
    port_env, kind}], "up_timeout_s", "open_url", "up_note"}}. Proc
    order is launch order. A malformed file raises SimError naming it -
    a stack typo must never abort with a traceback."""
    stacks = {}
    for f in sorted(glob.glob(os.path.join(STACKS_DIR, "*.yaml"))):
        base = os.path.basename(f)
        try:
            spec = simlib.load_yaml(f)
        except Exception as e:
            raise simlib.SimError(f"{base}: {e}")
        if not isinstance(spec, dict):
            raise simlib.SimError(f"{base}: top level is not a mapping")
        name = spec.get("name")
        stem = base[:-len(".yaml")]
        if not isinstance(name, str) or not simlib.NAME_RX.match(name):
            raise simlib.SimError(f"{base}: needs a `name:` "
                                  "(word chars)")
        if name != stem:
            raise simlib.SimError(
                f"{base}: name {name!r} must match the file name")
        procs = spec.get("procs")
        if not isinstance(procs, list) or not procs:
            raise simlib.SimError(f"{base}: needs a `procs:` list")
        timeout = spec.get("up_timeout_s", 30)
        if not isinstance(timeout, (int, float)):
            raise simlib.SimError(f"{base}: up_timeout_s must be a number")
        out, seen = [], set()
        for i, p in enumerate(procs):
            where = f"{base}: procs[{i}]"
            if not isinstance(p, dict):
                raise simlib.SimError(f"{where}: not a mapping")
            pname = p.get("name")
            if not isinstance(pname, str) or not pname:
                raise simlib.SimError(f"{where}: needs a `name:`")
            if pname in seen:
                raise simlib.SimError(f"{where}: duplicate proc {pname!r}")
            seen.add(pname)
            repo_env, repo = p.get("repo_env"), p.get("repo")
            if repo_env and not isinstance(repo_env, str):
                raise simlib.SimError(f"{where}: repo_env must be a string")
            if repo is None:
                if not repo_env:
                    raise simlib.SimError(
                        f"{where}: needs `repo:` or `repo_env:`")
                repo = pname
            if not isinstance(repo, str):
                raise simlib.SimError(f"{where}: repo must be a string")
            kind = p.get("kind", "exec")
            if kind not in _KINDS:
                raise simlib.SimError(
                    f"{where}: kind {kind!r} not in {_KINDS}")
            bin_ = p.get("bin")
            if kind == "exec" and not isinstance(bin_, str):
                raise simlib.SimError(
                    f"{where}: kind exec needs `bin:` "
                    "(the binary path inside the repo)")
            if bin_ is not None and kind != "exec":
                raise simlib.SimError(
                    f"{where}: `bin:` only with kind exec")
            port_env = p.get("port_env")
            if port_env is not None and not isinstance(port_env, str):
                raise simlib.SimError(
                    f"{where}: port_env must be a string")
            out.append({"name": pname, "repo": repo,
                        "repo_env": repo_env, "bin": bin_,
                        "port_env": port_env, "kind": kind})
        for k in ("open_url", "up_note"):
            if spec.get(k) is not None and not isinstance(spec[k], str):
                raise simlib.SimError(f"{base}: {k} must be a string")
        stacks[name] = {"procs": out, "up_timeout_s": int(timeout),
                        "open_url": spec.get("open_url"),
                        "up_note": spec.get("up_note")}
    return stacks


def _repo(p):
    """The proc's repo path: its sim.local.env/env override, else the
    yaml `repo:` - relative means under $DE_CODE."""
    if p["repo_env"]:
        return simlib.repo_dir(p["repo_env"], p["repo"])
    return p["repo"] if os.path.isabs(p["repo"]) \
        else os.path.join(simlib.DE_CODE, p["repo"])


STACKS = _load_stacks()


def stack_names():
    """The declared stacks, for `sim` dispatch (up/down/logs/status) and
    `./sim list stacks`."""
    return list(STACKS)


def procs(stack):
    """The stack's proc specs in launch order; SimError on a name that
    is not a stack."""
    if stack not in STACKS:
        raise simlib.SimError(f"unknown stack {stack!r} - "
                              "`./sim list stacks`")
    return STACKS[stack]["procs"]


def open_url(stack):
    """What `--open-web` / `sim status` show for the stack, with
    {webclient_url} substituted; None when the stack opens nothing."""
    u = STACKS.get(stack, {}).get("open_url")
    return u.replace("{webclient_url}", simlib.WEBCLIENT_URL) if u else None


def _stack_dir(stack):
    """de_servers -> var/servers, example_comm_only -> var/example_comm_only."""
    return os.path.join(simlib.VAR_DIR,
                        stack[3:] if stack.startswith("de_") else stack)


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


def _port(stack, p):
    """The port the proc will bind. exec procs: their repo's
    server.config + server.config.local (the same shallow merge the
    common loader does), then the env override some servers honour.
    npm procs: $PORT, else the SIM_WEBCLIENT_URL port."""
    if p["kind"] == "npm":
        if os.environ.get("PORT"):
            return int(os.environ["PORT"])
        u = urllib.parse.urlparse(simlib.WEBCLIENT_URL)
        return u.port or (443 if u.scheme == "https" else 80)
    repo = _repo(p)
    cfg = simlib.load_config(os.path.join(repo, "server.config"))
    loc = os.path.join(repo, "server.config.local")
    if os.path.isfile(loc):
        cfg.update(simlib.load_config(loc))
    env_key = p["port_env"]
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


def _launch_script(stack, p):
    """Run in the repo (its own server.config(.local) and relative
    paths resolve there); the --<stack> marker is ignored by the
    launched program but identifies the pid as this stack's. npm procs
    ride `npm start -- <marker>` so the npm pid carries it (and
    react-scripts ignores it); exec procs pass it straight to the
    binary (the Rust servers ignore unknown args)."""
    repo, proc = _repo(p), p["name"]
    if p["kind"] == "npm":
        # BROWSER=none: CRA opens the browser on first compile - the sim
        # stays headless, `--open-web` does it on demand
        cmd = "export BROWSER=none\nexec '%s' start -- %s" \
            % (_npm(), _marker(stack))
    else:
        cmd = "exec '%s' %s" % (os.path.join(repo, p["bin"]),
                                _marker(stack))
    path = os.path.join(_stack_dir(stack), f"launch.{proc}.sh")
    simlib.write_text(path,
                      "#!/bin/bash\ncd '%s'\necho $$ > '%s'\n%s\n"
                      % (repo, _pidfile(stack, proc), cmd), 0o755)
    return path


def _check_repo(stack, p):
    repo, proc = _repo(p), p["name"]
    hint = f"set {p['repo_env']} in sim.local.env" if p["repo_env"] \
        else f"fix repo: in stacks/{stack}.yaml"
    if p["kind"] == "npm":
        if not os.path.isfile(os.path.join(repo, "package.json")):
            raise simlib.SimError(
                f"{proc}: no package.json under {repo} - {hint}")
        if not os.path.isdir(os.path.join(repo, "node_modules",
                                          "react-scripts")):
            raise simlib.SimError(
                f"{proc}: no react-scripts under {repo}/node_modules - "
                "run npm install there")
    else:
        b = os.path.join(repo, p["bin"])
        if not (os.path.isfile(b) and os.access(b, os.X_OK)):
            raise simlib.SimError(
                f"{proc}: no executable {p['bin']} under {repo} - "
                f"cd {repo} && cargo build --release (or {hint})")


def up(stack="de_servers", fresh=False):
    """Start the stack's procs in order, waiting on each one's port."""
    procs_ = procs(stack)
    if fresh:
        down(stack, quiet=True)
    os.makedirs(_pids_dir(stack), exist_ok=True)
    os.makedirs(_logs_dir(stack), exist_ok=True)
    live = pids(stack)
    if len(live) == len(procs_):
        print(f"{stack} already up - `./sim servers` for status, "
              "--fresh to restart")
        return
    for p in procs_:
        proc = p["name"]
        if proc in live:
            print(f"{simlib.col_dim(f'{stack}.{proc}')}: "
                  f"already running pid {live[proc]}")
            continue
        _check_repo(stack, p)
        script = _launch_script(stack, p)
        log = os.path.join(_logs_dir(stack), proc + ".log")
        subprocess.Popen(["setsid", "nohup", "bash", script],
                         stdout=open(log, "a"), stderr=subprocess.STDOUT,
                         stdin=subprocess.DEVNULL)
        t0 = time.time()
        while time.time() - t0 < 3 \
                and not os.path.isfile(_pidfile(stack, proc)):
            time.sleep(0.1)
        port = _port(stack, p)
        if port and not _wait_port(port, STACKS[stack]["up_timeout_s"]):
            raise simlib.SimError(
                f"{proc} did not open :{port} - see {log}")
        pid = open(_pidfile(stack, proc)).read().strip() \
            if os.path.isfile(_pidfile(stack, proc)) else "?"
        print(f"{simlib.col_green(f'{stack}.{proc}')}: "
              f"pid {simlib.col_bold(pid)}  port {port}")
    note = STACKS[stack]["up_note"]
    if note:
        print(note.replace("{webclient_url}", simlib.WEBCLIENT_URL))


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
    procs(stack)   # SimError on a name that is not a stack
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
    procs(stack)   # SimError on a name that is not a stack
    d = _logs_dir(stack)
    files = [os.path.join(d, f) for f in sorted(os.listdir(d))
             if proc is None or f == proc + ".log"]
    if not files:
        sys.exit(f"no logs under {d}")
    subprocess.call(["tail", "-n", "40"] + files)


def _foreign_owner_pid(port):
    """The pid actually serving a stack port when no stack-owned proc
    holds it (hand-started servers on the same ports). Display only -
    teardown never touches it."""
    try:
        out = subprocess.run(["ss", "-tlnpH", f"sport = :{port}"],
                             capture_output=True, text=True,
                             timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    i = out.find("pid=")
    if i < 0:
        return None
    try:
        return int(out[i + 4:].split(",")[0].split(")")[0])
    except ValueError:
        return None


def _rows(stack):
    live = pids(stack)
    rows = []
    for p in procs(stack):
        proc = p["name"]
        try:
            port = _port(stack, p)
        except (simlib.SimError, ValueError):
            port = 0
        try:
            socket.create_connection(("127.0.0.1", port), 0.3).close()
            bound = True
        except OSError:
            bound = False
        rows.append({"proc": proc, "repo": _repo(p),
                     "pid": live.get(proc), "port": port,
                     "bound": bound,
                     "foreign": None if (live.get(proc) or not bound)
                                else _foreign_owner_pid(port)})
    return rows


def status(stack="de_servers"):
    rows = _rows(stack)
    bound = sum(1 for r in rows if r["bound"])
    print(stack + " " + simlib.col_dim(
        f"({bound}/{len(rows)} bound, "
        f"{len(pids(stack))} stack-owned)"))
    for r in rows:
        pid = r["pid"] or r["foreign"]
        tag = " (foreign)" if (r["foreign"] and not r["pid"]) else ""
        state = simlib.col_green("bound") if r["bound"] \
            else (simlib.col_yellow("not bound")
                  if r["pid"] else "down")
        print(f"  {r['proc']:14s} pid {pid or '-':>6}{tag}  "
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
