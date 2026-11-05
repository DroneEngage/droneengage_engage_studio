"""prelaunch_canary.py - read-only checks on the live slot, behind
`./sim prelaunch --target live --canary`.

A firewall or a certificate can change with no release, so a gate at release
time is not enough. This runs after each deploy and on a timer, and it gives a
notice when a result changes from PASS to FAIL.

It never arms a vehicle, writes config, restarts a service or changes a
firewall. It uses only the canary account, so live units see nothing. The
relay probe runs only when the target names a canary test unit.

With SSH it adds the inside view. The port list on the machine compared with
the probe from outside tells the two faults apart: listening but not reachable
is a firewall fault, and not listening is a service fault. Only `ss -ltn` and
`cat` of the named config files run on the machine.

Target keys: canary: {notify_cmd: <shell, gets the text on stdin>,
machines: {a: {ssh: ..., key: ...}}, config_remote: {comm: {machine: b, path:
...}}}, and services.<name>.machine.
"""

import datetime
import json
import os
import re
import subprocess
import sys

import simlib
from prelaunch_checks import reach, relay, turn, config
import prelaunch_install

CERT_WARN_DAYS = 14


def _state_path(name):
    return os.path.join(simlib.RESULTS_DIR, "prelaunch", "canary", name,
                        "state.json")


def _read(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def inside_view(ctx):
    """{machine: set of listening ports}, for each machine SSH reaches."""
    out = {}
    canary = ctx.target.get("canary") or {}
    for name, host in (canary.get("machines") or {}).items():
        rc, text = prelaunch_install.run_on(host, "ss -ltnH", ctx, timeout=30)
        if rc == 0:
            out[name] = {int(m) for m in re.findall(r":(\d+)\s", text)}
    return out


def diagnose(ctx, inside):
    """Plain words for each public port that is closed from outside."""
    faults, oks = [], []
    for name, s in (ctx.target.get("services") or {}).items():
        if not s or not s.get("port") or s.get("public") is False:
            continue
        st = reach.tcp_state(s["host"], s["port"])
        where = f"{name} {s['host']}:{s['port']}"
        if st == "open":
            oks.append(where)
            continue
        ports = inside.get(s.get("machine"))
        if ports is None:
            faults.append(f"{where}: {reach._why(st)}")
        elif s["port"] in ports:
            faults.append(f"{where}: the service listens on the machine, but "
                          f"this host cannot reach it. This is a firewall fault")
        else:
            faults.append(f"{where}: nothing listens on the machine. This is a "
                          f"service fault")
    if faults:
        return "FAIL", "; ".join(faults)
    if not oks:
        return "SKIP", "no public port is declared"
    return "PASS", "open: " + ", ".join(oks)


def cert_warnings(ctx):
    """(status, detail). Under CERT_WARN_DAYS is a warning, not a fault: the
    service still works today. An expired or bad certificate is a fault."""
    faults, warns, oks = [], [], []
    for name, s in (ctx.target.get("services") or {}).items():
        if not s or not s.get("port") or s.get("tls") is False:
            continue
        days, err = reach.cert_info(s["host"], s["port"], s.get("ca_file"),
                                    s.get("verify_tls", True))
        where = f"{name} {s['host']}:{s['port']}"
        if err:
            faults.append(f"{where}: {err}")
        elif days < CERT_WARN_DAYS:
            warns.append(f"WARNING {where}: the certificate has {days} days left")
        else:
            oks.append(f"{where} {days} days")
    if faults:
        return "FAIL", "; ".join(faults + warns)
    if warns:
        return "PASS", "; ".join(warns)
    return "PASS", ", ".join(oks) or "no TLS service"


def fetch_config(ctx, out_dir):
    """Copy the named config files from the machines for the lint. -> the
    number fetched. Reading only."""
    canary = ctx.target.get("canary") or {}
    machines = canary.get("machines") or {}
    paths, n = {}, 0
    for key, spec in (canary.get("config_remote") or {}).items():
        host = machines.get(spec.get("machine"))
        if not host:
            continue
        rc, text = prelaunch_install.run_on(host, f"cat {spec['path']}", ctx,
                                            timeout=30)
        if rc == 0:
            p = os.path.join(out_dir, f"{key}.config")
            fd = os.open(p, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, "w") as fh:
                fh.write(text)
            paths[key] = p
            n += 1
    if paths:
        ctx.target.setdefault("config", {}).update(paths)
    return n


def _guard(fn, ctx):
    try:
        return fn(ctx)
    except Exception as e:     # includes prelaunch.Skip, reported as text
        if type(e).__name__ == "Skip":
            return "SKIP", str(e)
        return "FAIL", f"the check crashed: {e!r}"


def transitions(old, new):
    """The checks whose status moved from PASS to FAIL since the last run. A
    check seen for the first time as FAIL counts. A repeated FAIL does not."""
    return [c for c, st in new.items()
            if st == "FAIL" and old.get(c) != "FAIL"]


def notify(ctx, lines):
    text = "\n".join(lines)
    print(f"NOTICE canary {ctx.name}:\n{text}", file=sys.stderr, flush=True)
    cmd = (ctx.target.get("canary") or {}).get("notify_cmd")
    if cmd:
        subprocess.run(cmd, shell=True, input=text, text=True, timeout=60)


def run(target, as_json=False, ctx=None, checks=None):
    import prelaunch
    now = datetime.datetime.now(datetime.timezone.utc)
    state_file = _state_path(target["name"])
    out = os.path.dirname(state_file)
    os.makedirs(out, exist_ok=True)
    ctx = ctx or prelaunch.Ctx(target, out, "canary")
    inside = inside_view(ctx)
    fetched = fetch_config(ctx, out)
    checks = checks or {
        "canary.ports": lambda c: diagnose(c, inside),
        "canary.tls": cert_warnings,
        "canary.login": reach.login,
        "canary.public_host": reach.public_host,
        "canary.turn": turn.allocate_probe,
    }
    if (target.get("test_unit") or {}).get("party") and not checks.get("skip"):
        checks["canary.relay"] = relay.mavlink_both_ways
    if fetched:
        checks["canary.config_ice"] = config.ice
        checks["canary.config_public_host"] = config.public_host
        checks["canary.config_udp_relay"] = config.udp_relay
    results = {}
    for cid, fn in checks.items():
        st, detail = _guard(fn, ctx)
        results[cid] = {"status": st, "detail": ctx.scrub(detail)}
        print(f"[{st:>4}] {cid}: {results[cid]['detail']}", flush=True)
    old = _read(state_file).get("checks") or {}
    new_stat = {c: r["status"] for c, r in results.items()}
    moved = transitions({c: r for c, r in old.items()}, new_stat)
    lines = [f"{c}: {results[c]['detail']}" for c in moved]
    warns = [f"{c}: {results[c]['detail']}" for c, r in results.items()
             if "WARNING" in r["detail"]
             and "WARNING" not in (_read(state_file).get("details") or {}).get(c, "")]
    if lines or warns:
        notify(ctx, lines + warns)
    rep = {"target": target["name"], "at": now.strftime("%Y-%m-%dT%H:%M:%SZ"),
           "checks": new_stat,
           "details": {c: r["detail"] for c, r in results.items()}}
    with open(state_file, "w") as fh:
        json.dump(rep, fh, indent=2)
    if as_json:
        print(json.dumps(rep, indent=2))
    return 1 if "FAIL" in new_stat.values() else 0
