#!/usr/bin/env python3
"""prelaunch_promote.py - switch the production slots, and roll back.

Two slots run on the same two machines. Each slot keeps its ports for all
time. Only the public de_auth API port (server_port) moves: the mirror takes
it, and the old live slot takes the test port. Units know only auth_ip and
auth_port and learn the rest from de_auth, so no DNS or unit config changes.

Read docs/PRELAUNCH.md first. The default is a dry run that prints each step
and changes nothing. A human starts each real run with --execute.

    python3 engine/prelaunch_promote.py --plan <plan.yaml> --version 1.2.0
    python3 engine/prelaunch_promote.py --plan <plan.yaml> --version 1.2.0 \\
        --message-sent --execute
    python3 engine/prelaunch_promote.py --plan <plan.yaml> --rollback --execute

Drones in flight do not block a switch: the human warns users with the global
message first, and the script only shows how many units are armed.

Plan file (the human's decisions, so credentials stay out of it):

    public_port: 19408
    test_port: 19500
    report_target: mirror
    live_slot_file: ~/.de_live_slot      # on each machine
    armed_cmd: <shell that prints a count, optional>
    machines:
      a: {ssh: user@auth.example.org, key: ~/.ssh/auth_key}
      b: {ssh: user@comm.example.org}
    slots:
      blue:
        auth:      {machine: a, config: <path>, stop: <shell>, start: <shell>}
        webclient: {machine: a, config: <path>, restart: <shell>}
        comm:      {machine: b, restart: <shell>}
        data_sync: [{machine: a, cmd: <copy accounts from this slot to green>}]
      green: {...}

data_sync of a slot copies from it to the other slot, just before it stops
being live. It must also remove test data. A slot with no data_sync stops the
real run, because logins would break for accounts made since the last copy.

Run the self test: python3 engine/prelaunch_promote_test.py
"""

import argparse
import json
import os
import shlex
import sys

import yaml

ENGINE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ENGINE)
import prelaunch_install  # noqa: E402

SERVER_PORT_RE = r'("server_port"[[:space:]]*:[[:space:]]*)"?[0-9]+"?'
WEB_PORT_RE = r'("CONST_PROD_MODE_PORT"[[:space:]]*:[[:space:]]*)"?[0-9]+"?'


class Stop(Exception):
    """The switch must not go on. The text says why."""


def load_plan(path):
    with open(path) as fh:
        plan = yaml.safe_load(fh)
    for key in ("public_port", "test_port", "machines", "slots",
                "live_slot_file"):
        if not plan.get(key):
            raise Stop(f"{path}: the plan needs {key}")
    if len(plan["slots"]) != 2:
        raise Stop(f"{path}: the plan needs exactly two slots")
    return plan


def _slot_file(plan):
    f = plan["live_slot_file"]
    return f if f.startswith("~") else shlex.quote(f)


def read_live(plan, ctx=None):
    """The slot name in the live-slot file, the same on every machine."""
    seen = {}
    for name, host in plan["machines"].items():
        rc, text = prelaunch_install.run_on(host, f"cat {_slot_file(plan)}",
                                            ctx, timeout=30)
        seen[name] = text.strip() if rc == 0 else None
    values = set(seen.values())
    if None in values:
        missing = [m for m, v in seen.items() if v is None]
        raise Stop(f"no live-slot file on machine {', '.join(missing)}. Set it "
                   f"by hand to the slot that is live now")
    if len(values) != 1:
        raise Stop(f"the machines disagree on the live slot: {seen}")
    live = values.pop()
    if live not in plan["slots"]:
        raise Stop(f"the live-slot file says {live!r}, not a slot in the plan")
    return live


def check_report(plan, version, results_dir):
    """A switch needs a GO report for this exact version, or an override."""
    target = plan.get("report_target", "mirror")
    path = os.path.join(results_dir, "prelaunch", version, target,
                        "report.json")
    try:
        with open(path) as fh:
            rep = json.load(fh)
    except (OSError, ValueError):
        raise Stop(f"no pre-launch report for version {version} on {target} "
                   f"({path}). Run the gate first")
    if rep.get("verdict") == "GO":
        return f"report {version} on {target}: GO"
    if rep.get("override"):
        return (f"report {version} on {target}: {rep.get('verdict')}, with "
                f"human override: {rep['override'].get('reason')}")
    raise Stop(f"report {version} on {target} says {rep.get('verdict')}. Fix "
               f"it, or record a human --override in the gate")


def _sed(pattern, value, path):
    p = path if path.startswith("~") else shlex.quote(path)
    return f"sed -i -E 's/{pattern}/\\1{value}/' {p}"


def _web_cmd(webclient, port):
    return _sed(WEB_PORT_RE, f'"{port}"', webclient["config"])


def build_steps(plan, frm, to):
    """The ordered switch from slot `frm` to slot `to`, as
    [(machine, shell, why)]. The port can have only one owner, so the old
    auth stops before the new auth starts. Logins fail for that short time."""
    old, new = plan["slots"][frm], plan["slots"][to]
    pub, test = plan["public_port"], plan["test_port"]
    steps = []
    for item in old.get("data_sync") or []:
        steps.append((item["machine"], item["cmd"],
                      f"copy live data from {frm} to {to}"))
    oa, na = old["auth"], new["auth"]
    steps += [
        (oa["machine"], oa["stop"], f"stop the {frm} auth: it frees port {pub}"),
        (na["machine"], na["stop"], f"stop the {to} auth: it frees port {test}"),
        (oa["machine"], _sed(SERVER_PORT_RE, test, oa["config"]),
         f"{frm} auth server_port = {test}"),
        (na["machine"], _sed(SERVER_PORT_RE, pub, na["config"]),
         f"{to} auth server_port = {pub}"),
        (na["machine"], na["start"], f"start the {to} auth on port {pub}"),
        (oa["machine"], oa["start"], f"start the {frm} auth on port {test}"),
    ]
    for name, slot, port in ((frm, old, test), (to, new, pub)):
        web = slot.get("webclient")
        if web:
            steps.append((web["machine"], _web_cmd(web, port),
                          f"{name} webclient CONST_PROD_MODE_PORT = {port}"))
            if web.get("restart"):
                steps.append((web["machine"], web["restart"],
                              f"restart the {name} webclient"))
    oc = old["comm"]
    steps.append((oc["machine"], oc["restart"],
                  f"restart the {frm} comm: connected units reconnect and "
                  f"reach {to} through port {pub}"))
    for m in plan["machines"]:
        steps.append((m, f"echo {to} > {_slot_file(plan)}",
                      f"record {to} as the live slot on machine {m}"))
    return steps


def run_steps(plan, steps, ctx=None):
    for i, (machine, cmd, why) in enumerate(steps, 1):
        print(f"[{i}/{len(steps)}] {why}", flush=True)
        rc, text = prelaunch_install.run_on(plan["machines"][machine], cmd, ctx)
        if rc != 0:
            tail = text[-300:].replace("\n", " ")
            raise Stop(f"step {i} failed with exit {rc} on machine {machine}: "
                       f"{why}: {tail}. The switch stopped here: read the state "
                       f"by hand. The rollback swaps by the live-slot file, "
                       f"which still names the old slot")


def armed_count(plan, ctx=None):
    cmd = plan.get("armed_cmd")
    if not cmd:
        return "not known (the plan has no armed_cmd)"
    first = next(iter(plan["machines"].values()))
    rc, text = prelaunch_install.run_on(first, cmd, ctx, timeout=30)
    return text.strip() if rc == 0 else f"not known (armed_cmd exit {rc})"


def main(argv=None, results_dir=None):
    ap = argparse.ArgumentParser(description="switch the production slots")
    ap.add_argument("--plan", required=True)
    ap.add_argument("--version")
    ap.add_argument("--execute", action="store_true",
                    help="really do it; the default only prints the steps")
    ap.add_argument("--rollback", action="store_true",
                    help="swap back; needs no report")
    ap.add_argument("--message-sent", action="store_true",
                    help="you sent the global message to users")
    a = ap.parse_args(argv)
    if results_dir is None:
        import simlib
        results_dir = simlib.RESULTS_DIR
    try:
        plan = load_plan(a.plan)
        live = read_live(plan)
        other = [s for s in plan["slots"] if s != live][0]
        print(f"live slot now: {live}. After this run: {other}.")
        if not a.rollback:
            if not a.version:
                raise Stop("--version is required for a switch")
            try:
                print(check_report(plan, a.version, results_dir))
            except Stop:
                if a.execute:
                    raise
                print("WOULD STOP: no GO report for this version")
        print(f"armed units now: {armed_count(plan)} (this does not stop the "
              f"switch)")
        if not a.rollback and not plan["slots"][live].get("data_sync") \
                and a.execute:
            raise Stop(f"slot {live} has no data_sync: logins would break for "
                       f"accounts made since the last copy")
        steps = build_steps(plan, live, other)
        if a.rollback:      # the data in the older slot stays as it was
            steps = [s for s in steps if not s[2].startswith("copy live data")]
        if not a.execute:
            print("DRY RUN: nothing changes. These steps would run:")
            for i, (m, cmd, why) in enumerate(steps, 1):
                print(f"  {i}. [{m}] {why}\n       {cmd}")
            return 0
        if not a.rollback and not a.message_sent:
            raise Stop("send the global message to users first, then pass "
                       "--message-sent")
        run_steps(plan, steps)
        print(f"done: {other} is live. Run the canary for this slot now: "
              f"./sim prelaunch --target live --canary. Keep {live} unchanged "
              f"until you approve the release.")
        return 0
    except Stop as e:
        print(f"STOP: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
