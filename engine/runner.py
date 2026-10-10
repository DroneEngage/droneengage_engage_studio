#!/usr/bin/env python3
"""runner.py - run a test (`./sim run tests/<name>.yaml`).

A test names the sim it needs and the steps + assertions to run on it.
The sim is reused when it is already up with the same hash and every
process alive; otherwise it is brought up fresh. Several tests can share
one sim file - that is the point of sims: a reusable fleet setup.

    version: 1
    description: ...
    sim: ../sims/three_copters.yaml      # or an inline sim {server, units}
    operation: ../packages/pkg.json      # default package for `deploy`
    binding: {copter1: scout}            # optional deploy binding
    steps:
      - deploy                           # or {deploy: {package: p, unit: 2}}
      - arm: all
      - start
      - wait_journal: {unit: 2, re: "mission main completed", timeout: 300}
      - wait_log: {unit: 2, module: de_precland, re: "EVENT_OUT",
                   timeout: 30}              # the same block over
                                             # logs/<module>.log (module
                                             # stdout - never journaled)
      - fire: {event: area.clear, payload: {x: 1}, unit: all}
      - fire: {event: go.now, unit: 1, until: "BUS_OUT.*n=area\\.clear"}
                                         # re-fire until the unit journals it
      - opstart                          # P4A-09 fleet START (1094); the
                                         # operation must be a roles spec
      - opabort                          # P4A-09 fleet ABORT
      - opstatus: {seconds: 8}           # P4A-09 status poll
      - opmsg: {tg: copter1, ms: {...}, expect: "re"}   # 1094 probe
      - reboot: {unit: 1}                # de_comm restart under a fresh
                                         # boot id (pending TTL signals
                                         # drop, sticky facts survive)
      - loss: {unit: all, percent: 30}   # link faults: server type fake only
      - link_cut: {unit: 1, seconds: 20}
      - link: {unit: 1, set: "stall 30"} # SL-0007 raw link-model knobs;
                                         # set: ["loss_tx 0", "loss_rx 100"]
                                         # sends several sub-lines
      - beacon: {unit: 1, seconds: 10, min: 3}   # count the unit's 1004
                                         # presence beacons on a fresh GCS
                                         # listen (SL-0007)
      - kill: {unit: 1, proc: de_comm}   # unit: server, proc: de_server
      - restart: {unit: 1, proc: de_comm}
      - freeze: {unit: 1, proc: de_mavlink, dur_s: 30}   # SIGSTOP - a hang,
      - unfreeze: {unit: 1, proc: de_mavlink}            # not a crash
      - reset: 1                         # SITL back to home, disarmed
      - mode: {unit: 1, mode: RTL}
      - wait_online: 60                  # every unit seen by the GCS
      - wait: 10
      - write_file: {unit: 1, module: de_gpio, path: gpio_inputs.json,
                     json: {"17": 1}}      # JSON into var/run/<unit>/<module>/;
                                          # also: [{path, json, delay_ms?}, ...]
                                          # writes more files in the same ctl
                                          # call; delay_ms sleeps N ms first
                                          # (a +N marker inside the call)
    assert:
      - journal_count: {unit: all, re: "EVENT_IN area.clear", equals: 1}
      - journal_order: {unit: 1, first: "A re", then: "B re"}
      - journal_time_delta: {u1: 1, re1: "A", u2: 2, re2: "B", max_s: 5}
      - mission_current: {unit: 1, seq: 4}
    teardown: land                       # land (default) | reset | none

de.scenario/1 files (tests/**/*.yaml, scenarios/**/*.yaml -
engine/scenario_runner.py)
add the truth/world assertions of engine/checks.py on top of these:
distance_to, detection_precision, detection_recall (P5-10: who was
found, who correctly stayed hidden), time_to_detect (P5-10: seconds
from the world epoch to the first find), signals_in_range,
coverage_complete, stub_deterministic, ... - the full list and the
per-assertion arguments live in the checks.py docstring.

`unit:` takes what ctl takes: a name, 1-based position, block:<b>, a comma
list or all (`vehicle:` / `v1` / `v2` are accepted as aliases).

A failing step stops the step list; assertions still run and are reported,
and the test exits non-zero. Journals, logs, the test and the sim manifest
are saved under var/results/<timestamp>-<test>/.
"""

import json
import os
import re
import shutil
import subprocess
import sys
import time

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simlib
import ctl
import launcher

CTL = os.path.join(simlib.ENGINE_DIR, "ctl.py")


SIM_NAME = None           # the test's sim - run() sets it


def run_ctl(*args):
    print("$ ctl", *args, flush=True)
    argv = [sys.executable, CTL]
    if SIM_NAME:
        argv += ["-s", SIM_NAME]
    r = subprocess.run(argv + [str(x) for x in args])
    if r.returncode != 0:
        print(f"  (exit {r.returncode})", flush=True)
    return r.returncode == 0


def unit_arg(arg, key="unit", default="all"):
    v = arg.get(key, arg.get("vehicle")) if isinstance(arg, dict) else None
    return default if v is None else str(v)


# ------------------------------------------------------------------ steps

_UNIT_STEPS = {"arm", "land", "reset", "wait_journal", "wait_log",
               "wait_ready", "loss",
               "delay", "link_cut", "kick", "commline", "link_clear", "kill",
               "link", "restart", "freeze", "unfreeze",
               "journal", "mode", "disarm", "fire", "fc",
               "write_file", "beacon"}


def _wait_journal(u, rx, timeout, marks):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if simlib.journal_match(u, rx, marks.get(u["name"])):
            print(f"  {u['name']}: matched {rx!r} after {time.time()-t0:.1f}s", flush=True)
            return True
        time.sleep(1)
    print(f"  TIMEOUT: {rx!r} not in {u['name']} journal after {timeout}s", flush=True)
    return False


def _wait_log(u, proc, rx, timeout, marks):
    """wait_log: block until the unit's logs/<proc>.log (the module's
    stdout) has a line matching rx since the test mark. The module-log
    sibling of _wait_journal - module stdout never reaches the de_comm
    journal (PL-07: de_precland prints its facts as EVENT_OUT lines)."""
    rxc = re.compile(rx)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if simlib.log_match(u, proc, rxc, marks.get(u["name"])):
            print(f"  {u['name']}.{proc}: matched {rx!r} after "
                  f"{time.time() - t0:.1f}s", flush=True)
            return True
        time.sleep(1)
    print(f"  TIMEOUT: {rx!r} not in {u['name']} logs/{proc}.log after "
          f"{timeout}s", flush=True)
    return False


def _wait_journal_any(units, rx, timeout, marks):
    """any: true - done as soon as ONE of the units journals rx (a line
    whose unit is decided at runtime, e.g. which searcher adopts)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        for u in units:
            if simlib.journal_match(u, rx, marks.get(u["name"])):
                print(f"  {u['name']}: matched {rx!r} after {time.time()-t0:.1f}s", flush=True)
                return True
        time.sleep(1)
    names = ",".join(u["name"] for u in units)
    print(f"  TIMEOUT: {rx!r} not in any of {names} after {timeout}s", flush=True)
    return False


def run_step(step, t, marks):
    if isinstance(step, str):
        step = {step: True}
    if not isinstance(step, dict) or len(step) != 1:
        print(f"bad step {step!r}: one {{name: args}} mapping per step",
              flush=True)
        return False
    (name, arg), = step.items()
    if arg is True or arg is None:
        arg = {}
    if isinstance(arg, (str, int, float)) and name in _UNIT_STEPS:
        arg = {"unit": arg}
    unit = unit_arg(arg)
    here = os.path.dirname(t["__file"])

    if name == "deploy":
        pkg = arg.get("package", t.get("operation")) if isinstance(arg, dict) else t.get("operation")
        if not pkg:
            print("deploy: no package (set `operation:` or deploy.package)")
            return False
        pkg = os.path.join(here, pkg)
        target = unit_arg(arg, default=None) if isinstance(arg, dict) else None
        binding = t.get("binding")
        if target not in (None, "all"):
            binding = {u["name"]: {} for u in ctl.resolve(target)}
        if binding:
            b = os.path.join(simlib.run_dir(ctl._ctx_name()),
                             "test_binding.json")
            json.dump(binding, open(b, "w"))
            return run_ctl("deploy", pkg, b)
        return run_ctl("deploy", pkg)
    if name == "opstart":
        # P4A-09 fleet START (1094) on the deployed run; optional
        # {"run","lead_ms","in_ms","t_start","start":{role:mis},"targets":[]}
        extra = [json.dumps(arg)] if isinstance(arg, dict) and arg else []
        return run_ctl("opstart", *extra)
    if name == "opabort":
        parts = []
        if isinstance(arg, dict):
            if arg.get("run"):
                parts.append(str(arg["run"]))
            if arg.get("why"):
                if not parts:
                    parts = ["-"]
                parts.append(str(arg["why"]))
        return run_ctl("opabort", *parts)
    if name == "opstatus":
        secs = arg.get("seconds", 8) if isinstance(arg, dict) else (arg or 8)
        return run_ctl("opstatus", str(secs))
    if name == "opmsg":
        # inject one 1094 frame (ms dict) at tg as a real GCS and check
        # the replies - "expect" is a regex a reply line must match
        tg = arg.get("tg", arg.get("to", "all"))
        extra = []
        if arg.get("mt"):
            extra += ["--mt", str(arg["mt"])]
        if arg.get("expect"):
            extra += ["--expect", str(arg["expect"])]
        extra += ["--secs", str(arg.get("secs", arg.get("seconds", 5)))]
        return run_ctl("opmsg", str(tg), json.dumps(arg.get("ms", {})),
                       *extra)
    if name == "reboot":
        # P4F-03: a de_comm restart under a fresh boot id - pending TTL
        # signals drop, sticky pending + stored facts must survive
        return run_ctl("bootid", unit) and \
            run_ctl("restart", unit, "de_comm")
    if name in ("start", "stop", "resume", "pause", "clear"):
        extra = [arg["mission"]] if isinstance(arg, dict) and arg.get("mission") else []
        return run_ctl(name, *extra)
    if name == "wait":
        time.sleep(float(arg if isinstance(arg, (int, float)) else arg.get("seconds", 0)))
        return True
    if name == "wait_journal":
        if arg.get("any"):
            return _wait_journal_any(ctl.resolve(unit), arg["re"],
                                     float(arg.get("timeout", 60)), marks)
        return all(_wait_journal(u, arg["re"], float(arg.get("timeout", 60)), marks)
                   for u in ctl.resolve(unit))
    if name == "wait_log":
        return all(_wait_log(u, str(arg["module"]), arg["re"],
                             float(arg.get("timeout", 60)), marks)
                   for u in ctl.resolve(unit))
    if name in ("arm", "land", "reset", "disarm", "journal", "kick",
                "link_clear", "fc"):
        return run_ctl(name, unit)
    if name == "wait_ready":
        return run_ctl("wait_ready", unit, arg.get("timeout", 120))
    if name == "loss":
        return run_ctl("loss", unit, arg["percent"])
    if name == "delay":
        return run_ctl("delay", unit, arg["ms"])
    if name in ("link_cut", "commline"):
        return run_ctl(name, unit, arg["seconds"])
    if name == "link":
        # SL-0007 link-model knobs: {set: "stall 30"} or a list of
        # control sub-lines {set: ["loss_tx 0", "loss_rx 100"]}
        ok = True
        for spec in ([arg["set"]] if isinstance(arg.get("set"), str)
                     else arg.get("set") or []):
            ok = run_ctl("link", unit, *str(spec).split()) and ok
        if not arg.get("set"):
            print("  link step needs set:", flush=True)
            return False
        return ok
    if name in ("kill", "restart"):
        return run_ctl(name, unit, arg.get("proc", "all"))
    if name in ("freeze", "unfreeze"):
        # freeze <unit> <proc> [dur_s]: a hang (SIGSTOP), not a crash;
        # dur_s runs the resume inside the same ctl call for a `do:` step
        return run_ctl(name, unit, arg.get("proc", arg.get("module")),
                       *([arg["dur_s"]] if name == "freeze" and
                         arg.get("dur_s") else []))
    if name == "write_file":
        # GE-07: JSON file(s) into var/run/<unit>/<module>/ - drives module
        # file seams (de_rpi_gpio's gpio_inputs.json). `also` folds extra
        # writes into the SAME ctl call, so a burst lands inside one
        # 100 ms read window (a bounce collapses into one state). GF-07:
        # an entry may carry delay_ms - ctl gets a +N marker and sleeps
        # inside the same process, so delayed writes stay ordered but
        # land in a later read window.
        argv = [unit, arg["module"]]
        for w in [{"path": arg["path"], "json": arg["json"],
                   "delay_ms": arg.get("delay_ms")}] + \
                list(arg.get("also", [])):
            delay_ms = int(w.get("delay_ms") or 0)
            if delay_ms > 0:
                argv.append("+" + str(delay_ms))
            argv += [w["path"], json.dumps(w["json"])]
        return run_ctl("write_file", *argv)
    if name == "server_up":
        return run_ctl("server_up")
    if name == "mode":
        return run_ctl("mode", unit, arg["mode"])
    if name == "fire":
        pay = [json.dumps(arg["payload"])] if arg.get("payload") else []
        until = arg.get("until")
        if not until:
            return run_ctl("fire", arg["event"], *(pay + [unit]))
        # the fire command (GCS -> unit, 1086) has no retransmit: under an
        # injected loss it can vanish before the unit acts. With
        # `until: {re, count, unit, tries, every_s}` it is re-fired until
        # that unit's journal (since the test start) holds `count`
        # matches of `re` - the trigger is made reliable so the test checks
        # what comes AFTER it.
        if isinstance(until, str):
            until = {"re": until}
        u = ctl.resolve(str(until.get("unit", unit)))[0]
        want = int(until.get("count", 1))
        tries = int(until.get("tries", 6))
        every = float(until.get("every_s", 8))
        for k in range(tries):
            run_ctl("fire", arg["event"], *(pay + [unit]))
            t0 = time.time()
            while time.time() - t0 < every:
                n = len(simlib.journal_match(u, until["re"], marks.get(u["name"])))
                if n >= want:
                    print(f"  {u['name']}: {n} x {until['re']!r} after fire #{k + 1}",
                          flush=True)
                    return True
                time.sleep(0.5)
            print(f"  fire #{k + 1}: no {until['re']!r} x{want} on {u['name']} yet"
                  " - re-firing", flush=True)
        print(f"  FIRE FAILED: {until['re']!r} x{want} never seen on {u['name']}",
              flush=True)
        return False
    if name == "event":
        return run_ctl("event", arg if isinstance(arg, str) else arg["event"])
    if name == "send":
        return run_ctl("send", arg if isinstance(arg, str) else json.dumps(arg))
    if name == "clock_offset":
        return run_ctl("clock", "offset", arg["ms"])
    if name == "links":
        return run_ctl("links")
    if name == "wait_online":
        secs = arg if isinstance(arg, (int, float)) else arg.get("timeout", 60)
        return run_ctl("wait_online", secs)
    if name == "roster_peers":
        return run_ctl("roster", arg.get("seconds", 5), arg.get("count", 1))
    if name == "beacon":
        # SL-0007: bounded "the GCS still hears this unit" evidence
        # inside an asymmetric fault window - count the unit's mt=1004
        # presence beacons on a fresh GCS listen for `seconds`, fail
        # under `min`.
        return run_ctl("beacon", unit, arg.get("seconds", 5),
                       arg.get("min", 1))
    print(f"unknown step {name!r}", flush=True)
    return False


# ------------------------------------------------------------------ asserts

def check_assertion(a, m, marks, t_file=None):
    """Legacy assertion -> bool. ONE implementation: checks.py (the same
    code the de.scenario/1 runner uses), so a fix there fixes both. SKIP
    is not a failure here, but is printed as such."""
    import checks
    ctx = {"manifest": m, "sim": m["name"], "marks": marks,
           "file": t_file or "", "results_dir": None,
           "telemetry": None, "bot": None, "baseline": None,
           "link_events": {}, "operator_log": None}
    r = checks.check_assertion(a, ctx)
    val = "" if r["value"] is None else f" value={r['value']}"
    print(f"    [{r['status']}] {r['name']}:{val} {r['detail']}")
    return r["status"] != "FAIL"


# ------------------------------------------------------------------ main

def collect_results(t_file, m):
    ts = time.strftime("%Y%m%d-%H%M%S")
    name = os.path.splitext(os.path.basename(t_file))[0]
    out = os.path.join(simlib.RESULTS_DIR, f"{ts}-{name}")
    for u in m["units"]:
        for sub in ("journal", "logs"):
            d = os.path.join(simlib.unit_dir(u), sub)
            if os.path.isdir(d):
                shutil.copytree(d, os.path.join(out, u["name"], sub),
                                dirs_exist_ok=True)
    srv = os.path.join(simlib.server_dir(m["name"]), "logs")
    if os.path.isdir(srv):
        shutil.copytree(srv, os.path.join(out, "server"),
                        dirs_exist_ok=True)
    shutil.copy(t_file, out)
    shutil.copy(simlib.manifest_path(m["name"]), out)
    return out


def load_test(t_file):
    t = yaml.safe_load(open(t_file)) or {}
    if t.get("version", 1) != 1:
        sys.exit("unsupported test version")
    if "sim" not in t:
        sys.exit("test: `sim:` is required (a sim file path or an inline sim)")
    t["__file"] = t_file
    sim = t["sim"]
    t["__sim"] = os.path.join(os.path.dirname(t_file), sim) if isinstance(sim, str) else sim
    return t


def run(t_file, keep=False):
    global SIM_NAME
    t_file = os.path.abspath(t_file)
    t = load_test(t_file)
    m = launcher.ensure(t["__sim"])
    ctl.CTX = SIM_NAME = m["name"]
    fake = m.get("server", {}).get("type") == "fake"
    # a previous test that died mid-way may have left link faults applied
    if fake:
        run_ctl("link_clear", "all")
    # journal assertions only see lines appended after this point
    marks = {u["name"]: simlib.journal_mark(u) for u in m["units"]}
    if os.path.isfile(ctl.deployed_file()):
        os.unlink(ctl.deployed_file())
    ok = True
    for step in t.get("steps", []):
        if not run_step(step, t, marks):
            print(f"STEP FAILED: {step} - skipping the remaining steps", flush=True)
            ok = False
            break

    print("\n== assertions ==")
    results = []
    for a in t.get("assert", []):
        r = check_assertion(a, m, marks, t_file)
        results.append(r)
        print(f"  [{'PASS' if r else 'FAIL'}] {list(a)[0]}: {list(a.values())[0]}")

    # faults never leak into the next test (pass, fail or broken-off step)
    if fake:
        run_ctl("link_clear", "all")

    td = "none" if keep else t.get("teardown", "land")
    if td in ("land", "reset"):
        print(f"\n== teardown: stop + {td} ==", flush=True)
        run_ctl("stop")
        if ctl.with_fc(m["units"]) and not run_ctl(td, "all"):
            print("TEARDOWN FAILED: not every vehicle is down + disarmed", flush=True)
            ok = False
    elif td != "none":
        print(f"unknown teardown {td!r} (use land | reset | none)", flush=True)
        ok = False

    out = collect_results(t_file, m)
    n_ok = sum(results)
    print(f"\n{n_ok}/{len(results)} assertions passed"
          + ("" if ok else " (a step or the teardown failed)") + f"\nresults: {out}")
    return 0 if ok and all(results) else 1
