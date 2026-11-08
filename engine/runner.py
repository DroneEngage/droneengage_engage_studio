#!/usr/bin/env python3
"""runner.py - the `do:` step set and ctl plumbing for de.scenario/1.

scenario_runner executes a scenario's timeline; every `do:` name it does
not handle itself lands in `run_step` below (one {name: args} mapping,
the step's `do:` key plus its other keys as args). `run_ctl` shells one
`./sim ctl` command against the run's pinned sim.

Step args (the `do:` item's extra keys, shown as {name: args}):

      deploy                            # or {deploy: {package: p, unit: 2}}
      arm: all
      start
      wait_journal: {unit: 2, re: "mission main completed", timeout: 300}
      wait_log: {unit: 2, module: de_precland, re: "EVENT_OUT",
                 timeout: 30}               # the same block over
                                            # logs/<module>.log (module
                                            # stdout - never journaled)
      fire: {event: area.clear, payload: {x: 1}, unit: all}
      fire: {event: go.now, unit: 1, until: "BUS_OUT.*n=area\\.clear"}
                                            # re-fire until the unit
                                            # journals it
      opstart                           # P4A-09 fleet START (1094); the
                                        # operation must be a roles spec
      opabort                           # P4A-09 fleet ABORT
      opstatus: {seconds: 8}            # P4A-09 status poll
      opmsg: {tg: copter1, ms: {...}, expect: "re"}   # 1094 probe
      reboot: {unit: 1}                 # de_comm restart under a fresh
                                        # boot id (pending TTL signals
                                        # drop, sticky facts survive)
      loss: {unit: all, percent: 30}    # message loss: server type fake
                                        # only (link faults otherwise
                                        # work on `server: {type: real,
                                        # link_proxy: true}` too)
      link_cut: {unit: 1, seconds: 20}
      link: {unit: 1, set: "stall 30"}  # SL-0007 raw link-model knobs;
                                        # set: ["loss_tx 0", "loss_rx 100"]
                                        # sends several sub-lines;
                                        # loss* knobs are fake-only
      beacon: {unit: 1, seconds: 10, min: 3}   # count the unit's 1004
                                        # presence beacons on a fresh GCS
                                        # listen (SL-0007)
      kill: {unit: 1, proc: de_comm}    # unit: server, proc: de_server
      restart: {unit: 1, proc: de_comm}
      freeze: {unit: 1, proc: de_mavlink, dur_s: 30}   # SIGSTOP - a hang,
      unfreeze: {unit: 1, proc: de_mavlink}            # not a crash
      fcb_cut: {unit: 1, seconds: 20}      # SL-0008 fcb proxy faults -
      fcb_stall: {unit: 1, ms: 5000}       # needs `fcb_proxy: true` on
      fcb_corrupt: {unit: 1, byte_drop_pct: 1, bit_flip_pct: 0, seed: 7}
                                           # the unit/sim; any transport
      fcb_drop: {unit: 1, msgid: 30, percent: 50}   # one msgid only
      fcb_inject: {unit: 1, hex: "fe0900..."}   # raw MAVLink bytes ->
                                           # de_mavlink (TASK-14 ADS-B hook)
      fcb_clear / fcb_stats: {unit: 1}     # remove rules / counters
      cpu_starve: {unit: 1, proc: de_mavlink, duty_pct: 80,  # SL-0014:
                  dur_s: 60, cycle_ms: 1000}       # SIGSTOP/SIGCONT duty
      cpu_unstarve: {unit: 1, proc: de_mavlink}     # cycle; dur_s bounds it
      journal_unwritable: {unit: 1, dur_s: 30}      # journal dir RO,
      journal_writable: {unit: 1}                   # auto-restored on down
      clock_step: {unit: 1, offset_s: 3600}         # libfaketime step on a
                                                  # `faketime: true` unit
      reset: 1                          # SITL back to home, disarmed
      mode: {unit: 1, mode: RTL}
      takeoff: {unit: 1, alt: 10}       # NAV_TAKEOFF (arm GUIDED first)
      param: {unit: 1, name: SIM_BATT_VOLTAGE, value: 10.5}
      param_get: {unit: 1, name: SIM_BATT_VOLTAGE}
      wait_online: 60                   # every unit seen by the GCS
      wait: 10
      write_file: {unit: 1, module: de_gpio, path: gpio_inputs.json,
                   json: {"17": 1}}       # JSON into var/run/<unit>/<module>/;
                                          # also: [{path, json, delay_ms?}, ...]
                                          # writes more files in the same ctl
                                          # call; delay_ms sleeps N ms first
                                          # (a +N marker inside the call)
      udp_chunks: {unit: 1, module: de_mavlink, drop_pct: 5,
                   reorder: 3, dup_pct: 10, seed: 1}
                                          # SL-0010: knobs on the unit's
                                          # module<->de_comm chunk proxy
                                          # (needs udp_proxy: on the unit)

de.scenario/1 assertions (engine/checks.py) cover journal_count,
journal_order, journal_time_delta, mission_current and the truth/world
assertions: distance_to, detection_precision, detection_recall (P5-10:
who was found, who correctly stayed hidden), time_to_detect (P5-10:
seconds from the world epoch to the first find), signals_in_range,
coverage_complete, stub_deterministic, ... - the full list and the
per-assertion arguments live in the checks.py docstring.

`unit:` takes what ctl takes: a name, 1-based position, block:<b>, a comma
list or all (`vehicle:` / `v1` / `v2` are accepted as aliases).

A failing step stops the scenario's timeline; assertions still run and
are reported, and the run exits non-zero.
"""

import json
import os
import re
import shlex
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simlib
import ctl

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
               "fcb_cut", "fcb_stall", "fcb_corrupt", "fcb_drop",
               "fcb_inject", "fcb_clear", "fcb_stats",
               "fcb_unplug", "fcb_plug",
               "journal", "mode", "disarm", "fire", "fc",
               "write_file", "beacon", "mission_save", "mission_load",
               "takeoff", "param", "param_get",
               # SL-0014 OS faults
               "cpu_starve", "cpu_unstarve", "clock_step",
               "journal_unwritable", "journal_writable"}


def _log_event(log_events, unit, src, rx, line):
    """Stamp one wait_journal/wait_log observation for the
    log_match_delta assertion (SL-0106): module/journal lines carry no
    usable clock of their own, so the instant the runner SAW the match
    is the measurement - unit + source + the matched line's text."""
    if log_events is not None:
        log_events.append({"t_ms": int(time.time() * 1000),
                           "unit": unit, "src": src, "re": rx,
                           "line": line})


def _wait_journal(u, rx, timeout, marks, log_events=None):
    t0 = time.time()
    while time.time() - t0 < timeout:
        hits = simlib.journal_match(u, rx, marks.get(u["name"]))
        if hits:
            print(f"  {u['name']}: matched {rx!r} after {time.time()-t0:.1f}s", flush=True)
            _log_event(log_events, u["name"], "journal", rx,
                       f"{hits[0][4]} {hits[0][5]}")
            return True
        time.sleep(1)
    print(f"  TIMEOUT: {rx!r} not in {u['name']} journal after {timeout}s", flush=True)
    return False


def _wait_log(u, proc, rx, timeout, marks, log_events=None):
    """wait_log: block until the unit's logs/<proc>.log (the module's
    stdout) has a line matching rx since the test mark. The module-log
    sibling of _wait_journal - module stdout never reaches the de_comm
    journal (PL-07: de_precland prints its facts as EVENT_OUT lines)."""
    rxc = re.compile(rx)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hits = simlib.log_match(u, proc, rxc, marks.get(u["name"]))
        if hits:
            print(f"  {u['name']}.{proc}: matched {rx!r} after "
                  f"{time.time() - t0:.1f}s", flush=True)
            _log_event(log_events, u["name"], str(proc), rx, hits[0])
            return True
        time.sleep(1)
    print(f"  TIMEOUT: {rx!r} not in {u['name']} logs/{proc}.log after "
          f"{timeout}s", flush=True)
    return False


def _wait_journal_any(units, rx, timeout, marks, log_events=None):
    """any: true - done as soon as ONE of the units journals rx (a line
    whose unit is decided at runtime, e.g. which searcher adopts)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        for u in units:
            hits = simlib.journal_match(u, rx, marks.get(u["name"]))
            if hits:
                print(f"  {u['name']}: matched {rx!r} after {time.time()-t0:.1f}s", flush=True)
                _log_event(log_events, u["name"], "journal", rx,
                           f"{hits[0][4]} {hits[0][5]}")
                return True
        time.sleep(1)
    names = ",".join(u["name"] for u in units)
    print(f"  TIMEOUT: {rx!r} not in any of {names} after {timeout}s", flush=True)
    return False


def run_step(step, t, marks, log_events=None):
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

    if name == "lab_exec":
        # SL-0223/SL-0225: run a lab driver script (tests/*/name.py)
        # with the scenario's target on SIM_TARGET; its exit code is
        # the step result
        script = os.path.join(here, str(arg["script"]))
        r = subprocess.run([sys.executable, script]
                           + [str(x) for x in
                              str(arg.get("args", "")).split() if x])
        return r.returncode == 0

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
    if name == "mission_save":
        # SL-0079: {do: mission_save, unit: u, name: n, missionId: id?,
        # data: {...}} - GCS->comm->storage 9011
        extra = [str(arg["name"])]
        extra.append(str(arg["missionId"]) if arg.get("missionId") else "-")
        if arg.get("data") is not None:
            extra.append(json.dumps(arg["data"]))
        return run_ctl("mission_save", unit, *extra)
    if name == "mission_load":
        # {do: mission_load, unit: u, missionId: id?, expect: re?} - 9010
        extra = []
        if arg.get("missionId"):
            extra.append(str(arg["missionId"]))
        if arg.get("expect"):
            extra += ["--expect", str(arg["expect"])]
        return run_ctl("mission_load", unit, *extra)
    if name in ("start", "stop", "resume", "pause", "clear"):
        extra = [arg["mission"]] if isinstance(arg, dict) and arg.get("mission") else []
        return run_ctl(name, *extra)
    if name == "relay":
        # SL-0179: 9021 operator acts as the sim's GCS -
        # {do: relay, act: approve|reject|kick|change_port|query,
        #  unit, ip?, port?, secs?}; `cmd` passes any relay subcommand
        # through verbatim (relay gcs start/stop/silence/...)
        if arg.get("cmd"):
            return run_ctl("relay", *shlex.split(str(arg["cmd"])))
        args = [arg["act"], arg["unit"]]
        if arg.get("gcs"):
            args += ["--gcs", arg["gcs"]]
        if arg.get("ip"):
            args.append(arg["ip"])
        if arg.get("port"):
            args.append(arg["port"])
        if arg.get("secs") is not None:
            args += ["--secs", arg["secs"]]
        return run_ctl("relay", *args)
    if name == "wait":
        time.sleep(float(arg if isinstance(arg, (int, float)) else arg.get("seconds", 0)))
        return True
    if name == "wait_journal":
        if arg.get("any"):
            return _wait_journal_any(ctl.resolve(unit), arg["re"],
                                     float(arg.get("timeout", 60)), marks,
                                     log_events)
        return all(_wait_journal(u, arg["re"], float(arg.get("timeout", 60)),
                                 marks, log_events)
                   for u in ctl.resolve(unit))
    if name == "wait_log":
        return all(_wait_log(u, str(arg["module"]), arg["re"],
                             float(arg.get("timeout", 60)), marks,
                             log_events)
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
    if name == "fcb_cut":
        # SL-0008: sever the de_mavlink<->SITL fcb link for N seconds
        return run_ctl("fcb_cut", unit,
                       arg.get("seconds", arg.get("dur_s", 30)))
    if name == "fcb_stall":
        return run_ctl("fcb_stall", unit,
                       arg.get("ms", float(arg.get("dur_s", 1)) * 1000))
    if name == "fcb_corrupt":
        extra = [str(arg["seed"])] if arg.get("seed") is not None else []
        return run_ctl("fcb_corrupt", unit,
                       arg.get("byte_drop_pct", 0),
                       arg.get("bit_flip_pct", 0), *extra)
    if name == "fcb_drop":
        return run_ctl("fcb_drop", unit, arg.get("msgid", "all"),
                       arg.get("percent", 100))
    if name == "fcb_inject":
        return run_ctl("fcb_inject", unit, arg["hex"])
    if name in ("fcb_clear", "fcb_stats"):
        return run_ctl(name, unit)
    if name in ("fcb_unplug", "fcb_plug"):
        # SL-0107: the one-shot pull/plug of the serial bridge - the
        # `fault: fcb_unplug` + dur_s form replugs on a timer; these
        # steps let a timeline script the pair itself
        return run_ctl(name, unit)
    if name == "cpu_starve":
        # cpu_starve <unit> <proc> <duty_pct> [dur_s] [cycle_ms]
        # (SL-0014) - spawns the detached duty-cycle worker; dur_s bounds
        # its lifetime, without it cpu_unstarve/`sim down` ends it
        extra = [arg.get("proc", arg.get("module", "de_comm")),
                 arg.get("duty_pct", 80)]
        if arg.get("dur_s"):
            extra += [arg["dur_s"], arg.get("cycle_ms", 1000)]
        return run_ctl(name, unit, *extra)
    if name == "cpu_unstarve":
        return run_ctl(name, unit, arg.get("proc", arg.get("module",
                                                          "de_comm")))
    if name == "journal_unwritable":
        return run_ctl(name, unit,
                       *([arg["dur_s"]] if arg.get("dur_s") else []))
    if name == "journal_writable":
        return run_ctl(name, unit)
    if name == "clock_step":
        return run_ctl(name, unit, arg["offset_s"])
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
    if name == "udp_chunks":
        # SL-0010: chunk-fault knobs on the unit's udp_proxy
        return run_ctl("udp_chunks", unit, arg["module"],
                       arg.get("drop_pct", 0), arg.get("reorder", 0),
                       arg.get("dup_pct", 0), arg.get("seed", 0))
    if name == "server_up":
        return run_ctl("server_up")
    if name == "mode":
        return run_ctl("mode", unit, arg["mode"])
    if name == "takeoff":
        return run_ctl("takeoff", unit, arg.get("alt", arg.get("alt_m", 10)))
    if name == "param":
        return run_ctl("param", unit, arg["name"], arg["value"])
    if name == "param_get":
        return run_ctl("param_get", unit, arg["name"])
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
    import registry
    v = registry.lookup(name, "do")
    if v is not None:
        # registered but not (yet) dispatchable here - say what it wants
        print(f"unknown step {name!r} (registry args: "
              f"{', '.join(v['req'] + v['opt'])})", flush=True)
    else:
        print(f"unknown step {name!r} (catalog: ./sim verbs)",
              flush=True)
    return False
