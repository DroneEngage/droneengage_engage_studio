#!/usr/bin/env python3
"""scenario_runner.py - run a de.scenario/1 rehearsal (TASK-P5-07).

`./sim run <file>` dispatches here: a scenario
(`version: 1` + `name:` + `fleet:` + `timeline:`) runs:

  verify -> fleet up -> operator bot + telemetry on -> timeline
  (t: scheduled + on: journal-triggered steps, faults included) ->
  assertions -> teardown -> var/results/<utc>/<name>/ with journals, logs,
  telemetry.jsonl (tlog), operator.log, truth.jsonl, verify.json,
  summary.json, junit.xml.

Timeline steps carry {t, do:<step-name>, ...args} or {t, fault:<kind>,
...args} or {on: {unit, re}, do:..., once:bool} or
{on: {world, re}, do:...} (SL-0013: a world-entity watcher tails the
truth.jsonl world_event rows instead of a unit journal); on: match
groups are filled into the step's strings as {M1}..{M9}. `t` is
seconds from the scenario start; negative t runs at once before the
clock starts.

Operator-facing `do:` names (deploy/prepare/start/abort/operator) go
through the operator bot's own GCS session so the fleet sees one
consistent operator party; every other `do:` name maps to the runner.py
step of the same name (arm, land, mode, wait_journal, fire, opmsg, ...).

P5-08 world hook: a scenario `world:` file is loaded and injected into
the fleet spec before `ensure` (stub config blocks, camera/SDR model
settings on the real modules). Once the fleet is up, `world_epoch.json`
is written (the stub start gun) and a TruthRecorder writes
`var/run/<sim>/truth.jsonl` at 1 Hz; stub detection rows land in the same
file. That tick is also the single authority for entity `react:`
triggers (SL-0013): it appends knots to world_overrides.jsonl and
world_event rows to truth.jsonl. The recorder and epoch are always
torn down with the run.
"""

import json
import math
import os
import re
import shutil
import stat
import subprocess
import sys
import threading
import time

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simlib
import ctl
import launcher
import runner
import scenario
import checks
import telemetry as telemetry_mod
import operator_bot
import adapter
import world as world_mod

VERIFY_CLI = os.path.join("scripts", "verify-op.js")


# ---------------------------------------------------------------- verify

def run_verify(scn, results_dir):
    """P5-06 op verifier before the deploy. verify.json always written.
    Returns True when the scenario may proceed."""
    out = {"operation": scn.get("operation")}
    if not scn.get("operation"):
        out["skipped"] = "no operation"
        return _verify_out(out, results_dir, True)
    spec = scn.get("verify") or {}
    # default: the wrapper (hydrates the ctx sidecar and skips compiled
    # packages / deploy specs, which verify-op.js alone fails VER-000);
    # the bare webclient CLI only when the wrapper is absent
    wrap = os.path.join(simlib.SIM_ROOT, "tools", "verify_wrap.js")
    cli = spec.get("cli") or (wrap if os.path.isfile(wrap) else
                              os.path.join(simlib.WEBCLIENT_REPO, VERIFY_CLI))
    if spec.get("cli") and not os.path.isabs(cli):
        cli = os.path.join(os.path.dirname(scn["file"]), cli)
    if not os.path.isfile(cli):
        out["skipped"] = "P5-06 CLI not yet available"
        out["cli"] = cli
        return _verify_out(out, results_dir, True)
    t0 = time.time()
    # --json: without it the CLI prints human text and the findings can
    # never be counted (a list of {sev, rule, path, msg} on stdout)
    try:
        # the wrapper locates the webclient repo by env or sibling path;
        # neither works from an sl-* worktree - hand it the resolved
        # simlib paths (same values the main checkout would find)
        env = dict(os.environ, DE_CODE=simlib.DE_CODE,
                   DE_WEBCLIENT_DIR=simlib.WEBCLIENT_REPO)
        r = subprocess.run(["node", cli, scn["operation"], "--json"],
                           capture_output=True, text=True, timeout=120,
                           env=env)
    except (subprocess.TimeoutExpired, OSError) as e:
        out["cli"] = cli
        out["error"] = f"{type(e).__name__}: {e}"
        out["ok"] = False
        return _verify_out(out, results_dir, False)
    out["cli"] = cli
    out["wall_s"] = round(time.time() - t0, 2)
    out["stdout_tail"] = r.stdout[-4000:]
    out["stderr_tail"] = r.stderr[-2000:]
    try:
        rep = json.loads(r.stdout)
    except ValueError:
        rep = None
    out["report"] = rep
    # CLI exit: 0 = no error findings, 1 = error findings, 2 = bad usage /
    # unreadable input. An unparseable report is a verifier failure, never
    # "zero errors".
    # two report shapes: scripts/verify-op.js --json prints a LIST of
    # findings {sev, rule, ...}; wrapper CLIs (tools/) print
    # {error_count, errors:[{rule}]}
    n_err = rules = None
    if isinstance(rep, list):
        errs = [f for f in rep
                if isinstance(f, dict) and f.get("sev") == "error"]
        n_err = len(errs)
    elif isinstance(rep, dict):
        errs = [e for e in (rep.get("errors") or []) if isinstance(e, dict)]
        n_err = rep["error_count"] if isinstance(rep.get("error_count"),
                                                 int) else len(errs)
    if n_err is None or r.returncode not in (0, 1):
        out["error"] = (f"verifier exit {r.returncode}, "
                        + ("no usable JSON report on stdout" if n_err is None
                           else "unexpected exit code"))
        out["ok"] = False
        return _verify_out(out, results_dir, False)
    rules = sorted({e.get("rule") or e.get("ruleId") or "?" for e in errs})
    out["errors"] = n_err
    out["rules"] = rules
    if spec.get("proceed_rules") is not None:
        # a negative-control scenario deploys past the gate: proceed
        # ONLY when the error findings are exactly the declared rules -
        # any extra or missing rule still stops the run before deploy
        ok = rules == sorted(spec["proceed_rules"])
        out["ok"] = ok
        out["gate_proceed"] = ok
        return _verify_out(out, results_dir, ok)
    if "expect_errors" in spec or spec.get("expect_rules") is not None:
        # a negative scenario EXPECTS error findings (exit 1 is fine):
        # expectations only classify the run at suite level (GATE-PASS
        # vs FAIL through verify.json), they never let a dirty op past
        # the gate - deploy refuses verifier errors, expected or not
        ok = True
        if "expect_errors" in spec:
            ok &= n_err == spec["expect_errors"]
        if spec.get("expect_rules") is not None:
            ok &= rules == sorted(spec["expect_rules"])
    else:
        ok = n_err == 0
    out["ok"] = ok
    return _verify_out(out, results_dir, ok and n_err == 0)


def _verify_out(out, results_dir, proceed):
    if results_dir:
        os.makedirs(results_dir, exist_ok=True)
        simlib.write_json(os.path.join(results_dir, "verify.json"),
                          out, indent=2)
    return proceed


# ------------------------------------------------------------- timelines

def _fill(obj, groups):
    """Deep-replace {M1}..{M9} in every string of the step."""
    if isinstance(obj, str):
        for i, g in enumerate(groups, 1):
            obj = obj.replace("{M%d}" % i, g or "")
        return obj
    if isinstance(obj, list):
        return [_fill(x, groups) for x in obj]
    if isinstance(obj, dict):
        return {k: _fill(v, groups) for k, v in obj.items()}
    return obj


def _step_args(step):
    return {k: v for k, v in step.items()
            if k not in ("t", "on", "do", "fault", "once")}


def _do(step, scn, ctx):
    """A do: step - dispatched through ctx['adapter'] (the vehicle-stack
    seam, engine/adapter.py + docs/ADAPTER.md)."""
    name = step["do"]
    a = _step_args(step)
    # an on-triggered step defaults its unit to the trigger's unit
    # (a world-entity trigger has none - the step must name its own)
    if "on" in step and "unit" not in a and step["on"].get("unit"):
        a["unit"] = step["on"]["unit"]
    import registry
    if registry.lookup(name, "do") is None:
        # _validate already gates this; belt-and-braces for steps a
        # caller builds without loading through scenario.py
        print(f"  unknown step {name!r} (catalog: ./sim verbs)",
              flush=True)
        return False
    return ctx["adapter"].do_step(name, a, step, scn, ctx)


def ctx_manifest():
    """The live manifest - kept for the fault helpers below that run
    outside a step's ctx."""
    return _CTX[0]["manifest"]


_CTX = [None]


# ---------------------------------------------------------------- faults

def _schedule(ctx, after_s, fn, *args):
    # `fired` marks "the callback began" - is_alive() alone cannot tell a
    # pending timer from one mid-callback, and the finally flush below
    # must not double-run restores that are already in flight
    fired = threading.Event()
    def once():
        fired.set()
        fn(*args)
    t = threading.Timer(after_s, once)
    t.daemon = True
    t.fired_once = fired
    t.start()
    ctx["timers"].append(t)


def _fc_param_restore(unit, spec):
    """The dur_s restore half of a param fault: write every read-back
    value through the verified ctl path."""
    ok = True
    for name, val in spec.items():
        ok &= runner.run_ctl("param", unit, name, val)
    return ok


def _fc_param_fault(ctx, unit, spec, dur, restore_over=None):
    """Generic FC param fault (SL-0005): one serial0 session per unit
    reads each param BEFORE setting it; dur_s schedules the restore of
    the values that were read (never a baked-in default). `restore_over`
    maps {param: value} explicit restores on top (battery's ok_* args)."""
    us = [u for u in ctl.resolve(unit, ctx["manifest"]) if u["fc"]]
    if not us:
        print("  fault param needs an FC unit", flush=True)
        return False
    ok = True
    for u in us:
        m = ctl.mav(u)
        before = {}
        try:
            for name in spec:
                v = ctl.fc_param_get(m, name)
                if v is None:
                    print(f"  {u['name']}: {name} not on the FC",
                          flush=True)
                    ok = False
                    continue
                before[name] = v
            for name, val in spec.items():
                if name in before:
                    ok &= ctl.fc_param_set(m, u, name, val)
        finally:
            m.close()
        if dur and before:
            restore = dict(before)
            for p, v in (restore_over or {}).items():
                if v is not None and p in restore:
                    restore[p] = v
            _schedule(ctx, float(dur), _fc_param_restore,
                      u["name"], restore)
    return ok


def _fc_param_spec(kind, a):
    """fault kind + step args -> {PARAM: value}. `param` takes its own
    set:; a preset expands its table row (scenario.FC_PRESETS)."""
    if kind == "param":
        return {str(k): v for k, v in (a.get("set") or {}).items()}
    out = {}
    for p, (arg, default) in scenario.FC_PRESETS[kind].items():
        v = a.get(arg, default) if arg else default
        if v is not None:
            out[p] = v
    return out


def _rc_pump(ctx, u, chans, dur_s):
    """RC_CHANNELS_OVERRIDE times out ~1 s after the last frame - one
    pilot connection re-sends the override at 2.5 Hz for dur_s, then
    releases every channel (65535 = no override)."""
    def pump():
        try:
            m = ctl.mav(u, src=255)   # SYSID_MYGCS or SITL drops the override
            end = time.time() + float(dur_s)
            while time.time() < end:
                for ch, pwm in chans.items():
                    c = [65535] * 18
                    c[int(ch) - 1] = int(pwm)
                    m.mav.rc_channels_override_send(
                        m.target_system, m.target_component, *c)
                time.sleep(0.4)
            c = [65535] * 18
            m.mav.rc_channels_override_send(
                m.target_system, m.target_component, *c)
            m.close()
        except Exception as e:
            print(f"  rc_override pump {u['name']}: "
                  f"{type(e).__name__}: {e}", flush=True)
    t = threading.Thread(target=pump, daemon=True)
    t.start()
    ctx["pumps"].append(t)


def _fault(step, scn, ctx):
    kind = step["fault"]
    import registry
    if registry.lookup(kind, "fault") is None:
        print(f"  unknown fault {kind!r} (catalog: ./sim verbs)",
              flush=True)
        return False
    a = _step_args(step)
    if "on" in step and "unit" not in a and step["on"].get("unit"):
        a["unit"] = step["on"]["unit"]
    unit = str(a.get("unit", "all"))
    dur = a.get("dur_s", a.get("seconds"))
    bot = ctx["bot"]

    def _proc_event(proc, u=unit):
        # a fault window that may legitimately change pids - the
        # no_unexpected_restart invariant allows changes only inside it
        ctx["proc_events"].append({"t_ms": int(time.time() * 1000),
                                   "unit": str(u), "proc": str(proc)})

    def _restore_at(after_s=0.0):
        # the instant the unit's connectivity should be back - the
        # recovers_after_fault invariant checks online + telemetry
        # after it
        ctx["restore_events"].append(
            {"t_ms": int(time.time() * 1000) + int(after_s * 1000),
             "unit": unit, "kind": kind})

    def _restart_later(after_s, owner, proc):
        """dur_s on kill/module_kill/server_down: bring the proc back
        inside a recorded fault window, so no_unexpected_restart still
        sees the new pid as expected."""
        def go():
            _proc_event(proc, "server" if owner == "server" else unit)
            if owner == "server":
                runner.run_ctl("server_up")
            else:
                runner.run_ctl("restart", owner, proc)
        _schedule(ctx, float(after_s), go)

    if kind == "link_cut":
        _restore_at(float(dur or 30))
        return runner.run_ctl("link_cut", unit, dur or 30)
    if kind == "link_stall":
        # SL-0007: a hard in-order hold of every byte on the link for
        # dur|seconds s - the ws stays up (only the unit's own watchdog
        # notices); the buffered frames flush when the window ends.
        seconds = float(dur or 30)
        _restore_at(seconds)
        return runner.run_ctl("link", unit, "stall", seconds)
    if kind == "link_profile":
        # SL-0007: a named link_model preset (lte_edge, uplink_dead,
        # ...) via `profile:`, and/or raw model knobs via
        # `cmds: ["loss_rx 100", ...]`; dur_s clears the link after.
        ok = True
        if a.get("profile"):
            ok = runner.run_ctl("link", unit, "profile",
                                a["profile"]) and ok
        for cmd in a.get("cmds") or []:
            ok = runner.run_ctl("link", unit,
                                *str(cmd).split()) and ok
        if not a.get("profile") and not a.get("cmds"):
            print("  link_profile needs profile:|cmds:", flush=True)
            return False
        if ok and dur:
            _schedule(ctx, float(dur), runner.run_ctl, "link_clear",
                      unit)
            _restore_at(float(dur))
        return ok
    if kind == "loss":
        ok = runner.run_ctl("loss", unit, a.get("percent", 50))
        if dur:
            # windowed degradation: link_clear restores; the link may
            # have dropped meanwhile - recovers_after_fault checks it
            _schedule(ctx, float(dur), runner.run_ctl, "link_clear", unit)
            _restore_at(float(dur))
        return ok
    if kind == "delay":
        ok = runner.run_ctl("delay", unit, a.get("ms", 500))
        if dur:
            _schedule(ctx, float(dur), runner.run_ctl, "link_clear", unit)
            _restore_at(float(dur))
        return ok
    if kind == "gcs_disconnect":
        ok = True
        if ctx["server"]["type"] != "fake":
            # the ctl/deploy GCS is the bridge on a real stack - drop it
            _proc_event("gcs_bridge", "server")
            ok = runner.run_ctl("kill", "server", "gcs_bridge")
            if dur:
                _restart_later(float(dur), "server", "gcs_bridge")
        if dur:
            bot.disconnect_for(float(dur))
        else:
            bot.disconnect_for(30)
        return ok
    if kind == "server_down":
        _proc_event(a.get("proc", "all"), "server")
        ok = runner.run_ctl("kill", "server", a.get("proc", "all"))
        if dur:
            _restart_later(float(dur), "server", a.get("proc", "all"))
            _restore_at(float(dur))
        return ok
    if kind == "kill":
        _proc_event(a.get("proc", "all"))
        ok = runner.run_ctl("kill", unit, a.get("proc", "all"))
        if dur:
            _restart_later(float(dur), unit, a.get("proc", "all"))
            _restore_at(float(dur))
        return ok
    if kind == "restart":
        _proc_event(a.get("proc", "all"))
        _restore_at()
        return runner.run_ctl("restart", unit, a.get("proc", "all"))
    if kind == "module_kill":
        _proc_event(a["module"])
        ok = runner.run_ctl("kill", unit, a["module"])
        if dur:
            _restart_later(float(dur), unit, a["module"])
            _restore_at(float(dur))
        return ok
    if kind == "freeze":
        # SIGSTOP - a hang, not a crash: sockets stay open, the process
        # is alive-but-silent (TASK-04). dur_s schedules the SIGCONT like
        # every other timed fault's restore; without it the unit stays
        # frozen until teardown (launcher SIGCONTs before killing).
        proc = a.get("proc") or a.get("module")
        if not proc:
            print("  fault freeze needs proc:|module:", flush=True)
            return False
        ok = runner.run_ctl("freeze", unit, proc)
        if ok and dur:
            _schedule(ctx, float(dur), runner.run_ctl,
                      "unfreeze", unit, proc)
            if proc in ("de_comm", "all"):
                _restore_at(float(dur))
        return ok
    if kind == "param" or kind in scenario.FC_PRESETS:
        # SL-0005 generic path: `param` writes its set:; the named
        # presets (gps_jam, motor_out, ...) and the re-implemented
        # battery/gps_off expand to one. Everything rides the
        # read-before-set + read-back restore machinery.
        spec = _fc_param_spec(kind, a)
        if not spec:
            print(f"  fault {kind}: empty param set", flush=True)
            return False
        if kind == "gps_glitch":
            # GLTCH.x/y are DEGREES of lat/lng on this ArduPilot
            # (SIM_GPS.cpp: d.latitude += glitch.x) - the step args are
            # metres (z already is); convert against the sim home
            home = (ctx["manifest"].get("home") or simlib.HOME)[0]
            spec["SIM_GPS1_GLTCH_X"] = \
                float(spec["SIM_GPS1_GLTCH_X"]) / 111320.0
            spec["SIM_GPS1_GLTCH_Y"] = float(spec["SIM_GPS1_GLTCH_Y"]) / (
                111320.0 * math.cos(math.radians(home)))
        # the supervisor's battery_low watches remaining PERCENT, which
        # the FC derives from consumed mAh vs BATT_CAPACITY - a voltage
        # sag alone never moves it. cap_mah shrinks the pack so remaining
        # collapses to 0% at once (must stay >10 mAh: below that the FC
        # reports the reading invalid, which would reset the handler).
        # ok_volts/ok_cap_mah (legacy args) pin the restore instead of
        # the read-back value.
        restore_over = None
        if kind == "battery":
            restore_over = {"SIM_BATT_VOLTAGE": a.get("ok_volts"),
                            "BATT_CAPACITY": a.get("ok_cap_mah")}
        return _fc_param_fault(ctx, unit, spec, dur, restore_over)
    if kind == "server_clock":
        if ctx["server"]["type"] != "fake":
            print("  fault server_clock needs transport: fake", flush=True)
            return False
        ok = runner.run_ctl("clock", "offset",
                            a.get("offset_ms", -3600000))
        if dur:
            _schedule(ctx, float(dur), runner.run_ctl,
                      "clock", "offset", 0)
        return ok
    if kind == "no_time_sync":
        if ctx["server"]["type"] != "fake":
            print("  fault no_time_sync needs transport: fake", flush=True)
            return False
        ok = runner.run_ctl("noclock", unit, 1)
        if dur:
            _schedule(ctx, float(dur), runner.run_ctl, "noclock", unit, 0)
        return ok
    if kind == "rc_override":
        chans = a.get("channels") or {a.get("chan", 3): a.get("pwm", 1600)}
        us = [u for u in ctl.resolve(unit, ctx["manifest"]) if u["fc"]]
        if not us:
            print("  rc_override needs an FC unit", flush=True)
            return False
        for u in us:
            _rc_pump(ctx, u, chans, dur or 15)
        return True
    if kind == "udp_chunks":
        # SL-0010: seeded chunk drop/reorder/dup on the module<->de_comm
        # s2s link. The proxy itself is wired at sim-up (the scenario
        # loader injects the unit's udp_proxy opt-in for this fault);
        # the fault only flips knobs - no pid changes, and the unit's
        # server link is untouched, so dur_s restores are a plain
        # ctl call plus a restore event for recovers_after_fault.
        mod = a.get("module")
        if not mod:
            print("  fault udp_chunks needs module:", flush=True)
            return False
        ok = runner.run_ctl("udp_chunks", unit, mod,
                            a.get("drop_pct", 0), a.get("reorder", 0),
                            a.get("dup_pct", 0),
                            a.get("seed", scn["seed"]))
        if ok and dur:
            _schedule(ctx, float(dur), runner.run_ctl,
                      "udp_chunks", unit, mod, "clear")
            _restore_at(float(dur))
        return ok
    if kind == "mode":
        return runner.run_ctl("mode", unit, a["mode"])
    if kind in ("fcb_cut", "fcb_stall", "fcb_rate", "fcb_corrupt"):
        # SL-0008: faults on the per-unit fcb proxy between de_mavlink
        # and SITL (unit/sim `fcb_proxy: true`). The proxy is ours, not
        # the server's - these work on any transport.
        us = [u for u in ctl.resolve(unit, ctx["manifest"]) if u["fc"]]
        if not us:
            print(f"  {kind} needs an FC unit", flush=True)
            return False
        if not all(u["fc"].get("fcb_proxy") for u in us):
            print(f"  {kind} needs `fcb_proxy: true` on the unit/sim",
                  flush=True)
            return False
        if kind == "fcb_cut":
            # the proxy itself expires the cut (accepted-then-dropped
            # reconnects); de_mavlink's TCP retry loop drives the restore
            _restore_at(float(dur or 30))
            return runner.run_ctl("fcb_cut", unit, dur or 30)
        if kind == "fcb_stall":
            ms = a.get("ms") or int(float(dur or 1) * 1000)
            _restore_at(ms / 1000.0)
            return runner.run_ctl("fcb_stall", unit, ms)
        if kind == "fcb_rate":
            # SL-0101: UART pacing (baud/10 bytes/s per direction);
            # dur_s restores the full-speed link through fcb_clear
            ok = runner.run_ctl("fcb_rate", unit, a.get("baud", 9600))
            if ok and dur:
                _schedule(ctx, float(dur), runner.run_ctl,
                          "fcb_clear", unit)
                _restore_at(float(dur))
            return ok
        ok = runner.run_ctl("fcb_corrupt", unit,
                            a.get("byte_drop_pct", 0),
                            a.get("bit_flip_pct", 0),
                            a.get("seed", ctx["scn"]["seed"]))
        if ok and dur:
            _schedule(ctx, float(dur), runner.run_ctl, "fcb_clear", unit)
            _restore_at(float(dur))
        return ok
    if kind == "fcb_unplug":
        # SL-0103: yank the serial device a `fcb_link` unit's de_mavlink
        # sits on - the fcb_tty bridge's whole process group is killed,
        # so the device node goes away (pty link + /dev/pts/N), like a
        # USB cable pull. dur_s schedules the replug (same link path)
        # like every timed fault's restore; both pid changes are marked
        # for no_unexpected_restart.
        us = [u for u in ctl.resolve(unit, ctx["manifest"]) if u["fc"]]
        if not us:
            print("  fcb_unplug needs an FC unit", flush=True)
            return False
        if not all(simlib.bridge_proc(u) for u in us):
            print("  fcb_unplug needs an `fcb_link` bridged unit "
                  "(kind pty/host_serial/bt_spp/udp)", flush=True)
            return False
        # record each unit's real bridge proc so no_unexpected_restart
        # windows cover its pid changes (fcb_tty / bt_spp by kind)
        def _bridge_event():
            for u in us:
                _proc_event(simlib.bridge_proc(u), u["name"])
        _bridge_event()
        ok = runner.run_ctl("fcb_unplug", unit)
        if ok and dur:
            def _replug():
                _bridge_event()
                runner.run_ctl("fcb_plug", unit)
            _schedule(ctx, float(dur), _replug)
            _restore_at(float(dur))
        return ok
    if kind == "cpu_starve":
        # SL-0014: SIGSTOP/SIGCONT duty cycle - the proc keeps its pid
        # and sockets but gets (100-duty_pct)% of each cycle_ms window.
        # A detached ctl worker does the cycling so the timeline never
        # blocks; dur_s both ends the worker and schedules the restore
        # like every timed fault.
        proc = a.get("proc") or a.get("module") or "de_comm"
        # the worker itself is a pidfile-tracked proc ("<proc>.starve")
        # - its appearance, and its exit at dur_s, must sit inside a
        # fault window for no_unexpected_restart
        _proc_event(proc + ".starve")
        ok = runner.run_ctl("cpu_starve", unit, proc,
                            a.get("duty_pct", 80), dur or 0,
                            a.get("cycle_ms", 1000))
        if ok and dur:
            _schedule(ctx, float(dur), runner.run_ctl,
                      "cpu_unstarve", unit, proc)
            _schedule(ctx, float(dur), _proc_event, proc + ".starve")
            if proc in ("de_comm", "all"):
                _restore_at(float(dur))
        return ok
    if kind == "journal_unwritable":
        # SL-0014: the unit's journal dir goes read-only - de_comm stays
        # alive but cannot open a new journal sink. dur_s schedules the
        # restore; `sim down` also restores so nothing stays broken.
        ok = runner.run_ctl("journal_unwritable", unit)
        if ok and dur:
            _schedule(ctx, float(dur), runner.run_ctl,
                      "journal_writable", unit)
        return ok
    if kind == "clock_step":
        # SL-0014: step a `faketime: true` unit's de_comm clock via its
        # FAKETIME_TIMESTAMP_FILE - per-process, no relaunch. dur_s
        # schedules the step back to "+0".
        if "offset_s" not in a:
            print("  fault clock_step needs offset_s:", flush=True)
            return False
        ok = runner.run_ctl("clock_step", unit, a["offset_s"])
        if ok and dur:
            _schedule(ctx, float(dur), runner.run_ctl,
                      "clock_step", unit, 0)
        return ok
    print(f"  unknown fault {kind!r}", flush=True)
    return False


# --------------------------------------------------------- link monitor

def _sampler(ctx, stop):
    """1 Hz wall-clock over this sim's pidfile pids (read_pids already
    gates on pid_is_ours): rss_kb + open-fd count + the pid itself ->
    results/samples.jsonl. Feeds the invariant assertions
    (no_unexpected_restart, rss_growth, fd_growth)."""
    path = ctx["samples_file"]
    rss_re = re.compile(r"VmRSS:\s*(\d+)")
    while not stop.is_set():
        now = int(time.time() * 1000)
        rows = []
        for key, pid in sorted(simlib.read_pids(ctx["sim"]).items()):
            try:
                st = open(f"/proc/{pid}/status").read()
                rss = int(rss_re.search(st).group(1))
                fd = len(os.listdir(f"/proc/{pid}/fd"))
            except (OSError, ValueError, AttributeError):
                continue                  # died between read_pids and now
            rows.append({"t_ms": now, "key": key, "pid": pid,
                         "rss_kb": rss, "fd": fd})
        try:
            with open(path, "a") as f:
                for r in rows:
                    f.write(json.dumps(r) + "\n")
        except OSError:
            pass
        stop.wait(1.0)


def _link_monitor(ctx, stop):
    """Poll the server's online list; journal every unit's link
    transitions to links.jsonl (link_offline reads ctx["link_events"]).
    Works on fake (control `list`) and real (bridge `list`) alike. A fake
    server is also tailed for "[server] ... connected/disconnected": a
    drop+reconnect inside one poll window (a hung de_comm resuming) still
    lands as an offline->online pair.

    SL-0009: units behind a link_proxy are read off the proxy instead -
    the bridge `list` is a 60 s "heard lately" window that cannot see a
    30 s cut, while the proxy knows the exact session state. Its proc
    log is tailed like the fake's for sub-poll transitions."""
    path = os.path.join(ctx["results_dir"], "links.jsonl")
    events = ctx["link_events"]
    # a fake server is also tailed for "[server] <party> connected/
    # disconnected": a drop+reconnect inside one poll window (a hung
    # de_comm resuming) still lands as an offline->online pair
    log_path = (os.path.join(simlib.run_dir(ctx["sim"]), "server", "logs",
                             "fake_server.log")
                if ctx["server"]["type"] == "fake" else None)
    unit_parties = {u["party_id"] for u in ctx["manifest"]["units"]}
    log_re = re.compile(r"\[server\] (?:d:)?(\S+) "
                        r"(connected|disconnected)")
    log_pos = 0
    # SL-0009: proxied units - per-party proxy session state, plus a
    # tail of each link_proxy log ("session established/closed")
    proxied = {u["party_id"]: u for u in ctx["manifest"]["units"]
               if u.get("link_proxy")}
    px_logs = {p: {"path": os.path.join(
                    simlib.unit_dir(u), "logs", "link_proxy.log"),
                   "pos": 0}
               for p, u in proxied.items()}
    px_re = re.compile(r"session (established|closed)")
    # a falsy proxy `list` (transient connect blip) must not flap the
    # party - offline needs two consecutive falsy polls; the px log's
    # "session closed" stays the precise edge
    px_falsy = {}
    was = None

    def record(p, online, now):
        events.setdefault(p, []).append((now, online))
        try:
            with open(path, "a") as f:
                f.write(json.dumps(
                    {"t_ms": now, "party": p, "online": online}) + "\n")
        except OSError:
            pass

    while not stop.is_set():
        try:
            online = ctl.online_parties(ctx["sim"]) - set(proxied)
        except BaseException:
            # server/control port unreachable (server_down fault, restart
            # in flight): nobody is online at a server that is not there
            online = set()
        for p, u in proxied.items():
            try:
                st = ctl.link_proxy_online(u)
            except BaseException:
                st = None
            if st:
                px_falsy[p] = 0
                online.add(p)
            else:
                px_falsy[p] = px_falsy.get(p, 0) + 1
                if was is not None and p in was and px_falsy[p] < 2:
                    online.add(p)        # suspect once, offline twice
        if was is None and not online:
            stop.wait(1.5)
            continue
        if was is None and (log_path or px_logs):
            # the first sample is the baseline; skip every log's
            # history - a reused sim replays the last run's transitions
            # otherwise
            if log_path:
                try:
                    log_pos = os.path.getsize(log_path)
                except OSError:
                    log_pos = 0
            for st in px_logs.values():
                try:
                    st["pos"] = os.path.getsize(st["path"])
                except OSError:
                    st["pos"] = 0
        now = int(time.time() * 1000)
        if was is not None:
            if log_path:
                # every new log line arrived since the last poll - replay
                # in order, deduped against was (poll may see it too)
                try:
                    with open(log_path, errors="ignore") as f:
                        f.seek(log_pos)
                        lines = f.readlines()
                        log_pos = f.tell()
                except OSError:
                    lines = []
                for ln in lines:
                    mm = log_re.search(ln)
                    if not mm or mm.group(1) not in unit_parties:
                        continue
                    p, on = mm.group(1), mm.group(2) == "connected"
                    if on == (p in was):
                        continue
                    record(p, on, now)
                    if on:
                        was.add(p)
                    else:
                        was.discard(p)
            for p, st in px_logs.items():
                # a proxied session that died and re-established inside
                # one poll still lands as an offline->online pair
                try:
                    with open(st["path"], errors="ignore") as f:
                        f.seek(st["pos"])
                        lines = f.readlines()
                        st["pos"] = f.tell()
                except OSError:
                    lines = []
                for ln in lines:
                    mm = px_re.search(ln)
                    if not mm:
                        continue
                    on = mm.group(1) == "established"
                    if on == (p in was):
                        continue
                    record(p, on, now)
                    if on:
                        was.add(p)
                    else:
                        was.discard(p)
            for p in was - online:
                record(p, False, now)
            for p in online - was:
                record(p, True, now)
        was = online
        stop.wait(1.5)


# ------------------------------------------------------------- execution

def _run_step(step, scn, ctx):
    try:
        if "do" in step:
            return _do(step, scn, ctx)
        return _fault(step, scn, ctx)
    except SystemExit as e:
        print(f"  step aborted: {e}", flush=True)
        return False
    except Exception as e:
        print(f"  step error: {type(e).__name__}: {e}", flush=True)
        return False


def _watcher_fires(w, ctx):
    """Poll the watcher's unit journal; run the step on new matches.
    Returns False when the step itself failed."""
    ok = True
    u = ctx["units_by_name"].get(w["unit"])
    if u is None:
        return True
    if w["offsets"] is None:
        w["offsets"] = dict(ctx["marks"].get(u["name"]) or {})
    for ln in simlib.journal_read_new(u, w["offsets"]):
        m = w["rx"].search(ln[4] + " " + ln[5]) or w["rx"].search(ln[5])
        if not m:
            continue
        w["fired"] += 1
        step = _fill(w["step"], m.groups())
        print(f"  on:{w['unit']} {w['rx'].pattern!r} matched "
              f"({ln[4]} {ln[5][:60]}) - firing", flush=True)
        ok &= _run_step(step, ctx["scn"], ctx)
        if w["once"]:
            w["armed"] = False
            break
    return ok


def _fleet_poses(ctx):
    """{name: {"lat","lng","cls"}} for the react authority (SL-0013):
    FC units report the telemetry sampler's latest row, statics sit at
    their manifest position. cls is the scenario fleet cls (unit_cls
    matches what the scenario author wrote - copter|static|a block)."""
    out = {}
    fleet_cls = {str(f.get("id")): f.get("cls")
                 for f in ctx["scn"].get("fleet") or []}
    tele = ctx.get("telemetry")
    for u in ctx["manifest"]["units"]:
        cls = fleet_cls.get(u["name"]) or u["block"]
        if u.get("fc"):
            row = tele.latest(u["name"]) if tele else {}
            if row.get("lat") is not None:
                out[u["name"]] = {"lat": row["lat"], "lng": row["lng"],
                                  "cls": cls}
        else:
            p = u.get("position") or {}
            out[u["name"]] = {"lat": p.get("lat"), "lng": p.get("lng"),
                              "cls": cls}
    return out


def _world_watcher_fires(w, ctx):
    """SL-0013: an `on: {world: <entity>, re}` watcher tails
    truth.jsonl for that entity's world_event rows (the react
    authority's log) and runs the step on new regex matches - the
    match runs against the row's msg text."""
    ok = True
    path = world_mod.truth_path(ctx["sim"])
    try:
        with open(path, "rb") as f:
            f.seek(w["offset"])
            data = f.read()
    except OSError:
        return True
    end = data.rfind(b"\n") + 1
    w["offset"] += end                    # torn tail re-read next poll
    for ln in data[:end].decode(errors="replace").splitlines():
        try:
            row = json.loads(ln)
        except ValueError:
            continue
        if row.get("type") != "world_event" or \
                str(row.get("tid")) != w["world"]:
            continue
        m = w["rx"].search(str(row.get("msg")
                               or json.dumps(row, sort_keys=True)))
        if not m:
            continue
        w["fired"] += 1
        step = _fill(w["step"], m.groups())
        print(f"  on:world {w['world']} {w['rx'].pattern!r} matched "
              f"({str(row.get('msg'))[:60]}) - firing", flush=True)
        ok &= _run_step(step, ctx["scn"], ctx)
        if w["once"]:
            w["armed"] = False
            break
    return ok


def run_timeline(scn, ctx):
    """Scheduled t: steps + armed on: watchers until last_t + settle_s."""
    steps = scn["timeline"]
    t_steps = sorted((s for s in steps if "t" in s), key=lambda s: s["t"])
    watchers = []
    for s in steps:
        if "on" not in s:
            continue
        w = {"rx": re.compile(s["on"]["re"]),
             "step": s, "fired": 0,
             "once": bool(s.get("once", True)), "armed": True}
        if "world" in s["on"]:
            # SL-0013: a world-entity watcher tails truth.jsonl's
            # world_event rows; offset 0 is fine - clear_run_state
            # truncated the file for this run
            w.update(world=str(s["on"]["world"]), offset=0)
        else:
            w.update(unit=str(s["on"]["unit"]), offsets=None)
        watchers.append(w)
    ok = True
    neg = [s for s in t_steps if s["t"] < 0]
    pos = [s for s in t_steps if s["t"] >= 0]
    for s in neg:
        print(f"  t={s['t']}: {json.dumps(_step_args(s))}", flush=True)
        ok &= _run_step(s, scn, ctx)
    t0 = time.time()
    last_t = max([s["t"] for s in pos], default=0)
    end = t0 + last_t + scn["settle_s"]
    pending = list(pos)
    while pending or time.time() < end:
        now = time.time() - t0
        while pending and pending[0]["t"] <= now:
            s = pending.pop(0)
            print(f"  t={s['t']}: {json.dumps(_step_args(s))}", flush=True)
            ok &= _run_step(s, scn, ctx)
            # a blocking step (wait_journal, deploy) may run past last_t:
            # settle_s is measured from when the last step really ended
            if not pending:
                end = max(end, time.time() + scn["settle_s"])
        for w in watchers:
            if w["armed"]:
                ok &= (_world_watcher_fires(w, ctx) if "world" in w
                       else _watcher_fires(w, ctx))
        time.sleep(0.25)
    return ok


def _hold(m, world_stream=False, open_web=False):
    """--hold: park the run after the timeline with the world stream,
    truth recorder, operator bot and render procs still live - the
    interactive Mavlink3DMap session. Ctrl-C releases into the normal
    assertions/teardown. `world_stream` says a de.worldstream/1 feed is
    live on render.stream_port - only then does the printed URL carry
    ?world= (a bare sim has no stream). `open_web` xdg-opens the
    webclient link."""
    render = m.get("render") or {}
    print(simlib.col_bold("\n== holding - Ctrl-C to finish the run =="),
          flush=True)
    # the parked sim's webclient deep links - a held run is exactly when
    # the operator wants to click into the GCS (each page gets its own
    # GCS senderID)
    print("webclient: " + simlib.col_cyan(simlib.webclient_url(m)),
          flush=True)
    print("planning:  " + simlib.col_cyan(
          simlib.webclient_url(m, "/planning", "GCSPLN_" + m["name"])),
          flush=True)
    if open_web:
        ctl.xdg_open(simlib.webclient_url(m))
    if render:
        url = simlib.render_viewer_url(render, world_stream)
        print("render viewer: " + simlib.col("1;36", url), flush=True)
        # SL-PG-03: the held run IS the viewer session - open it rather
        # than making the operator copy-paste; no opener -> the printed
        # URL still stands
        try:
            subprocess.Popen(["xdg-open", url],
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        except OSError as e:
            print(f"xdg-open failed: {e} - open the URL above by hand",
                  flush=True)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("hold released\n", flush=True)


# ---------------------------------------------------------------- results

def _results_dir(scn):
    ts = time.strftime("%Y-%m-%dT%H-%M-%SZ", time.gmtime())
    return os.path.join(simlib.RESULTS_DIR, ts, scn["name"])


def _commits():
    repos = {"sim": simlib.SIM_ROOT, "de_comm": simlib.COMM_REPO,
             "de_server": simlib.SERVER_REPO, "de_auth": simlib.AUTH_REPO,
             "ardupilot": simlib.ARDUPILOT_DIR,
             "webclient": simlib.WEBCLIENT_REPO}
    out = {}
    for name, d in repos.items():
        try:
            out[name] = subprocess.run(
                ["git", "-C", d, "rev-parse", "--short", "HEAD"],
                capture_output=True, text=True, timeout=5
            ).stdout.strip() or None
        except Exception:
            out[name] = None
    return out


def _junit(name, results, wall_s, path):
    def esc(s):
        return str(s).replace("&", "&amp;").replace("<", "&lt;") \
            .replace(">", "&gt;").replace('"', "&quot;")
    cases = []
    n_fail = n_skip = 0
    for r in results:
        body = ""
        if r["status"] == "SKIP":
            n_skip += 1
            body = f'<skipped message="{esc(r["detail"])}"/>'
        elif r["status"] == "FAIL":
            n_fail += 1
            body = f'<failure message="{esc(r["detail"])}"/>'
        cases.append(
            f'  <testcase classname="{esc(name)}" name="{esc(r["name"])}"'
            f' time="0">{body}</testcase>')
    xml = ('<?xml version="1.0" encoding="UTF-8"?>\n'
           f'<testsuite name="{esc(name)}" tests="{len(results)}" '
           f'failures="{n_fail}" skipped="{n_skip}" '
           f'time="{wall_s:.1f}">\n' + "\n".join(cases)
           + "\n</testsuite>\n")
    simlib.write_text(path, xml)


def _collect(scn, m, out, results, verify, extra):
    """Move the run's artifacts into var/results/<utc>/<name>/."""
    os.makedirs(out, exist_ok=True)
    for u in m["units"]:
        for sub in ("journal", "logs"):
            d = os.path.join(simlib.unit_dir(u), sub)
            if os.path.isdir(d):
                shutil.copytree(d, os.path.join(out, u["name"], sub),
                                dirs_exist_ok=True)
    srv_logs = os.path.join(simlib.server_dir(m["name"]), "logs")
    if os.path.isdir(srv_logs):
        shutil.copytree(srv_logs, os.path.join(out, "server"),
                        dirs_exist_ok=True)
    for src, dst in extra:
        if src and os.path.isfile(src):
            shutil.copy(src, os.path.join(out, dst))
    truth = os.path.join(simlib.run_dir(m["name"]), "truth.jsonl")
    if os.path.isfile(truth):
        shutil.copy(truth, os.path.join(out, "truth.jsonl"))
    # P5-13: the render feed's connect/event log
    ws_log = os.path.join(simlib.run_dir(m["name"]), "worldstream.log")
    if os.path.isfile(ws_log):
        shutil.copy(ws_log, os.path.join(out, "worldstream.log"))
    # P5-14: Mavlink3DMap bridge/web logs when the viewer launched
    rdir = os.path.join(simlib.render_dir(m["name"]), "logs")
    if os.path.isdir(rdir):
        shutil.copytree(rdir, os.path.join(out, "render"),
                        dirs_exist_ok=True)
    # P5-08 stub logs (stubs/<unit>.log) - the byte-compare artifacts for
    # the determinism acceptance
    sdir = os.path.join(simlib.run_dir(m["name"]), "stubs")
    if os.path.isdir(sdir):
        shutil.copytree(sdir, os.path.join(out, "stubs"),
                        dirs_exist_ok=True)
    shutil.copy(scn["file"], out)
    if os.path.isfile(simlib.manifest_path(m["name"])):
        shutil.copy(simlib.manifest_path(m["name"]), out)
    n_pass = sum(1 for r in results if r["status"] == "PASS")
    n_skip = sum(1 for r in results if r["status"] == "SKIP")
    return n_pass, n_skip


# ------------------------------------------------------------------ main

def _skipped_result(scn, t_file, out, t0_wall, why):
    """A hardware-gated run whose requirements are absent (SL-0104): the
    machine contract still gets summary.json + the RESULT line last,
    but exit 0 and a skip count - missing hardware is not a failure."""
    simlib.write_json(os.path.join(out, "summary.json"), {
        "scenario": scn["name"], "file": t_file, "seed": scn["seed"],
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                     time.gmtime(t0_wall)),
        "wall_s": round(time.time() - t0_wall, 1),
        "module_overrides": dict(simlib.module_overrides()),
        "step_ok": True, "skipped": why,
        "assertions": [{"name": "requires", "status": "SKIP",
                        "value": None, "detail": why}],
        "counts": {"pass": 0, "fail": 0, "skip": 1}, "exit": 0,
    }, indent=2)
    print(f"SKIP {why}\nRESULT {os.path.join(out, 'summary.json')}",
          flush=True)
    return 0


def _early_result(scn, t_file, out, t0_wall, why):
    """A run that stops before the timeline still honours the machine
    contract (docs/MACHINE_API.md): a summary.json with the reason and
    the RESULT line last."""
    simlib.write_json(os.path.join(out, "summary.json"), {
        "scenario": scn["name"], "file": t_file, "seed": scn["seed"],
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                     time.gmtime(t0_wall)),
        "wall_s": round(time.time() - t0_wall, 1),
        "module_overrides": dict(simlib.module_overrides()),
        "step_ok": False, "error": why, "assertions": [],
        "counts": {"pass": 0, "fail": 0, "skip": 0}, "exit": 1,
    }, indent=2)
    print(f"\nRESULT {os.path.join(out, 'summary.json')}", flush=True)
    return 1


def run_scenario(t_file, seed=None, keep=False, hold=False,
                 record=None, open_web=False):
    """One de.scenario/1 run -> exit code (0 = every non-SKIP assertion
    passed and no step failed). hold parks the run after the timeline
    (the world stream and the render procs stay live) until Ctrl-C.
    record is an output yaml path: the run's webclient actions are
    captured through the comm server's command_plugin seam (record.py)
    and written there as a de.scenario/1 file when the timeline/hold
    ends."""
    t_file = os.path.abspath(t_file)
    scn = scenario.load_scenario(t_file, seed=seed)
    if (scn.get("operation") or "").endswith(".deop"):
        # TASK-23: compile the source on demand BEFORE the first step -
        # scn["operation"] stays the .deop (verify:/send {PKG:} use it)
        # and the build also refreshes the by-name/ links those {PKG:}
        # tokens resolve against
        import opbuild
        scn["operation_spec"] = opbuild.build_deop(
            scn["operation"], scn.get("fleet"))
    t0_wall = time.time()
    out = _results_dir(scn)
    os.makedirs(out, exist_ok=True)
    print(f"== {scn['name']} (seed {scn['seed']}) -> {out}", flush=True)
    if scn.get("chaos_steps"):
        # TASK-10: the seeded schedule the run will execute, printed and
        # saved as a standalone scenario - a failing seed becomes a fixed
        # regression file (`./sim run <out>/expanded.yaml`)
        print(f"== chaos: seed {scn['seed']} drew "
              f"{len(scn['chaos_steps'])} fault step(s)", flush=True)
        for s in scn["chaos_steps"]:
            print("  " + json.dumps(s, sort_keys=True), flush=True)
        simlib.write_text(os.path.join(out, "expanded.yaml"),
                          scenario.expanded_text(scn))
    # SL-0104 hardware gate: a `fcb_link: {kind: host_serial}` unit
    # needs its USB-serial adapter on this host (char device, rw for
    # us); SL-0105's `kind: bt_spp` needs an UP Bluetooth controller -
    # without them the run can prove nothing, so the whole scenario
    # skips cleanly before the fleet even starts.
    missing = []
    bt_ok = None
    for ent in scn["sim"].get("units") or []:
        fl = ent.get("fcb_link") or scn["sim"].get("fcb_link") or {}
        if fl.get("kind") == "host_serial":
            dev = str(fl.get("device") or "")
            try:
                usable = stat.S_ISCHR(os.stat(dev).st_mode) and \
                    os.access(dev, os.R_OK | os.W_OK)
            except OSError:
                usable = False
            if not usable:
                missing.append(
                    f"{ent.get('name', '?')}:{dev or '?'} - attach the "
                    "USB-serial adapter (dialout group for rw)")
        elif fl.get("kind") == "bt_spp":
            if bt_ok is None:
                bt_ok = simlib.bt_adapter_up()
            if not bt_ok:
                missing.append(
                    f"{ent.get('name', '?')}:ch{fl.get('channel')} - "
                    "no UP Bluetooth controller (attach/enable one: "
                    "bluetoothctl power on)")
            else:
                # SL-0105: our bridge binds the channel LAST - a
                # foreign RFCOMM listener already on it (kdeconnectd's
                # SPP server is the known thief) keeps winning the
                # client's dials and fcb_stats stays zero. Warn now so
                # the connect window is not wasted.
                sq = simlib.bt_rfcomm_squatters()
                if sq:
                    print("  WARNING: foreign RFCOMM holder(s) may "
                          "steal the bt_spp channel: " + ", ".join(
                              f"{s['name']}({s['pid']})x{s['sockets']}"
                              for s in sq) +
                          " - kill them or the client dial lands "
                          "elsewhere", flush=True)
    if missing:
        return _skipped_result(
            scn, t_file, out, t0_wall,
            "serial link hardware not usable: " + ", ".join(missing))

    # P5-08: load the world and inject it into the sim spec BEFORE the
    # fleet is ensured - the stub config blocks (world path, seed, unit,
    # pose port) and the geo_projection model land in the generated
    # module configs, so the stubs and the real projectors share one
    # camera model and one seeded truth.
    wld = None
    if scn.get("world"):
        if not os.path.isfile(scn["world"]):
            print(f"  world {scn['world']} missing - "
                  "world-dependent checks will SKIP", flush=True)
        else:
            wld = world_mod.load(scn["world"], scn["seed"])
            world_mod.prepare_sim(scn, wld)

    # pre-deploy verifier (P5-06 CLI; absence is recorded, not fatal)
    if not run_verify(scn, out):
        print("VERIFIER FAILED - scenario stops before deploy", flush=True)
        _collect(scn, {"units": [], "name": scn["name"],
                       "server_procs": []}, out, [], True, [])
        return _early_result(scn, t_file, out, t0_wall, "verifier failed")

    try:
        m = launcher.ensure(scn["sim"], world_stream=(wld is not None))
    except Exception as e:
        print(f"  BRING-UP ERROR: {type(e).__name__}: {e}", flush=True)
        return _early_result(scn, t_file, out, t0_wall,
                             f"bring-up failed: {e}")
    ctl.CTX = runner.SIM_NAME = m["name"]
    rec_t0 = None
    if record:
        # --record: arm the command_plugin capture before the bot and
        # the timeline connect, so a held run's webclient driving lands
        # in the file (arm() bounces de_server once on a real server)
        import record as record_mod
        rec_t0 = record_mod.arm(m)
    ctx = {"manifest": m, "sim": m["name"], "scn": scn,
           "server": m["server"], "bot": None,
           "telemetry": None, "marks": {}, "timers": [], "pumps": [],
           "file": t_file, "results_dir": out,
           "units_by_name": {u["name"]: u for u in m["units"]},
           "run_start_ms": int(t0_wall * 1000),
           "operator_log": os.path.join(out, "operator.log"),
           "samples_file": os.path.join(out, "samples.jsonl"),
           "proc_events": [], "restore_events": [], "log_events": []}
    _CTX[0] = ctx
    # link-capable transports start clean: the fake server's links{} and
    # every unit's link_proxy survive sim reuse
    link_capable = m["server"]["type"] == "fake" or \
        any(u.get("link_proxy") for u in m["units"])
    if link_capable:
        runner.run_ctl("link_clear", "all")
    ctx["marks"] = {u["name"]: simlib.journal_mark(u) for u in m["units"]}
    bot = operator_bot.OperatorBot(m, scn["operator"],
                                   log_path=ctx["operator_log"])
    ctx["bot"] = bot
    ctx["adapter"] = adapter.new_adapter(ctx)
    bot.start()
    tele_path = os.path.join(simlib.run_dir(m["name"]), "telemetry.jsonl")
    tele = telemetry_mod.Telemetry(m["units"], tele_path)
    ctx["telemetry"] = tele
    tele.start()
    ctx["link_events"] = {}
    link_stop = threading.Event()
    link_thread = threading.Thread(target=_link_monitor,
                                   args=(ctx, link_stop), daemon=True)
    link_thread.start()
    sample_stop = threading.Event()
    sample_thread = threading.Thread(target=_sampler,
                                     args=(ctx, sample_stop), daemon=True)
    sample_thread.start()
    ctx["baseline"] = checks.capture_baseline(ctx)
    ctx["world"] = wld
    recorder = None
    if wld is not None:
        # scenario t0: the stubs pause while world_epoch.json is absent,
        # so this is also their start gun. Truth rows land in
        # var/run/<sim>/truth.jsonl at 1 Hz from now on.
        world_mod.clear_run_state(m["name"])
        epoch_ms = world_mod.write_epoch(m["name"], scn["world"],
                                         scn["seed"])
        # SL-0013: the 1 Hz truth tick is the SINGLE authority for
        # react: triggers - it reads the fleet poses (telemetry for FC
        # units, the manifest position for statics), appends knots to
        # world_overrides.jsonl and world_event rows to truth.jsonl;
        # the stubs and the world stream read the same file.
        recorder = world_mod.TruthRecorder(
            wld, epoch_ms, world_mod.truth_path(m["name"]),
            overrides_path=world_mod.overrides_path(m["name"]),
            fleet_poses=lambda: _fleet_poses(ctx))
        recorder.start()
        ctx["world_epoch_ms"] = epoch_ms
        # SL-0016: cls:aircraft entities emit ADSB_VEHICLE frames at
        # 1 Hz through each fcb-proxied FC unit's inject control - the
        # frames land on de_mavlink exactly like SITL's own traffic
        if wld.aircraft():
            adsb_units = [u for u in m["units"]
                          if u["fc"] and u["fc"].get("fcb_proxy")]
            if adsb_units:
                feeder = world_mod.AdsbFeeder(
                    wld, epoch_ms, adsb_units, m["home"][2])
                feeder.start()
                ctx["adsb_feeder"] = feeder
            else:
                print("  world has cls:aircraft entities but no FC "
                      "unit runs fcb_proxy - no ADS-B injection",
                      flush=True)
        # P5-13: a render-enabled scenario also opens the world stream
        # (de.worldstream/1 on render.stream_port) now that the epoch -
        # the truth clock - is fixed
        render = m.get("render") or {}
        if render:
            import world_stream
            stream = world_stream.WorldStream(
                wld, m, epoch_ms, render["stream_port"],
                world_mod.truth_path(m["name"]),
                world_stream.log_path(m["name"]), bot=bot)
            stream.start()
            ctx["world_stream"] = stream

    ok = True
    try:
        ok &= run_timeline(scn, ctx)
        if hold:
            # --hold parks with the world stream, truth recorder and
            # telemetry still live for the viewer; rows stamped after
            # this boundary are hold traffic, not scenario evidence, so
            # the check readers cap at it (unix ms - the truth/
            # detection row clock)
            ctx["hold_ms"] = int(time.time() * 1000)
            tele.until_ms = ctx["hold_ms"]
            _hold(m, world_stream=ctx.get("world_stream") is not None,
                  open_web=open_web)
    except Exception as e:
        # a runner bug must still reach assertions/teardown/stop(): the
        # bot, telemetry and link threads and the fleet are never leaked
        print(f"  TIMELINE ERROR: {type(e).__name__}: {e}", flush=True)
        ok = False
    finally:
        if ctx.get("world_stream") is not None:
            ctx["world_stream"].stop()
        if ctx.get("adsb_feeder") is not None:
            ctx["adsb_feeder"].stop()
        if recorder is not None:
            recorder.stop()
        # a fault's dur_s restore that never fired inside the timeline
        # runs now instead - faults never leak into a reused fleet
        for t in ctx["timers"]:
            if t.is_alive() and not t.fired_once.is_set():
                # an in-flight callback counts as is_alive() too - skip
                # it or the restore would run twice, concurrently
                t.cancel()
                try:
                    t.function(*t.args, **t.kwargs)
                except Exception as e:
                    print(f"  fault restore failed: {e}", flush=True)
        if rec_t0 is not None:
            try:
                record_mod.disarm(m)
                record_mod.finalize(m, rec_t0, record)
            except Exception as e:
                print(f"  record finalize failed: {e}", flush=True)

    # realtime gate: only when the scenario asked for it
    speedups = {}
    for u in m["units"]:
        if u["fc"]:
            speedups[u["name"]] = tele.speedup(u["name"])
    ctx["realtime_required"] = scn["realtime"]
    got = [v for v in speedups.values() if v is not None]
    ctx["realtime_ok"] = bool(got) and all(0.85 <= v <= 1.15 for v in got)
    ctx["realtime_note"] = \
        ", ".join(f"{n}={v}" for n, v in speedups.items()) or "no FC telemetry"
    if scn["realtime"] and not ctx["realtime_ok"]:
        print(f"  realtime required but speed-up != 1 ({ctx['realtime_note']})"
              " - realtime assertions SKIP", flush=True)

    print("\n== assertions ==", flush=True)
    results = []
    for a in scn["assert"]:
        r = checks.check_assertion(a, ctx)
        results.append(r)
        val = "" if r["value"] is None else f" value={r['value']}"
        det = f" ({r['detail']})" if r["detail"] else ""
        print(f"  [{r['status']:4s}] {list(a)[0]}:{val}{det}", flush=True)

    # a failed step is a failed test: surface it as a result so junit/
    # summary agree with the exit code, and an all-SKIP run proves nothing
    if not ok:
        results.append(checks._r("timeline_steps", "FAIL", None,
                                 "a timeline step failed (see the log)"))
    elif scn["assert"] and not any(r["status"] == "PASS" for r in results) \
            and not scn["raw"].get("allow_all_skip"):
        results.append(checks._r(
            "evidence", "FAIL", None,
            "every assertion SKIPped - the run proved nothing "
            "(set allow_all_skip: true to accept)"))
    if link_capable:
        runner.run_ctl("link_clear", "all")
    td = "none" if keep else scn["teardown"]
    if td in ("land", "reset"):
        print(f"\n== teardown: {td} ==  (Ctrl-C skips)", flush=True)
        # rc_override pumps still holding serial0 would starve land's
        # pilot connection - let them release first
        for t in ctx["pumps"]:
            t.join(20)
        try:
            bot.op_abort(why="scenario teardown")
        except Exception:
            pass
        try:
            if ctl.with_fc(m["units"]) and not runner.run_ctl(td, "all"):
                print("TEARDOWN FAILED: not every vehicle is down",
                      flush=True)
                ok = False
        except KeyboardInterrupt:
            # the operator asked to be let go now - leaving vehicles
            # armed/airborne is safe for a sim but the next run reuses
            # this fleet as-is, so say how to finish or scrap it
            print(f"teardown interrupted - the fleet may still be "
                  f"airborne; ./sim ctl -s {m['name']} land all to "
                  f"finish, ./sim down {m['name']} to scrap", flush=True)
    bot.stop()
    tele.stop()
    link_stop.set()
    link_thread.join(5)
    sample_stop.set()
    sample_thread.join(5)

    # P5-12: the coverage artefact pair per computed grid, when any
    # coverage assertion ran (coverage.geojson opens in any GIS)
    if ctx.get("coverage_grids"):
        import coverage as coverage_mod
        multi = len(ctx["coverage_grids"]) > 1
        for g in ctx["coverage_grids"].values():
            tag = f"_{g['area']}" if multi else ""
            simlib.write_json(
                os.path.join(out, f"coverage{tag}.json"),
                dict(coverage_mod.summary(g["grid"]), area=g["area"],
                     cell_m=g["cell_m"], units=g["units"]))
            simlib.write_json(
                os.path.join(out, f"coverage{tag}.geojson"),
                coverage_mod.geojson(g["grid"]))

    n_pass, n_skip = _collect(
        scn, m, out, results, True,
        [(tele_path, "telemetry.jsonl"),
         (tele_path, "tlog.jsonl")])
    wall = time.time() - t0_wall
    n_fail = sum(1 for r in results if r["status"] == "FAIL")
    simlib.write_json(os.path.join(out, "summary.json"), {
        "scenario": scn["name"], "file": t_file, "seed": scn["seed"],
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                     time.gmtime(t0_wall)),
        "wall_s": round(wall, 1),
        "commits": _commits(),
        "module_overrides": dict(simlib.module_overrides()),
        "speedup": speedups,
        "speedup_requested": m.get("speedup", 1),
        "realtime": {"required": scn["realtime"],
                     "ok": ctx["realtime_ok"],
                     "note": ctx["realtime_note"]},
        "step_ok": ok,
        "assertions": results,
        "counts": {"pass": n_pass, "fail": n_fail, "skip": n_skip},
        "exit": 0 if ok and n_fail == 0 else 1,
    }, indent=2)
    _junit(scn["name"], results, wall, os.path.join(out, "junit.xml"))
    print(f"\n{n_pass} PASS, {n_fail} FAIL, {n_skip} SKIP"
          + ("" if ok else " (a step or the teardown failed)")
          + f"\nresults: {out}"
          # the stable machine contract (docs/MACHINE_API.md): the last
          # line is always the run's summary.json path
          + f"\nRESULT {os.path.join(out, 'summary.json')}", flush=True)
    return 0 if ok and n_fail == 0 else 1


def run(t_file, keep=False, hold=False, seed=1, record=None,
        open_web=False):
    """`./sim run` dispatcher: scenarios -> the de.scenario/1 engine (a
    bare sims/*.yaml file has no steps - it is just ensured up, additive
    like ./sim up but hash-reused, it leaves the other live sims
    alone). --record <file> captures the parked window's webclient
    actions into a de.scenario/1 file (record.py). --open-web opens the
    sim's webclient link once it is up."""
    raw = simlib.load_yaml(os.path.abspath(t_file))
    if scenario.is_sim_spec(raw):
        print(simlib.col_yellow(
            f"{t_file} is a sim, not a test - no steps to run; "
            "bringing the fleet up"), flush=True)
        # additive like `sim up`: a bare sim is nobody's test - it must
        # not down the other live sims
        m = launcher.ensure(t_file, exclusive=False)
        if open_web:
            ctl.xdg_open(simlib.webclient_url(m))
        if record:
            import record as record_mod
            record_mod.run_held(m, record)
            print(simlib.col_green("sim is still up")
                  + " - ./sim down to stop it", flush=True)
        elif hold:
            _hold(m)
            print(simlib.col_green("sim is still up")
                  + " - ./sim down to stop it", flush=True)
        return 0
    if open_web and not hold:
        # a scenario run tears the sim down on exit - the link only
        # lives while the run parks (--hold/--view)
        print(simlib.col_yellow(
            "--open-web ignored without --hold - the sim is torn down "
            "when the run ends"), flush=True)
    return run_scenario(t_file, seed=seed, keep=keep, hold=hold,
                        record=record, open_web=open_web)


def run_suite(suite_dir, seeds, keep=False):
    """Every *.yaml in the suite x every seed; serial (a run owns the
    machine). -> suite exit code."""
    files = sorted(f for f in os.listdir(suite_dir)
                   if f.endswith((".yaml", ".yml")))
    if not files:
        sys.exit(f"suite {suite_dir}: no scenario files")
    runs = []
    for f in files:
        for s in seeds:
            p = os.path.join(suite_dir, f)
            print(f"\n######## {f} seed={s} ########", flush=True)
            rc = run_scenario(p, seed=s, keep=keep)
            runs.append({"file": f, "seed": s, "exit": rc})
    ok = all(r["exit"] == 0 for r in runs)
    print("\n==== suite ====")
    for r in runs:
        print(f"  [{'ok' if r['exit'] == 0 else 'FAIL'}] "
              f"{r['file']} seed={r['seed']}")
    return 0 if ok else 1


def main(argv):
    keep = "--keep" in argv
    hold = "--hold" in argv
    argv = [a for a in argv if a not in ("--keep", "--hold")]
    seeds = [1]
    suite = None
    t_file = None
    i = 0
    while i < len(argv):
        if argv[i] == "--suite":
            suite = argv[i + 1]
            i += 2
        elif argv[i] == "--seeds":
            seeds = [int(x) for x in argv[i + 1].split(",")]
            i += 2
        elif argv[i] == "--seed":
            seeds = [int(argv[i + 1])]
            i += 2
        elif argv[i] == "--speedup":
            os.environ["SIM_SPEEDUP"] = argv[i + 1]
            i += 2
        else:
            t_file = argv[i]
            i += 1
    if suite:
        return run_suite(os.path.abspath(suite), seeds, keep)
    if not t_file:
        sys.exit("usage: run_scenario.py <file.yaml> [--seed N] [--keep] "
                 "[--hold] [--speedup N] | --suite <dir> [--seeds 1,2,3]")
    return run_scenario(t_file, seed=seeds[0], keep=keep, hold=hold)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
