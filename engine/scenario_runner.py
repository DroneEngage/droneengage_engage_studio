#!/usr/bin/env python3
"""scenario_runner.py - run a de.scenario/1 rehearsal (TASK-P5-07).

`./sim run <file>` dispatches here: a legacy tests/*.yaml (version 1,
`sim:` + `steps:`) still goes through runner.py unchanged; a scenario
(`version: 1` + `name:` + `fleet:` + `timeline:`) runs the new path:

  verify -> fleet up -> operator bot + telemetry on -> timeline
  (t: scheduled + on: journal-triggered steps, faults included) ->
  assertions -> teardown -> results/<utc>/<name>/ with journals, logs,
  telemetry.jsonl (tlog), operator.log, truth.jsonl, verify.json,
  summary.json, junit.xml.

Timeline steps carry {t, do:<step-name>, ...args} or {t, fault:<kind>,
...args} or {on: {unit, re}, do:..., once:bool}; on: match groups are
filled into the step's strings as {M1}..{M9}. `t` is seconds from the
scenario start; negative t runs at once before the clock starts.

Operator-facing `do:` names (deploy/prepare/start/abort/operator) go
through the operator bot's own GCS session so the fleet sees one
consistent operator party; every other `do:` name maps to the runner.py
step of the same name (arm, land, mode, wait_journal, fire, opmsg, ...).

P5-08 world hook: a scenario `world:` file is loaded and injected into
the fleet spec before `ensure` (stub config blocks, camera/SDR model
settings on the real modules). Once the fleet is up, `world_epoch.json`
is written (the stub start gun) and a TruthRecorder writes
`run/<sim>/truth.jsonl` at 1 Hz; stub detection rows land in the same
file. The recorder and epoch are always torn down with the run.
"""

import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simlib
import ctl
import launcher
import runner
import scenario
import checks
import telemetry as telemetry_mod
import operator_bot
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
    wrap = os.path.join(simlib.SIM_ROOT, "scenarios", "support",
                        "verify_wrap.js")
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
        r = subprocess.run(["node", cli, scn["operation"], "--json"],
                           capture_output=True, text=True, timeout=120)
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
    # findings {sev, rule, ...}; wrapper CLIs (scenarios/support/) print
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
    """A do: step - operator-facing names via the bot, everything else
    delegates to the runner.py step of the same name."""
    name = step["do"]
    a = _step_args(step)
    # an on-triggered step defaults its unit to the trigger's unit
    if "on" in step and "unit" not in a:
        a["unit"] = step["on"]["unit"]
    bot = ctx["bot"]
    if name in ("deploy", "prepare"):
        op = a.get("operation") or scn.get("operation")
        if not op:
            print("  deploy: no operation:", flush=True)
            return False
        op = op if os.path.isabs(op) else os.path.join(
            os.path.dirname(scn["file"]), op)
        if ctl._is_deploy_spec(op):
            bot.log("DEPLOY", f"spec {os.path.basename(op)}")
            return bot.op_deploy(op, float(a.get("timeout", 60)))
        return runner.run_ctl("deploy", op)
    if name == "start":
        # the scenario's own t0: time_to_detect measures from the first
        # start step, not from the earlier fleet-up world epoch (the
        # arm/EKF wait in between is not detection latency)
        ctx.setdefault("start_ms", int(time.time() * 1000))
        st = {}
        try:
            st = json.load(open(simlib.op_state_path(ctx["sim"])))
        except (OSError, ValueError):
            pass
        if st.get("run"):
            ov = {k: a[k] for k in
                  ("lead_ms", "in_ms", "t_start", "run") if k in a}
            if isinstance(a.get("start"), dict):
                ov["start"] = a["start"]      # {role: mission} map (1094)
            if isinstance(a.get("targets"), list):
                ov["targets"] = [
                    (ctx["units_by_name"].get(str(n)) or {})
                    .get("party_id", str(n)) for n in a["targets"]]
            if isinstance(a.get("expect_fail"), list):
                ov["expect_fail"] = [
                    (ctx["units_by_name"].get(str(n)) or {})
                    .get("party_id", str(n)) for n in a["expect_fail"]]
            return bot.op_start(
                ov, float(a["timeout"]) if "timeout" in a else None)
        return runner.run_ctl("start")
    if name == "abort":
        return bot.op_abort(a.get("run"), a.get("why", ""))
    if name == "operator":
        return _operator_act(a, bot)
    # every other name: the runner.py step, unchanged semantics
    t = {"__file": scn["file"], "operation": scn.get("operation")}
    return runner.run_step({name: a}, t, ctx["marks"])


def _operator_act(a, bot):
    """do: operator - act: approve|deny|confirm|dismiss|fire|<1086 a>."""
    act = a.get("act")
    unit = a.get("unit")
    tg = "_AGN_" if not unit or str(unit) == "all" else \
        ctl.resolve(str(unit), ctx_manifest())[0]["party_id"]
    if act in ("approve", "deny"):
        if not unit:
            print("  operator approve/deny needs unit:", flush=True)
            return False
        return bot.decide(tg, req=a.get("req"), kind=a.get("kind"),
                          decision=act, wait_s=float(a.get("wait_s", 10)))
    if act in ("confirm", "dismiss"):
        state = "confirmed" if act == "confirm" else "dismissed"
        return bot.confirm(a["tid"], tg=tg, state=state)
    if act == "fire":
        return bot.fire(a["event"], a.get("payload"), tg)
    if act in ("activate", "pause", "resume", "stop", "clear"):
        ms = {"a": act}
        if a.get("mission"):
            ms["mission"] = a["mission"]
        if a.get("hold") is not None:
            ms["hold"] = bool(a["hold"])
        return bot.action(ms, tg)
    if act == "deploy":
        return _do({"do": "deploy", **{k: v for k, v in a.items()
                                       if k != "act"}}, _SCN[0], _CTX[0])
    if act in ("start", "abort"):
        return _do({"do": act, **{k: v for k, v in a.items()
                                  if k != "act"}}, _SCN[0], _CTX[0])
    print(f"  operator: unknown act {act!r}", flush=True)
    return False


_CTX = [None]          # let _operator_act reach the unit resolver
_SCN = [None]


def ctx_manifest():
    return _CTX[0]["manifest"]


# ---------------------------------------------------------------- faults

def _schedule(ctx, after_s, fn, *args):
    t = threading.Timer(after_s, fn, args=args)
    t.daemon = True
    t.start()
    ctx["timers"].append(t)


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
    a = _step_args(step)
    if "on" in step and "unit" not in a:
        a["unit"] = step["on"]["unit"]
    unit = str(a.get("unit", "all"))
    dur = a.get("dur_s", a.get("seconds"))
    bot = ctx["bot"]
    if kind == "link_cut":
        return runner.run_ctl("link_cut", unit, dur or 30)
    if kind == "loss":
        return runner.run_ctl("loss", unit, a.get("percent", 50))
    if kind == "delay":
        return runner.run_ctl("delay", unit, a.get("ms", 500))
    if kind == "gcs_disconnect":
        ok = True
        if ctx["server"]["type"] != "fake":
            # the ctl/deploy GCS is the bridge on a real stack - drop it
            ok = runner.run_ctl("kill", "server", "gcs_bridge")
            if dur:
                _schedule(ctx, float(dur), runner.run_ctl, "server_up")
        if dur:
            bot.disconnect_for(float(dur))
        else:
            bot.disconnect_for(30)
        return ok
    if kind == "server_down":
        ok = runner.run_ctl("kill", "server", a.get("proc", "all"))
        if dur:
            _schedule(ctx, float(dur), runner.run_ctl, "server_up")
        return ok
    if kind == "kill":
        return runner.run_ctl("kill", unit, a.get("proc", "all"))
    if kind == "restart":
        return runner.run_ctl("restart", unit, a.get("proc", "all"))
    if kind == "module_kill":
        return runner.run_ctl("kill", unit, a["module"])
    if kind == "battery":
        ok = runner.run_ctl("param", unit, "SIM_BATT_VOLTAGE",
                            a.get("volts", 10.5))
        # the supervisor's battery_low watches remaining PERCENT, which
        # the FC derives from consumed mAh vs BATT_CAPACITY - a voltage
        # sag alone never moves it. cap_mah shrinks the pack so remaining
        # collapses to 0% at once (must stay >10 mAh: below that the FC
        # reports the reading invalid, which would reset the handler).
        if a.get("cap_mah") is not None:
            ok &= runner.run_ctl("param", unit, "BATT_CAPACITY",
                                 a["cap_mah"])
        if dur:
            _schedule(ctx, float(dur), runner.run_ctl,
                      "param", unit, "SIM_BATT_VOLTAGE", a.get("ok_volts", 12.6))
            if a.get("cap_mah") is not None:
                _schedule(ctx, float(dur), runner.run_ctl,
                          "param", unit, "BATT_CAPACITY",
                          a.get("ok_cap_mah", 3300))
        return ok
    if kind == "gps_off":
        # SIM_GPS1_FIXTYPE forces the fix the sim GPS reports (6=3D_DGPS,
        # 0=NO_GPS) - deterministic on every SITL version; the old
        # SIM_GPS_DISABLE knob no longer exists.
        ok = runner.run_ctl("param", unit, "SIM_GPS1_FIXTYPE", 0)
        if dur:
            _schedule(ctx, float(dur), runner.run_ctl,
                      "param", unit, "SIM_GPS1_FIXTYPE", 6)
        return ok
    if kind == "server_clock":
        if ctx["server"]["type"] != "fake":
            print("  fault server_clock needs transport: fake", flush=True)
            return False
        return runner.run_ctl("clock", "offset",
                              a.get("offset_ms", -3600000))
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
    if kind == "mode":
        return runner.run_ctl("mode", unit, a["mode"])
    print(f"  unknown fault {kind!r}", flush=True)
    return False


# --------------------------------------------------------- link monitor

def _link_monitor(ctx, stop):
    """Poll the server's online list; journal every unit's link
    transitions to links.jsonl (link_offline reads ctx["link_events"]).
    Works on fake (control `list`) and real (bridge `list`) alike."""
    path = os.path.join(ctx["results_dir"], "links.jsonl")
    events = ctx["link_events"]
    was = None
    while not stop.is_set():
        try:
            online = ctl.online_parties(ctx["sim"])
        except BaseException:
            # server/control port unreachable (server_down fault, restart
            # in flight): nobody is online at a server that is not there
            online = set()
        if was is None and not online:
            stop.wait(1.5)
            continue
        now = int(time.time() * 1000)
        if was is not None:
            for p in was - online:
                events.setdefault(p, []).append((now, False))
                try:
                    with open(path, "a") as f:
                        f.write(json.dumps(
                            {"t_ms": now, "party": p,
                             "online": False}) + "\n")
                except OSError:
                    pass
            for p in online - was:
                events.setdefault(p, []).append((now, True))
                try:
                    with open(path, "a") as f:
                        f.write(json.dumps(
                            {"t_ms": now, "party": p,
                             "online": True}) + "\n")
                except OSError:
                    pass
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


def run_timeline(scn, ctx):
    """Scheduled t: steps + armed on: watchers until last_t + settle_s."""
    steps = scn["timeline"]
    t_steps = sorted((s for s in steps if "t" in s), key=lambda s: s["t"])
    watchers = [{"unit": str(s["on"]["unit"]),
                 "rx": re.compile(s["on"]["re"]),
                 "step": s, "offsets": None, "fired": 0,
                 "once": bool(s.get("once", True)), "armed": True}
                for s in steps if "on" in s]
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
                ok &= _watcher_fires(w, ctx)
        time.sleep(0.25)
    return ok


def _hold(m, world_stream=False):
    """--hold: park the run after the timeline with the world stream,
    truth recorder, operator bot and render procs still live - the
    interactive Mavlink3DMap session. Ctrl-C releases into the normal
    assertions/teardown. `world_stream` says a de.worldstream/1 feed is
    live on render.stream_port - only then does the printed URL carry
    ?world= (a bare sim has no stream)."""
    render = m.get("render") or {}
    print(simlib.col_bold("\n== holding - Ctrl-C to finish the run =="),
          flush=True)
    if render:
        print("render viewer: " + simlib.col("1;36",
              simlib.render_viewer_url(render, world_stream)),
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
    """Move the run's artifacts into results/<utc>/<name>/."""
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

def run_scenario(t_file, seed=None, keep=False, hold=False):
    """One de.scenario/1 run -> exit code (0 = every non-SKIP assertion
    passed and no step failed). hold parks the run after the timeline
    (the world stream and the render procs stay live) until Ctrl-C."""
    t_file = os.path.abspath(t_file)
    scn = scenario.load_scenario(t_file, seed=seed)
    if scn["legacy"]:
        return runner.run(t_file, keep=keep)
    t0_wall = time.time()
    out = _results_dir(scn)
    os.makedirs(out, exist_ok=True)
    print(f"== {scn['name']} (seed {scn['seed']}) -> {out}", flush=True)
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
        return 1

    m = launcher.ensure(scn["sim"], world_stream=(wld is not None))
    ctl.CTX = runner.SIM_NAME = m["name"]
    ctx = {"manifest": m, "sim": m["name"], "scn": scn,
           "server": m["server"], "bot": None,
           "telemetry": None, "marks": {}, "timers": [], "pumps": [],
           "file": t_file, "results_dir": out,
           "units_by_name": {u["name"]: u for u in m["units"]},
           "operator_log": os.path.join(out, "operator.log")}
    _CTX[0] = ctx
    _SCN[0] = scn
    fake = m["server"]["type"] == "fake"
    if fake:
        runner.run_ctl("link_clear", "all")
    ctx["marks"] = {u["name"]: simlib.journal_mark(u) for u in m["units"]}
    bot = operator_bot.OperatorBot(m, scn["operator"],
                                   log_path=ctx["operator_log"])
    ctx["bot"] = bot
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
    ctx["baseline"] = checks.capture_baseline(ctx)
    ctx["world"] = wld
    recorder = None
    if wld is not None:
        # scenario t0: the stubs pause while world_epoch.json is absent,
        # so this is also their start gun. Truth rows land in
        # run/<sim>/truth.jsonl at 1 Hz from now on.
        world_mod.clear_run_state(m["name"])
        epoch_ms = world_mod.write_epoch(m["name"], scn["world"],
                                         scn["seed"])
        recorder = world_mod.TruthRecorder(
            wld, epoch_ms, world_mod.truth_path(m["name"]))
        recorder.start()
        ctx["world_epoch_ms"] = epoch_ms
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
            _hold(m, world_stream=ctx.get("world_stream") is not None)
    except Exception as e:
        # a runner bug must still reach assertions/teardown/stop(): the
        # bot, telemetry and link threads and the fleet are never leaked
        print(f"  TIMELINE ERROR: {type(e).__name__}: {e}", flush=True)
        ok = False
    finally:
        if ctx.get("world_stream") is not None:
            ctx["world_stream"].stop()
        if recorder is not None:
            recorder.stop()
        # a fault's dur_s restore that never fired inside the timeline
        # runs now instead - faults never leak into a reused fleet
        for t in ctx["timers"]:
            if t.is_alive():
                t.cancel()
                try:
                    t.function(*t.args, **t.kwargs)
                except Exception as e:
                    print(f"  fault restore failed: {e}", flush=True)

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
    if fake:
        runner.run_ctl("link_clear", "all")
    td = "none" if keep else scn["teardown"]
    if td in ("land", "reset"):
        print(f"\n== teardown: {td} ==", flush=True)
        # rc_override pumps still holding serial0 would starve land's
        # pilot connection - let them release first
        for t in ctx["pumps"]:
            t.join(20)
        try:
            bot.op_abort(why="scenario teardown")
        except Exception:
            pass
        if ctl.with_fc(m["units"]) and not runner.run_ctl(td, "all"):
            print("TEARDOWN FAILED: not every vehicle is down", flush=True)
            ok = False
    bot.stop()
    tele.stop()
    link_stop.set()
    link_thread.join(5)

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
        "speedup": speedups,
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
          + f"\nresults: {out}", flush=True)
    return 0 if ok and n_fail == 0 else 1


def run(t_file, keep=False, hold=False):
    """`./sim run` dispatcher: legacy tests -> runner.py, scenarios ->
    the de.scenario/1 engine. hold is scenario-only (a legacy test has
    no world stream to keep alive). A bare sims/*.yaml file has no
    steps - it is just ensured up (additive: like ./sim up but
    hash-reused, it leaves the other live sims alone)."""
    raw = simlib.load_yaml(os.path.abspath(t_file))
    if scenario.is_sim_spec(raw):
        print(simlib.col_yellow(
            f"{t_file} is a sim, not a test - no steps to run; "
            "bringing the fleet up"), flush=True)
        # additive like `sim up`: a bare sim is nobody's test - it must
        # not down the other live sims
        m = launcher.ensure(t_file, exclusive=False)
        if hold:
            _hold(m)
            print(simlib.col_green("sim is still up")
                  + " - ./sim down to stop it", flush=True)
        return 0
    if scenario.is_legacy_spec(raw):
        return runner.run(t_file, keep=keep)
    return run_scenario(t_file, keep=keep, hold=hold)


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
        else:
            t_file = argv[i]
            i += 1
    if suite:
        return run_suite(os.path.abspath(suite), seeds, keep)
    if not t_file:
        sys.exit("usage: run_scenario.py <file.yaml> [--seed N] [--keep] "
                 "[--hold] | --suite <dir> [--seeds 1,2,3]")
    return run_scenario(t_file, seed=seeds[0], keep=keep, hold=hold)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
