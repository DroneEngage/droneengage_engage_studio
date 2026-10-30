#!/usr/bin/env python3
"""ci/run_suite.py - nightly/CI suite driver for the de.scenario/1
library (TASK-P5-09).

    python3 ci/run_suite.py [--roots tests] [--seeds 1,2,3] [--keep]
                            [--out var/results/nightly]

Every *.yaml under each root (top-level and every nested subdir -
tests/precland/standalone is two levels deep) runs once per seed,
serially - a run owns the machine. Verdicts:

  PASS       run_scenario exit 0
  GATE-PASS  exit !=0 AND the file's verify: declares expected findings
             AND verify.json's recorded rules match them - the deploy
             gate blocked by design (tests/verify/*)
  FAIL       anything else
  FLAKY      first attempt FAILed, the single permitted rerun produced
             PASS or GATE-PASS - reported with BOTH attempts, never
             silently green

Outputs <out>/summary.md (owner table) and <out>/junit.xml (CI). Exit 1
when any verdict is FAIL; FLAKY keeps the suite amber, not green - it
also exits 1 so the workflow surfaces it.
"""

import argparse
import glob
import json
import os
import re
import subprocess
import sys
import time
import xml.sax.saxutils as sax

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUNNER = os.path.join(ROOT, "run_scenario.py")

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.join(ROOT, "engine"))
import simlib          # noqa: E402
import scenario        # noqa: E402

RESULTS = simlib.RESULTS_DIR


# dir names excluded from a plain suite run - opt-in only via an
# explicit --roots <path>/<name>: soak = long wall-clock runs,
# experimental = not release-gating (SL-0050)
_OPT_IN_DIRS = ("soak", "experimental")


def yaml_files(roots, tier=None, task=None):
    """Every *.yaml under each root - except opt-in dirs: a */soak/ dir
    holds wall-clock runs (tests/soak is 30 min at speed-up 1) and a
    */experimental/ dir holds tests excluded from release gating; both
    run only via an explicit --roots <path>/<name>. Fleet specs
    (units-only files a scenario names via `sim: {base:}` - e.g.
    tests/**/phone_stack.yaml) are dropped too: they are not run files.
    `tier` keeps only files whose `tier:` key matches (absent =
    regression); `task` keeps only files whose `tasks:` list carries
    the TeamEngage id (SL-0137)."""
    out = []
    for r in roots:
        d = os.path.join(ROOT, r)
        files = sorted(glob.glob(os.path.join(d, "**", "*.yaml"),
                                 recursive=True) +
                       glob.glob(os.path.join(d, "**", "*.yml"),
                                 recursive=True))
        if os.path.basename(os.path.normpath(d)) not in _OPT_IN_DIRS:
            files = [f for f in files
                     if not set(_OPT_IN_DIRS)
                     & set(os.path.relpath(f, d).split(os.sep))]
        out += files
    out = [f for f in out
           if not scenario.is_sim_spec(simlib.load_yaml(f) or {})]
    if tier:
        out = [f for f in out
               if (simlib.load_yaml(f) or {}).get("tier", "regression")
               == tier]
    if task:
        def _tasks(f):
            t = (simlib.load_yaml(f) or {}).get("tasks")
            return t if isinstance(t, list) else []
        out = [f for f in out if task in _tasks(f)]
    return out


def expected_verify(path):
    """The file's declared verifier expectation, or None."""
    try:
        raw = simlib.load_yaml(path) or {}
    except Exception:
        return None
    spec = raw.get("verify")
    if not isinstance(spec, dict):
        return None
    if "expect_rules" not in spec and "expect_errors" not in spec:
        return None
    return spec


def latest_results_dir(name, since):
    """Newest var/results/<utc>/<name>/ dir written by THIS run (mtime >=
    `since`) - runs are serial, but a run that died before creating its
    dir must not be judged on an older run's verify.json (same basename
    in another folder, or a previous seed)."""
    hits = sorted((d for d in glob.glob(os.path.join(RESULTS, "*", name))
                   if os.path.getmtime(d) >= since),
                  key=os.path.getmtime)
    return hits[-1] if hits else None


def gate_check(path, results_dir):
    """A blocked-by-design run proves itself through verify.json."""
    spec = expected_verify(path)
    if spec is None or not results_dir:
        return False, "no verify expectation or results"
    vj = os.path.join(results_dir, "verify.json")
    if not os.path.isfile(vj):
        return False, "no verify.json"
    rep = json.load(open(vj))
    rules = rep.get("rules") or []
    errs = rep.get("errors")
    if "expect_rules" in spec and \
            sorted(spec["expect_rules"]) != sorted(rules):
        return False, f"rules {rules} != expect {spec['expect_rules']}"
    if "expect_errors" in spec and errs != spec["expect_errors"]:
        return False, f"errors {errs} != expect {spec['expect_errors']}"
    return True, f"gate blocked: {errs} error(s) rules {rules}"


def run_once(yaml_path, seed, keep):
    name = os.path.splitext(os.path.basename(yaml_path))[0]
    cmd = [sys.executable, RUNNER, yaml_path, "--seed", str(seed)]
    if keep:
        cmd.append("--keep")
    t0 = time.time()
    r = subprocess.run(cmd, cwd=ROOT)
    wall = round(time.time() - t0, 1)
    rd = latest_results_dir(name, t0 - 1.0)
    if r.returncode == 0:
        return "PASS", wall, rd, ""
    ok, why = gate_check(yaml_path, rd)
    if ok:
        return "GATE-PASS", wall, rd, why
    return "FAIL", wall, rd, why or f"exit {r.returncode}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--roots", nargs="+", default=["tests"])
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--out", default=os.path.join(RESULTS, "nightly"))
    ap.add_argument("--keep", action="store_true")
    ap.add_argument("--tier", default=None,
                    choices=["smoke", "regression"],
                    help="only files marked `tier: <name>` (absent = "
                         "regression; smoke = the fast first-pass set)")
    ap.add_argument("--task", default=None,
                    help="only files whose `tasks:` list carries this "
                         "TeamEngage id (e.g. SL-0099)")
    ap.add_argument("--speedup", type=float, default=None,
                    help="SITL --speedup for every unpinned scenario "
                         "(default 3; realtime:/speedup:-pinned files "
                         "keep theirs - rerun at 1 for evidence)")
    a = ap.parse_args()
    if a.speedup:
        os.environ["SIM_SPEEDUP"] = f"{a.speedup:g}"
    seeds = [int(s) for s in a.seeds.split(",") if s.strip()]
    files = yaml_files(a.roots, tier=a.tier, task=a.task)
    if not files:
        sys.exit(f"no scenario yaml under {a.roots}")
    os.makedirs(a.out, exist_ok=True)

    rows = []
    for f in files:
        rel = os.path.relpath(f, ROOT)
        for s in seeds:
            v, wall, rd, why = run_once(f, s, a.keep)
            rec = {"file": rel, "seed": s, "attempt1": v, "attempt2": "",
                   "verdict": v, "wall_s": wall, "why": why,
                   "results": rd}
            if v == "FAIL":
                v2, wall2, rd2, why2 = run_once(f, s, a.keep)
                rec["attempt2"] = v2
                if v2 in ("PASS", "GATE-PASS"):
                    rec["verdict"] = "FLAKY"
                    rec["why"] = f"attempt1 FAIL ({why}); attempt2 {v2}"
                    rec["results"] = rd2
                else:
                    rec["why"] = f"{why}; rerun {v2} ({why2})"
            rows.append(rec)
            print(f"[{rec['verdict']:>9}] {rel} seed={s} "
                  f"({rec['attempt1']}"
                  + (f" -> {rec['attempt2']}" if rec["attempt2"] else "")
                  + f") {rec['wall_s']}s", flush=True)

    # ---- summary.md ----
    n_pass = sum(1 for r in rows if r["verdict"] == "PASS")
    n_gate = sum(1 for r in rows if r["verdict"] == "GATE-PASS")
    n_fail = sum(1 for r in rows if r["verdict"] == "FAIL")
    n_flaky = sum(1 for r in rows if r["verdict"] == "FLAKY")
    lines = [
        "# scenario suite summary",
        "",
        f"roots: {', '.join(a.roots)}   seeds: {a.seeds}   "
        f"runs: {len(rows)}",
        f"**PASS {n_pass}  GATE-PASS {n_gate}  FLAKY {n_flaky}  "
        f"FAIL {n_fail}**",
        "",
        "| verdict | scenario | seed | attempt1 | attempt2 | wall_s |",
        "|---|---|---|---|---|---|",
    ]
    lines += [f"| {r['verdict']} | {r['file']} | {r['seed']} | "
              f"{r['attempt1']} | {r['attempt2'] or '-'} | {r['wall_s']} |"
              for r in rows]
    flaky = [r for r in rows if r["verdict"] == "FLAKY"]
    if flaky:
        lines += ["", "## flaky evidence (fail then pass - kept amber)"]
        lines += [f"- `{r['file']}` seed {r['seed']}: {r['why']}"
                  for r in flaky]
    fails = [r for r in rows if r["verdict"] == "FAIL"]
    if fails:
        lines += ["", "## failures"]
        lines += [f"- `{r['file']}` seed {r['seed']}: {r['why']} "
                  f"(results: {r['results']})" for r in fails]
    open(os.path.join(a.out, "summary.md"), "w").write("\n".join(lines))

    # ---- junit.xml: FLAKY is a failure node so CI can never green it -
    # the message carries both attempts; GATE-PASS is a clean pass. ----
    cases = []
    for r in rows:
        tc = f'  <testcase classname="scenario" name="{sax.escape(r["file"])}#seed{r["seed"]}" time="{r["wall_s"]}">'
        if r["verdict"] == "FAIL":
            tc += f'<failure message="{sax.escape(r["why"])}"/>'
        elif r["verdict"] == "FLAKY":
            tc += ('<failure type="flaky" message=' +
                   f'"flaky: {sax.escape(r["why"])}"/>')
        tc += "</testcase>"
        cases.append(tc)
    suite = (f'<testsuite name="scenario-suite" tests="{len(rows)}" '
             f'failures="{n_fail + n_flaky}" time='
             f'"{round(sum(r["wall_s"] for r in rows), 1)}">\n'
             + "\n".join(cases) + "\n</testsuite>\n")
    open(os.path.join(a.out, "junit.xml"), "w").write(
        '<?xml version="1.0" encoding="utf-8"?>\n' + suite)
    open(os.path.join(a.out, "runs.json"), "w").write(
        json.dumps(rows, indent=1))

    print(f"\n==== {n_pass} PASS {n_gate} GATE-PASS {n_flaky} FLAKY "
          f"{n_fail} FAIL -> {a.out}")
    return 1 if (n_fail or n_flaky) else 0


if __name__ == "__main__":
    sys.exit(main())
