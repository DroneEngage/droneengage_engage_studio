#!/usr/bin/env python3
"""ci/check_library.py - pure (no-SITL) validation of the scenario
library, run on every push.

  - engine/*.py + run_scenario.py + ci/*.py compile clean
  - every sims/tests/scenarios yaml parses and - for de.scenario/1
    files - loads through engine/scenario.py (schema + fleet expansion)
  - every json parses - the shared demo/example ops in packages/
    plus the test-only fixtures under tests/**/*.json and the
    scenario jsons under scenarios/**/*.json (the SL-0059 split)
  - every scenario carries the one-paragraph "what it proves" header

Exit 1 on the first class of problems with every offender printed.
"""

import glob
import json
import os
import py_compile
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.join(ROOT, "engine"))
os.environ.setdefault("SIM_ROOT", ROOT)

import simlib          # noqa: E402
import scenario        # noqa: E402

bad = []


def fail(kind, path, why):
    bad.append(f"{kind} {os.path.relpath(path, ROOT)}: {why}")


# ---- python compiles -------------------------------------------------
for f in sorted(glob.glob(os.path.join(ROOT, "engine", "*.py")) +
                glob.glob(os.path.join(ROOT, "ci", "*.py")) +
                [os.path.join(ROOT, "run_scenario.py")]):
    try:
        py_compile.compile(f, doraise=True)
    except py_compile.PyCompileError as e:
        fail("py", f, e.msg.splitlines()[0] if e.msg else str(e))

# ---- yaml -----------------------------------------------------------
yamls = sorted(glob.glob(os.path.join(ROOT, "sims", "*.yaml")) +
               glob.glob(os.path.join(ROOT, "tests", "**", "*.yaml"),
                         recursive=True) +
               glob.glob(os.path.join(ROOT, "scenarios", "**", "*.yaml"),
                         recursive=True))
for f in yamls:
    try:
        raw = simlib.load_yaml(f)
    except Exception as e:
        fail("yaml", f, str(e))
        continue
    if not isinstance(raw, dict):
        fail("yaml", f, "top level is not a mapping")
        continue
    # every scenario file opens with a one-paragraph header comment
    head = open(f, errors="replace").read(400)
    if not head.lstrip().startswith("#"):
        fail("hdr", f, "missing header comment")
    if scenario.is_legacy_spec(raw):
        fail("schema", f, "`sim:`+`steps:` is the removed legacy format "
             "(SL-0020) - rewrite as de.scenario/1")
        continue
    if f.startswith(os.path.join(ROOT, "sims") + os.sep):
        continue                    # sims/*.yaml are fleet specs, not
                                    # de.scenario/1 - resolved by ./sim show
    try:
        scenario.load_scenario(f, seed=1, probe_ports=False)
    except SystemExit as e:
        fail("schema", f, f"load_scenario exited {e.code}")
    except Exception as e:
        fail("schema", f, str(e))

# ---- package / deploy-spec / op json ---------------------------------
for f in sorted(glob.glob(os.path.join(ROOT, "packages", "*.json")) +
                glob.glob(os.path.join(ROOT, "tests", "**", "*.json"),
                          recursive=True) +
                glob.glob(os.path.join(ROOT, "scenarios", "**", "*.json"),
                          recursive=True)):
    try:
        json.load(open(f))
    except Exception as e:
        fail("json", f, str(e))

# ---- no committed compile_op.js output with an in-repo .deop (SL-0026)
# the runner compiles a referenced .deop on demand into
# var/build/packages/ - a committed twin only drifts from its source
deop_ops = set()
for f in glob.glob(os.path.join(ROOT, "tests", "**", "*.deop"),
                   recursive=True) + \
         glob.glob(os.path.join(ROOT, "scenarios", "**", "*.deop"),
                   recursive=True):
    deop_ops.add(os.path.basename(f)[:-len(".deop")])
    try:
        d = json.loads(simlib.strip_comments(open(f).read()))
        if isinstance(d.get("op"), str):
            deop_ops.add(d["op"])
    except Exception:
        pass
for f in sorted(glob.glob(os.path.join(ROOT, "packages", "*.json"))):
    try:
        d = json.load(open(f))
    except Exception:
        continue
    if isinstance(d, dict) and d.get("op") in deop_ops:
        fail("pkg", f,
             f"compiled output of in-repo .deop {d['op']!r} - delete it "
             "(the runner compiles it on demand)")

for line in bad:
    print("FAIL", line)
print(f"{len(yamls)} yaml, "
      f"{len(glob.glob(os.path.join(ROOT, 'packages', '*.json')))} packages - "
      f"{len(bad)} problem(s)")
sys.exit(1 if bad else 0)
