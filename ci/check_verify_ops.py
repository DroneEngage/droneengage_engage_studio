#!/usr/bin/env python3
"""ci/check_verify_ops.py - the push-time verifier gate (TASK-P5-09).

Pure check - node + verify_wrap.js only, no fleet, no servers, no sim:

  - every tests/**/*.yaml (and scenarios/**/*.yaml) with a `verify:`
    block declaring expect_rules / proceed_rules names an op under
    `operation:` - that op MUST be blocked by the verifier and the
    findings MUST carry exactly the declared rules
  - every other *.op.json / *.deop found under tests/ (and scenarios/)
    MUST verify clean (zero error-severity findings)

Mirrors what the webclient's scripts/verify-all.js does for its own
fixture dir. Exit 1 on any violation.
"""

import glob
import json
import os
import subprocess
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
import simlib          # noqa: E402

WRAP = os.path.join(ROOT, "tools", "verify_wrap.js")

bad = []
checked = 0


def verify_op(op_path):
    """Returns (rules, error_count) from verify_wrap stdout."""
    r = subprocess.run(["node", WRAP, op_path], cwd=ROOT,
                       capture_output=True, text=True)
    out = r.stdout.strip().splitlines()
    rep = {}
    for line in reversed(out):
        try:
            rep = json.loads(line)
            break
        except json.JSONDecodeError:
            continue
    rules = rep.get("rules")
    if rules is None:
        rules = sorted({f.get("rule") for f in rep.get("findings", [])
                        if f.get("sev") == "error"} - {None})
    # verify_wrap reports "errors" as the finding LIST (empty on a clean
    # op) and "error_count" as the number - a clean op must not compare
    # [] == 0
    errs = rep.get("error_count")
    if errs is None:
        e = rep.get("errors", 0)
        errs = len(e) if isinstance(e, list) else e
    return rules, errs


# expected-block ops: verify yaml -> operation -> declared rules
expected = {}
for y in glob.glob(os.path.join(ROOT, "tests", "**", "*.yaml"),
                   recursive=True) + \
         glob.glob(os.path.join(ROOT, "scenarios", "**", "*.yaml"),
                   recursive=True):
    raw = simlib.load_yaml(y) or {}
    spec = raw.get("verify")
    if not isinstance(spec, dict):
        continue
    want = spec.get("expect_rules")
    if want is None:
        want = spec.get("proceed_rules")
    if want is None:
        continue
    op = raw.get("operation")
    if not op:
        bad.append(f"{y}: verify rule expectation without operation:")
        continue
    expected[os.path.normpath(os.path.join(os.path.dirname(y), op))] = want

ops = sorted(set(
    glob.glob(os.path.join(ROOT, "tests", "**", "*.op.json"),
              recursive=True) +
    glob.glob(os.path.join(ROOT, "tests", "**", "*.deop"),
              recursive=True) +
    glob.glob(os.path.join(ROOT, "scenarios", "**", "*.op.json"),
              recursive=True) +
    glob.glob(os.path.join(ROOT, "scenarios", "**", "*.deop"),
              recursive=True)))
for op in ops:
    checked += 1
    rel = os.path.relpath(op, ROOT)
    try:
        rules, errs = verify_op(op)
    except Exception as e:
        bad.append(f"{rel}: verifier threw {e}")
        continue
    want = expected.get(os.path.normpath(op))
    if want is not None:
        if sorted(rules) == sorted(want):
            print(f"ok   {rel}: blocked by {','.join(sorted(rules))}")
        else:
            bad.append(f"{rel}: expected rules {want}, got {rules}")
    else:
        if errs == 0:
            print(f"ok   {rel}: clean")
        else:
            bad.append(f"{rel}: {errs} error(s) {rules} - only verify/ "
                       "ops may fail, and only with declared rule ids")

# a declared expectation whose op never materialised is a violation too
seen = {os.path.normpath(o) for o in ops}
for op in expected:
    if op not in seen:
        bad.append(f"missing op file {os.path.relpath(op, ROOT)}")

for line in bad:
    print("FAIL", line)
print(f"{checked} op(s) checked, {len(bad)} violation(s)")
sys.exit(1 if bad or checked == 0 else 0)
