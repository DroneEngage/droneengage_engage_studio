#!/usr/bin/env python3
"""ci/check_verbs.py - the verb-registry coverage gate (SL-0024).

Pure check - no fleet, no servers, no sim:

  - every `do:` step, `fault:` kind and `assert:` name used anywhere
    under tests/ and scenarios/ MUST exist in engine/registry.py
  - every registry verb name MUST appear in docs/SCENARIOS.md (the
    "## Verb reference" table is generated from the registry - keep
    it in sync with `python3 tools/gen_verbs_doc.py >> docs/...` or by
    re-running ./sim verbs)

Exit 1 on any violation.
"""

import glob
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        ROOT, "var"), "pycache"))

sys.path.insert(0, os.path.join(ROOT, "engine"))
import registry        # noqa: E402

bad = []
used = {"do": set(), "fault": set(), "assert": set()}

for path in sorted(glob.glob(os.path.join(ROOT, "tests", "**", "*.yaml"),
                             recursive=True)
                   + glob.glob(os.path.join(ROOT, "scenarios", "**",
                                            "*.yaml"),
                               recursive=True)):
    rel = os.path.relpath(path, ROOT)
    try:
        raw = yaml.safe_load(open(path))
    except Exception:
        continue                    # not a scenario yaml / unparseable
    if not isinstance(raw, dict) or not (
            raw.get("timeline") or raw.get("assert") or
            raw.get("chaos")):
        continue
    for i, step in enumerate(raw.get("timeline") or []):
        if not isinstance(step, dict):
            continue
        if step.get("do"):
            used["do"].add(step["do"])
        if step.get("fault"):
            used["fault"].add(step["fault"])
    for asr in raw.get("assert") or []:
        if isinstance(asr, dict) and asr:
            used["assert"].add(next(iter(asr)))
    for f in (raw.get("chaos") or {}).get("faults") or []:
        if isinstance(f, dict) and f.get("kind"):
            used["fault"].add(f["kind"])

for group, names in used.items():
    for nm in sorted(names):
        if registry.lookup(nm, group) is None:
            bad.append(f"{group}: {nm!r} used under tests//scenarios "
                       "but not in the registry")

# the registry's check names must be exactly checks.DISPATCH's
import checks                   # noqa: E402
for nm in checks.DISPATCH:
    if registry.lookup(nm, "check") is None:
        bad.append(f"check: {nm!r} in DISPATCH but not the registry")
for v in registry.by_kind("check"):
    if v["name"] not in checks.DISPATCH:
        bad.append(f"check: {v['name']!r} in the registry but not "
                   "DISPATCH")

# docs/SCENARIOS.md verb reference must name every registered verb
doc = open(os.path.join(ROOT, "docs", "SCENARIOS.md")).read()
for v in registry.VERBS:
    if f"`{v['name']}`" not in doc and f"| {v['name']} " not in doc:
        bad.append(f"{v['kind']}: {v['name']!r} not in "
                   "docs/SCENARIOS.md")

if bad:
    for b in bad:
        print("not ok  " + b)
    sys.exit(f"{len(bad)} problem(s)")
print(f"ok  {sum(len(s) for s in used.values())} used names covered, "
      f"{len(registry.VERBS)} registry verbs documented")
