#!/usr/bin/env python3
"""tools/golden_simjson.py - golden-master check for sim.json generation
(phase 0 of docs/port_allocation.md): record every generatable sim's
manifest once, then prove a re-run produces the same sim.json and
sim_hash.

  python3 tools/golden_simjson.py --write <dir>   record a baseline
  python3 tools/golden_simjson.py --check <dir>   re-generate and diff;
                                                  exit 1 on any diff
  --only <substr>                limit to files whose repo-relative
                                 path contains it

Enumerates sims/*.yaml (top level only - overlays/ are fragments),
scenarios/**/*.yaml and tests/**/*.yaml. Each loads and generates the
way ./sim does, into <dir>/run_scratch (SIM_RUN_DIR/SIM_RESULTS_DIR are
bound at simlib import, so they are set before it) - never the repo's
real var/run. <dir>/golden.json holds
{relpath: {ok, error, sim_hash, sim_json}}; `owner` (pid/sid/t_ms) is
dropped and a repo-local sim_file made relative before storing, so the
baseline survives a moved checkout and a new process. A sim that cannot
generate here (a missing module repo checkout) records its error as the
entry - the check compares the failure too.
"""

import difflib
import glob
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _baseline_arg(argv):
    """The <dir> after --write/--check, before real parsing - the scratch
    env vars must be set before simlib binds RUN_DIR/RESULTS_DIR."""
    for i, a in enumerate(argv):
        if a in ("--write", "--check") and i + 1 < len(argv):
            return os.path.abspath(argv[i + 1])
    return None


_scratch = _baseline_arg(sys.argv[1:])
if _scratch:
    os.environ["SIM_RUN_DIR"] = os.path.join(_scratch, "run_scratch")
    os.environ["SIM_RESULTS_DIR"] = os.path.join(_scratch,
                                               "results_scratch")

# bytecode under var/pycache from the first import on (same preamble as
# ./sim - else __pycache__ dirs land next to the engine sources)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(REPO, "var"),
                 "pycache"))
os.environ.setdefault("SIM_ROOT", REPO)
# "any" would make load_sim probe live sims + host ports - the golden
# must resolve the same slot every run
os.environ.pop("SIM_SLOT_BASE_OVERRIDE", None)

sys.path.insert(0, os.path.join(REPO, "engine"))
import simlib                        # noqa: E402
import scenario as scenario_mod      # noqa: E402

# the golden covers manifest content, not whether this machine has every
# module binary (module templates still must exist - a missing repo
# checkout raises into a failure entry below)
simlib.check_binaries = lambda sim: None


def _enum():
    """Every generatable yaml as (repo-relative path, kind): 'sim' for
    the fleet files sims/*.yaml, 'scenario' for de.scenario/1 files."""
    out = []
    for f in sorted(glob.glob(os.path.join(REPO, "sims", "*.yaml"))):
        out.append((os.path.relpath(f, REPO), "sim"))
    for root in ("scenarios", "tests"):
        pat = os.path.join(REPO, root, "**", "*.yaml")
        for f in sorted(glob.glob(pat, recursive=True)):
            out.append((os.path.relpath(f, REPO), "scenario"))
    return out


def _rel_repo(p):
    """sim_file inside the repo -> relative (the baseline must not pin
    the checkout's absolute path); anything else stays as generated."""
    if not isinstance(p, str) or not os.path.isabs(p):
        return p
    rp = os.path.relpath(p, REPO)
    return p if rp.startswith("..") else rp


def _produce(rel, kind):
    """Load + generate one yaml -> {ok, error, sim_hash, sim_json}.
    sims/*.yaml generate straight from the file; a scenario resolves
    through load_scenario (probe_ports=False keeps the seed's slot pick
    deterministic - no host probing) and generates its inline sim dict,
    labelled "inline" like a real `run` does."""
    abspath = os.path.join(REPO, rel)
    try:
        if kind == "sim":
            sim = simlib.load_sim(abspath)
            simlib.generate(sim, abspath)
        else:
            scn = scenario_mod.load_scenario(abspath, probe_ports=False)
            sim = simlib.load_sim(scn["sim"],
                                  base_dir=os.path.dirname(abspath))
            simlib.generate(sim, "inline")
        with open(simlib.manifest_path(sim["name"])) as fh:
            m = json.load(fh)
    except (SystemExit, Exception) as e:
        return {"ok": False, "error": f"{type(e).__name__}: {e}",
                "sim_hash": None, "sim_json": None}
    m.pop("owner", None)            # pid/sid/t_ms - never reproducible
    m["sim_file"] = _rel_repo(m.get("sim_file"))
    return {"ok": True, "error": None,
            "sim_hash": m.get("sim_hash"), "sim_json": m}


def _diff(want, cur, limit=60):
    """Readable unified diff of two normalized manifests; both sides
    dumped sort-keyed so dict order never diffs."""
    ta = json.dumps(want, sort_keys=True, indent=1).splitlines()
    tb = json.dumps(cur, sort_keys=True, indent=1).splitlines()
    d = list(difflib.unified_diff(ta, tb, "baseline", "current",
                                  lineterm=""))
    return d[:limit] + ([f"... ({len(d) - limit} more)"]
                        if len(d) > limit else [])


def _verdict(want, cur):
    """same | CHANGED | newly-failing | newly-passing for one baseline
    entry vs its re-generated entry (a matching failure is 'same')."""
    if want["ok"] and not cur["ok"]:
        print(f"      error: {cur['error']}")
        return "newly-failing"
    if not want["ok"] and cur["ok"]:
        return "newly-passing"
    if not want["ok"]:
        if want["error"] == cur["error"]:
            return "same"
        print(f"      baseline error: {want['error']}")
        print(f"      current error:  {cur['error']}")
        return "CHANGED"
    if want["sim_hash"] == cur["sim_hash"] \
            and want["sim_json"] == cur["sim_json"]:
        return "same"
    if want["sim_hash"] != cur["sim_hash"]:
        print(f"      sim_hash: {want['sim_hash']} -> {cur['sim_hash']}")
    for ln in _diff(want["sim_json"], cur["sim_json"]):
        print("      " + ln)
    return "CHANGED"


def _write(bdir, only):
    os.makedirs(bdir, exist_ok=True)
    out, nfail = {}, 0
    for rel, kind in _enum():
        if only and only not in rel:
            continue
        e = _produce(rel, kind)
        out[rel] = e
        if e["ok"]:
            print(f"ok    {rel}")
        else:
            nfail += 1
            print(f"FAIL  {rel}: {e['error']}")
    path = os.path.join(bdir, "golden.json")
    with open(path, "w") as fh:
        json.dump(out, fh, sort_keys=True, indent=1)
    print(f"{len(out)} entries ({nfail} failures) -> {path}")
    return 0


def _check(bdir, only):
    path = os.path.join(bdir, "golden.json")
    if not os.path.isfile(path):
        sys.exit(f"no baseline at {path} - run --write first")
    with open(path) as fh:
        base = json.load(fh)
    on_disk = dict(_enum())
    counts = {"same": 0, "CHANGED": 0, "newly-failing": 0,
              "newly-passing": 0, "missing": 0}
    for rel in sorted(base):
        if only and only not in rel:
            continue
        if rel not in on_disk:
            verdict = "missing"
        else:
            verdict = _verdict(base[rel], _produce(rel, on_disk[rel]))
        counts[verdict] += 1
        print(f"{verdict:<14} {rel}")
    for rel in sorted(r for r in on_disk
                      if r not in base and (not only or only in r)):
        print(f"new            {rel}  (not in baseline - ignored)")
    checked = sum(counts.values())
    print(f"{checked} checked: " + ", ".join(
        f"{v} {k}" for k, v in counts.items() if v))
    # exit 0 only when every baseline entry matches and none is missing
    return 1 if checked - counts["same"] else 0


def main(argv):
    write = check = only = None
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("--write", "--check") and i + 1 < len(argv):
            if a == "--write":
                write = argv[i + 1]
            else:
                check = argv[i + 1]
            i += 2
        elif a == "--only" and i + 1 < len(argv):
            only = argv[i + 1]
            i += 2
        else:
            sys.exit(__doc__)
    if (write is None) == (check is None):
        sys.exit(__doc__)
    if write is not None:
        return _write(os.path.abspath(write), only)
    return _check(os.path.abspath(check), only)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
