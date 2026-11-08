#!/usr/bin/env python3
"""tools/golden_requests.py - golden-master check for the port-request
list each sim produces (phase 3 of docs/port_allocation.md): every port
a sim binds is a named allocator request, so the REQUEST LIST - names,
protocols, kinds, pins, prefer hints, block sizes - is the stable,
machine-independent contract a yaml's port behaviour boils down to.
This replaces golden_simjson.py: allocated port values are deliberately
NOT recorded (they are per-machine allocator picks).

  python3 tools/golden_requests.py --write [file]   record a baseline
                                                    (default:
                                                    tools/golden_requests.json,
                                                    checked in)
  python3 tools/golden_requests.py --check [file]   re-resolve and diff;
                                                    exit 1 on any diff
  --only <substr>                    limit to files whose repo-relative
                                     path contains it

Enumerates sims/*.yaml (top level only - overlays/ are fragments),
scenarios/**/*.yaml and tests/**/*.yaml. Each resolves the way ./sim
does - a scenario through load_scenario, a sim file straight - then
simlib.lease_requests gives the request list; nothing is generated or
leased. <file> holds {relpath: {ok, error, sim_hash, requests}} sorted;
a sim that cannot resolve here (a missing module catalog entry, a bad
unit name) records its error as the entry - the check compares the
failure too.
"""

import difflib
import glob
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASELINE = os.path.join(REPO, "tools", "golden_requests.json")

# bytecode under var/pycache from the first import on (same preamble as
# ./sim - else __pycache__ dirs land next to the engine sources)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(REPO, "var"),
                 "pycache"))
os.environ.setdefault("SIM_ROOT", REPO)
# "any" would only change identity - drop overrides so the golden
# resolves the spec's own base every run
os.environ.pop("SIM_SLOT_BASE_OVERRIDE", None)
os.environ.pop("SIM_WITH", None)

sys.path.insert(0, os.path.join(REPO, "engine"))
import simlib                        # noqa: E402
import scenario as scenario_mod      # noqa: E402


def _enum():
    """Every resolvable yaml as (repo-relative path, kind): 'sim' for
    the fleet files sims/*.yaml, 'scenario' for de.scenario/1 files."""
    out = []
    for f in sorted(glob.glob(os.path.join(REPO, "sims", "*.yaml"))):
        out.append((os.path.relpath(f, REPO), "sim"))
    for root in ("scenarios", "tests"):
        pat = os.path.join(REPO, root, "**", "*.yaml")
        for f in sorted(glob.glob(pat, recursive=True)):
            out.append((os.path.relpath(f, REPO), "scenario"))
    return out


def _requests(sim):
    """lease_requests normalized for the golden: sorted, only the fields
    that define the request (name/proto/kind/pin/prefer/size) - no
    allocator state, no remembered ports."""
    rows = []
    for r in simlib.lease_requests(sim):
        row = {"name": r["name"], "proto": r["proto"], "kind": r["kind"]}
        for k in ("pin", "prefer", "size"):
            if r.get(k) is not None:
                row[k] = r[k]
        rows.append(row)
    rows.sort(key=lambda r: r["name"])
    return rows


def _produce(rel, kind):
    """Load one yaml -> {ok, error, sim_hash, requests}. sims/*.yaml
    resolve straight from the file; a scenario resolves through
    load_scenario and resolves its inline sim dict, labelled "inline"
    like a real `run` does."""
    abspath = os.path.join(REPO, rel)
    try:
        if kind == "sim":
            sim = simlib.load_sim(abspath)
        else:
            scn = scenario_mod.load_scenario(abspath, probe_ports=False)
            sim = simlib.load_sim(scn["sim"],
                                  base_dir=os.path.dirname(abspath))
    except (SystemExit, Exception) as e:
        return {"ok": False, "error": f"{type(e).__name__}: {e}",
                "sim_hash": None, "requests": None}
    return {"ok": True, "error": None,
            "sim_hash": sim["hash"], "requests": _requests(sim)}


def _diff(want, cur, limit=60):
    """Readable unified diff of two normalized entries; both sides
    dumped sort-keyed so dict order never diffs."""
    ta = json.dumps(want, sort_keys=True, indent=1).splitlines()
    tb = json.dumps(cur, sort_keys=True, indent=1).splitlines()
    d = list(difflib.unified_diff(ta, tb, "baseline", "current",
                                  lineterm=""))
    return d[:limit] + ([f"... ({len(d) - limit} more)"]
                        if len(d) > limit else [])


def _verdict(want, cur):
    """same | CHANGED | newly-failing | newly-passing for one baseline
    entry vs its re-resolved entry (a matching failure is 'same')."""
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
            and want["requests"] == cur["requests"]:
        return "same"
    if want["sim_hash"] != cur["sim_hash"]:
        print(f"      sim_hash: {want['sim_hash']} -> {cur['sim_hash']}")
    for ln in _diff({"sim_hash": want["sim_hash"],
                     "requests": want["requests"]},
                    {"sim_hash": cur["sim_hash"],
                     "requests": cur["requests"]}):
        print("      " + ln)
    return "CHANGED"


def _write(path, only):
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
    with open(path, "w") as fh:
        json.dump(out, fh, sort_keys=True, indent=1)
        fh.write("\n")
    print(f"{len(out)} entries ({nfail} failures) -> {path}")
    return 0


def _check(path, only):
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
        if a in ("--write", "--check"):
            if i + 1 < len(argv) and not argv[i + 1].startswith("-"):
                path = os.path.abspath(argv[i + 1])
                i += 2
            else:
                path = BASELINE
                i += 1
            if a == "--write":
                write = path
            else:
                check = path
        elif a == "--only" and i + 1 < len(argv):
            only = argv[i + 1]
            i += 2
        else:
            sys.exit(__doc__)
    if (write is None) == (check is None):
        sys.exit(__doc__)
    if write is not None:
        return _write(write, only)
    return _check(check, only)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
