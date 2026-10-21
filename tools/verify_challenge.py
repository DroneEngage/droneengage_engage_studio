#!/usr/bin/env python3
"""verify_challenge.py - SL-0046 scoreboard verifier.

A submission is a directory under scenarios/challenges/entries/<handle>/
holding scenario.yaml (the official challenge with only `name:` and
`operation:` changed), the op files that operation references, and the
summary.json the run produced. Usage:

  python3 tools/verify_challenge.py <dir>            # structure + score
  python3 tools/verify_challenge.py <dir> --replay   # rerun + compare

--check (default): the submitted scenario must equal the official
scenarios/challenges/sar_challenge.yaml in every key except `name` and
`operation` (world/verify refs are compared path-resolved), and its
summary.json must carry seed 11. Prints the score per docs/CHALLENGE.md.

--replay: additionally downs any same-name sim (a reused sim keeps its
world epoch, so seeded draws land on different frame indices - cold
start only), then reruns `./sim run <dir>/scenario.yaml` (additive -
`--shared --slot-base any` so it works beside other live sims) and
compares each assertion's measured value against the submitted
summary's - fractions within 0.05, time_to_detect within 15 % or 10 s.
Only a --replay VERIFIED counts for a scoreboard row; without it the
check is structure + self-reported numbers only.
"""
import json
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OFFICIAL = os.path.join(ROOT, "scenarios", "challenges",
                        "sar_challenge.yaml")
OFFICIAL_SEED = 11
SCORE_TTD_TRUTH = "person_meadow"
SCORE_TTD_CAP_S = 120.0

sys.path.insert(0, os.path.join(ROOT, "engine"))
import yaml  # noqa: E402  (pyyaml is an engine dep)


def _load(path):
    with open(path) as fh:
        return yaml.safe_load(fh)


def _resolved(doc, key, base):
    v = doc.get(key)
    if isinstance(v, str):
        return os.path.normpath(os.path.join(base, v))
    if isinstance(v, dict) and isinstance(v.get("cli"), str):
        v = dict(v)
        v["cli"] = os.path.normpath(os.path.join(base, v["cli"]))
    return v


def check_scenario(sub_yaml):
    """Every key except name/operation must equal the official file;
    world/verify refs compare path-resolved. -> list of diffs."""
    off = _load(OFFICIAL)
    sub = _load(sub_yaml)
    diffs = []
    for key in set(off) | set(sub):
        if key in ("name", "operation"):
            continue
        a, b = (os.path.dirname(os.path.abspath(p)) for p in
                (OFFICIAL, sub_yaml))
        va = _resolved(off, key, a) if key in ("world", "verify") \
            else off.get(key)
        vb = _resolved(sub, key, b) if key in ("world", "verify") \
            else sub.get(key)
        if va != vb:
            diffs.append(f"key {key!r}: official {va!r} != {vb!r}")
    return diffs


def score_of(summary):
    """score = 60*pod_mean + 25*speed + 15*precision (docs/CHALLENGE.md)."""
    vals = {"pod_mean": None, "precision": None, "ttd_s": None}
    for a in summary.get("assertions", []):
        n, v, d = a.get("name"), a.get("value"), a.get("detail", "")
        if n == "pod_mean":
            vals["pod_mean"] = v
        elif n == "detection_precision" and v is not None:
            vals["precision"] = v if vals["precision"] is None \
                else min(vals["precision"], v)
        elif n == "time_to_detect" and SCORE_TTD_TRUTH in d:
            vals["ttd_s"] = v
    pod = vals["pod_mean"] or 0.0
    prec = vals["precision"] or 0.0
    ttd = vals["ttd_s"]
    speed = max(0.0, 1.0 - (ttd or SCORE_TTD_CAP_S) / SCORE_TTD_CAP_S)
    total = 60.0 * pod + 25.0 * speed + 15.0 * prec
    return {"pod_mean": pod, "ttd_s": ttd, "precision": prec,
            "score": round(total, 1)}


def _close(a, b, name):
    if a is None or b is None:
        return a == b
    if name == "time_to_detect":
        return abs(a - b) <= max(10.0, 0.15 * abs(b))
    # 0.05: observed cold-start spread on pod_at is ~0.025 (wall-clock
    # frame-keyed draws shift a few looks); 0.02 rejects honest entries
    return math.isclose(a, b, abs_tol=0.05)


def replay(sub_yaml, submitted):
    """Rerun the submission's scenario and compare assertion values."""
    # A same-name sim left up reuses its processes - and its world
    # epoch, so the seeded draws land on different frame indices.
    # Down it first: every replay starts cold, like the original run.
    sname = _load(sub_yaml).get("name")
    if sname:
        subprocess.run([os.path.join(ROOT, "sim"), "down", str(sname)],
                       cwd=ROOT, capture_output=True, text=True)
    out = subprocess.run(
        [os.path.join(ROOT, "sim"), "run", sub_yaml,
         "--seed", str(OFFICIAL_SEED),
         "--shared", "--slot-base", "any"],
        cwd=ROOT, capture_output=True, text=True, timeout=1800)
    m = None
    for line in out.stdout.splitlines():
        if line.startswith("RESULT "):
            m = line.split(None, 1)[1].strip()
    if not m:
        return ["replay produced no RESULT line "
                f"(exit {out.returncode})"]
    fresh = json.load(open(m))
    diffs = []
    old = [(a.get("name"), a.get("detail", ""), a.get("value"))
           for a in submitted.get("assertions", [])]
    new = [(a.get("name"), a.get("detail", ""), a.get("value"))
           for a in fresh.get("assertions", [])]
    for (n, d, v) in old:
        peers = [nv for (nn, nd, nv) in new
                 if nn == n and _same_assert(n, d, nd)]
        if not peers:
            diffs.append(f"{n} ({d[:40]}): absent in replay")
        elif not any(_close(v, nv, n) for nv in peers):
            diffs.append(f"{n} ({d[:40]}): submitted {v} "
                         f"vs replayed {peers}")
    return diffs


def _same_assert(name, d1, d2):
    """Same grader instance = same name; for per-truth checks the truth
    id must appear in both details."""
    for tid in ("person_meadow", "person_wood", "person_barn"):
        if (tid in d1) != (tid in d2):
            return False
    return True


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if len(args) != 1 or not os.path.isdir(args[0]):
        sys.exit("usage: verify_challenge.py <submission_dir> [--replay]")
    d = args[0]
    sub_yaml = os.path.join(d, "scenario.yaml")
    summ_p = os.path.join(d, "summary.json")
    for p in (sub_yaml, summ_p):
        if not os.path.isfile(p):
            sys.exit(f"missing {p} - see docs/CHALLENGE.md")
    problems = check_scenario(sub_yaml)
    summ = json.load(open(summ_p))
    if summ.get("seed") != OFFICIAL_SEED:
        problems.append(f"seed {summ.get('seed')} != official "
                        f"{OFFICIAL_SEED}")
    sc = score_of(summ)
    print(f"pod_mean {sc['pod_mean']}  ttd {sc['ttd_s']}s  "
          f"precision {sc['precision']}  -> SCORE {sc['score']}")
    if "--replay" in sys.argv and not problems:
        problems += replay(sub_yaml, summ)
    for p in problems:
        print(f"  MISMATCH {p}")
    if problems:
        print(f"REJECTED ({len(problems)})")
    elif "--replay" in sys.argv:
        print("VERIFIED")
    else:
        print("STRUCTURE OK - score is self-reported; "
              "only a --replay VERIFIED counts for the scoreboard")
    sys.exit(0 if not problems else 1)


if __name__ == "__main__":
    main()
