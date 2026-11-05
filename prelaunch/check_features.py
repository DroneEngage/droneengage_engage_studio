#!/usr/bin/env python3
"""prelaunch/check_features.py - pure validation of the feature register.

load(path) returns the parsed register; validate(reg, root) returns a
list of problems. No network and no sim. The check id test is the core:
a feature row that names a check that does not exist must fail, because
a row with no real check gives a false feeling of cover.

Usage: python3 prelaunch/check_features.py [features.yaml]
"""

import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PATH = os.path.join(ROOT, "prelaunch", "features.yaml")
STAGES = ("0", "1", "install", "2", "3", "4")
SEVERITIES = ("P0", "P1")
KIND_FIELDS = {"probe": "impl", "scenario": "file",
               "command": "run", "manual": "text"}


def load(path=DEFAULT_PATH):
    with open(path) as fh:
        return yaml.safe_load(fh)


def _check_problems(cid, c, root):
    out = []
    if not isinstance(c, dict):
        return [f"check {cid}: not a mapping"]
    if str(c.get("stage")) not in STAGES:
        out.append(f"check {cid}: stage {c.get('stage')!r} is not one of "
                   f"{', '.join(STAGES)}")
    kind = c.get("kind")
    if kind not in KIND_FIELDS:
        out.append(f"check {cid}: kind {kind!r} is not one of "
                   f"{', '.join(KIND_FIELDS)}")
        return out
    if not c.get(KIND_FIELDS[kind]):
        out.append(f"check {cid}: kind {kind} needs {KIND_FIELDS[kind]}")
    if not c.get("title"):
        out.append(f"check {cid}: no title")
    if kind == "scenario" and c.get("file") and \
            not os.path.isfile(os.path.join(root, c["file"])):
        out.append(f"check {cid}: scenario file {c['file']} does not exist")
    if kind == "command" and not c.get("repo"):
        out.append(f"check {cid}: kind command needs repo")
    if kind == "probe" and c.get("impl") and ":" not in c["impl"]:
        out.append(f"check {cid}: impl must be <module>:<function>")
    return out


def validate(reg, root=ROOT):
    """Every problem in the register, as text. Empty list = valid."""
    if not isinstance(reg, dict):
        return ["the top level is not a mapping"]
    checks = reg.get("checks")
    features = reg.get("features")
    if not isinstance(checks, dict) or not checks:
        return ["no checks: map"]
    if not isinstance(features, list) or not features:
        return ["no features: list"]
    out = []
    for cid, c in checks.items():
        out += _check_problems(cid, c, root)
    seen, used = set(), set()
    for f in features:
        if not isinstance(f, dict) or not f.get("id"):
            out.append(f"feature row without id: {f!r}")
            continue
        fid = f["id"]
        if fid in seen:
            out.append(f"feature {fid}: duplicate id")
        seen.add(fid)
        if f.get("severity") not in SEVERITIES:
            out.append(f"feature {fid}: severity {f.get('severity')!r} is "
                       f"not P0 or P1")
        if not f.get("title"):
            out.append(f"feature {fid}: no title")
        if not f.get("components"):
            out.append(f"feature {fid}: no components")
        if not f.get("checks"):
            out.append(f"feature {fid}: no checks")
        for cid in f.get("checks") or []:
            used.add(cid)
            if cid not in checks:
                out.append(f"feature {fid}: check {cid} does not exist")
    return out


def main(argv):
    path = argv[1] if len(argv) > 1 else DEFAULT_PATH
    try:
        problems = validate(load(path))
    except (OSError, yaml.YAMLError) as e:
        problems = [str(e)]
    for p in problems:
        print(f"prelaunch: {p}")
    if not problems:
        print(f"prelaunch: {os.path.relpath(path, ROOT)} OK")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
