#!/usr/bin/env python3
"""prelaunch.py - the pre-launch gate runner behind `./sim prelaunch`.

Reads a target file (prelaunch/targets/<name>.yaml: which host runs which
service) and the feature register (prelaunch/features.yaml), runs the
checks stage by stage, and writes report.md + report.json under
var/results/prelaunch/<version>/<target>/. docs/PRELAUNCH.md is the
procedure this implements.

The rule that drives the design: a P0 row passes only when every one of
its checks PASSed. A SKIP proves nothing, so it blocks the same as a FAIL.
A probe with no code yet gives SKIP, never a pass.

A probe is a function fn(ctx) -> (status, detail), or it raises Skip.
ctx is a Ctx: the target, secrets lookup and the output dir. Everything
written to the report goes through Ctx.scrub so credentials stay out.
"""

import argparse
import datetime
import importlib
import json
import os
import re
import subprocess
import sys
import time

import yaml

ENGINE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ENGINE)
import simlib  # noqa: E402

ROOT = os.path.dirname(ENGINE)
sys.path.insert(0, os.path.join(ROOT, "prelaunch"))
import check_features  # noqa: E402

PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"
STAGES = check_features.STAGES
TARGETS_DIR = os.path.join(ROOT, "prelaunch", "targets")

# component -> (repo dir setting, default path under DE_CODE); used for
# the version lines of the report
COMPONENT_REPOS = {
    "de_comm": ("DE_COMM_DIR", "droneengage_communication"),
    "de_mavlink": ("DE_MAVLINK_DIR", "droneengage_mavlink"),
    "de_camera": ("DE_CAMERA_DIR", "droneengage_camera"),
    "webclient": ("DE_WEBCLIENT_DIR", "droneengage_webclient_react"),
    "andruav": ("DE_ANDRUAV_DIR", "andruav_android_app"),
    "de_auth": ("DE_AUTH_DIR", "servers/droneengage_authenticator"),
    "de_comm_server": ("DE_SERVER_DIR", "servers/droneengage_server"),
    "de_storage": ("DE_STORAGE_DIR", "servers/droneengage_storage_server"),
    "de_udp": ("DE_UDP_RELAY_DIR", "servers/droneengage_udp_proxy"),
}
# a target file names credentials by env key only: these keys never hold
# a value
_FORBIDDEN_TARGET_KEYS = re.compile(
    r"^(password|passwd|pwd|code|secret|token|credential)$", re.I)


class Skip(Exception):
    """A probe cannot run here (missing setting, no phone). Blocks a P0."""


class Ctx:
    """What a probe gets: the target, secrets and the output dir."""

    def __init__(self, target, out_dir, version, features=None):
        self.target = target
        self.out_dir = out_dir
        self.version = version
        self.features = features or {}
        self.secrets = set()
        self.versions = {}       # a probe may record installed versions

    @property
    def name(self):
        return self.target.get("name", "?")

    @property
    def role(self):
        return self.target.get("role", "airgap")

    def service(self, name):
        s = (self.target.get("services") or {}).get(name)
        if not s:
            raise Skip(f"target {self.name} has no services.{name}")
        return s

    def secret(self, env_key):
        """A secret's value from the env or sim.local.env. The value is
        added to the scrub list, so it cannot reach a report."""
        v = simlib.setting(env_key)
        if not v:
            raise Skip(f"{env_key} is not set in the env or sim.local.env")
        self.secrets.add(v)
        return v

    def account(self):
        """(user, code) of the test account for this target."""
        a = self.target.get("account") or {}
        if not a.get("user_env") or not a.get("code_env"):
            raise Skip(f"target {self.name} has no account.user_env/code_env")
        return self.secret(a["user_env"]), self.secret(a["code_env"])

    def scrub(self, text):
        out = str(text)
        for s in sorted(self.secrets, key=len, reverse=True):
            out = out.replace(s, "<redacted>")
        return out

    def log_path(self, name):
        d = os.path.join(self.out_dir, "logs")
        os.makedirs(d, exist_ok=True)
        return os.path.join(d, re.sub(r"[^\w.-]", "_", name) + ".log")


def _now():
    return datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")


# ---- target ---------------------------------------------------------

def _forbidden_keys(node, path=""):
    if isinstance(node, dict):
        for k, v in node.items():
            p = f"{path}.{k}" if path else str(k)
            if _FORBIDDEN_TARGET_KEYS.match(str(k)):
                yield p
            yield from _forbidden_keys(v, p)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from _forbidden_keys(v, f"{path}[{i}]")


def load_target(name_or_path):
    path = name_or_path
    if not os.path.isfile(path):
        path = os.path.join(TARGETS_DIR, f"{name_or_path}.yaml")
    if not os.path.isfile(path):
        have = sorted(os.path.splitext(f)[0] for f in os.listdir(TARGETS_DIR)
                      if f.endswith(".yaml")) \
            if os.path.isdir(TARGETS_DIR) else []
        sys.exit(f"no target {name_or_path!r} (have: {', '.join(have)})")
    with open(path) as fh:
        t = yaml.safe_load(fh)
    if not isinstance(t, dict) or not t.get("name"):
        sys.exit(f"{path}: a target needs a name")
    if not isinstance(t.get("services"), dict):
        sys.exit(f"{path}: a target needs a services map with a host for "
                 f"each service")
    unset = [f"services.{n}.{k}" for n, sv in t["services"].items()
             for k, v in (sv or {}).items() if v is None]
    if unset:
        sys.exit(f"{path}: set these values first: {', '.join(unset)}")
    bad = list(_forbidden_keys(t))
    if bad:
        sys.exit(f"{path}: {', '.join(bad)}: a target file must not hold "
                 f"credentials. Use *_env keys and sim.local.env")
    return t


# ---- register -------------------------------------------------------

def load_register(path=None):
    path = path or check_features.DEFAULT_PATH
    reg = check_features.load(path)
    problems = check_features.validate(reg)
    if problems:
        sys.exit("feature register is not valid:\n  " + "\n  ".join(problems))
    return reg


def check_applies(check, role):
    """A check may name the target roles it is for (the slot checks exist
    only on a mirror). With no `roles:` it runs everywhere."""
    roles = check.get("roles")
    return not roles or role in roles


def stage_of(reg, cid):
    return str(reg["checks"][cid]["stage"])


def parse_stages(text):
    if not text:
        return list(STAGES)
    out = []
    for s in str(text).split(","):
        s = s.strip()
        if s not in STAGES:
            sys.exit(f"unknown stage {s!r} (have: {', '.join(STAGES)})")
        out.append(s)
    return [s for s in STAGES if s in out]


def used_check_ids(reg):
    """Every check a feature row names, plus each unused one: an unused
    check still runs, so a report never hides a red probe."""
    used = []
    for f in reg["features"]:
        for c in f["checks"]:
            if c not in used:
                used.append(c)
    for c in reg["checks"]:
        if c not in used:
            used.append(c)
    return used


# ---- running one check ----------------------------------------------

def _run_probe(check, ctx):
    mod_name, fn_name = check["impl"].split(":", 1)
    try:
        mod = importlib.import_module(mod_name)
    except ModuleNotFoundError as e:
        if e.name and (e.name == mod_name or mod_name.startswith(e.name)):
            return SKIP, f"not built yet ({mod_name} does not exist)"
        raise
    fn = getattr(mod, fn_name, None)
    if fn is None:
        return SKIP, f"not built yet ({check['impl']} does not exist)"
    return fn(ctx)


def _scenario_runner():
    sys.path.insert(0, os.path.join(ROOT, "ci"))
    import run_suite
    return run_suite.run_once


def _run_scenario(check, ctx):
    run_once = _scenario_runner()
    path = os.path.join(ROOT, check["file"])
    v, wall, rd, why = run_once(path, 1, False, shared=True)
    rel = os.path.relpath(rd, ROOT) if rd else "no results dir"
    if v in ("PASS", "GATE-PASS"):
        return PASS, f"{v} in {wall}s ({rel})"
    return FAIL, f"{why or v} in {wall}s ({rel})"


def _repo_dir(repo):
    return os.path.join(simlib.setting("DE_CODE", os.path.dirname(ROOT)), repo)


def _run_command(cid, check, ctx):
    d = _repo_dir(check["repo"])
    if not os.path.isdir(d):
        return SKIP, f"repo {check['repo']} is not at {d}"
    log = ctx.log_path(cid)
    timeout = int(check.get("timeout_s", 1800))
    try:
        with open(log, "w") as fh:
            r = subprocess.run(check["run"], shell=True, cwd=d, stdout=fh,
                               stderr=subprocess.STDOUT, timeout=timeout)
    except subprocess.TimeoutExpired:
        return FAIL, f"no result after {timeout}s. See {log}"
    if r.returncode == 0:
        return PASS, f"`{check['run']}` exit 0"
    return FAIL, f"`{check['run']}` exit {r.returncode}. See {log}"


def run_check(cid, check, ctx, signoffs):
    """One check -> {status, detail, stage, ...}. Never raises: a crash
    in a probe is a FAIL with its text, so one bad probe cannot hide the
    results of the others."""
    t0 = time.time()
    kind = check["kind"]
    try:
        if kind == "manual":
            status, detail = SKIP, f"needs a human: {check['text']}"
        elif kind == "probe":
            status, detail = _run_probe(check, ctx)
        elif kind == "scenario":
            status, detail = _run_scenario(check, ctx)
        else:
            status, detail = _run_command(cid, check, ctx)
    except Skip as e:
        status, detail = SKIP, str(e)
    except (Exception, SystemExit) as e:
        status, detail = FAIL, f"the check crashed: {e!r}"
    res = {"status": status, "detail": ctx.scrub(detail),
           "stage": str(check["stage"]), "kind": kind,
           "title": check.get("title", ""), "at": _now(),
           "wall_s": round(time.time() - t0, 1)}
    return apply_signoff(cid, res, signoffs)


def apply_signoff(cid, res, signoffs):
    """A human signoff can turn a SKIP into a PASS, or anything into a
    FAIL. It cannot turn a FAIL into a PASS: that is what --override is
    for, and an override is shown as one."""
    s = signoffs.get(cid)
    if not s:
        return res
    if s["result"] == "fail":
        res["status"], res["detail"] = FAIL, f"human signoff: fail ({s['at']})"
    elif res["status"] == SKIP:
        res["status"] = PASS
        res["detail"] = f"human signoff: pass ({s['at']})"
    res["signed"] = True
    return res


# ---- versions -------------------------------------------------------

def collect_versions():
    out = {}
    for comp, (env, default) in COMPONENT_REPOS.items():
        d = simlib.setting(env) or _repo_dir(default)
        try:
            sha = subprocess.check_output(
                ["git", "-C", d, "rev-parse", "--short", "HEAD"],
                stderr=subprocess.DEVNULL, text=True).strip()
            dirty = subprocess.check_output(
                ["git", "-C", d, "status", "--porcelain", "-uno"],
                stderr=subprocess.DEVNULL, text=True).strip()
            out[comp] = sha + ("+dirty" if dirty else "")
        except (OSError, subprocess.CalledProcessError):
            out[comp] = "unknown"
    return out


# ---- report ---------------------------------------------------------

def feature_rows(reg, results, stages=None, role=None):
    """Each feature's status from its checks. With `stages`, only checks
    of those stages count, and a row with none of them is left out. With
    `role`, a check for other roles does not count."""
    rows = []
    for f in reg["features"]:
        cids = [c for c in f["checks"]
                if (stages is None or stage_of(reg, c) in stages)
                and (role is None or check_applies(reg["checks"][c], role))]
        if not cids:
            continue
        stat = [(results.get(c) or {}).get("status", "NOT RUN") for c in cids]
        if FAIL in stat:
            s = FAIL
        elif all(x == PASS for x in stat):
            s = PASS
        else:
            s = SKIP if SKIP in stat else "NOT RUN"
        rows.append({"id": f["id"], "title": f["title"],
                     "severity": f["severity"], "status": s,
                     "checks": {c: st for c, st in zip(cids, stat)}})
    return rows


def verdict_of(rows):
    """GO only when every P0 row is PASS. A row not run gives INCOMPLETE,
    unless another P0 already failed or skipped."""
    p0 = [r for r in rows if r["severity"] == "P0"]
    if any(r["status"] in (FAIL, SKIP) for r in p0):
        return "NO-GO"
    if any(r["status"] != PASS for r in p0):
        return "INCOMPLETE"
    return "GO"


def _md_cell(t):
    return str(t).replace("|", "/").replace("\n", " ")


def render_md(rep):
    v = rep["verdict"] + (" (HUMAN OVERRIDE)" if rep.get("override") else "")
    lines = [f"# Pre-launch report: {rep['version']} on {rep['target']}", "",
             f"**Verdict: {v}**", "",
             f"Target role: {rep['role']}. Required stages: "
             f"{', '.join(rep['required_stages'])}. Updated {rep['updated']}."]
    if rep.get("override"):
        lines += ["", f"Override at {rep['override']['at']}: "
                      f"{rep['override']['reason']}"]
    lines += ["", "## Features", "", "| Severity | Result | Feature |",
              "|---|---|---|"]
    lines += [f"| {r['severity']} | {r['status']} | {_md_cell(r['title'])} |"
              for r in rep["features"]]
    bad = {c: x for c, x in rep["checks"].items()
           if x["status"] != PASS}
    if bad:
        lines += ["", "## What is not passing", "",
                  "| Result | Check | Stage | Detail |", "|---|---|---|---|"]
        lines += [f"| {x['status']} | {c} | {x['stage']} | "
                  f"{_md_cell(x['detail'])} |" for c, x in bad.items()]
    lines += ["", "## Every check", "",
              "| Result | Check | Stage | Time | Detail |",
              "|---|---|---|---|---|"]
    lines += [f"| {x['status']} | {c} | {x['stage']} | {x['wall_s']}s | "
              f"{_md_cell(x['detail'])} |" for c, x in rep["checks"].items()]
    lines += ["", "## Versions", "", "| Component | Version |", "|---|---|"]
    lines += [f"| {k} | {x} |" for k, x in sorted(rep["versions"].items())]
    return "\n".join(lines) + "\n"


def out_dir_for(version, target_name):
    return os.path.join(simlib.RESULTS_DIR, "prelaunch", version, target_name)


def _read_json(path, default):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return default


def write_report(out, rep):
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "report.json"), "w") as fh:
        json.dump(rep, fh, indent=2, sort_keys=True)
        fh.write("\n")
    with open(os.path.join(out, "report.md"), "w") as fh:
        fh.write(render_md(rep))


def build_report(reg, target, version, results, versions, override=None):
    required = [str(s) for s in target.get("stages") or STAGES]
    rows = feature_rows(reg, results, required, target.get("role", "airgap"))
    return {"version": version, "target": target["name"],
            "role": target.get("role", "airgap"),
            "required_stages": required, "updated": _now(),
            "verdict": verdict_of(rows), "features": rows,
            "checks": results, "versions": versions, "override": override}


# ---- CLI ------------------------------------------------------------

def _print_list(reg):
    for f in reg["features"]:
        print(f"{f['severity']}  {f['id']}: {f['title']}")
        for c in f["checks"]:
            ch = reg["checks"][c]
            print(f"      stage {str(ch['stage']):<8}{ch['kind']:<9}{c}: "
                  f"{ch.get('title', '')}")


def _parse_signoff(text):
    m = re.fullmatch(r"([\w.]+)=(pass|fail)", text or "")
    if not m:
        sys.exit(f"--signoff wants <check>=pass or <check>=fail, not {text!r}")
    return m.group(1), m.group(2)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="./sim prelaunch")
    ap.add_argument("--target", help="prelaunch/targets/<name>.yaml or a path")
    ap.add_argument("--stage", help="comma-separated: "
                    + ",".join(STAGES) + " (default: the target's stages)")
    ap.add_argument("--version", help="release version (default dev-<date>)")
    ap.add_argument("--features", help="feature register path")
    ap.add_argument("--list", action="store_true",
                    help="print features and checks; no side effects")
    ap.add_argument("--json", action="store_true",
                    help="print report.json, or the register with --list")
    ap.add_argument("--signoff", action="append", default=[],
                    help="<check>=pass|fail: record a human result")
    ap.add_argument("--override", metavar="REASON",
                    help="a human accepts a no-go: stored in the report")
    ap.add_argument("--canary", action="store_true",
                    help="read-only probes for a live target; see "
                         "engine/prelaunch_canary.py")
    ap.add_argument("--fresh", action="store_true",
                    help="drop earlier results of this version and target")
    a = ap.parse_args(argv)

    reg = load_register(a.features)
    if a.list:
        if a.json:
            print(json.dumps(reg, indent=2))
        else:
            _print_list(reg)
        return 0
    if not a.target:
        ap.error("--target is required (or use --list)")

    target = load_target(a.target)
    if a.canary:
        import prelaunch_canary
        return prelaunch_canary.run(target, as_json=a.json)
    if target.get("role") == "live":
        sys.exit("a live target takes only --canary: the other stages can "
                 "change state")
    version = a.version or "dev-" + datetime.date.today().isoformat()
    out = out_dir_for(version, target["name"])
    old = {} if a.fresh else _read_json(os.path.join(out, "report.json"), {})
    results = dict(old.get("checks") or {})
    override = old.get("override")
    versions = old.get("versions") or collect_versions()

    # signoffs live next to the report and survive a re-run of a stage
    so_path = os.path.join(out, "signoff.json")
    signoffs = {} if a.fresh else _read_json(so_path, {})
    for text in a.signoff:
        cid, result = _parse_signoff(text)
        if cid not in reg["checks"]:
            sys.exit(f"--signoff: check {cid} does not exist")
        signoffs[cid] = {"result": result, "at": _now()}
    if a.override:
        override = {"reason": a.override, "at": _now()}

    ctx = Ctx(target, out, version, reg)
    os.makedirs(out, exist_ok=True)
    only_record = (a.signoff or a.override) and not a.stage
    if only_record:
        stages = []
        for cid in signoffs:
            if cid in results:
                results[cid] = apply_signoff(cid, results[cid], signoffs)
            else:
                ch = reg["checks"][cid]
                results[cid] = run_check(cid, ch, ctx, signoffs)
    else:
        stages = parse_stages(a.stage) if a.stage else \
            [s for s in STAGES if s in
             [str(x) for x in target.get("stages") or STAGES]]
        for st in stages:
            for cid in used_check_ids(reg):
                ch = reg["checks"][cid]
                if str(ch["stage"]) != st or \
                        not check_applies(ch, target.get("role", "airgap")):
                    continue
                r = run_check(cid, ch, ctx, signoffs)
                results[cid] = r
                print(f"[{r['status']:>4}] stage {st:<8}{cid}: {r['detail']}",
                      flush=True)
    with open(so_path, "w") as fh:
        json.dump(signoffs, fh, indent=2)

    versions.update(ctx.versions)
    rep = build_report(reg, target, version, results, versions, override)
    write_report(out, rep)
    if a.json:
        print(json.dumps(rep, indent=2, sort_keys=True))
    else:
        print(f"verdict: {rep['verdict']}"
              + (" (human override)" if override else "")
              + f"  report: {os.path.relpath(out, ROOT)}/report.md")

    # exit: the rows of the stages this run covered must all pass; the
    # report's verdict is the release-level answer across all stages
    if override:
        return 0
    if only_record:
        return 0 if rep["verdict"] == "GO" else 1
    rows = feature_rows(reg, results, stages, target.get("role", "airgap"))
    bad = [r for r in rows if r["severity"] == "P0" and r["status"] != PASS]
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
