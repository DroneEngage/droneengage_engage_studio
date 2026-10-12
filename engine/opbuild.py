"""opbuild.py - on-demand .deop compile (TASK-23 / SL-0026).

A scenario's `operation:` may name a de.operation/2 source
(ops/<name>.deop) instead of a committed deploy spec. The runner
compiles it on demand with tools/compile_op.js into

    var/build/packages/<key>/

where <key> is sha256(deop bytes + compile_op.js + the .ctx.json
sidecar + the compile args) - editing the .deop (or the compiler, or
the ctx) changes the key and recompiles; an unchanged op hits the same
key and is reused as-is (no node spawn).

The <name>.ctx.json sidecar supplies the compile-time binding
{"binding": {role: [unit, ...]}} and the capability registries
({"registries": {<unit>: <the 1089 caps view>}} - the first entry is
fed to compile_op.js --caps); {"fleet": false} opts the op out of the
--fleet metadata the deploy-spec pipeline needs (the committed
packages show fleet is the default - only intentionally unit-local
ops opt out). Without a ctx the binding is derived from the scenario
fleet: units whose `role:` equals an op role, then unassigned units
in fleet order.
"""

import hashlib
import json
import os
import subprocess

import simlib

COMPILE_OP = os.path.join(simlib.SIM_ROOT, "tools", "compile_op.js")


def _ctx_file(deop):
    return deop[:-len(".deop")] + ".ctx.json"


def build_dir(key):
    return os.path.join(simlib.VAR_DIR, "build", "packages", key)


def _role_count(spec):
    n = (spec or {}).get("count", 1)
    if isinstance(n, dict):
        n = n.get("max", 1)
    try:
        return max(1, int(n))
    except (TypeError, ValueError):
        return 1


def _derive_binding(deop_path, fleet):
    """role -> [unit ids] from the scenario fleet: `role:`-tagged units
    first, then unassigned units in fleet order (the role1..N name+index
    convention covers fleets that just name a unit after its role)."""
    try:
        op = json.loads(simlib.strip_comments(open(deop_path).read()))
    except Exception:
        return {}
    units = [{"id": str(u.get("id") or u.get("name")),
              "role": u.get("role")} for u in fleet or []]
    taken, b = set(), {}
    for r in op.get("roles") or {}:
        b[r] = [u["id"] for u in units
                if u["role"] == r and u["id"] not in taken]
        taken.update(b[r])
    for r, spec in (op.get("roles") or {}).items():
        if b[r]:
            continue
        free = [u["id"] for u in units if u["id"] not in taken]
        b[r] = free[:_role_count(spec)]
        taken.update(b[r])
    return b


def build_deop(deop_path, fleet=None):
    """<op>.deop -> the compiled deploy spec path (compiles or reuses).
    `fleet` is the scenario's unit list - it supplies the role binding
    when the op has no .ctx.json binding."""
    deop_path = os.path.abspath(deop_path)
    if not deop_path.endswith(".deop") or not os.path.isfile(deop_path):
        raise simlib.SimError(f"opbuild: not a .deop file: {deop_path}")
    stem = os.path.basename(deop_path)[:-len(".deop")]
    ctx, ctx_raw = {}, b""
    cf = _ctx_file(deop_path)
    if os.path.isfile(cf):
        ctx_raw = open(cf, "rb").read()
        ctx = json.loads(simlib.strip_comments(ctx_raw.decode()))
    binding = ctx.get("binding") or _derive_binding(deop_path, fleet)
    registries = ctx.get("registries") or {}
    caps_file = None
    if registries:
        # --caps needs a file; stage the first registry's caps view
        # under the build dir so the key below covers it (registries
        # change -> new key -> restaged anyway)
        caps_file = "unit_caps.json"
    else:
        # ctx-less ops resolve against the ops dir's capability view:
        # exactly one <group>.unit.caps.json beside the .deop feeds
        # --caps (two or more means the op needs a ctx to pick one)
        import glob
        cands = sorted(glob.glob(os.path.join(
            os.path.dirname(deop_path), "*.unit.caps.json")))
        if len(cands) == 1:
            caps_file = cands[0]
        elif len(cands) > 1:
            raise simlib.SimError(
                f"opbuild: {deop_path}: {len(cands)} unit.caps.json "
                "files beside it - a .ctx.json registries entry must "
                "pick one")

    argv = ["node", COMPILE_OP, deop_path,
            "--pkg-prefix", stem]
    if binding:
        argv += ["--binding", ";".join(
            f"{r}={','.join(u)}" for r, u in binding.items())]
    if ctx.get("fleet", True):
        argv += ["--fleet"]
    key_src = (open(deop_path, "rb").read() + ctx_raw +
               open(COMPILE_OP, "rb").read() +
               json.dumps(sorted(argv[3:]), sort_keys=True).encode() +
               (open(caps_file, "rb").read()
                if caps_file and caps_file != "unit_caps.json" else b""))
    key = hashlib.sha256(key_src).hexdigest()[:16]
    out = build_dir(key)
    spec = os.path.join(out, f"{stem}.deploy.json")
    manifest = os.path.join(out, ".built")
    if os.path.isfile(manifest) and os.path.isfile(spec):
        _by_name(out)
        print(f"  opbuild: reusing {os.path.relpath(spec)}", flush=True)
        return spec

    os.makedirs(out, exist_ok=True)
    if caps_file == "unit_caps.json":
        caps_file = os.path.join(out, caps_file)
        json.dump(next(iter(registries.values())), open(caps_file, "w"),
                  indent=2)
    if caps_file:
        argv += ["--caps", caps_file]
    argv += ["--out-dir", out]
    r = subprocess.run(argv, cwd=simlib.SIM_ROOT,
                       capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(spec):
        tail = (r.stdout + r.stderr).strip().splitlines()[-8:]
        raise simlib.SimError(
            f"opbuild: compile_op failed on {deop_path} "
            f"(exit {r.returncode})\n    " + "\n    ".join(tail))
    json.dump({"deop": os.path.relpath(deop_path, simlib.SIM_ROOT),
               "key": key, "argv": argv[3:]}, open(manifest, "w"))
    _by_name(out)
    print(f"  opbuild: compiled {os.path.relpath(deop_path)} -> "
          f"{os.path.relpath(spec)}", flush=True)
    return spec


def _by_name(out):
    """var/build/packages/by-name/<file> -> ../<key>/<file> for every
    produced artifact: a stable path for {PKG:}/{HASH:} send tokens
    (they cannot name a hash-keyed dir). Dangling links are pruned."""
    root = os.path.join(os.path.dirname(out), "by-name")
    os.makedirs(root, exist_ok=True)
    for f in os.listdir(root):
        p = os.path.join(root, f)
        if os.path.islink(p) and not os.path.exists(p):
            os.unlink(p)
    for f in os.listdir(out):
        if f == ".built" or not f.endswith(
                (".deploy.json", ".pkg.json", ".opgraph.json")):
            continue
        link = os.path.join(root, f)
        want = os.path.relpath(os.path.join(out, f), root)
        if os.path.islink(link) and os.readlink(link) == want:
            continue
        if os.path.lexists(link):
            os.unlink(link)
        os.symlink(want, link)
