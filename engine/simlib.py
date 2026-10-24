#!/usr/bin/env python3
"""simlib.py - core of droneengage_simulation_land.

Three layers, all data:

  blocks/<name>/       a reusable unit type: which modules it runs, its
                       flight controller (if any) and config overlays per
                       module. Blocks can `extends:` another block.
  sims/<name>.yaml     a simulation: blocks composed into a fleet
                       (count, names, placement, per-unit overrides).
  tests/**/*.yaml      automated test cases: the de.scenario/1 library
                       (engine/scenario_runner.py) - what CI runs.
  scenarios/**/*.yaml  human-run scenarios: demos, examples, anything a
                       user kicks off by hand (same engines).

This module resolves blocks and sims, allocates ports, generates every
config + launch script under var/run/<sim name>/, writes the manifest
(var/run/<name>/sim.json) and holds the journal / pid helpers used by ctl.py
and runner.py. Several sims can be up at once: each owns its own run dir
and pid dir, and `sim up` refuses to start a sim whose ports or unit
names clash with a live one.

Ports (slot k = slot_base + unit index stays the fleet's IDENTITY -
SITL -I k / sysid k / unit index - it no longer derives any port):
every port a sim binds is a named lease from the machine-wide
allocator in port_allocation_management.py (`lease_requests` ->
`lease_commit` before generate, `./sim ports` shows them). A
yaml-explicit port pins; the previous run's ports come back through
the allocator's remembered-port memory; everything else comes from
the free pool. A resolved-but-unleased sim shows the pin/remembered
hint in each field, or None when the allocator has nothing to prefer.
"""

import copy
import glob
import hashlib
import json
import os
import re
import secrets
import socket
import sys
import time
import urllib.parse

import yaml

SIM_ROOT = os.environ.get("SIM_ROOT") or os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))
LOCAL_ENV = os.path.join(SIM_ROOT, "sim.local.env")


def _read_local_env(path):
    """KEY=VALUE lines of the per-machine sim.local.env (gitignored); see
    sim.local.env.example. # comments and blank lines are skipped."""
    out = {}
    if os.path.isfile(path):
        for n, ln in enumerate(open(path), 1):
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            k, sep, v = ln.partition("=")
            if not sep:
                raise SystemExit(f"{path}:{n}: expected KEY=VALUE")
            out[k.strip()] = v.strip().strip('"').strip("'")
    return out


_LOCAL = _read_local_env(LOCAL_ENV)


def setting(key, default=None):
    """Machine path / option: environment variable > sim.local.env >
    default. ~ and $VARS (incl. keys set earlier in the file) expand."""
    v = os.environ.get(key, _LOCAL.get(key))
    if v is None:
        return default
    env = dict(_LOCAL)
    env.update(os.environ)
    v = re.sub(r"\$\{?(\w+)\}?", lambda m: env.get(m.group(1), m.group(0)), v)
    return os.path.expanduser(v)


ENGINE_DIR = os.path.join(SIM_ROOT, "engine")
BLOCKS_DIR = os.path.join(SIM_ROOT, "blocks")
MODULES_DIR = os.path.join(SIM_ROOT, "modules")
TEMPLATES_DIR = os.path.join(MODULES_DIR, "templates")
VAR_DIR = os.path.abspath(setting("SIM_VAR_DIR", os.path.join(SIM_ROOT, "var")))
RUN_DIR = os.path.abspath(setting("SIM_RUN_DIR", os.path.join(VAR_DIR, "run")))
RESULTS_DIR = os.path.abspath(setting("SIM_RESULTS_DIR", os.path.join(VAR_DIR, "results")))

# python bytecode lives under var/pycache, not __pycache__ next to the
# sources: PYTHONPYCACHEPREFIX covers every child process spawned from
# here (ctl, launchers, runners), sys.pycache_prefix covers whatever
# this process imports from now on.
_pycache = os.path.join(VAR_DIR, "pycache")
os.environ.setdefault("PYTHONPYCACHEPREFIX", _pycache)
if hasattr(sys, "pycache_prefix"):
    sys.pycache_prefix = _pycache

# where the code lives on this machine (sim.local.env)
DE_CODE = setting("DE_CODE", os.path.expanduser("~/de_code"))
ARDUPILOT_DIR = setting("ARDUPILOT_DIR", os.path.expanduser("~/code/ardupilot"))
NODE = setting("NODE", "node")


def repo_dir(env_key, name):
    """A DroneEngage repo: its own setting, else $DE_CODE/<name>."""
    return setting(env_key, os.path.join(DE_CODE, name))


COMM_REPO = repo_dir("DE_COMM_DIR", "droneengage_communication")
SERVER_REPO = repo_dir("DE_SERVER_DIR", "droneengage_server")
AUTH_REPO = repo_dir("DE_AUTH_DIR", "droneengage_authenticator")
UDP_RELAY_REPO = repo_dir("DE_UDP_RELAY_DIR", "droneengage_udp_proxy")
# SL-0079: the storage server lives under servers/ in the DE_CODE tree
STORAGE_REPO = repo_dir("DE_STORAGE_DIR",
                        "servers/droneengage_storage_server")
WEBCLIENT_REPO = repo_dir("DE_WEBCLIENT_DIR", "droneengage_webclient_react")
# P5-14: Mavlink3DMap2 lives inside the DE_CODE tree; only needed for
# render modes that launch the viewer
MAVLINK3DMAP_DIR = setting("MAVLINK3DMAP_DIR",
                           os.path.join(DE_CODE, "Mavlink3DMap2"))
# SL-0054 (SL-PG-21): the viewer is versioned gear now - the sim expects
# this `mavlink3dmap` release (backend/package.json `version`, the npm
# package identity; a git tag of the same shape also matches). A
# mismatch warns "expected X, found Y" at render gen and in `doctor` -
# a warning, never a block: render modes degrade, they do not hard-fail.
M3D_REQUIRED_VERSION = setting("SIM_M3D_VERSION", "1.4.0")
# P5-15: the headless render farm is a stock Chromium/Chrome plus the
# sink's per-unit ffmpeg - both machine binaries
CHROMIUM_BIN = setting("CHROMIUM_BIN", "google-chrome")
FFMPEG_BIN = setting("FFMPEG", "ffmpeg")

SLOT_BASE = int(setting("SIM_SLOT_BASE", 10))
HOME = (-35.363262, 149.165237, 584.0, 353.0)   # SITL default home
SPACING_M = 20.0                    # default: units 20 m apart, northward

GCS_PARTY = "GCS_FLEET"
FULL_PERM = 0xFFFFFFFF

# where a webclient is served on this machine (npm start default) - the
# base of the deep link printed by `sim status` / opened by `--open-web`
WEBCLIENT_URL = setting("SIM_WEBCLIENT_URL", "https://localhost:3000").rstrip("/")
# hostname the deep links use for the sim's own 127.0.0.1 server ports.
# Set to "airgap.local" (with a /etc/hosts entry + the ssl_airgap
# root.crt trusted once) to silence every self-signed-cert dialog; the
# ssl_airgap cert's SAN covers airgap.local only, not 127.0.0.1.
# Only local targets are rewritten - external sims keep their own host.
SIM_URL_HOST = setting("SIM_URL_HOST", "127.0.0.1")

# Command-line login overrides (./sim --user/--password/--auth-server):
# they win over the sim yaml, sim.local.env and the defaults. The `sim`
# front-end fills this via cli_auth(); load_server applies it.
CLI_AUTH = {}

DE_COMM_BIN = setting("DE_COMM_BIN", os.path.join(COMM_REPO, "bin", "de_comm"))

# SL-0009: opt-in byte-level link-fault proxy between de_comm and the
# real comm server (server: {type: real, link_proxy: true}). de_comm
# dials the proxy's leased listen port instead of the server wss port;
# the leased `ctrl` port runs the fault control port (fake-server
# line protocol).
#
# TASK-87: opt-in standalone UDP relay (server: {type: real,
# udp_relay: true}). Its wss control port and the UDP pair window are
# leases like everything else - the window is one `block` request
# sized from the fleet's de_mavlink count.

# flight controller kinds: SITL binary, -M model, ArduPilot defaults,
# pymavlink mode table, mode used to arm, how `land` brings it down
FC_KINDS = {
    "copter": {"bin": "arducopter", "model": "quad",
               "defaults": ["Tools/autotest/default_params/copter.parm"],
               "extra": ["sitl_copter.parm"],
               "modes": "mode_mapping_acm", "arm_mode": "GUIDED",
               "land": "LAND"},
    "plane": {"bin": "arduplane", "model": "plane",
              "defaults": ["Tools/autotest/models/plane.parm"],
              "extra": [],
              "modes": "mode_mapping_apm", "arm_mode": "MANUAL",
              "land": "reset"},       # a plane can't LAND in place: restart SITL
    "rover": {"bin": "ardurover", "model": "rover",
              "defaults": ["Tools/autotest/default_params/rover.parm"],
              "extra": [],
              "modes": "mode_mapping_rover", "arm_mode": "GUIDED",
              "land": "HOLD"},
}

NAME_RX = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


class SimError(Exception):
    pass


# ANSI colors for the operator-facing output - plain text when stdout is
# not a tty (piped runs, captured logs)
def col(code, s):
    return f"\033[{code}m{s}\033[0m" if sys.stdout.isatty() else str(s)


def col_green(s):  return col("32", s)     # ok: online, ready, unit up
def col_yellow(s): return col("33", s)     # notices / warnings
def col_red(s):    return col("1;31", s)   # failures
def col_cyan(s):   return col("36", s)     # hosts, ports, URLs
def col_bold(s):   return col("1", s)      # key values: names, pids
def col_dim(s):    return col("2", s)      # progress noise


def print_usage(doc):
    """A module __doc__ printed as CLI help, colorized on a tty: the
    command in green, its <params>/[flags] in yellow, the description in
    white. Shared by `sim` and `ctl` so both help screens look alike."""
    if not sys.stdout.isatty():
        print(doc)
        return
    green, yellow, white, reset = \
        "\033[32m", "\033[33m", "\033[37m", "\033[0m"
    param_rx = re.compile(r"<[^>]*>|\[[^\]]*\]|\.\.\.")
    for line in doc.splitlines():
        # a usage row is "  <cmd><2+ spaces><description>"; a command
        # filling the column has no description on its own line. Anything
        # else (headers, wrapped descriptions, blanks) prints plain white
        m = re.match(r"^(  \S.*?)(?:( {2,})(\S.*))?$", line)
        if not m:
            print(white + line + reset if line.strip() else line)
            continue
        cmd, pad, desc = m.group(1), m.group(2), m.group(3)
        cmd = param_rx.sub(yellow + r"\g<0>" + green, cmd)
        print(green + cmd + reset
              + (pad + white + desc + reset if desc else ""))


# ---------------------------------------------------------------- json/yaml

def strip_comments(text):
    """de_* config files allow // and /* */ comments; drop them outside strings."""
    out, i, n, in_str = [], 0, len(text), False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1]); i += 1
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True; out.append(c)
        elif text.startswith("//", i):
            while i < n and text[i] != "\n":
                i += 1
            continue
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            i = n if j < 0 else j + 2
            continue
        else:
            out.append(c)
        i += 1
    return "".join(out)


def load_config(path):
    try:
        with open(path) as f:
            return json.loads(strip_comments(f.read()))
    except ValueError as e:
        raise SimError(f"{path}: bad JSON: {e}")


def load_yaml(path):
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except OSError as e:
        raise SimError(f"cannot read {path}: {e}")


def deep_merge(base, over):
    """Recursive dict merge; `over` wins. A key set to null in `over` is
    removed. Lists are replaced, not concatenated."""
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if v is None:
            out.pop(k, None)
        elif isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def write_json(path, obj, indent=4):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(obj, f, indent=indent)


def write_text(path, text, mode=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    if mode:
        os.chmod(path, mode)


# ---------------------------------------------------------------- catalog

def load_catalog():
    """modules/catalog.yaml plus every blocks/<name>/catalog.yaml (SL-0030:
    a sensor block carries its own module definition, so a new sensor is
    one folder - no global catalog edit). Bare relative bin/template
    paths in a block fragment resolve inside the block dir; a block
    module name colliding with a global one is an error."""
    cat = load_yaml(os.path.join(MODULES_DIR, "catalog.yaml"))
    for name in ("de_comm", "sitl"):
        if name in cat:
            raise SimError(f"catalog.yaml: '{name}' is built in, remove it")
    for d in list_blocks():
        frag_path = os.path.join(BLOCKS_DIR, d, "catalog.yaml")
        if not os.path.isfile(frag_path):
            continue
        frag = load_yaml(frag_path) or {}
        for name, spec in frag.items():
            if name in cat:
                raise SimError(
                    f"blocks/{d}/catalog.yaml: module {name!r} already "
                    "defined in modules/catalog.yaml")
            spec = dict(spec)
            for k in ("bin", "template"):
                v = spec.get(k)
                if v and not os.path.isabs(v):
                    cand = os.path.join("blocks", d, v)
                    if os.path.isfile(os.path.join(SIM_ROOT, cand)):
                        spec[k] = cand
            cat[name] = spec
    return cat


def module_dir(spec):
    """Repo folder of a catalog module: its `dir_env` setting, else
    $DE_CODE/<repo>."""
    return repo_dir(spec["dir_env"], spec["repo"]) if spec.get("dir_env") \
        else os.path.join(DE_CODE, spec["repo"])


def module_bin(name, spec):
    b = setting(spec["bin_env"]) if spec.get("bin_env") else None
    b = b or spec["bin"]
    return b if os.path.isabs(b) else os.path.join(module_dir(spec), b)


# --module name=path (./sim run|try): per-run binary overrides - the
# agent-iteration hook ("rebuilt de_mavlink, try it on the warm sim").
# They feed the sim hash, so a different override never silently reuses a
# sim running the old binary; ensure() swaps just the overridden procs
# when the fleet spec itself is unchanged.
_MODULE_BINS = {}


def set_module_overrides(overrides):
    """Install {module: abs bin path} for this process. Called by the
    sim CLI before load_sim; {} clears."""
    global _MODULE_BINS
    _MODULE_BINS = dict(overrides or {})


def module_overrides():
    return _MODULE_BINS


def resolve_bin(name, spec=None):
    """The binary a proc would launch with NOW: --module override, else
    the bin_env setting / catalog default (de_comm is built in, not in
    the catalog)."""
    if name in _MODULE_BINS:
        return _MODULE_BINS[name]
    if name == "de_comm":
        return DE_COMM_BIN
    return module_bin(name, spec)


def bin_with_overrides(name, spec, overrides):
    """resolve_bin() as it would have answered with `overrides` installed
    - the binary a manifest was generated with."""
    saved = _MODULE_BINS
    try:
        globals()["_MODULE_BINS"] = dict(overrides or {})
        return resolve_bin(name, spec)
    finally:
        globals()["_MODULE_BINS"] = saved


def swap_proc_bin(u, proc, old_bin, new_bin):
    """Point a generated launch.<proc>.sh at a different binary (the
    warm --module swap path): replace the old path token in place. The
    launch template only ever spells the bin as '<abs path>', so a
    literal replace is exact."""
    path = os.path.join(unit_dir(u), f"launch.{proc}.sh")
    try:
        text = open(path).read()
    except OSError as e:
        raise SimError(f"{u['name']}.{proc}: {e}")
    if old_bin not in text:
        raise SimError(f"{u['name']}.{proc}: launch script does not "
                       f"reference {old_bin}")
    write_text(path, text.replace(old_bin, new_bin), 0o755)


def module_template(spec):
    t = spec["template"]
    for p in (t if os.path.isabs(t) else os.path.join(module_dir(spec), t),
              os.path.join(TEMPLATES_DIR, os.path.basename(t))):
        if os.path.isfile(p):
            return p
    raise SimError(f"config template {t} not found (repo {spec['repo']})")


# ---------------------------------------------------------------- blocks

def list_blocks():
    if not os.path.isdir(BLOCKS_DIR):
        return []
    # a "_" dir is a scaffold template (blocks/_template_sensor/), not a
    # usable block - ./sim new --sensor copies it
    return sorted(d for d in os.listdir(BLOCKS_DIR)
                  if not d.startswith("_") and
                  os.path.isfile(os.path.join(BLOCKS_DIR, d, "block.yaml")))


def load_block(name, _chain=()):
    """Resolve a block with its `extends:` chain. Returns
    {name, description, chain, modules, fc, config{module: overlay},
     parm_files, location}."""
    if name in _chain:
        raise SimError(f"block extends loop: {' -> '.join(_chain + (name,))}")
    bdir = os.path.join(BLOCKS_DIR, name)
    bfile = os.path.join(bdir, "block.yaml")
    if not os.path.isfile(bfile):
        raise SimError(f"unknown block {name!r} (have: {', '.join(list_blocks())})")
    spec = load_yaml(bfile)
    unknown = set(spec) - {"description", "extends", "modules", "add_modules",
                           "remove_modules", "fc", "config", "location"}
    if unknown:
        raise SimError(f"block {name}: unknown keys {sorted(unknown)}")

    if spec.get("extends"):
        b = load_block(spec["extends"], _chain + (name,))
    else:
        b = {"modules": ["de_comm"], "fc": None, "config": {},
             "parm_files": [], "location": None, "chain": []}
    b["name"] = name
    b["description"] = spec.get("description", "").strip()
    b["chain"] = b["chain"] + [name]

    if "modules" in spec:
        b["modules"] = list(spec["modules"])
    b["modules"] += [m for m in spec.get("add_modules", []) if m not in b["modules"]]
    b["modules"] = [m for m in b["modules"] if m not in spec.get("remove_modules", [])]
    if "de_comm" not in b["modules"]:
        raise SimError(f"block {name}: every unit needs de_comm")

    if "fc" in spec:
        b["fc"] = None if spec["fc"] is None else deep_merge(b["fc"] or {}, spec["fc"])

    # overlays: parent first, then this block's <module>.json files, then
    # its inline `config:`
    for f in sorted(os.listdir(bdir)):
        if f.endswith(".json"):
            mod = f[:-5]
            b["config"][mod] = deep_merge(b["config"].get(mod, {}),
                                          load_config(os.path.join(bdir, f)))
    for mod, over in (spec.get("config") or {}).items():
        b["config"][mod] = deep_merge(b["config"].get(mod, {}), over)

    parm = os.path.join(bdir, "sitl.parm")
    if os.path.isfile(parm):
        b["parm_files"] = b["parm_files"] + [parm]
    if "location" in spec:
        b["location"] = spec["location"]
    return b


def validate_block(b, catalog):
    for m in b["modules"]:
        if m != "de_comm" and m not in catalog:
            raise SimError(f"block {b['name']}: unknown module {m!r} "
                           f"(catalog: de_comm, {', '.join(catalog)})")
        if catalog.get(m, {}).get("needs_fc") and not b["fc"]:
            raise SimError(f"block {b['name']}: module {m} needs an `fc:`")
    for m in b["config"]:
        if m not in b["modules"] and m != "sitl":
            raise SimError(f"block {b['name']}: config for {m!r} but the block "
                           f"does not run it (modules: {b['modules']})")
    if b["fc"]:
        kind = b["fc"].get("vehicle")
        if kind not in FC_KINDS:
            raise SimError(f"block {b['name']}: fc.vehicle must be one of "
                           f"{list(FC_KINDS)}, got {kind!r}")


# ---------------------------------------------------------------- sims

SERVER_TYPES = ("real", "fake", "external")


def load_sim(path_or_spec, base_dir=None, overlays=None,
             exclusive=False):
    """Load a sim file (or an inline dict) and expand it into concrete units.
    Returns {name, server{}, home, slot_base, units[], hash}. The name -
    the sim file's stem or an explicit `name:` - owns var/run/<name>/. An
    unnamed spec gets each applied overlay appended (copters3 --with
    faults -> copters3_faults), so variants keep their own var/run dir
    and a changed overlay set always changes the hash."""
    spec, src = _sim_spec(path_or_spec, base_dir)
    applied = []
    for ov in _spec_overlays(spec, overlays):
        spec = _apply_overlay(spec, ov, applied)
    unknown = set(spec) - {"version", "name", "description", "server", "home",
                           "slot_base", "units", "render", "speedup",
                           "fcb_proxy", "bus"}
    if unknown:
        raise SimError(f"sim: unknown keys {sorted(unknown)}")
    if spec.get("version", 1) != 1:
        raise SimError("sim: unsupported version")
    sim_bus = spec.get("bus", "udp")
    if sim_bus not in ("udp", "unix"):
        raise SimError(f"sim: bus must be 'udp' or 'unix', got {sim_bus!r}")
    name = spec.get("name") or \
        (os.path.splitext(os.path.basename(src))[0] if src else "sim")
    if applied and "name" not in spec:
        # a file/inline spec without its own name takes the variant
        # suffix (base_overlay1_overlay2) - variants cannot share a
        # var/run dir and reuse never mixes them
        name += "_" + "_".join(applied)
    if not re.fullmatch(r"[A-Za-z0-9][\w-]*", name):
        raise SimError(f"sim name {name!r}: letters, digits, _ and - only")
    home = tuple(spec.get("home", HOME))
    if len(home) == 3:
        home = home + (HOME[3],)
    # ./sim --slot-base <N|any> (SIM_SLOT_BASE_OVERRIDE) tops the file's
    # slot_base: and the SIM_SLOT_BASE fallback. Ports no longer come
    # from the slot, so "any" simply resolves at the spec's own base -
    # it only still means "I don't pin the identity"
    ov = str(setting("SIM_SLOT_BASE_OVERRIDE", ""))
    slot_base = int(spec.get("slot_base", SLOT_BASE)) \
        if ov in ("", "any") else int(ov)
    srv_spec = spec.get("server") or {}
    sim = {
        "name": name,
        "description": spec.get("description", "").strip(),
        "server": load_server(srv_spec),
        "home": list(home),
        "slot_base": slot_base,
        # SITL --speedup: sim `speedup:` > SIM_SPEEDUP > 1. de_comm's
        # timers stay wall-clock - only a fast first pass, never timing
        # evidence. Scenario runs resolve their own rate in
        # scenario.py (default 3; realtime:/speedup: pin 1) - this
        # fallback serves a direct `sim up` of a sims/ file.
        "speedup": float(spec.get("speedup")
                         or setting("SIM_SPEEDUP", "1")),
        # SL-0008: run the fcb MAVLink proxy on every FC unit; a unit may
        # still override with its own `fcb_proxy:` - not hashed itself,
        # the per-unit fc.fcb_proxy dict it produces is
        "fcb_proxy": bool(spec.get("fcb_proxy", False)),
        # SL-0086: module bus transport per sim (unit `bus:` overrides) -
        # "unix" puts unix-capable modules on per-unit AF_UNIX socket
        # dirs instead of leased s2s UDP ports. Default "udp".
        "bus": sim_bus,
        "units": [],
        "render": _render_spec(spec.get("render")),
    }
    if sim["speedup"] < 1:
        raise SimError("sim: speedup must be >= 1")
    if not sim["server"]["gcs_party"]:
        # parallel external sims share one real server: give each its own
        # GCS party so their bridges don't fight over GCS_FLEET
        sim["server"]["gcs_party"] = f"GCS_{name.upper()}"
    catalog = load_catalog()
    blocks = {}
    if not spec.get("units"):
        raise SimError("sim: no units")
    for entry in spec["units"]:
        if isinstance(entry, str):
            entry = {"block": entry}
        bad = set(entry) - {"block", "name", "count", "at", "config", "fc",
                            "account", "add_modules", "role", "fcb_proxy",
                            "faketime", "udp_proxy", "link_proxy", "bus"}
        if bad:
            raise SimError(f"sim unit {entry}: unknown keys {sorted(bad)}")
        bname = entry["block"]
        if bname not in blocks:
            blocks[bname] = load_block(bname)
            validate_block(blocks[bname], catalog)
        b = blocks[bname]
        count = entry.get("count")
        base = entry.get("name", bname)
        names = [base] if count is None else \
            [f"{base}{i}" for i in range(1, int(count) + 1)]
        for i, uname in enumerate(names):
            at = dict(entry.get("at") or {})
            if count is not None and "at" in entry:
                at["north_m"] = at.get("north_m", 0) + SPACING_M * i
            sim["units"].append(_expand_unit(sim, b, uname, entry, at, catalog))
    seen = set()
    for u in sim["units"]:
        if u["name"] in seen:
            raise SimError(f"sim: duplicate unit name {u['name']!r}")
        seen.add(u["name"])
    sim["port_pins"] = _port_pins(sim)
    sim["hash"] = sim_hash(sim, _MODULE_BINS)
    _apply_hints(sim)
    return sim


def _port_pins(sim):
    """lease name -> the yaml-explicit port it pins (phase 3: the ONLY
    hard values - every other request is prefer/free and lands wherever
    the allocator puts it). Recorded while the resolved fields still
    carry spec values only - before _apply_hints writes the
    remembered-port guesses into them."""
    pins = {}
    srv = sim["server"]
    if srv["type"] == "external":
        if srv["control_port"] is not None:
            pins["bridge.ctrl"] = srv["control_port"]
    else:
        if srv["port"] is not None:
            pins["server.wss"] = srv["port"]
        if srv["control_port"] is not None:
            pins["server.ctrl"] = srv["control_port"]
        ur = srv.get("udp_relay")
        if ur:
            if ur.get("port") is not None:
                pins[UDP_RELAY_LEASE + ".wss"] = ur["port"]
            if ur.get("udp_port_range"):
                pins[UDP_RELAY_LEASE + ".range"] = \
                    int(ur["udp_port_range"][0])
        sto = srv.get("storage")
        if sto and sto.get("port") is not None:
            pins[STORAGE_LEASE] = sto["port"]
    if sim.get("render"):
        for role, key in (("stream", "stream_port"), ("mav", "mav_port"),
                          ("mav_ws", "mav_ws_port"), ("web", "web_port"),
                          ("sink", "sink_port")):
            if sim["render"][key] is not None:
                pins[f"render.{role}"] = sim["render"][key]
    return pins


def _apply_hints(sim):
    """Fill every lease-fed field with the value the allocator would
    most likely hand back - its request's pin, else static prefer hint,
    else the remembered/leased port (allocator.hints) - and record the
    shown values on sim["port_hints"] for the `moved` lines. A field
    stays None when nothing prefers anything (first run on a quiet
    machine) - generated configs only ever see the leased ints."""
    reqs = lease_requests(sim)
    remembered = allocator().hints(sim["name"], SIM_ROOT)
    hints = {}
    for r in reqs:
        v = r.get("pin", r.get("prefer", r.get("path")))
        if v is None:
            v = remembered.get(r["name"])
        hints[r["name"]] = v
    sim["port_hints"] = hints
    _apply_ports(sim, {r["name"]: hints[r["name"]] for r in reqs}, reqs)


def sim_hash(sim, overrides):
    """The fleet-identity hash over the REQUEST, not the allocation
    (SL-0076): the resolved spec with every lease-fed port field
    stripped (explicit yaml pins keep theirs - they're spec, not
    allocation), plus the lease request list - names, protos, kinds,
    sizes and pin values - so request existence/kinds and yaml pins
    still move the hash. A sim re-resolved at another slot_base hashes
    the same, and `run`/`ensure` warm-reuse a fleet that merely moved
    ports. `ensure` rehashes with the manifest's stored overrides to
    tell "same fleet, different binaries" apart from a real spec
    change."""
    return hashlib.sha256(json.dumps(
        [_hash_server(sim["server"]),
         sim["home"],
         [_hash_unit(u) for u in sim["units"]],
         _hash_render(sim.get("render")),
         overrides or {},
         sorted((r["name"], r["proto"], r["kind"],
                 r.get("pin"), r.get("size"))
                for r in lease_requests(sim))],
        sort_keys=True).encode()).hexdigest()[:16]


def _hash_server(srv):
    """srv as spec for the hash: lease-fed port fields (None until the
    commit fills them) -> 0; yaml-explicit values keep theirs."""
    srv = copy.deepcopy(srv)
    if srv["type"] == "external":
        # ip/port/auth_* describe the user's remote stack - spec, not
        # leases; only the local bridge control port is leased
        if srv["control_port"] is None:
            srv["control_port"] = 0
        return srv
    for k in ("port", "control_port", "auth_port", "auth_s2s_port"):
        if srv.get(k) is None:
            srv[k] = 0
    ur = srv.get("udp_relay")
    if ur:
        if ur.get("port") is None:
            ur["port"] = 0
        if ur.get("udp_port_range") is None:
            ur["udp_port_range"] = 0
    sto = srv.get("storage")
    if sto and sto.get("port") is None:
        sto["port"] = 0
    return srv


def _hash_unit(u):
    """u as spec for the hash: slot/sysid and every leased port value
    stripped; names, modules, config, params, positions stay."""
    u = copy.deepcopy(u)
    u.pop("slot", None)
    u["ports"] = {m: 0 for m in u["ports"]}
    if u.get("link_proxy"):
        u["link_proxy"] = {"port": 0, "ctrl": 0}
    fc = u.get("fc")
    if fc:
        for k in ("sysid", "instance", "base_port", "pilot_port",
                  "aux_port", "fcb_port", "pose_port", "pose_serial",
                  "rcin_port", "irlock_port", "sim_in_port",
                  "sim_out_port", "mav_port"):
            fc.pop(k, None)
        if fc.get("fcb_proxy"):
            fc["fcb_proxy"] = {"port": 0, "ctrl": 0}
    # world.prepare_sim's stamped pose feed / sysid are slot- and
    # lease-derived (re-stamped by _apply_ports after the commit)
    for mcfg in (u.get("config") or {}).values():
        stub = mcfg.get("stub") if isinstance(mcfg, dict) else None
        if isinstance(stub, dict):
            stub.pop("sysid", None)
            stub.pop("pose_port", None)
    return u


def _hash_render(render):
    """render as spec for the hash: lease-fed port fields -> 0, explicit
    yaml ports keep theirs; absent render hashes as None."""
    if not render:
        return None
    r = dict(render)
    for key in ("stream_port", "mav_port", "mav_ws_port", "web_port",
                "sink_port"):
        if r[key] is None:
            r[key] = 0
    return r


def _sim_spec(path_or_spec, base_dir=None, _chain=()):
    """Raw sim dict (with `extends: <other sim file>` applied) plus the
    path it came from (None for an inline dict). The parent's keys are
    deep-merged with this file's (a `units` list replaces the parent's,
    so a child can re-use a fleet and only swap the server)."""
    if isinstance(path_or_spec, dict):
        spec, here, src = dict(path_or_spec), base_dir or os.getcwd(), None
    else:
        src = path = os.path.abspath(path_or_spec)
        if path in _chain:
            raise SimError(f"sim extends loop at {path}")
        _chain += (path,)
        spec, here = load_yaml(path), os.path.dirname(path)
    parent = spec.pop("extends", None)
    if not parent:
        return spec, src
    merged, _ = _sim_spec(os.path.join(here, parent), None, _chain)
    return deep_merge(merged, spec), src


# ---------------------------------------------------------------- overlays
# sims/overlays/<name>.yaml - one twist applied on TOP of a resolved sim
# spec (extends: inherits, an overlay overrides). `./sim <cmd> --with a,b`,
# SIM_WITH=a,b, a spec's own `with: [a, b]`, or a scenario's
# `sim: {base: copters3, with: [faults]}` all end up here. A bare name is
# an overlay file; a path (with a separator or .yaml) is read as given.

OVERLAYS_DIR = os.path.join(SIM_ROOT, "sims", "overlays")


def _overlay_path(name):
    p = name if (os.sep in str(name)
                 or str(name).endswith((".yaml", ".yml"))) \
        else os.path.join(OVERLAYS_DIR, str(name) + ".yaml")
    p = os.path.abspath(p)
    if not os.path.isfile(p):
        raise SimError(f"overlay {name!r}: no {p}")
    return p


def _apply_overlay(spec, name, applied, _chain=()):
    """Merge one overlay onto spec (the overlay wins). Keys:
    server/home/slot_base/render/speedup/fcb_proxy - deep-merged like
    extends; units - APPENDED; config - deep-merged into every unit
    entry's config:; fc - likewise into every fc:; with - more overlays
    (applied before this one's own keys... no: overlay order = file
    order, a `with:` inside is applied after its own keys). `applied`
    collects overlay names for the derived sim name."""
    path = _overlay_path(name)
    if path in _chain:
        raise SimError(f"overlay with loop at {path}")
    o = dict(load_yaml(path))
    for drop in ("version", "description", "name"):
        o.pop(drop, None)
    nested = o.pop("with", None) or []
    if isinstance(nested, str):
        nested = [x.strip() for x in nested.split(",") if x.strip()]
    units_extra = o.pop("units", None)
    cfg = o.pop("config", None)
    fc = o.pop("fc", None)
    unknown = set(o) - {"server", "home", "slot_base", "render",
                        "speedup", "fcb_proxy"}
    if unknown:
        raise SimError(f"overlay {name!r}: unknown keys {sorted(unknown)}")
    spec = deep_merge(spec, o)
    if cfg or fc:
        for e in spec.get("units") or []:
            if not isinstance(e, dict):
                continue
            if cfg:
                e["config"] = deep_merge(dict(e.get("config") or {}), cfg)
            if fc:
                e["fc"] = deep_merge(dict(e.get("fc") or {}), fc)
    if units_extra is not None:
        spec["units"] = list(spec.get("units") or []) + list(units_extra)
    applied.append(os.path.splitext(os.path.basename(path))[0])
    for n in nested:
        spec = _apply_overlay(spec, n, applied, _chain + (path,))
    return spec


def _spec_overlays(spec, overlays):
    """The overlay list for one load: the spec's own `with:` (file or
    inline dict), then the explicit arg, else the SIM_WITH env/CLI
    flag. Later entries win."""
    own = spec.pop("with", None) or []
    if isinstance(own, str):
        own = [x.strip() for x in own.split(",") if x.strip()]
    rest = overlays if overlays is not None else \
        [x.strip() for x in str(setting("SIM_WITH", "")).split(",")
         if x.strip()]
    return list(own) + list(rest)


def load_server(srv):
    """server: {type: real|fake|external, port, auth, account{user, code},
    seed, control_port, gcs_party}.

    Every port field is the yaml-explicit value or None - the lease
    commit fills the Nones; a yaml value is the port's pin.

    real (default): the actual droneengage_server from its repo, on the
      sim's own port with a generated config. auth: false runs it in
      local_server_enabled mode (units connect directly, like
      use_local_comm_server); auth: true also starts droneengage_authenticator
      and every unit logs in with the sim account. link_proxy: true
      (auth: false only, SL-0009) inserts engine/link_proxy.py between
      each unit's de_comm and the server wss port - byte-level
      stall/GE/bw/jitter/delay/cut with a per-unit control port; a unit
      may override with `link_proxy: false`.
    fake: engine/fake_fleet_server.py - same routing plus seeded per-link
      loss / delay / cut. Use it for link-fault tests. Extra fake-only
      keys: clock_offset_ms (skew the stamped fleet clock), noclock
      (unit names whose 9007/9005 frames carry no "ts" - they stay
      tq=0, the P4A-10 closed-environment case).
    external: a server stack the user runs himself (e.g.
      servers/sh_start_servers.sh). Nothing is spawned except this sim's
      GCS bridge, which logs in like a GCS client. auth: true (default)
      logs every unit in through the authenticator at
      auth_ip:auth_port; auth: false connects de_comm straight to
      ip:port (the server's local_server_enabled mode). control_port is
      this sim's local ctl port (leased); gcs_party
      defaults to GCS_<sim name> so parallel sims on one server don't
      share a GCS party. `account` is the GCS login (needs the GCS
      permission bit); `unit_account` is the default login for units
      (unit bit only is enough - a unit account cannot login as GCS).
      A unit may carry its own `account: {user, code}` (`{name}` in user
      expands to the unit name). Fallbacks: sim.local.env
      SIM_AUTH_USER / SIM_AUTH_CODE for the GCS, SIM_UNIT_AUTH_USER /
      SIM_UNIT_AUTH_CODE for units, then `account` for units too - so
      credentials stay out of git and single-account sims keep working.
      ./sim --user/--password (CLI_AUTH) top every layer: `account`,
      `unit_account` and per-unit `account:` - one login for the whole
      sim. --auth-server <host[:port]> overrides auth_ip/auth_port on
      external sims."""
    bad = set(srv) - {"type", "ip", "port", "auth", "account",
                      "unit_account", "seed", "clock_offset_ms", "noclock",
                      "auth_ip", "auth_port", "auth_verify_ssl",
                      "control_port", "gcs_party", "link_proxy",
                      "udp_relay", "storage", "persist", "storage_port"}
    if bad:
        raise SimError(f"sim server: unknown keys {sorted(bad)}")
    t = srv.get("type", "real")
    if t not in SERVER_TYPES:
        raise SimError(f"sim server.type must be one of {SERVER_TYPES}")
    if t == "fake" and srv.get("auth"):
        raise SimError("sim server: auth needs type real")
    if srv.get("link_proxy") and t != "real":
        raise SimError("sim server: link_proxy needs type real - it "
                       "relays bytes to a spawned comm server")
    if srv.get("udp_relay") and t != "real":
        raise SimError("sim server: udp_relay needs type real - the "
                       "relay is spawned next to the comm server")
    if (srv.get("storage") or srv.get("persist") or
            srv.get("storage_port") is not None) and t != "real":
        raise SimError("sim server: storage needs type real - the "
                       "storage server is spawned next to the comm server")
    if (srv.get("persist") or srv.get("storage_port") is not None) and \
            not srv.get("storage"):
        raise SimError("sim server: persist/storage_port need storage: true")
    if srv.get("link_proxy") and srv.get("auth"):
        raise SimError("sim server: link_proxy needs auth: false - the "
                       "authenticator hands de_comm the server address at "
                       "login, which cannot be redirected per unit")
    acc = srv.get("account") or {}
    # one CLI credential alone silently pairs with the yaml/env
    # counterpart - a wrong-password login reads as a server failure
    if bool(CLI_AUTH.get("user")) != bool(CLI_AUTH.get("code")):
        raise SimError("--user and --password must be passed together")
    user = CLI_AUTH.get("user") or acc.get("user") or \
        setting("SIM_AUTH_USER", "sim@sim.droneengage.com")
    code = str(CLI_AUTH.get("code") or acc.get("code") or
               setting("SIM_AUTH_CODE", "sim1234"))
    check_account(user, code, "server.account")
    cli_login = bool(CLI_AUTH.get("user") or CLI_AUTH.get("code"))
    auth_on = bool(srv.get("auth", t == "external"))
    if cli_login and not auth_on:
        raise SimError("--user/--password have no effect - server auth "
                       "is off")
    if (CLI_AUTH.get("auth_ip") or CLI_AUTH.get("auth_port")) and \
            (t != "external" or not auth_on):
        raise SimError("--auth-server needs server: {type: external, "
                       "auth: true}")
    # the units' own login - on an external server it is usually a
    # different, unit-only account (a unit account cannot login as GCS).
    # unit_account -> SIM_UNIT_AUTH_* -> account (single-account sims keep
    # working). Only external servers use it: a spawned authenticator
    # knows the single `account` only. --user/--password replace it - one
    # login for the whole sim.
    if cli_login:
        unit_login = {"user": user, "code": code}
    else:
        unit_acc = srv.get("unit_account") or {}
        unit_user = unit_acc.get("user") or setting("SIM_UNIT_AUTH_USER")
        unit_code = str(unit_acc.get("code") or
                        setting("SIM_UNIT_AUTH_CODE", ""))
        if unit_user:
            if not unit_code:
                raise SimError("server.unit_account: user set but no code "
                               "(unit_account.code or SIM_UNIT_AUTH_CODE)")
            check_account(unit_user, unit_code, "server.unit_account")
        unit_login = {"user": unit_user, "code": unit_code} \
            if unit_user else None
    if t == "external":
        return {
            "type": t,
            "ip": srv.get("ip", "127.0.0.1"),
            "port": int(srv.get("port", 9966)),
            "control_port": (int(srv["control_port"])
                             if srv.get("control_port") is not None
                             else None),
            "auth": bool(srv.get("auth", True)),
            "auth_ip": CLI_AUTH.get("auth_ip") or srv.get("auth_ip",
                                                         "127.0.0.1"),
            "auth_port": int(CLI_AUTH.get("auth_port") or
                             srv.get("auth_port", 19408)),
            "auth_verify_ssl": bool(srv.get("auth_verify_ssl", False)),
            "account": {"user": user, "code": code},
            "unit_account": unit_login,
            "gcs_party": srv.get("gcs_party"),
            "seed": 0, "clock_offset_ms": 0, "noclock": [],
            "link_proxy": False,
        }
    port = int(srv["port"]) if srv.get("port") is not None else None
    urelay = srv.get("udp_relay")
    if urelay is True:
        urelay = {}
    elif urelay in (None, False):
        urelay = None
    elif isinstance(urelay, dict):
        bad = set(urelay) - {"port", "udp_port_range", "gcs_peer_policy",
                             "id", "ssl"}
        if bad:
            raise SimError(f"sim server.udp_relay: unknown keys "
                           f"{sorted(bad)}")
    else:
        raise SimError("sim server.udp_relay: must be true or a mapping "
                       "({port, udp_port_range, gcs_peer_policy})")
    if urelay is not None:
        # the relay's wss control port and UDP pair window are leases
        # like everything else; the window sizes at gen time from the
        # fleet's de_mavlink count
        urelay = dict(urelay)
        urelay.setdefault("port", None)
        urelay.setdefault("gcs_peer_policy", "open")
        urelay.setdefault("ssl", True)
    out = {
        "type": t, "ip": "127.0.0.1", "port": port,
        "control_port": (int(srv["control_port"])
                         if srv.get("control_port") is not None else None),
        "auth": bool(srv.get("auth", False)),
        "auth_ip": "127.0.0.1", "auth_port": None,
        "auth_s2s_port": None,
        "auth_verify_ssl": False,
        "account": {"user": user, "code": code},
        "gcs_party": srv.get("gcs_party") or GCS_PARTY,
        "seed": int(srv.get("seed", 1337)),
        "clock_offset_ms": int(srv.get("clock_offset_ms", 0)),
        "noclock": list(srv.get("noclock") or []),
        "link_proxy": bool(srv.get("link_proxy", False)),
        "udp_relay": urelay,
    }
    if srv.get("storage"):
        # SL-0079: a real storage node next to the comm server. persist:
        # keeps the DB across `sim down`/`up` (restart tests); port is
        # a yaml pin or the storage.wss lease.
        out["storage"] = {
            "persist": bool(srv.get("persist", False)),
            "port": (int(srv["storage_port"])
                     if srv.get("storage_port") is not None else None),
        }
    return out


RENDER_MODES = ("viewer", "cameras", "physics", "full")

# P5-13/14: the render side's five ports are allocator leases like
# every other. Two keep a static prefer hint - Mavlink3DMap's own
# defaults, so a viewer launched outside the sim still finds the
# MAVLink websocket and its dev-server page where it looks first.
RENDER_MAV_WS_PREFER = 8811
RENDER_WEB_PREFER = 8080


def _render_spec(raw):
    """render: {mode, headless, stream_port, mav_port, mav_ws_port,
    web_port} -> a normalized dict, or None when the scenario did not
    opt in (D1: absent render means no process, port or param changes -
    the sim hash, generated from the unit specs, stays byte-identical
    to before). The port fields are the yaml-explicit pins; None until
    the lease commit fills them."""
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise SimError("render: must be a mapping")
    bad = set(raw) - {"mode", "headless", "stream_port", "mav_port",
                      "mav_ws_port", "web_port", "sink_port"}
    if bad:
        raise SimError(f"render: unknown keys {sorted(bad)}")
    mode = raw.get("mode", "viewer")
    if mode not in RENDER_MODES:
        raise SimError(f"render: unknown mode {mode!r} "
                       f"(known: {', '.join(RENDER_MODES)})")
    def _p(key):
        return int(raw[key]) if raw.get(key) is not None else None
    return {"mode": mode,
            "headless": bool(raw.get("headless", False)),
            "stream_port": _p("stream_port"),
            "mav_port": _p("mav_port"),
            "mav_ws_port": _p("mav_ws_port"),
            "web_port": _p("web_port"),
            "sink_port": _p("sink_port")}


def render_viewer_url(render, world_stream=True):
    """The printed Mavlink3DMap URL. `world_stream` is only True when a
    de.worldstream/1 feed will serve stream_port - i.e. a scenario with
    a world. The ?world= param selects the DE world scene, whose vehicle
    frame is gated on the stream's hello: advertising it without the
    stream pins vehicles at the origin (attitude still works)."""
    q = (f"world=ws://127.0.0.1:{render['stream_port']}&"
         if world_stream else "")
    return (f"http://127.0.0.1:{render['web_port']}/?{q}"
            f"mavlink=ws://127.0.0.1:{render['mav_ws_port']}")


def check_account(user, code, where):
    """droneengage_authenticator rule (hlp_strings.fn_isValidAccountName /
    fn_isAlphanumeric): a bad name is refused as "API endpoint not found"."""
    if not (re.fullmatch(r"[A-Za-z0-9]+", user)
            or re.fullmatch(r"[^@\s]+@[^@\s]+\.[A-Za-z]{2,}", user)):
        raise SimError(f"{where}.user {user!r}: alphanumeric or an "
                       "e-mail with a TLD (the authenticator refuses others)")
    if not re.fullmatch(r"[A-Za-z0-9]+", code):
        raise SimError(f"{where}.code must be alphanumeric")


def cli_auth(user=None, password=None, auth_server=None):
    """Store the ./sim --user/--password/--auth-server overrides.
    `auth_server` is host[:port] - a bare number is the port, an optional
    http(s):// scheme is dropped."""
    if user:
        CLI_AUTH["user"] = user
    if password:
        CLI_AUTH["code"] = password
    if auth_server:
        s = str(auth_server).split("://", 1)[-1]
        if ":" in s:
            host, _, port = s.rpartition(":")
        elif s.isdigit():
            host, port = "", s          # bare number = port only
        else:
            host, port = s, ""          # bare hostname, default port
        if host:
            CLI_AUTH["auth_ip"] = host
        if port:
            try:
                CLI_AUTH["auth_port"] = int(port)
            except ValueError:
                raise SimError(f"--auth-server {auth_server!r}: bad port")


def webclient_url(sim, page="/", gcs_party=None):
    """Click-to-login webclient deep link for a sim (manifest or resolved
    sim dict - both carry ["name"] and ["server"]).

    page     -> webclient route: "/" for the main view, "/planning" for
                the planner. Same deep link, different page.
    auth on  -> ?email&accesscode&authserver=host:port: the webclient does
                the normal /w/wl login against that authenticator.
    auth off -> ?server=host:port: no login exists (fake server / real
                server in local_server_enabled) - the webclient connects
                straight to the comm server's wss with a bare de_auth.
    The GCS party defaults to GCSWEB_<sim>: deliberately NOT the bridge's
    gcs_party and NOT shared between pages - the comm server kicks a
    connection whose senderID is already online."""
    srv = sim["server"]
    # SIM_URL_HOST rewrites loopback targets (ports bound on this
    # machine - spawned servers, and external stacks that happen to be
    # local); a real remote address is left alone
    def _h(h):
        return SIM_URL_HOST if h in ("127.0.0.1", "localhost", "::1") else h
    q = {"unitName": gcs_party or ("GCSWEB_" + sim["name"]), "connect": "1"}
    if srv["auth"]:
        q["email"] = srv["account"]["user"]
        q["accesscode"] = srv["account"]["code"]
        q["authserver"] = f"{_h(srv['auth_ip'])}:{srv['auth_port']}"
    else:
        q["server"] = f"{_h(srv['ip'])}:{srv['port']}"
    return WEBCLIENT_URL + page + "?" + urllib.parse.urlencode(q, safe=":@")


def mav_chan_index(params, serial):
    """The n in the MAVn_*/SRn_* stream-rate params a SERIAL<serial>
    MAVLink stream answers to: ArduPilot binds one MAVLink channel per
    serial whose protocol is MAVLink (1 or 2), so serial4's n is 1 +
    the count of lower-numbered MAVLink serials. A block re-purposing
    SERIAL1-3 (GPS, None, ...) shifts it; params a unit never set take
    the ArduPilot SITL defaults (AP_SerialManager.cpp: serial0-2
    MAVLink2, serial3/4 GPS, the rest disabled)."""
    defaults = {0: 2, 1: 2, 2: 2, 3: 5, 4: 5}
    n = 1
    for i in range(serial):
        try:
            proto = int(params.get(f"SERIAL{i}_PROTOCOL",
                                   defaults.get(i, -1)))
        except (TypeError, ValueError):
            proto = -1
        if proto in (1, 2):
            n += 1
    return n


def read_parm_files(paths):
    """{NAME: value} from ArduPilot .parm files (`NAME VALUE` or
    `NAME,VALUE`, # comments), later files winning - the same layering
    SITL's --defaults applies. Missing files are skipped."""
    out = {}
    for path in paths:
        try:
            with open(path) as f:
                for ln in f:
                    ln = ln.split("#", 1)[0].strip()
                    if not ln:
                        continue
                    parts = ln.replace(",", " ").split()
                    if len(parts) >= 2:
                        out[parts[0]] = parts[1]
        except OSError:
            continue
    return out


def _unit_sockdir(sim_name, unit_name):
    """The unix-bus unit's socket dir (SL-0086):
    $XDG_RUNTIME_DIR/de_sim/s/<8-char hash of worktree+sim+unit>/ -
    deliberately short: every socket inside must fit AF_UNIX sun_path
    (108 bytes), which the worktree path under var/run/ would blow past.
    Falls back to ~/.cache like the allocator's registry; the dir itself
    is created by de_common at bind time and removed when the lease is
    released (`sim down`)."""
    xdg = os.environ.get("XDG_RUNTIME_DIR")
    base = xdg if xdg else os.path.expanduser("~/.cache")
    h = hashlib.sha256(
        f"{os.path.realpath(SIM_ROOT)}|{sim_name}|{unit_name}"
        .encode()).hexdigest()[:8]
    return os.path.join(base, "de_sim", "s", h)


def _expand_unit(sim, b, name, entry, at, catalog):
    if not NAME_RX.match(name):
        raise SimError(f"unit name {name!r}: letters, digits, _ only")
    slot = sim["slot_base"] + len(sim["units"])
    u = {"name": name, "sim": sim["name"], "block": b["name"],
         "chain": b["chain"], "slot": slot,
         "party_id": name, "unitID": name,
         "modules": list(b["modules"]), "ports": {}, "fc": None,
         "config": copy.deepcopy(b["config"]),
         "parm_files": list(b["parm_files"]),
         "location": None, "role": entry.get("role"),
         # SL-0014: launch de_comm under libfaketime so `ctl clock_step`
         # can skew its clock (FAKETIME_TIMESTAMP_FILE)
         "faketime": bool(entry.get("faketime"))}
    # P5-07: a unit may add modules on top of its block (scenario fleet
    # entries carry `modules:` for the P5-08 stubs). Unknown modules fail
    # here with the catalog list, before ports are allocated.
    for m in entry.get("add_modules", []):
        if m in u["modules"]:
            continue
        if m not in catalog:
            raise SimError(f"unit {name}: unknown add_modules entry {m!r} "
                           f"(catalog: {', '.join(catalog)})")
        if catalog.get(m, {}).get("needs_fc") and not b["fc"]:
            raise SimError(f"unit {name}: module {m} needs an `fc:`")
        u["modules"].append(m)
    for mod, over in (entry.get("config") or {}).items():
        if mod not in u["modules"] and mod != "sitl":
            raise SimError(f"unit {name}: config for {mod!r} but block "
                           f"{b['name']} does not run it")
        u["config"][mod] = deep_merge(u["config"].get(mod, {}), over)
    # P5-15 D7: rendered-camera units get a stable per-sim index; the
    # de_yolo_ai catalog wire turns it into the source/AI card labels
    # (cam_labels - DE-SRC-k/DE-AI-k or the machine's named-VC labels)
    # the module finds by name, and the paired de_camera reads yolo's
    # annotated output device.
    # TASK-EV-CAM: a de_camera unit with no de_yolo_ai is also a
    # rendered camera when the sim runs the frame farm (render.mode
    # cameras|full) - it reads the sink's DE-SRC-k device directly.
    if _is_render_camera_unit(u["modules"], sim):
        k = sum(1 for x in sim["units"]
                if _is_render_camera_unit(x["modules"], sim))
        u["cam_index"] = k
        u["cam_labels"] = cam_labels(k)
        if "de_camera" in u["modules"]:
            # "key" pins the camera.<key>.* event surface name
            feed_label = u["cam_labels"][
                1 if "de_yolo_ai" in u["modules"] else 0]
            u["config"]["de_camera"] = deep_merge(
                {"camera": {"camera_list":
                            [{"name": "CAM",
                              "device_name": feed_label,
                              "key": "cam_0"}]}},
                u["config"].get("de_camera") or {})
    # per-unit account: the drone's own login, like a real vehicle.
    # `{name}` in user expands to the unit name so `count:` fleets get
    # quad1@.., quad2@.. - None falls back to server.account.
    # ./sim --user/--password override it like every other login setting.
    acc = entry.get("account")
    u["account"] = None
    if acc and not (CLI_AUTH.get("user") or CLI_AUTH.get("code")):
        user = str(acc.get("user", "")).replace("{name}", name)
        code = str(acc.get("code") or
                   setting("SIM_AUTH_CODE", "sim1234"))
        check_account(user, code, f"unit {name} account")
        u["account"] = {"user": user, "code": code}
    # every unit-side port is a lease filled by the commit - None until
    # then (u["ports"] keys ARE the unit.<u>.* request names)
    u["ports"]["de_comm"] = None
    # SL-0009: `link_proxy` (server block on a real sim, per-unit
    # override) inserts engine/link_proxy.py between de_comm and the
    # comm server wss port for byte-level link faults
    lp = entry.get("link_proxy")
    if lp is None:
        lp = sim["server"].get("link_proxy")
    elif not isinstance(lp, bool):
        raise SimError(f"unit {name}: link_proxy must be true/false")
    if lp and sim["server"]["type"] != "real":
        raise SimError(f"unit {name}: link_proxy needs "
                       "server: {type: real}")
    u["link_proxy"] = {"port": None, "ctrl": None} if lp else None
    for m in u["modules"]:
        if m != "de_comm":
            u["ports"][m] = None
    for m in entry.get("udp_proxy") or []:
        # SL-0010: a chunk-fault proxy on this module's s2s link - its
        # own leased port; gen_unit then points the module's
        # s2s_udp_target_port at it and the proxy owns the whole
        # module<->de_comm path - de_comm answers the sender's
        # address, so both directions cross it.
        if m == "de_comm" or m not in u["modules"]:
            raise SimError(f"unit {name}: udp_proxy {m!r} is not one of "
                           f"its modules ({u['modules']})")
        u["ports"]["udp_proxy_" + m] = None

    # SL-0086: `bus: unix` puts the unit's module bus on a per-unit
    # AF_UNIX socket dir (a `unit.<u>.sockdir` path lease) instead of
    # per-module s2s UDP ports. Only catalog modules flagged `unix:
    # true` move over - the Python stubs, de_sound, and `udp_proxy:`
    # modules (SL-0010 faults the UDP path) stay on UDP and keep their
    # leases. de_comm always keeps its UDP lease (the broker binds UDP
    # regardless); it also gets the dir so its broker socket lands there.
    bus = entry.get("bus") or sim.get("bus") or "udp"
    if bus not in ("udp", "unix"):
        raise SimError(f"unit {name}: bus must be 'udp' or 'unix', "
                       f"got {bus!r}")
    if bus == "unix":
        u["bus"] = "unix"
        u["sockdir"] = _unit_sockdir(sim["name"], name)
        u["unix_modules"] = [m for m in u["modules"]
                             if m != "de_comm"
                             and catalog.get(m, {}).get("unix")
                             and "udp_proxy_" + m not in u["ports"]]
        for m in u["unix_modules"]:
            del u["ports"][m]

    # position: explicit `at` (meters from home), block `location`, or
    # the default spacing by unit order
    pos = dict(b.get("location") or {})
    pos.update(at)
    if not pos:
        pos = {"north_m": SPACING_M * (slot - sim["slot_base"])}
    lat, lng, alt, yaw = offset_home(sim["home"], pos)
    u["position"] = {"lat": lat, "lng": lng, "alt": alt, "yaw": yaw}

    fc = deep_merge(b["fc"] or {}, entry.get("fc") or {}) if b["fc"] else None
    if entry.get("fc") and not b["fc"]:
        raise SimError(f"unit {name}: `fc:` override but block {b['name']} has no fc")
    # SL-0008: `fcb_proxy: true` (unit or sim level) inserts the MAVLink
    # proxy between de_mavlink and SITL's fcb port. Sim-level applies to
    # FC units only (a static skips it silently); an explicit unit flag
    # on a no-fc unit is a spec error. An explicit `false` opts one unit
    # back out of a sim-level `true`. The proxy ports land in fc.* so
    # enabling it changes the sim hash.
    if entry.get("fcb_proxy") and not fc:
        raise SimError(f"unit {name}: `fcb_proxy` but the block has no fc")
    if fc:
        if not 1 <= slot <= 250:
            raise SimError(f"unit {name}: slot {slot} is not a valid MAVLink sysid")
        kind = FC_KINDS[fc["vehicle"]]
        u["fc"] = {
            "vehicle": fc["vehicle"],
            "model": fc.get("model", kind["model"]),
            "sysid": slot, "instance": slot,
            "speedup": sim["speedup"],
            # all ports are leases - None until the commit fills them.
            # The SITL TCP serials are one 10-port block
            # `unit.<name>.sitl` (pilot = base+0, aux = base+2,
            # pose = base+5, the rest reserved); serial1's fcb, the
            # RC-in UDP socket, the irlock and the sim-link in/out UDP
            # are separate leases.
            "base_port": None,
            "pilot_port": None,
            "aux_port": None,
            "fcb_port": None,
            "rcin_port": None,
            "irlock_port": None,
            "sim_in_port": None,
            "sim_out_port": None,
            "arm_mode": fc.get("arm_mode", kind["arm_mode"]),
            "land": fc.get("land", kind["land"]),
            "defaults": fc.get("defaults", kind["defaults"]),
            "params": fc.get("params") or {},
        }
        # a catalog module flagged `pose` reads the vehicle pose on its
        # own SITL serial (serial5, tcp:5 -> the SITL block's base+5) -
        # the P5-08 stubs use it; serial0/1/2 stay with
        # ctl / de_mavlink / telemetry and serial3 stays the SITL GPS
        # device on every unit. pose_serial marks the want; pose_port
        # is the leased value the commit writes
        if any(catalog.get(m, {}).get("pose") for m in u["modules"]):
            u["fc"]["pose_serial"] = True
            u["fc"]["pose_port"] = None
            u["fc"]["params"] = deep_merge(
                {"SERIAL5_PROTOCOL": 2, "SERIAL5_BAUD": 57},
                u["fc"]["params"])
        # P5-13 D4: a render-enabled sim fans MAVLink out to
        # Mavlink3DMap on serial4 (udpclient -> render.mav_port);
        # attitude/position at 10 Hz, servo outputs for P5-16. Newer
        # ArduPilot renamed SRn_* to MAVn_* (n = MAVLink channel, which
        # is the n-th MAVLink serial - compute it: a block overriding
        # SERIAL1-3_PROTOCOL shifts it) and ignores the old names. Only
        # that channel's names are set - a fixed MAV4_* would raise the
        # rates on whatever other link is channel 4. The protocols come
        # from every layer SITL boots with (ArduPilot defaults, the
        # template/vehicle/block .parm files, then inline params). Rates
        # apply at boot only.
        if sim.get("render"):
            u["fc"]["mav_port"] = sim["render"]["mav_port"]
            parm_layers = read_parm_files(
                [d if os.path.isabs(d) else os.path.join(ARDUPILOT_DIR, d)
                 for d in u["fc"]["defaults"]]
                + [os.path.join(TEMPLATES_DIR, "sitl_common.parm")]
                + [os.path.join(TEMPLATES_DIR, x) for x in kind["extra"]]
                + u["parm_files"])
            eff_params = deep_merge(
                deep_merge(parm_layers, u["fc"]["params"]),
                u["config"].get("sitl"))
            mav_n = mav_chan_index(eff_params, 4)
            u["fc"]["params"] = deep_merge(
                {"SERIAL4_PROTOCOL": 2,
                 f"SR{mav_n}_POSITION": 10, f"SR{mav_n}_EXTRA1": 10,
                 f"SR{mav_n}_RC_CHAN": 2, f"SR{mav_n}_EXT_STAT": 2,
                 f"MAV{mav_n}_POSITION": 10, f"MAV{mav_n}_EXTRA1": 10,
                 f"MAV{mav_n}_RC_CHAN": 2, f"MAV{mav_n}_EXT_STAT": 2},
                u["fc"]["params"])
        px = entry.get("fcb_proxy")
        if px is None:
            px = bool(sim.get("fcb_proxy"))
        if px:
            u["fc"]["fcb_proxy"] = {"port": None, "ctrl": None}
        # world.prepare_sim stamps config.<stub>.stub.{sysid,pose_port}
        # before the fleet resolves and the ports lease - the resolved
        # fc values always win here (a stale stamp leaves the stub
        # dialing a dead port: pose feed refused all run ->
        # level-attitude fallback -> wrong geo projections).
        for m in u["modules"]:
            stub = (u["config"].get(m) or {}).get("stub")
            if isinstance(stub, dict) and "pose_port" in stub:
                stub["sysid"] = u["fc"]["sysid"]
                stub["pose_port"] = u["fc"].get("pose_port") or 0
    if u.get("faketime") and not faketime_lib():
        raise SimError(
            f"unit {u['name']}: `faketime: true` needs libfaketime - "
            "apt install libfaketime, or set SIM_FAKETIME_LIB in "
            "sim.local.env")
    return u


def offset_home(home, pos):
    lat, lng, alt, yaw = home
    import math
    n, e = float(pos.get("north_m", 0)), float(pos.get("east_m", 0))
    lat2 = lat + n / 111320.0
    lng2 = lng + e / (111320.0 * math.cos(math.radians(lat)))
    return (round(lat2, 7), round(lng2, 7), alt + float(pos.get("alt_m", 0)),
            float(pos.get("yaw", yaw)))


# ---------------------------------------------------------------- manifest
# Every sim owns var/run/<name>/: sim.json, pids/, server/, <unit>/, and the
# per-sim state files (deployed.json, op_deploy.json). Sims run in
# parallel: one manifest each, no shared state.

def run_dir(sim):
    return os.path.join(RUN_DIR, sim)


def pids_dir(sim):
    return os.path.join(run_dir(sim), "pids")


def server_dir(sim):
    return os.path.join(run_dir(sim), "server")


def render_dir(sim):
    return os.path.join(run_dir(sim), "render")


def manifest_path(sim):
    return os.path.join(run_dir(sim), "sim.json")


def cert_path(sim):
    return os.path.join(server_dir(sim), "server.crt")


def key_path(sim):
    return os.path.join(server_dir(sim), "server.key")


def deployed_path(sim):
    return os.path.join(run_dir(sim), "deployed.json")


def op_state_path(sim):
    return os.path.join(run_dir(sim), "op_deploy.json")


def sim_names():
    """Sims with a generated manifest (may or may not be running)."""
    if not os.path.isdir(RUN_DIR):
        return []
    return sorted(d for d in os.listdir(RUN_DIR)
                  if os.path.isfile(manifest_path(d)))


def all_manifests():
    return {n: load_manifest(n) for n in sim_names()}


def load_manifest(name=None, required=True):
    """One sim's manifest. With no name: the single generated sim, or an
    error when several exist (callers that can be ambiguous must pass the
    sim name - ctl exposes it as `-s <name>`)."""
    if name is None:
        names = sim_names()
        if len(names) == 1:
            name = names[0]
        elif not names:
            if required:
                sys.exit("no sim manifest under var/run/ - "
                         "run `./sim up <sim.yaml>` first")
            return None
        else:
            sys.exit("several sims generated (" + ", ".join(names) +
                     ") - pick one with `-s <name>`")
    path = manifest_path(name)
    if not os.path.isfile(path):
        if required:
            sys.exit(f"no sim manifest at {path} - "
                     "run `./sim up <sim.yaml>` first")
        return None
    m = json.load(open(path))
    m.setdefault("name", name)
    return m


def live_sim_names():
    """Sims with at least one process still alive."""
    return [n for n in sim_names() if read_pids(n)]


def owner_record():
    """SL-0053 - who brought the sim up: host + login session id. A SID
    outlives the CLI itself (a terminal tab's whole process tree shares
    it), so the holder's own rerun reuses/restarts freely while a
    second session is refused instead of silently downing the fleet."""
    return {"host": socket.gethostname(), "sid": os.getsid(0),
            "pid": os.getpid(), "t_ms": int(time.time() * 1000)}


def _sid_alive(p_sid):
    """The recorded session's leader (the SID doubles as its pid) still
    runs - the cheap proxy for 'that terminal/session is still open'.
    EPERM means alive-but-not-ours to signal; only ESRCH is dead."""
    try:
        os.kill(int(p_sid), 0)
        return True
    except PermissionError:
        return True
    except (OSError, ValueError):
        return False


def foreign_owner(p_manifest):
    """The manifest's `owner` block when it names a DIFFERENT session
    that is still alive on this host; None for same-session use, a
    dead/stale owner and owner-less (pre-SL-0053) manifests."""
    o = (p_manifest or {}).get("owner")
    if not isinstance(o, dict):
        return None
    if o.get("host") != socket.gethostname():
        return None
    sid = o.get("sid")
    if sid in (None, "", os.getsid(0)):
        return None
    return o if _sid_alive(sid) else None


def units_by_name(manifest=None):
    m = manifest or load_manifest()
    return {u["name"]: u for u in m["units"]}


def find_unit(name, sim=None):
    """A unit by name - `sim` pins the search to one manifest (ctl -s),
    otherwise live sims first: a dead sim's manifest keeps its units on
    disk and may reuse the same names, but its launch scripts/configs are
    stale. Unit names may repeat across live sims on disjoint server
    namespaces (check_conflicts allows it), so a name found in several
    live sims is ambiguous - the caller has to pin the sim."""
    manifests = all_manifests()
    if sim:
        for u in manifests.get(sim, {"units": []})["units"]:
            if u["name"] == name:
                return u
        return None
    live = live_sim_names()
    hits = [u for n in live for u in manifests[n]["units"]
            if u["name"] == name]
    if len({u["sim"] for u in hits}) > 1:
        raise SimError(f"ambiguous unit {name!r} - live in sims "
                       + ", ".join(sorted({u["sim"] for u in hits}))
                       + "; pin one with -s <sim>")
    if hits:
        return hits[0]
    for n in sim_names():
        if n in live:
            continue
        for u in manifests[n]["units"]:
            if u["name"] == name:
                return u
    return None


def unit_dir(u):
    return os.path.join(run_dir(u.get("sim") or _single_sim()), u["name"])


def _single_sim():
    names = sim_names()
    if len(names) == 1:
        return names[0]
    if not names:
        raise SimError("no generated sim - run `./sim up <sim.yaml>` first")
    raise SimError("several sims generated (" + ", ".join(names) +
                   ") - pass the sim name")


# Row owner for the relay in `./sim ports`: the lease names of
# udp_relay.<node>.wss / .range (SL-0081). The single real server has one
# implicit relay node, "relay"; no allocator yet, so rows are bind probes.
UDP_RELAY_LEASE = "udp_relay.relay"
# SL-0079: the storage node's wss port (one storage node per sim today;
# the mesh doc's storage.<node>.wss naming lands with it)
STORAGE_LEASE = "storage.wss"


def _udp_relay_range(sim):
    """The relay's resolved [lo, hi] udp_port_range - explicit config or
    the value the `udp_relay.relay.range` block lease wrote back at
    commit; None while the sim is resolved but unleased."""
    ur = sim["server"]["udp_relay"]
    r = ur.get("udp_port_range")
    return [int(r[0]), int(r[1])] if r else None


def _udp_relay_size(sim):
    """The relay window size: the explicit range's span, else sized for
    the fleet's de_mavlink pair count."""
    r = _udp_relay_range(sim)
    if r:
        return r[1] - r[0] + 1
    n_pairs = sum(1 for u in sim["units"] if "de_mavlink" in u["modules"])
    return max(4, 2 * n_pairs)


def used_ports(sim):
    """Every local port a sim binds: unit s2s + SITL ports, and (for
    engine-run servers) the server/auth ports. The external server's own
    port is remote - only its local bridge control_port counts. A
    manifest's flat `ports` map (the lease map lease_commit writes)
    unions in too, so an allocated port always counts even when a
    nested field doesn't carry it (SITL-internal ports). Fields still
    None - an unleased sim's unresolved values - simply don't count."""
    def _is_port(v):
        return isinstance(v, int) and not isinstance(v, bool)
    ports = set()
    srv = sim["server"]
    if _is_port(srv["control_port"]):
        ports.add(srv["control_port"])
    if srv["type"] != "external":
        for v in (srv["port"], srv["auth_port"], srv["auth_s2s_port"]):
            if _is_port(v):
                ports.add(v)
        ur = srv.get("udp_relay")
        if ur:
            if _is_port(ur.get("port")):
                ports.add(ur["port"])
            rng = _udp_relay_range(sim)
            if rng:
                ports.update(range(rng[0], rng[1] + 1))
        sto = srv.get("storage")
        if sto and _is_port(sto.get("port")):
            ports.add(sto["port"])
    if sim.get("render"):
        ports.update(v for v in
                     (sim["render"]["stream_port"],
                      sim["render"]["mav_port"],
                      sim["render"]["mav_ws_port"],
                      sim["render"]["web_port"],
                      sim["render"]["sink_port"]) if _is_port(v))
    for u in sim["units"]:
        ports.update(p for p in u["ports"].values() if _is_port(p))
        if u.get("link_proxy"):
            ports.update(p for p in (u["link_proxy"]["port"],
                                     u["link_proxy"]["ctrl"])
                         if _is_port(p))
        if u["fc"]:
            fc = u["fc"]
            # the whole SITL decade is leased (the `unit.<u>.sitl`
            # block), not just the serials in use
            if _is_port(fc.get("base_port")):
                ports.update(range(fc["base_port"], fc["base_port"] + 10))
            if _is_port(fc.get("fcb_port")):
                ports.add(fc["fcb_port"])
            if _is_port(fc.get("pose_port")):
                ports.add(fc["pose_port"])
            if fc.get("fcb_proxy"):
                ports.update(p for p in (fc["fcb_proxy"]["port"],
                                         fc["fcb_proxy"]["ctrl"])
                             if _is_port(p))
            for key in ("rcin_port", "irlock_port",
                        "sim_in_port", "sim_out_port"):
                if _is_port(fc.get(key)):
                    ports.add(fc[key])
    for v in (sim.get("ports") or {}).values():
        p = v.get("port") if isinstance(v, dict) else v
        if p is not None:
            ports.add(int(p))
    return ports


def port_rows(sim):
    """[(owner, port, proto)] - every local port a sim binds, labeled.
    Same coverage as used_ports(); the relay's udp_port_range is one
    row per port so `sim ports` can show how much of the block is live.
    Fields still None (an unleased sim's unresolved values) produce no
    row."""
    def _is_port(v):
        return isinstance(v, int) and not isinstance(v, bool)
    rows = []
    srv = sim["server"]
    if _is_port(srv["control_port"]):
        rows.append(("server.control", srv["control_port"], "tcp"))
    if srv["type"] != "external":
        if _is_port(srv["port"]):
            rows.append(("server.wss", srv["port"], "tcp"))
        if srv["auth"]:
            for name, p in (("server.auth", srv["auth_port"]),
                            ("server.auth_s2s", srv["auth_s2s_port"])):
                if _is_port(p):
                    rows.append((name, p, "tcp"))
        ur = srv.get("udp_relay")
        if ur:
            if _is_port(ur.get("port")):
                rows.append((f"{UDP_RELAY_LEASE}.wss", ur["port"], "tcp"))
            rng = _udp_relay_range(sim)
            if rng:
                rows += [(f"{UDP_RELAY_LEASE}.range", p, "udp")
                         for p in range(rng[0], rng[1] + 1)]
        sto = srv.get("storage")
        if sto and _is_port(sto.get("port")):
            rows.append((STORAGE_LEASE, sto["port"], "tcp"))
    if sim.get("render"):
        r = sim["render"]
        for name, key in (("render.stream", "stream_port"),
                          ("render.mav", "mav_port"),
                          ("render.mav_ws", "mav_ws_port"),
                          ("render.web", "web_port"),
                          ("render.sink", "sink_port")):
            if _is_port(r[key]):
                rows.append((name, r[key], "tcp"))
    for u in sim["units"]:
        rows += [(f"{u['name']}.{m}", p, "udp")
                 for m, p in u["ports"].items() if _is_port(p)]
        if u.get("link_proxy"):
            rows += [(f"{u['name']}.{n}", p, "tcp")
                     for n, p in (("link_proxy", u["link_proxy"]["port"]),
                                  ("link_proxy.ctrl",
                                   u["link_proxy"]["ctrl"]))
                     if _is_port(p)]
        if u["fc"]:
            fc = u["fc"]
            base = fc.get("base_port")
            if _is_port(base):
                rows += [(f"{u['name']}.sitl.p{i}", base + i, "tcp")
                         for i in range(10)]
            if _is_port(fc.get("fcb_port")):
                rows.append((f"{u['name']}.sitl.fcb", fc["fcb_port"],
                             "tcp"))
            for name, key in (("rcin", "rcin_port"),
                              ("irlock", "irlock_port"),
                              ("sim_in", "sim_in_port"),
                              ("sim_out", "sim_out_port")):
                if _is_port(fc.get(key)):
                    rows.append((f"{u['name']}.sitl.{name}",
                                 fc[key], "udp"))
            if fc.get("fcb_proxy"):
                rows += [(f"{u['name']}.{n}", p, "tcp")
                         for n, p in (("fcb_proxy",
                                       fc["fcb_proxy"]["port"]),
                                      ("fcb_proxy.ctrl",
                                       fc["fcb_proxy"]["ctrl"]))
                         if _is_port(p)]
    return rows


# ------------------------------------------------- port leases (SL-0074)
# The machine-wide allocator in port_allocation_management.py keeps one
# registry shared by every worktree (docs/port_allocation.md). Phase 3
# (SL-0077): the formulas are gone - a request carries NO port value
# unless the yaml wrote one. Only yaml-explicit values pin
# (server.port/control_port, the relay's port/range, render ports);
# the only static prefer hints left are the viewer's own defaults
# (render.mav_ws 8811 / render.web 8080). Re-run stability comes from
# the allocator's remembered ports - the last run's port for the same
# (worktree, sim, name) is tried first on every free/prefer request.
# `sim up` prints `moved <name> <hint> -> <port>` when a hinted value
# did not land. SITL launches with --base-port/--rc-in-port so -I is
# the identity slot only, and sim_hash covers the request set - ports
# no longer change the fleet's identity.

_ALLOCATOR = None


def allocator():
    """The machine-wide port-lease allocator. Registry location:
    SIM_PORT_REGISTRY > $XDG_RUNTIME_DIR/de_sim/ports.json >
    ~/.cache/de_sim/ports.json - deliberately outside var/ so worktrees
    share it. SIM_PORT_LAST names the previous-run port memory;
    SIM_PORT_POOL / SIM_PORT_DENY shape free picks (phase 1)."""
    global _ALLOCATOR
    if _ALLOCATOR is None:
        import port_allocation_management as pam
        _ALLOCATOR = pam.Allocator(
            registry=setting("SIM_PORT_REGISTRY"),
            last=setting("SIM_PORT_LAST"),
            pool=setting("SIM_PORT_POOL"),
            deny=setting("SIM_PORT_DENY"))
    return _ALLOCATOR


def lease_requests(sim):
    """The port requests a resolved sim makes of the allocator, named
    hierarchically (server.*, auth.*, udp_relay.relay.*, render.*,
    unit.<u>.*). Phase 3 kinds (SL-0077): `pin` = yaml-explicit values
    only (recorded on sim["port_pins"]), exact port or fail; `prefer` =
    the viewer's own defaults render.mav_ws/render.web; `free` =
    everything else - the allocator tries the remembered port first,
    then the pool.

    Per-unit SITL serials are ONE 10-port TCP block `unit.<u>.sitl`:
    the launch line gets --base-port <base> and SITL binds pilot at
    base+0, aux at base+2 and pose at base+5; the other members stay
    reserved so no neighbour port can slip under the vehicle. serial1
    (fcb) is its own lease - the launch line passes it absolute
    (`--serial1 tcp:P`), so it does not land inside the block. The SITL
    UDP side leases separately: `unit.<u>.sitl.rcin` (bound
    unconditionally by SITL, passed --rc-in-port) plus
    `unit.<u>.sitl.irlock` / `.sim_in` / `.sim_out` - leased for every
    FC unit so a params- or model-enabled socket (precision-landing
    irlock, external-physics models) can never collide with another
    sim's ports; all are passed as --irlock-port/--sim-port-in/
    --sim-port-out, which -I then never re-derives. sysid stays
    slot-derived (the <= 250 check in _expand_unit): unique inside a
    sim by construction, and the alloc_id space is a phase-4 concern.
    """
    pins = sim.get("port_pins") or {}
    reqs = []

    def req(name, proto, prefer=None, size=None):
        r = {"name": name, "proto": proto}
        if size:
            r["kind"] = "block"
            r["size"] = size
        elif prefer is not None:
            r["kind"] = "prefer"
        else:
            r["kind"] = "pin" if name in pins else "free"
        if name in pins:
            r["pin"] = pins[name]
        elif prefer is not None:
            r["prefer"] = prefer
        return r

    srv = sim["server"]
    if srv["type"] == "external":
        # nothing local but the GCS bridge's control port
        reqs.append(req("bridge.ctrl", "tcp"))
    else:
        # fake too: it takes --ctrl-port now, so its pair is two
        # ordinary leases, not a port+1 block
        reqs.append(req("server.wss", "tcp"))
        reqs.append(req("server.ctrl", "tcp"))
        if srv["type"] == "real" and srv["auth"]:
            reqs.append(req("auth.https", "tcp"))
            reqs.append(req("auth.s2s", "tcp"))
        ur = srv.get("udp_relay")
        if ur:
            reqs.append(req(UDP_RELAY_LEASE + ".wss", "tcp"))
            reqs.append(req(UDP_RELAY_LEASE + ".range", "udp",
                            size=_udp_relay_size(sim)))
        if srv.get("storage"):
            reqs.append(req(STORAGE_LEASE, "tcp"))
    if sim.get("render"):
        reqs.append(req("render.stream", "tcp"))
        reqs.append(req("render.mav", "tcp"))
        # Mavlink3DMap's own defaults - a stock viewer still connects
        reqs.append(req("render.mav_ws", "tcp",
                        prefer=RENDER_MAV_WS_PREFER))
        reqs.append(req("render.web", "tcp", prefer=RENDER_WEB_PREFER))
        reqs.append(req("render.sink", "tcp"))
    for u in sim["units"]:
        prefix = f"unit.{u['name']}."
        reqs += [req(prefix + mod, "udp") for mod in u["ports"]]
        if u.get("sockdir"):
            # SL-0086: the unix-bus unit's socket dir as a path lease -
            # no port picked; the allocator records the dir and removes
            # it on release (`sim down`)
            reqs.append({"name": prefix + "sockdir", "proto": "path",
                         "kind": "path", "path": u["sockdir"]})
        if u.get("link_proxy"):
            reqs += [req(prefix + "link_proxy", "tcp"),
                     req(prefix + "link_proxy.ctrl", "tcp")]
        if u["fc"]:
            # the SITL TCP serial decade as one block (pilot=+0,
            # aux=+2, pose=+5, the rest reserved) - passed --base-port
            reqs.append(req(prefix + "sitl", "tcp", size=10))
            reqs.append(req(prefix + "sitl.fcb", "tcp"))
            for name in ("rcin", "irlock", "sim_in", "sim_out"):
                reqs.append(req(prefix + "sitl." + name, "udp"))
            if u["fc"].get("fcb_proxy"):
                reqs += [req(prefix + "fcb_proxy", "tcp"),
                         req(prefix + "fcb_proxy.ctrl", "tcp")]
    return reqs


def pinned_ports(sim):
    """The ports the resolved sim REQUIRES exactly (every pinned request
    and every pinned block member) - what check_conflicts must find
    free. free/prefer requests take whatever the allocator hands them,
    so they never conflict."""
    ports = set()
    for r in lease_requests(sim):
        if r.get("pin") is None:
            continue
        ports.update(range(r["pin"], r["pin"] + (r.get("size") or 1)))
    return ports


def _lease_conflict_msg(name, conflicts):
    """The usual clash message, sourced from the allocator: ports another
    sim leases read as 'conflicts with running sim' (naming the worktree
    when it is not this one); ports a non-sim process binds read as
    'in use by something else'."""
    foreign = sorted({c["port"] for c in conflicts
                      if c.get("holder") == "foreign"})
    held = {}
    for c in conflicts:
        h = c.get("holder")
        if isinstance(h, dict):
            held.setdefault(h.get("sim", "?"), {}) \
                .setdefault("ports", set()).add(c["port"])
            held[h.get("sim", "?")]["worktree"] = h.get("worktree")
    parts = []
    for other, info in sorted(held.items()):
        wt = info.get("worktree")
        suffix = "" if not wt or os.path.realpath(wt) == os.path.realpath(SIM_ROOT) \
            else f" (worktree {wt})"
        parts.append(
            f"sim {name!r} conflicts with running sim {other!r}{suffix}:"
            f" ports {sorted(info['ports'])} - `./sim down {other}`"
            + (" there first" if suffix else " first"))
    if foreign:
        parts.append(f"ports {foreign} are in use by something else - "
                     "down what holds them (./sim status), or change "
                     "the yaml's explicit port")
    return "; ".join(parts)


def lease_check(sim):
    """Dry-run of lease_commit, run before `sim up` tears down the
    same-named sim: the allocator reports which sim (in any worktree)
    or foreign process already holds each port, and the refusal keeps
    the usual clash message. A same-named sim already running here binds
    its own ports - those 'foreign' reports are ours, not a clash."""
    conflicts = allocator().check(sim["name"], SIM_ROOT,
                                  lease_requests(sim))
    if sim["name"] in live_sim_names():
        m = load_manifest(sim["name"], required=False)
        own = used_ports(m) if m else set()
        ours = os.path.realpath(SIM_ROOT)
        conflicts = [c for c in conflicts
                     if not (c.get("holder") == "foreign" and
                             c["port"] in own) and
                     not (isinstance(c.get("holder"), dict) and
                          c["holder"].get("sim") == sim["name"] and
                          os.path.realpath(
                              c["holder"].get("worktree") or "") == ours)]
    if conflicts:
        raise SimError(_lease_conflict_msg(sim["name"], conflicts))


def lease_commit(sim):
    """Lease every port the resolved sim needs in one atomic
    transaction - the sim either owns its whole port set or errors and
    nothing is reserved (allocator AllocError -> the clash message) -
    then write the picked values back into the nested fields
    (_apply_ports): gen_unit/gen_server read those fields, so the
    launcher calls this BEFORE generate(). The flat {name: port} map is
    kept on sim["ports"] and lands in sim.json (the lease source of
    truth); block members land there too (<name>.p<i>), so used_ports
    sees the reserved runs whole. sim["hash"] deliberately does NOT
    change - it covers the request set (SL-0076), which allocated
    values are no part of. `moved_ports` records the hinted values
    (pins, prefers, remembered ports - what the fields showed) the
    allocator could not hand back - the launcher prints one `moved`
    line per entry."""
    import port_allocation_management as pam
    reqs = lease_requests(sim)
    try:
        with allocator().transaction(sim["name"], SIM_ROOT) as tx:
            for r in reqs:
                if r.get("size"):
                    tx.alloc_block(r["name"], r["size"], proto=r["proto"],
                                   pin=r.get("pin"), prefer=r.get("prefer"))
                else:
                    tx.alloc(r["name"], proto=r["proto"], kind=r["kind"],
                             pin=r.get("pin"), prefer=r.get("prefer"),
                             path=r.get("path"))
            ports = tx.commit()
    except pam.AllocError as e:
        raise SimError(_lease_conflict_msg(sim["name"], e.conflicts))
    sim["ports"] = {}
    for r in reqs:
        if r["kind"] == "path":
            sim["ports"][r["name"]] = {"path": ports[r["name"]],
                                       "proto": "path"}
            continue
        sim["ports"][r["name"]] = {"port": ports[r["name"]],
                                   "proto": r["proto"]}
        for i in range(r.get("size") or 0):
            sim["ports"][f"{r['name']}.p{i}"] = \
                {"port": ports[f"{r['name']}.p{i}"], "proto": r["proto"]}
    hints = sim.get("port_hints") or {}
    sim["moved_ports"] = [
        {"name": r["name"], "hint": hints[r["name"]],
         "port": ports[r["name"]]}
        for r in reqs
        if hints.get(r["name"]) is not None
        and ports[r["name"]] != hints[r["name"]]]
    _apply_ports(sim, ports, reqs)
    return ports


def _apply_ports(sim, ports, requests):
    """The flat lease map back into the nested fields generation reads -
    every field a generated config or launch script uses takes the
    allocated value: server/auth/relay ports, render ports, the module
    s2s map, the proxies, the SITL block base and the members it derives
    (pilot=+0, aux=+2, pose=+5), serial1/fcb, the SITL UDP ports, and
    the stub blocks world.prepare_sim stamped at resolve time. Lease
    names not covered here are block members - the block's base carries
    their value. A name missing from `ports` leaves its field None
    (the map _apply_hints builds leaves unknown values out)."""
    srv = sim["server"]
    if srv["type"] == "external":
        srv["control_port"] = ports.get("bridge.ctrl")
    else:
        srv["port"] = ports.get("server.wss")
        srv["control_port"] = ports.get("server.ctrl")
        if srv["type"] == "real" and srv["auth"]:
            srv["auth_port"] = ports.get("auth.https")
            srv["auth_s2s_port"] = ports.get("auth.s2s")
        ur = srv.get("udp_relay")
        if ur:
            ur["port"] = ports.get(UDP_RELAY_LEASE + ".wss")
            size = next(r["size"] for r in requests
                        if r["name"] == UDP_RELAY_LEASE + ".range")
            lo = ports.get(UDP_RELAY_LEASE + ".range")
            ur["udp_port_range"] = \
                [int(lo), int(lo) + size - 1] if lo is not None else None
        if srv.get("storage"):
            srv["storage"]["port"] = ports.get(STORAGE_LEASE)
    if sim.get("render"):
        r = sim["render"]
        for role, key in (("stream", "stream_port"), ("mav", "mav_port"),
                          ("mav_ws", "mav_ws_port"), ("web", "web_port"),
                          ("sink", "sink_port")):
            r[key] = ports.get(f"render.{role}")
    for u in sim["units"]:
        prefix = f"unit.{u['name']}."
        for mod in u["ports"]:
            u["ports"][mod] = ports.get(prefix + mod)
        if u.get("sockdir") is not None:
            # SL-0086: the leased (deterministic) socket dir - the value
            # the path request carried in
            u["sockdir"] = ports.get(prefix + "sockdir")
        if u.get("link_proxy"):
            u["link_proxy"]["port"] = ports.get(prefix + "link_proxy")
            u["link_proxy"]["ctrl"] = ports.get(prefix + "link_proxy.ctrl")
        fc = u["fc"]
        if fc:
            base = ports.get(prefix + "sitl")
            fc["base_port"] = fc["pilot_port"] = base     # serial0: +0
            fc["aux_port"] = base + 2 if base is not None else None
            fc["fcb_port"] = ports.get(prefix + "sitl.fcb")
            if fc.get("pose_serial"):
                fc["pose_port"] = base + 5 if base is not None else None
            for name, key in (("rcin", "rcin_port"),
                              ("irlock", "irlock_port"),
                              ("sim_in", "sim_in_port"),
                              ("sim_out", "sim_out_port")):
                fc[key] = ports.get(prefix + "sitl." + name)
            if fc.get("mav_port") is not None or "render.mav" in ports:
                fc["mav_port"] = ports.get("render.mav")
            if fc.get("fcb_proxy"):
                fc["fcb_proxy"]["port"] = ports.get(prefix + "fcb_proxy")
                fc["fcb_proxy"]["ctrl"] = \
                    ports.get(prefix + "fcb_proxy.ctrl")
            # the world stubs' stamped pose feed follows the block -
            # _expand_unit marked the want (pose_serial) at resolve time
            for m in u["modules"]:
                stub = ((u.get("config") or {}).get(m) or {}).get("stub")
                if isinstance(stub, dict) and "pose_port" in stub:
                    stub["sysid"] = fc["sysid"]
                    stub["pose_port"] = fc.get("pose_port") or 0


def lease_release(name):
    """Drop every lease this worktree holds for sim `name` - `sim down`
    and `sim clean` both free the registry, and a no-op call is fine
    (a sim whose ports were never leased)."""
    allocator().release(name, SIM_ROOT)


def lease_mark_bound(name):
    """reserved -> bound: the fleet verified up, so the leases now name
    actually-bound ports (a kill -9'd `sim up` leaves `reserved` leases
    the next bring-up reclaims)."""
    allocator().mark_bound(name, None, SIM_ROOT)


def port_bound(port, proto):
    """Whether anything on this host already binds `port` (bind probe;
    TCP SO_REUSEADDR still fails on a LISTENing socket)."""
    s = socket.socket(
        socket.AF_INET,
        socket.SOCK_STREAM if proto == "tcp" else socket.SOCK_DGRAM)
    try:
        if proto == "tcp":
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        return False
    except OSError:
        return True
    finally:
        s.close()


def ports_held(ports):
    """The ports some other process already binds on this host (TCP or
    UDP, any address). The stub/module blocks (54000-60999) sit inside
    Linux's ephemeral range, so a long-lived OS socket (wsdd, a browser)
    can own one - the module then dies on bind and the run fails far
    from the cause. Ports of live sims are the caller's business
    (check_conflicts / the exclusive down)."""
    held = []
    for p in sorted(ports):
        for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
            s = socket.socket(socket.AF_INET, kind)
            try:
                if kind == socket.SOCK_STREAM:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind(("0.0.0.0", p))
            except OSError:
                held.append(p)
                break
            finally:
                s.close()
    return held


def _names_must_be_unique(sim, other):
    """Whether overlapping unit names between sim and the live sim
    `other` are a real conflict. Unit names double as party_id, so they
    must stay unique only where the two fleets could actually meet: on
    a shared `server: {type: external}` stack (party_id is the login
    identity there and auth accounts expand {name}), or on
    rendered-camera units whose v4l2loopback card labels
    (DE-SRC-k/DE-AI-k, per-sim indexed) are machine-global devices. Sims
    spawning their own server (fake or real) keep private party
    registries - the same names never route to each other."""
    return (sim["server"]["type"] == "external" or
            other["server"]["type"] == "external" or
            _has_render_cameras(sim) or _has_render_cameras(other))


def _has_render_cameras(sim):
    """A unit the render frame farm feeds (de_yolo_ai, or de_camera under
    render.mode cameras|full) occupies machine-global v4l2loopback card
    labels - two such fleets would fight over the same /dev devices."""
    return any(_is_render_camera_unit(u.get("modules", []), sim)
               for u in sim["units"])


def check_conflicts(sim):
    """Refuse to bring a sim up when its ports collide with another sim
    that is actually running (a manifest with live pids). Unit names
    may repeat across parallel sims whose servers are spawned per sim
    (fake or real - each owns a private party registry); they stay
    unique only where the fleets could meet - a shared external server
    stack - or share machine-global camera devices
    (_names_must_be_unique). Only the PINNED set counts as a clash -
    a prefer hint landing on a held port is no conflict, the allocator
    moves it into the pool at commit."""
    ports = pinned_ports(sim)
    names = {u["name"] for u in sim["units"]}
    for other in live_sim_names():
        if other == sim["name"]:
            continue
        m = load_manifest(other)
        clash = sorted(ports & used_ports(m))
        dup = (sorted(names & {u["name"] for u in m["units"]})
               if _names_must_be_unique(sim, m) else [])
        if clash or dup:
            raise SimError(
                f"sim {sim['name']!r} conflicts with running sim {other!r}:"
                + (f" ports {clash}" if clash else "")
                + (f" unit names {dup}" if dup else "")
                + (f" - rerun with --shared --slot-base any to park on a "
                   f"free block, or `./sim down {other}` first" if clash
                   else f" - a shared external server or rendered cameras "
                   f"make unit names machine-global; `./sim down {other}` "
                   f"first"))


def journal_dir(u):
    return os.path.join(unit_dir(u), "journal")


def faketime_lib():
    """Path to libfaketime.so.1 used by `faketime: true` units and the
    clock_step fault (SL-0014), or None when libfaketime is not on this
    machine. SIM_FAKETIME_LIB pins a non-standard location."""
    p = setting("SIM_FAKETIME_LIB")
    if p:
        return p if os.path.isfile(p) else None
    hits = glob.glob(os.path.join(VAR_DIR, "faketime", "libfaketime.so.1"))
    hits += glob.glob("/usr/lib/faketime/libfaketime.so.1")
    hits += glob.glob("/usr/local/lib/faketime/libfaketime.so.1")
    hits += glob.glob("/usr/lib/*/faketime/libfaketime.so.1")
    hits += glob.glob("/usr/lib64/faketime/libfaketime.so.1")
    return hits[0] if hits else None


def faketime_rc_path(u):
    """Per-unit FAKETIME_TIMESTAMP_FILE - `ctl clock_step` rewrites it
    at runtime, libfaketime re-reads it."""
    return os.path.join(unit_dir(u), "faketime.rc")


def procs_of(u):
    """Process names of a unit in launch order. fcb_proxy (SL-0008) rides
    between sitl and the modules: it must listen before de_mavlink's
    fcb_connection_uri is dialed. link_proxy (SL-0009) starts before the
    modules too: it is what de_comm dials for the comm server.
    udp_proxy_<m> (SL-0010) goes right before its module so the module's
    s2s dial hits the proxy."""
    out = ["sitl"] if u["fc"] else []
    if u["fc"] and u["fc"].get("fcb_proxy"):
        out.append("fcb_proxy")
    if u.get("link_proxy"):
        out.append("link_proxy")
    for m in u["modules"]:
        if "udp_proxy_" + m in u["ports"]:
            out.append("udp_proxy_" + m)    # SL-0010: proxy before its module
        out.append(m)
    return out


def ready_checks(u, catalog=None):
    """[(module, regex, sink)] that `sim up` waits for. sink "journal"
    matches de_comm journal lines; sink "log" matches the module's own
    proc log (stdout) - for modules like de_yolo_ai whose ready signal
    never reaches the databus."""
    catalog = catalog or load_catalog()
    out = []
    for m in u["modules"]:
        if m == "de_comm" or m not in catalog:
            continue
        spec = catalog[m]
        if spec.get("ready_re"):
            out.append((m, spec["ready_re"], "journal"))
        if spec.get("ready_log_re"):
            out.append((m, spec["ready_log_re"], "log"))
    return out


# ---------------------------------------------------------------- generation

def _fill(v, ctx):
    """Fill "{key}" placeholders in catalog `wire` values. A string that is
    exactly one placeholder takes the value's own type (ports stay ints)."""
    if isinstance(v, dict):
        return {k: _fill(x, ctx) for k, x in v.items()}
    if isinstance(v, list):
        return [_fill(x, ctx) for x in v]
    if isinstance(v, str):
        m = re.fullmatch(r"\{(\w+)\}", v)
        if m and m.group(1) in ctx:
            return ctx[m.group(1)]
        return v.format(**ctx) if "{" in v else v
    return v


def _launch_script(u, proc, cwd, cmd, delay=0, env=()):
    """Launch script that records its own pid then execs the binary."""
    path = os.path.join(unit_dir(u), f"launch.{proc}.sh")
    write_text(path,
               "#!/bin/bash\ncd '%s'\necho $$ > '%s'\n%s%sexec %s\n"
               % (cwd, pid_path(u["sim"], u["name"], proc),
                  f"sleep {delay}\n" if delay else "",
                  "".join(f"export {k}='{v}'\n" for k, v in env), cmd), 0o755)
    return path


def gen_unit(u, sim, catalog):
    udir = unit_dir(u)
    for d in ("journal", "logs"):
        os.makedirs(os.path.join(udir, d), exist_ok=True)
    # a fresh `sim up` must not inherit the previous execution's op state:
    # de_comm restores de_run.json (an unfinished run replays RUNNING and
    # rejects the next deploy with "stop the current run first") and the
    # prepared package/plan ride along. In-scenario `ctl restart` still
    # restores - the files only go away at scenario start.
    for stale in ("de_run.json", "de_package.json", "de_plan.json"):
        p = os.path.join(udir, stale)
        if os.path.isfile(p):
            os.remove(p)
    srv = sim["server"]

    # ---- de_comm ----
    cfg = load_config(os.path.join(TEMPLATES_DIR, "de_comm.json"))
    cfg = deep_merge(cfg, u["config"].get("de_comm"))
    cfg.update({
        "s2s_udp_listening_port": str(u["ports"]["de_comm"]),
        "use_unix_socket": u.get("sockdir") is not None,
        "unitID": u["unitID"],
        "de_plan_file": os.path.join(udir, "de_plan.json"),
        "de_package_file": os.path.join(udir, "de_package.json"),
        "de_run_file": os.path.join(udir, "de_run.json"),
        "de_journal_dir": journal_dir(u) + "/",
    })
    if u.get("sockdir") is not None:
        # SL-0086: the unit's own socket dir - the broker binds
        # <sockdir>/de_comm_broker.sock instead of the machine-wide
        # /run|/tmp/de_comm one
        cfg["unix_socket_dir"] = u["sockdir"]
    if srv["auth"]:
        # full login: authenticator -> comm server address + temp key.
        # The unit's own account, else the sim's unit_account, else the
        # (GCS) account
        acc = u.get("account") or srv.get("unit_account") or srv["account"]
        cfg.update({"use_local_comm_server": False,
                    "auth_ip": srv["auth_ip"], "auth_port": srv["auth_port"],
                    "auth_verify_ssl": srv["auth_verify_ssl"],
                    "userName": acc["user"],
                    "accessCode": acc["code"]})
    else:
        # a proxied unit dials its link_proxy port; the proxy relays the
        # TLS stream to the real server untouched. Only this unit's
        # de_comm is redirected - the GCS bridge and other units keep the
        # direct path.
        cfg.update({"use_local_comm_server": True,
                    "local_comm_server_ip": srv["ip"],
                    "local_comm_server_port":
                        u["link_proxy"]["port"]
                        if u.get("link_proxy") else srv["port"]})
    if not u["fc"]:
        p = u["position"]
        cfg["location"] = {
            "lat": int(round(p["lat"] * 1e7)), "lng": int(round(p["lng"] * 1e7)),
            "alt_abs": int(p["alt"] * 1000), "alt_rel": 0,
            "heading": p["yaw"], "acc_mm": 1000,
            # TASK-SU-06: de_comm only projects the configured location
            # into the 1 Hz static 9102 when the object declares itself
            # fixed - every no-FC sim unit is one (mast/relay/GCS node)
            "static": True,
        }
    cdir = os.path.join(udir, "de_comm")
    comm_cfg = os.path.join(cdir, "de_comm.config.json")
    write_json(comm_cfg, cfg)
    comm_local = os.path.join(cdir, "de_comm.local")
    write_json(comm_local, {
        "module_key": f"{u['name']}-comm",
        "party_id": u["party_id"], "unitID": u["unitID"],
        "unitDescription": f"sim unit {u['name']} (block {u['block']})",
        "userName": cfg.get("userName", "sim@sim.droneengage.com"),
        "accessCode": cfg.get("accessCode", "sim1234"),
        "auth_ip": cfg.get("auth_ip", "127.0.0.1"),
        "auth_port": cfg.get("auth_port", 19408),
    }, indent=1)
    # P4F-03: a per-unit boot id file stands in for the kernel boot id -
    # `ctl bootid` rewrites it so a de_comm restart reads as a reboot
    # (pending TTL signals drop, sticky pending + stored facts survive)
    boot_id = os.path.join(udir, "boot_id")
    if not os.path.isfile(boot_id):
        write_text(boot_id, "boot-1\n")
    # DE_BOARD_ID names the unit as its own board - without it every
    # module reports the host /etc/machine-id, and the GCS CPU summary /
    # per-board views show a 32-char hash instead of "cam1".
    env = [("DE_BOOT_ID_FILE", boot_id), ("DE_BOARD_ID", u["name"])]
    if u.get("faketime"):
        # SL-0014: run de_comm under libfaketime. FAKETIME_TIMESTAMP_FILE
        # holds a signed relative offset ("+3600") that ctl clock_step
        # rewrites, so a runtime step needs no relaunch. A short cache
        # lets the step land within a second or two.
        lib = faketime_lib()
        if not lib:
            raise SimError(
                f"unit {u['name']}: `faketime: true` needs libfaketime - "
                "apt install libfaketime, or set SIM_FAKETIME_LIB in "
                "sim.local.env")
        rc = faketime_rc_path(u)
        if not os.path.isfile(rc):
            write_text(rc, "+0\n")
        env += [("LD_PRELOAD", lib),
                ("FAKETIME_TIMESTAMP_FILE", rc),
                ("FAKETIME_CACHE_DURATION", "2")]
    _launch_script(u, "de_comm", cdir,
                   f"'{resolve_bin('de_comm')}' -c '{comm_cfg}'"
                   f" -b '{comm_local}'",
                   env=tuple(env))

    # ---- link proxy (SL-0009) ----
    # de_comm's generated server address already points here; the proxy
    # relays TLS bytes to the real comm server and drives the shared
    # link_model faults (stall/GE/bw/jitter/delay/cut) per direction.
    if u.get("link_proxy"):
        pdir = os.path.join(udir, "link_proxy")
        os.makedirs(pdir, exist_ok=True)
        _launch_script(
            u, "link_proxy", pdir,
            "'%s' -u '%s' --unit %s --listen %d --ctrl %d "
            "--target %s:%d --statedir '%s' --seed %d"
            % (sys.executable, os.path.join(ENGINE_DIR, "link_proxy.py"),
               u["name"], u["link_proxy"]["port"], u["link_proxy"]["ctrl"],
               srv["ip"], srv["port"], pdir, srv["seed"]))

    # ---- generic s2s modules ----
    ctx = {"unit": u["name"], "comm_port": u["ports"]["de_comm"],
           "cam_index": u.get("cam_index", 0),
           "src_label": u.get("cam_labels", ("", ""))[0],
           "ai_label": u.get("cam_labels", ("", ""))[1]}
    if u["fc"]:
        ctx.update({k: u["fc"][k] for k in ("fcb_port", "pilot_port", "sysid")})
        # SL-0008: where de_mavlink actually dials the FC - the proxy's
        # listen port when the unit runs one, else the SITL fcb port
        ctx["fcb_connect_port"] = (u["fc"].get("fcb_proxy") or {}) \
            .get("port", u["fc"]["fcb_port"])
    for m in u["modules"]:
        if m == "de_comm":
            continue
        spec = catalog[m]
        # normpath: DE_CODE ends with "/" so $DE_CODE/<repo> renders
        # "dir//dir"; inside a JSON string de_common's comment stripper
        # then eats the line from "//" on and the module fails to parse
        ctx["module_dir"] = os.path.normpath(module_dir(spec))
        if m == "de_precland":
            # the generated scene library the wire points test_source +
            # camera at: DE_PRECLAND_SCENES, else the repo's own
            # tools/sim_scenes
            ctx["precland_scenes"] = os.path.normpath(setting(
                "DE_PRECLAND_SCENES",
                os.path.join(module_dir(spec), "tools", "sim_scenes")))
        mdir = os.path.join(udir, m)
        mcfg = load_config(module_template(spec))
        mcfg = deep_merge(mcfg, spec.get("set"))
        mcfg = deep_merge(mcfg, u["config"].get(m))
        # SL-0010: a proxied module aims at its udp_proxy port instead
        # of de_comm's - engine wiring, so block/unit overlays cannot
        # break it either way
        target = u["ports"].get("udp_proxy_" + m, u["ports"]["de_comm"])
        unix_mod = m in (u.get("unix_modules") or ())
        mcfg.update({
            "s2s_udp_target_ip": "127.0.0.1",
            "s2s_udp_target_port": str(target),
            "s2s_udp_listening_ip": "127.0.0.1",
            # SL-0086: a unix-bus module leases no s2s port - "0" lets
            # the module's UDP fallback (dir rejected / bind failure)
            # take an ephemeral port instead of pretending a lease
            "s2s_udp_listening_port": "0" if unix_mod
                                      else str(u["ports"][m]),
            "use_unix_socket": unix_mod,
        })
        if unix_mod:
            mcfg["unix_socket_dir"] = u["sockdir"]
        mcfg = deep_merge(mcfg, _fill(spec.get("wire") or {},
                                      dict(ctx,
                                           port=u["ports"].get(m) or 0)))
        if "udp_proxy_" + m in u["ports"]:
            # a catalog `wire:` s2s_udp_target_port would win that merge -
            # the proxy keeps the last word
            mcfg["s2s_udp_target_port"] = str(target)
        cfg_path = os.path.join(mdir, f"{m}.config.json")
        write_json(cfg_path, mcfg)
        if "udp_proxy_" + m in u["ports"]:
            # SL-0010: the pass-through default - ctl udp_chunks flips
            # the knobs at fault time. The cmdline carries ENGINE_DIR
            # so pid_is_ours claims it.
            pdir = os.path.join(udir, "udp_proxy_" + m)
            os.makedirs(pdir, exist_ok=True)
            _launch_script(
                u, "udp_proxy_" + m, pdir,
                "python3 -u '%s' --listen '127.0.0.1:%d' "
                "--upstream '127.0.0.1:%d' --peer '127.0.0.1:%d' "
                "--label '%s.%s'"
                % (os.path.join(ENGINE_DIR, "udp_proxy.py"),
                   u["ports"]["udp_proxy_" + m],
                   u["ports"]["de_comm"], u["ports"][m],
                   u["name"], m))
        local = spec.get("local") or {}
        cmd = f"'{resolve_bin(m, spec)}' -c '{cfg_path}'"
        if local:
            lpath = os.path.join(mdir, local["file"])
            write_json(lpath, {"module_key": f"{u['name']}-{m}"}, indent=1)
            if local.get("arg"):
                cmd += f" {local['arg']} '{lpath}'"
        # P5-15 ordering: exclusive_caps loopbacks only advertise
        # CAPTURE once a producer is attached, and both consumers abort
        # on an output-only device - probe a frame first. yolo's source
        # is primed by render_sink (black frames until real frames come);
        # de_camera's input is produced by yolo itself. Missing label ->
        # gen_render errors before launch
        # de_camera waits on its configured source: the annotated AI
        # device when a de_yolo_ai produces it, else the render sink's
        # own src device (TASK-EV-CAM)
        wait_idx = {"de_yolo_ai": 0,
                    "de_camera": 1 if "de_yolo_ai" in u["modules"] else 0}
        if m in wait_idx and u.get("cam_labels"):
            wait_dev = v4l2_label_map().get(u["cam_labels"][wait_idx[m]])
            if wait_dev:
                # one bash -c: `exec` in the launch template applies to
                # the whole line, and exec-until is not a thing
                cmd = (f"bash -c \"until timeout 3 '{FFMPEG_BIN}' "
                       f"-loglevel error -f v4l2 -i '{wait_dev}' "
                       f"-frames:v 1 -f null - 2>/dev/null; "
                       f"do sleep 2; done; exec {cmd}\"")
        _launch_script(u, m, mdir, cmd, spec.get("start_delay", 0),
                       env=(("DE_BOARD_ID", u["name"]),))

    # ---- SITL ----
    if u["fc"]:
        fc, p = u["fc"], u["position"]
        kind = FC_KINDS[fc["vehicle"]]
        sdir = os.path.join(udir, "sitl")
        os.makedirs(sdir, exist_ok=True)
        # generated parm: common + vehicle extras + block sitl.parm files +
        # inline fc.params (later lines win in ArduPilot's --defaults)
        lines = [f"# generated for {u['name']} (block chain {' > '.join(u['chain'])})"]
        for f in ([os.path.join(TEMPLATES_DIR, "sitl_common.parm")]
                  + [os.path.join(TEMPLATES_DIR, x) for x in kind["extra"]]
                  + u["parm_files"]):
            lines += [f"# --- {f}", open(f).read().rstrip()]
        merged_params = deep_merge(fc["params"], u["config"].get("sitl"))
        if merged_params:
            lines.append("# --- block/unit fc.params")
            lines += [f"{k} {v}" for k, v in merged_params.items()]
        parm = os.path.join(sdir, "sim.parm")
        write_text(parm, "\n".join(lines) + "\n")
        defaults = ",".join([d if os.path.isabs(d) else os.path.join(ARDUPILOT_DIR, d)
                             for d in fc["defaults"]] + [parm])
        sitl_bin = os.path.join(ARDUPILOT_DIR, "build", "sitl", "bin", kind["bin"])
        # SITL kills itself when its parent dies (SITL_State kill(ppid,0)) -
        # run it as a child and wait, keeping this shell as its parent.
        # SL-0076: --base-port/--rc-in-port and the sim/irlock ports carry
        # the leased values BEFORE -I, so the instance is only the
        # eeprom/identity slot (it no longer shifts any port). serial0/2/5
        # stay relative inside the leased block (tcp:0 -> base, tcp:2 ->
        # base+2, tcp:5 -> base+5, pose only on `pose`-flagged units for
        # the P5-08 stubs); serial1 carries its own fcb lease absolute
        # (always >1000 so SITL does not rebase it). serial4 is the P5-13
        # render fan-out (udpclient to the render.mav_port).
        serial5 = " --serial5 tcp:5" if fc.get("pose_port") else ""
        serial4 = (" --serial4 udpclient:127.0.0.1:%d" % fc["mav_port"]
                   if fc.get("mav_port") else "")
        speedup = (" --speedup %g" % fc["speedup"]
                   if fc.get("speedup", 1) != 1 else "")
        write_text(os.path.join(udir, "launch.sitl.sh"),
                   "#!/bin/bash\ncd '%s'\n'%s' -w -M %s "
                   "--base-port %d --rc-in-port %d --sim-port-in %d "
                   "--sim-port-out %d --irlock-port %d -I %d --sysid %d "
                   "--home %.7f,%.7f,%.1f,%.0f --defaults '%s' "
                   "--serial0 tcp:0 --serial1 tcp:%d --serial2 tcp:2%s%s%s &\n"
                   "child=$!\necho $child > '%s'\nwait $child\n"
                   % (sdir, sitl_bin, fc["model"],
                      fc["base_port"], fc["rcin_port"], fc["sim_in_port"],
                      fc["sim_out_port"], fc["irlock_port"],
                      fc["instance"], fc["sysid"],
                      p["lat"], p["lng"], p["alt"], p["yaw"], defaults,
                      fc["fcb_port"], serial5, serial4, speedup,
                      pid_path(u["sim"], u["name"], "sitl")), 0o755)

        # ---- fcb proxy (SL-0008) ----
        # Opt-in MAVLink proxy between de_mavlink and serial1: it listens
        # on fc.fcb_proxy.port and forwards to fcb_port; ctl drives its
        # fault control port (fc.fcb_proxy.ctrl). A plain daemon like the
        # modules: exec'd, own pidfile, killed by `sim down`. --statedir
        # keeps the sim's var/run path on its cmdline for stray-pid sweeps.
        px = fc.get("fcb_proxy")
        if px:
            pdir = os.path.join(udir, "fcb_proxy")
            os.makedirs(pdir, exist_ok=True)
            _launch_script(
                u, "fcb_proxy", pdir,
                "'%s' -u '%s' --unit %s --listen %d --ctrl %d "
                "--target 127.0.0.1:%d --statedir '%s' --seed %d"
                % (sys.executable, os.path.join(ENGINE_DIR, "fcb_proxy.py"),
                   u["name"], px["port"], px["ctrl"], fc["fcb_port"],
                   pdir, u["slot"]))
    u.pop("config", None)
    u.pop("parm_files", None)


def _node_script(sim, repo, name, cfg):
    """Launch script for a node server: run in its repo (node_modules,
    plugins and the --config path are resolved there); the extra
    --sim_run arg is ignored by the server but marks the pid as ours."""
    rel = os.path.relpath(cfg, repo)
    path = os.path.join(server_dir(sim["name"]), f"launch.{name}.sh")
    write_text(path,
               "#!/bin/bash\ncd '%s'\necho $$ > '%s'\n"
               "exec '%s' ./src/server.js --config=%s --sim_run=%s\n"
               % (repo, pid_path(sim["name"], "server", name), NODE, rel,
                  run_dir(sim["name"])), 0o755)
    return path


def _bridge_script(sim):
    """gcs_bridge launch script: the bridge gets its sim name on argv and
    reads the manifest itself (server address, control port, GCS party)."""
    write_text(os.path.join(server_dir(sim["name"]), "launch.gcs_bridge.sh"),
               "#!/bin/bash\necho $$ > '%s'\nsleep 2\nexec python3 -u '%s' '%s'\n"
               % (pid_path(sim["name"], "server", "gcs_bridge"),
                  os.path.join(ENGINE_DIR, "gcs_bridge.py"), sim["name"]),
               0o755)
    return ["gcs_bridge"]


def gen_server(sim):
    """Write the server side: configs + launch scripts in
    var/run/<sim>/server/. Returns the server process names in launch
    order. An external server spawns nothing but this sim's GCS bridge."""
    srv = sim["server"]
    sdir = server_dir(sim["name"])
    os.makedirs(sdir, exist_ok=True)
    procs = []
    if srv["type"] == "external":
        return _bridge_script(sim)
    if srv["type"] == "fake":
        extra = ""
        if srv["clock_offset_ms"]:
            extra += " --clock-offset %d" % srv["clock_offset_ms"]
        if srv["noclock"]:
            extra += " --noclock " + ",".join(srv["noclock"])
        write_text(os.path.join(sdir, "launch.fake_server.sh"),
                   "#!/bin/bash\necho $$ > '%s'\nexec python3 -u '%s' --port %d "
                   "--ctrl-port %d --seed %d --cert '%s' --key '%s' "
                   "--sim '%s'%s --pretty\n"
                   % (pid_path(sim["name"], "server", "fake_server"),
                      os.path.join(ENGINE_DIR, "fake_fleet_server.py"),
                      srv["port"], srv["control_port"], srv["seed"],
                      cert_path(sim["name"]),
                      key_path(sim["name"]), sim["name"], extra), 0o755)
        return ["fake_server"]

    ssl_dir = os.path.join(SERVER_REPO, "ssl_local")
    if srv["auth"]:
        acfg = load_config(os.path.join(AUTH_REPO, "server.config"))
        acfg.update({
            "server_ip": "127.0.0.1", "server_port": srv["auth_port"],
            "account_storage_type": "single",
            "single_account_user_name": srv["account"]["user"],
            "single_account_access_code": srv["account"]["code"],
            "s2s_ws_listening_ip": "127.0.0.1",
            "s2s_ws_listening_port": str(srv["auth_s2s_port"]),
            "s2s_auth_enabled": False,
            "enable_SSL": True,
            "ssl_key_file": os.path.join(AUTH_REPO, "ssl_local/ssl_airgap/domain.key"),
            "ssl_cert_file": os.path.join(AUTH_REPO, "ssl_local/ssl_airgap/domain.crt"),
            "webadmin_enable": False, "enableLog": False,
            "dbdatabase": os.path.join(sdir, "auth.db"),
            "file_db": os.path.join(sdir, "auth_file_db.json"),
        })
        cfg = os.path.join(sdir, "de_auth.config")
        write_json(cfg, acfg)
        _node_script(sim, AUTH_REPO, "de_auth", cfg)
        procs.append("de_auth")

    scfg = load_config(os.path.join(SERVER_REPO, "server.config"))
    scfg.update({
        "server_ip": "127.0.0.1", "server_port": srv["port"],
        "public_host": "127.0.0.1",
        "local_server_enabled": not srv["auth"],
        "s2s_ws_target_ip": "127.0.0.1",
        # no authenticator -> no auth.s2s lease: 0 keeps the target a
        # valid URL that refuses, like the pre-allocator dead port did
        "s2s_ws_target_port": str(srv["auth_s2s_port"] or 0),
        "ssl_key_file": os.path.join(ssl_dir, "ssl_airgap/domain.key"),
        "ssl_cert_file": os.path.join(ssl_dir, "ssl_airgap/domain.crt"),
        "ca_cert_path": os.path.join(ssl_dir, "ssl_airgap/root.crt"),
        "s2s_my_private_key": os.path.join(ssl_dir, "DE_ServerComm_private.pem"),
        "command_plugin": [],
        # SL-0079: on only when server.storage requests a real storage
        # node - the storage block below fills host/port + shared secret
        "enable_storage_server": bool(srv.get("storage")),
        "allow_udpproxy_fixed_port": False,
        "ignoreLog": True,
        "log_directory": os.path.join(sdir, "logs") + "/",
    })

    urelay = srv.get("udp_relay")
    if urelay:
        # TASK-87: the standalone relay runs as a third server proc. The
        # comm server and the relay share a per-run HMAC secret file; the
        # comm server then routes 9008 pair opens to it (`udp_relay` in
        # its config). Started before de_server so the client connects
        # on the first try.
        if not os.path.isdir(
                os.path.join(UDP_RELAY_REPO, "node_modules", "ws")):
            raise SimError(f"{UDP_RELAY_REPO}: run `npm install` there "
                           "first (the relay needs its ws dependency)")
        secret_path = os.path.join(sdir, "udp_relay_s2s_secret.key")
        write_text(secret_path, secrets.token_hex(32) + "\n")
        os.chmod(secret_path, 0o600)
        lo = _udp_relay_range(sim)
        # the manifest records the resolved range - the
        # udp_proxy_mavlink check gates relay-vs-builtin on it
        urelay["udp_port_range"] = [int(lo[0]), int(lo[1])]
        rcfg = load_config(os.path.join(UDP_RELAY_REPO, "server.config"))
        rcfg.update({
            "server_id": urelay.get("id") or "DE_UdpRelay_" + sim["name"],
            "server_ip": "127.0.0.1",
            "server_port": urelay["port"],
            "enable_SSL": bool(urelay.get("ssl", True)),
            "ssl_key_file": os.path.join(
                UDP_RELAY_REPO, "ssl_local/ssl_airgap/domain.key"),
            "ssl_cert_file": os.path.join(
                UDP_RELAY_REPO, "ssl_local/ssl_airgap/domain.crt"),
            "s2s_cert_enabled": False,
            "s2s_shared_secret_file": secret_path,
            "udp_bind_ip": "0.0.0.0",
            "public_host": "127.0.0.1",
            "udp_port_range": [int(lo[0]), int(lo[1])],
            "gcs_peer_policy": urelay["gcs_peer_policy"],
            "state_file": os.path.join(sdir, "udp_relay_state.json"),
            "log_directory": os.path.join(sdir, "logs") + "/",
        })
        cfg = os.path.join(sdir, "udp_relay.config")
        write_json(cfg, rcfg)
        _node_script(sim, UDP_RELAY_REPO, "udp_relay", cfg)
        procs.append("udp_relay")
        scfg["udp_relay"] = [{
            "host": "127.0.0.1", "port": urelay["port"],
            "id": rcfg["server_id"], "ssl": bool(urelay.get("ssl", True)),
        }]
        scfg["s2s_shared_secret_file"] = secret_path

    sto = srv.get("storage")
    if sto:
        # SL-0079: a real droneengage_storage_server beside the comm
        # server - generated config, per-run shared secret (>=16 chars,
        # chmod 600), per-run DB under var/run/<sim>/server/ unless
        # persist: keeps it across sim down/up. Starts before de_server
        # so the comm server's storage link connects on the first try.
        if not os.path.isfile(os.path.join(STORAGE_REPO, "src",
                                           "server.js")):
            raise SimError(f"{STORAGE_REPO}: storage server not found "
                           "(set DE_STORAGE_DIR in sim.local.env)")
        # reuse the relay's per-run secret when both attach (one s2s
        # secret per comm server); a relative value is the repo config's
        # default, not a per-run file - write our own under server/
        secret_path = scfg.get("s2s_shared_secret_file")
        if not secret_path or not os.path.isabs(secret_path):
            secret_path = os.path.join(sdir, "storage_s2s_secret.key")
        if not os.path.isfile(secret_path):
            write_text(secret_path, secrets.token_hex(32) + "\n")
            os.chmod(secret_path, 0o600)
        if sto.get("persist"):
            dbdir = os.path.join(VAR_DIR, "persist", sim["name"],
                                 "storage")
        else:
            dbdir = sdir
        os.makedirs(dbdir, exist_ok=True)
        stcfg = load_config(os.path.join(STORAGE_REPO, "server.config"))
        stcfg.update({
            "server_id": "DE_StorageSrv_" + sim["name"],
            "server_ip": "127.0.0.1",
            "public_host": "127.0.0.1",
            "server_port": sto["port"],
            "ssl_key_file": os.path.join(
                STORAGE_REPO, "ssl_local/ssl_airgap/domain.key"),
            "ssl_cert_file": os.path.join(
                STORAGE_REPO, "ssl_local/ssl_airgap/domain.crt"),
            "s2s_cert_enabled": False,
            "s2s_shared_secret_file": secret_path,
            "database": {"path": os.path.join(dbdir, "storage.db"),
                         "maxConnections": 10},
            "log_directory": os.path.join(sdir, "logs") + "/",
            "dashboard_enable": False,
        })
        cfg = os.path.join(sdir, "de_storage.config")
        write_json(cfg, stcfg)
        _node_script(sim, STORAGE_REPO, "storage", cfg)
        procs.append("storage")
        scfg["enable_storage_server"] = True
        scfg["storage_server_host"] = "127.0.0.1"
        scfg["storage_server_port"] = sto["port"]
        scfg["s2s_shared_secret_file"] = secret_path

    cfg = os.path.join(sdir, "de_server.config")
    write_json(cfg, scfg)
    _node_script(sim, SERVER_REPO, "de_server", cfg)
    procs.append("de_server")

    # one persistent GCS session on the real server, exposing the same
    # control protocol as the fake server (ctl.py talks to either)
    procs += _bridge_script(sim)
    return procs


def binaries_needed(sim, catalog):
    out = {resolve_bin("de_comm")}
    if sim["server"]["type"] == "real":
        out.add(os.path.join(SERVER_REPO, "src", "server.js"))
        if sim["server"]["auth"]:
            out.add(os.path.join(AUTH_REPO, "src", "server.js"))
        if sim["server"].get("udp_relay"):
            out.add(os.path.join(UDP_RELAY_REPO, "src", "server.js"))
        if sim["server"].get("storage"):
            out.add(os.path.join(STORAGE_REPO, "src", "server.js"))
    for u in sim["units"]:
        for m in u["modules"]:
            if m != "de_comm":
                out.add(resolve_bin(m, catalog[m]))
        if u["fc"]:
            out.add(os.path.join(ARDUPILOT_DIR, "build", "sitl", "bin",
                                 FC_KINDS[u["fc"]["vehicle"]]["bin"]))
    return sorted(out)


def check_binaries(sim):
    missing = [b for b in binaries_needed(sim, load_catalog())
               if not os.path.isfile(b)]
    if missing:
        raise SimError("missing binaries - build them, or point sim.local.env "
                       "at them (see sim.local.env.example):\n  "
                       + "\n  ".join(missing))


def _render_script(sim, proc, cwd, cmd):
    """launch.<proc>.sh under var/run/<sim>/render/ for the synthetic
    'render' owner (pidfiles render.<proc>.pid)."""
    path = os.path.join(render_dir(sim["name"]), f"launch.{proc}.sh")
    write_text(path,
               "#!/bin/bash\ncd '%s'\necho $$ > '%s'\nexec %s\n"
               % (cwd, pid_path(sim["name"], "render", proc), cmd), 0o755)
    return path


def _is_render_camera_unit(modules, sim):
    """A unit the P5-15 render sink feeds. de_yolo_ai units always are
    (they read DE-SRC-k and annotate to DE-AI-k). A de_camera unit with
    no de_yolo_ai is one only when the sim actually runs the frame farm
    (render.mode cameras|full, not headless) - then de_camera reads
    DE-SRC-k directly and the annotated AI device stays unused
    (TASK-EV-CAM)."""
    if "de_yolo_ai" in modules:
        return True
    if "de_camera" not in modules:
        return False
    render = sim.get("render") or {}
    return (render.get("mode") in ("cameras", "full")
            and not render.get("headless"))


def cam_labels(k):
    """v4l2loopback card labels for rendered-camera unit k: default
    DE-SRC-k / DE-AI-k (what tools/setup_v4l2loopback.sh creates). On a
    machine that already runs a named-VC setup, comma lists in
    sim.local.env re-point the sim at the existing labels - e.g.
    SIM_V4L2_SRC_LABELS=SIM-CAM1 and SIM_V4L2_AI_LABELS=DE-AI reuse the
    camera repo's sh_camera_create_named_vc.sh devices instead."""
    srcs = [s.strip() for s in
            (setting("SIM_V4L2_SRC_LABELS", "") or "").split(",")
            if s.strip()]
    ais = [s.strip() for s in
           (setting("SIM_V4L2_AI_LABELS", "") or "").split(",")
           if s.strip()]
    return ((srcs[k] if k < len(srcs) else f"DE-SRC-{k}"),
            (ais[k] if k < len(ais) else f"DE-AI-{k}"))


# /sys scan is a function so tests can point it at a fixture tree
def v4l2_label_map(sysfs_root="/sys/class/video4linux"):
    """card label -> /dev/videoN for every V4L2 device on the box."""
    out = {}
    if not os.path.isdir(sysfs_root):
        return out
    for name in os.listdir(sysfs_root):
        try:
            with open(os.path.join(sysfs_root, name, "name")) as f:
                out[f.read().strip()] = f"/dev/{name}"
        except OSError:
            continue
    return out


def _chromium_proc(sim, rdir, url):
    # --use-angle=gl-egl is the headless GPU path (SwiftShader fallback
    # needs --enable-unsafe-swiftshader); --enable-logging puts the
    # page's console in the proc log. --use-gl=egl is NOT valid on
    # Chrome 150+ (egl-gles2 is not an allowed implementation)
    _render_script(sim, "chromium", rdir,
                   "'%s' --headless=new --use-angle=gl-egl "
                   "--enable-unsafe-swiftshader "
                   "--enable-logging=stderr "
                   "--disable-background-timer-throttling "
                   "--disable-renderer-backgrounding "
                   "--user-data-dir='%s/chrome' --no-first-run "
                   "'%s'" % (CHROMIUM_BIN, rdir, url))


def _physics_render_procs(sim, render, rdir):
    """P5-16: render.mode physics runs the world page in a headless
    Chromium (?render=physics) with no cameras, sink or probes - the
    page's physics rig reads the world stream and MAVLink fan-out and
    posts payload_landed / collision events back."""
    url = ("http://127.0.0.1:%d/index_4w.html"
           "?world=ws://127.0.0.1:%d&mavlink=ws://127.0.0.1:%d"
           "&render=physics"
           % (render["web_port"], render["stream_port"],
              render["mav_ws_port"]))
    _chromium_proc(sim, rdir, url)
    return ["chromium"]


def _camera_render_procs(sim, render, rdir, marker, m3d, vite_js):
    """P5-15: render.mode cameras|full adds the frame farm on top of the
    viewer procs: render_sink.js demuxes the binary WS frames into one
    ffmpeg per camera unit writing that unit's DE-SRC-k loopback device,
    a headless Chromium runs the ?render=cameras page, and one
    render_probe per unit reads the strip back off the device for the
    render_latency rows."""
    sink_js = os.path.join(m3d, "backend", "src", "render_sink.js")
    cam_units = [u for u in sim["units"] if u.get("cam_labels")]
    if not cam_units:
        # cameras mode on a sim with no rendered-camera unit: the viewer
        # procs still answer (the page renders the world), but there
        # is nothing to feed loopback devices with
        return []
    labels = v4l2_label_map()
    mapping = []
    for u in cam_units:
        src_label, ai_label = u.get("cam_labels",
                                    cam_labels(u["cam_index"]))
        # a de_yolo_ai unit needs the src/ai pair; a bare de_camera
        # unit (TASK-EV-CAM) is fed on src alone
        needed = ((src_label, ai_label) if "de_yolo_ai" in u["modules"]
                  else (src_label,))
        missing = [l for l in needed if l not in labels]
        if missing:
            n = len(cam_units)
            raise SimError(
                f"render: v4l2loopback label(s) {missing} not found - "
                f"run 'sudo tools/setup_v4l2loopback.sh {max(1, n)}' "
                "for the sim's own DE-SRC/DE-AI pair, or reuse an "
                "existing named-VC setup via SIM_V4L2_SRC_LABELS / "
                "SIM_V4L2_AI_LABELS in sim.local.env (P5-15)")
        mapping.append({"unit_idx": sim["units"].index(u),
                        "name": u["name"], "cam_index": u["cam_index"],
                        "device": labels[src_label], "fps": 10})
    if not os.path.isfile(sink_js):
        raise SimError(f"render: {sink_js} not found - Mavlink3DMap needs "
                       "the P5-15 render_sink backend")
    sink_map = os.path.join(rdir, "sink_map.json")
    write_json(sink_map, {"units": mapping}, indent=1)
    _render_script(sim, "render_sink", os.path.join(m3d, "backend"),
                   "'%s' --require '%s' ./src/render_sink.js -w %d "
                   "--map '%s' --ffmpeg '%s'"
                   % (NODE, marker, render["sink_port"], sink_map,
                      FFMPEG_BIN))
    url = ("http://127.0.0.1:%d/index_4w.html"
           "?world=ws://127.0.0.1:%d&mavlink=ws://127.0.0.1:%d"
           "&render=cameras&sink=ws://127.0.0.1:%d"
           % (render["web_port"], render["stream_port"],
              render["mav_ws_port"], render["sink_port"]))
    _chromium_proc(sim, rdir, url)
    probe = os.path.join(ENGINE_DIR, "render_probe.py")
    for m in mapping:
        _render_script(
            sim, f"render_probe_{m['cam_index']}", rdir,
            "'%s' '%s' --device '%s' --unit '%s' "
            "--stream 'ws://127.0.0.1:%d'"
            % (sys.executable, probe, m["device"], m["name"],
               render["stream_port"]))
    procs = ["render_sink", "chromium"]
    procs += [f"render_probe_{m['cam_index']}" for m in mapping]
    return procs


def m3d_version(p_m3d=None):
    """The version a Mavlink3DMap checkout reports: the npm package
    version in backend/package.json (the monorepo root is private and
    carries none), else `git describe --tags` (a tagged release), else
    None - nothing readable to compare."""
    m3d = p_m3d or MAVLINK3DMAP_DIR
    try:
        v = json.load(open(os.path.join(m3d, "backend", "package.json"))
                      ).get("version")
        if v:
            return str(v)
    except (OSError, ValueError):
        pass
    try:
        import subprocess
        r = subprocess.run(
            ["git", "-C", m3d, "describe", "--tags", "--abbrev=0"],
            capture_output=True, text=True, timeout=5)
        if r.returncode == 0 and r.stdout.strip():
            return r.stdout.strip().lstrip("v")
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def m3d_version_mismatch(p_m3d=None):
    """None when the checkout reports M3D_REQUIRED_VERSION; else the
    'expected X, found Y' detail (found <none> when unreadable)."""
    found = m3d_version(p_m3d)
    want = str(M3D_REQUIRED_VERSION)
    if found is None:
        return f"expected {want}, found <unreadable>"
    if found == want or found.lstrip("v") == want.lstrip("v"):
        return None
    return f"expected {want}, found {found}"


def gen_render(sim):
    """P5-14 D7: render.mode viewer|cameras|physics|full (and not headless,
    except physics, which always needs the page)
    launches Mavlink3DMap: the UDP->WebSocket bridge bound to
    render.mav_port serving MAVLink on ws render.mav_ws_port, and a
    vite dev server on render.web_port. cameras|full adds the P5-15
    frame farm (sink + chromium + probes). Scripts and logs live under
    var/run/<sim>/render/; the de_sim_marker.cjs preload puts the run dir
    in the node cmdline so pid_is_ours/`sim down` treat them like every
    other process."""
    render = sim.get("render") or {}
    if not render or render["mode"] not in ("viewer", "cameras", "physics",
                                            "full") \
            or (render["headless"] and render["mode"] != "physics"):
        return []
    m3d = MAVLINK3DMAP_DIR
    bridge = os.path.join(m3d, "backend", "src", "udp2websocket.js")
    # npm workspaces hoist vite to the repo root; a standalone
    # frontend install keeps it under frontend/node_modules
    vite_js = next((p for p in (
            os.path.join(m3d, "node_modules", "vite", "bin", "vite.js"),
            os.path.join(m3d, "frontend", "node_modules", "vite",
                         "bin", "vite.js"))
            if os.path.isfile(p)), None)
    for p in (bridge, vite_js):
        if not p or not os.path.isfile(p):
            raise SimError(
                f"render: {p} not found - set MAVLINK3DMAP_DIR in "
                "sim.local.env (or render.headless: true to run the "
                "stream without the viewer)")
    mism = m3d_version_mismatch(m3d)
    if mism is not None:
        print(col_yellow(f"render: Mavlink3DMap {mism} - "
                         "checkout the pinned release or set "
                         "SIM_M3D_VERSION"), flush=True)
    rdir = render_dir(sim["name"])
    os.makedirs(rdir, exist_ok=True)
    os.makedirs(pids_dir(sim["name"]), exist_ok=True)
    marker = os.path.join(rdir, "de_sim_marker.cjs")
    write_text(marker,
               "// sim pid marker: preloaded via node --require so the "
               "render\n// procs' cmdline names var/run/<sim>/ "
               "(simlib.pid_is_ours).\n")
    _render_script(sim, "mav_bridge", os.path.join(m3d, "backend"),
                   "'%s' --require '%s' ./src/udp2websocket.js -p %d -w %d"
                   % (NODE, marker, render["mav_port"],
                      render["mav_ws_port"]))
    _render_script(sim, "web", os.path.join(m3d, "frontend"),
                   "'%s' --require '%s' '%s' --port %d --strictPort "
                   "--host 127.0.0.1"
                   % (NODE, marker, vite_js, render["web_port"]))
    procs = ["mav_bridge", "web"]
    if render["mode"] in ("cameras", "full"):
        procs += _camera_render_procs(sim, render, rdir, marker, m3d,
                                      vite_js)
    if render["mode"] == "physics":
        procs += _physics_render_procs(sim, render, rdir)
    return procs


def generate(sim, sim_file=None):
    """Write var/run/<sim>/<unit>/... for every unit and the sim's manifest."""
    catalog = load_catalog()
    check_binaries(sim)
    os.makedirs(run_dir(sim["name"]), exist_ok=True)
    units = copy.deepcopy(sim["units"])
    for u in units:
        gen_unit(u, sim, catalog)
    server_procs = gen_server(sim)
    render_procs = gen_render(sim)
    manifest = {"version": 1, "name": sim["name"], "sim_file": sim_file,
                "sim_hash": sim["hash"], "owner": owner_record(),
                "server": sim["server"],
                "server_procs": server_procs,
                "render_procs": render_procs,
                "render": sim.get("render"),
                "module_overrides": dict(_MODULE_BINS),
                "home": sim["home"], "speedup": sim.get("speedup", 1),
                "units": units}
    if sim.get("ports"):
        # the flat lease map (name -> {port, proto}) - the source of
        # truth for what the allocator reserved; absent when generate()
        # runs without a commit (./sim show, golden diff)
        manifest["ports"] = sim["ports"]
    write_json(manifest_path(sim["name"]), manifest, indent=2)
    return manifest


# ---------------------------------------------------------------- journal

_LINE = re.compile(r"^(\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+([A-Z_]+)\s+(.*)$")


def journal_mark(u):
    """Snapshot of a unit's journal AND proc-log file offsets - pass as
    `since` to only see lines appended after this point. Proc logs are
    included so log-based ready checks (ready_log_re) can use the same
    mark dict; journal_lines() ignores non-journal keys."""
    marks = {}
    for sub in ("journal", "logs"):
        d = os.path.join(unit_dir(u), sub)
        if not os.path.isdir(d):
            continue
        for f in os.listdir(d):
            if f.endswith(".log"):
                p = os.path.join(d, f)
                marks[p] = os.path.getsize(p)
    return marks


_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")


def log_match(u, proc, rx, since=None):
    """Lines of var/run/<sim>/<unit>/logs/<proc>.log appended since the mark
    matching the regex (a pattern string or a compiled re) - ANSI
    colour escapes stripped first: de_common prints coloured stdout
    (de_precland's "PRECLAND state -> <n>" has escapes inside the line,
    so a bare substring regex would miss). For module ready checks and
    assertions whose signal lives on stdout (de_yolo_ai, de_precland)
    instead of the de_comm journal."""
    path = os.path.join(unit_dir(u), "logs", proc + ".log")
    if not os.path.isfile(path):
        return []
    off = since.get(path, 0) if since else 0
    rxc = re.compile(rx) if isinstance(rx, str) else rx
    with open(path, errors="replace") as fh:
        fh.seek(off)
        return [ln for ln in (_ANSI.sub("", l) for l in fh.read().splitlines())
                if rxc.search(ln)]


def _parse_journal_line(ln, f):
    m = _LINE.match(ln.strip())
    if m:
        return (int(m.group(1)), m.group(2), m.group(3),
                m.group(4), m.group(5), m.group(6), f)
    return (0, "-", "-", "-", "RAW", ln.strip(), f)


def _journal_files(u):
    jdir = journal_dir(u)
    if not os.path.isdir(jdir):
        return []
    return sorted(os.path.join(jdir, f) for f in os.listdir(jdir)
                  if f.endswith(".log"))


def _merge_by_time(per_file):
    """One time-ordered list from per-file line lists. A unit appends to
    several journal files at once (none.pre / <pkg>.pre / <run>), so file
    order (mtime or name) is not event order. RAW lines (ts 0) inherit the
    timestamp of the line before them in their own file."""
    keyed = []
    for fi, lines in enumerate(per_file):
        last = 0
        for li, ln in enumerate(lines):
            last = ln[0] or last
            keyed.append((last, fi, li, ln))
    keyed.sort(key=lambda k: k[:3])
    return [k[3] for k in keyed]


def journal_lines(u, since=None):
    """[(unix_ms, engine, mission, seq, kind, msg, file)] in event-time
    order across every journal file of the unit."""
    per_file = []
    for f in _journal_files(u):
        off = since.get(f, 0) if since else 0
        with open(f, errors="replace") as fh:
            fh.seek(off)
            per_file.append([_parse_journal_line(ln, f) for ln in fh])
    return _merge_by_time(per_file)


def journal_read_new(u, offsets):
    """Lines appended since `offsets` ({file: byte offset}, updated in
    place), time-ordered. Only complete lines are consumed, so a line
    being written is picked up whole on the next call. For pollers (the
    scenario `on:` watchers) - an index into journal_lines() is not stable
    because new lines land in the middle of the merged order."""
    per_file = []
    for f in _journal_files(u):
        try:
            with open(f, "rb") as fh:
                fh.seek(offsets.get(f, 0))
                data = fh.read()
        except OSError:
            continue
        end = data.rfind(b"\n") + 1
        if not end:
            continue
        offsets[f] = offsets.get(f, 0) + end
        text = data[:end].decode(errors="replace")
        per_file.append([_parse_journal_line(ln, f)
                         for ln in text.splitlines()])
    return _merge_by_time(per_file)


def line_hit(ln, rx):
    """A regex matches "<KIND> <msg>" or <msg> alone."""
    return bool(rx.search(ln[4] + " " + ln[5]) or rx.search(ln[5]))


def journal_match(u, pattern, since=None):
    rx = re.compile(pattern)
    return [ln for ln in journal_lines(u, since) if line_hit(ln, rx)]


def boot_mark_path(u):
    return os.path.join(unit_dir(u), "journal.boot_mark.json")


def save_boot_mark(u):
    json.dump(journal_mark(u), open(boot_mark_path(u), "w"))


def boot_mark(u):
    try:
        return json.load(open(boot_mark_path(u)))
    except (OSError, ValueError):
        return {}


# ---------------------------------------------------------------- pids

def pid_path(sim, name, proc):
    return os.path.join(pids_dir(sim), f"{name}.{proc}.pid")


def pid_is_ours(pid):
    """True only if the live pid's command line points into this repo
    (every launch script, config and parm file lives under SIM_ROOT/RUN_DIR).
    Never matches another harness's de_comm/SITL with the same binary."""
    try:
        cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
    except OSError:
        return False
    return RUN_DIR in cmd or ENGINE_DIR in cmd


def read_pids(sim):
    """{pidfile_stem: pid} of one sim, for entries whose process is alive
    and ours."""
    out = {}
    d = pids_dir(sim)
    if not os.path.isdir(d):
        return out
    for f in os.listdir(d):
        if not f.endswith(".pid"):
            continue
        try:
            pid = int(open(os.path.join(d, f)).read().strip())
            os.kill(pid, 0)
        except (ValueError, OSError):
            continue
        if pid_is_ours(pid):
            out[f[:-4]] = pid
    return out


def read_all_pids():
    """{sim_name: {pidfile_stem: pid}} over every generated sim."""
    return {n: read_pids(n) for n in sim_names()}
