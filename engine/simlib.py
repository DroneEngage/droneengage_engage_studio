#!/usr/bin/env python3
"""simlib.py - core of droneengage_studio.

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

Port scheme (slot k = slot_base + unit index, default slot_base 10 keeps
sim ports clear of any developer's own servers):
    SITL -I k:  pilot 5760+10k (serial0)  aux 5762+10k (serial2)
                fcb   7660+10k (serial1, de_mavlink)   sysid k
    fcb proxy   opt-in (`fcb_proxy: true` on the sim or a unit, SL-0008):
                data 7760+10k -> fcb_port, control 7860+10k
    fcb pty     opt-in (`fcb_link: {kind: pty}` on the sim or a unit,
                SL-0102): a socat pty front for the fcb proxy that
                de_mavlink opens as a real serial device
    de_comm     62000 + 10k     other modules: catalog port_base + 10k
    server      wss 62008, control 62009 (sim `server.port`)
    external    no local server; the sim's GCS bridge listens on
                control_port (default 62009 + slot_base)
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
STORAGE_REPO = repo_dir("DE_STORAGE_DIR", "droneengage_storage_server")
WEBCLIENT_REPO = repo_dir("DE_WEBCLIENT_DIR", "droneengage_webclient_react")
# SL-0144: the servers are Rust 1:1 replacements of the old node ones
# (same config files, server.config.local merge, env overrides, ports
# and endpoints; the old code stays on each repo's
# pr_nodejs_last_version branch). `cargo build --release` in the repo
# produces <repo>/target/release/<binary>; every binary takes
# --config=<file> and ignores unknown args, so the --sim_run /
# --<stack> pid markers keep working.
SERVER_BINS = {
    "de_auth": "andruavauth",
    "de_server": "andruavmainserver",
    "de_storage": "de_storage_server",
    "udp_relay": "droneengage-udp-proxy",
}


def server_bin(repo, name):
    """A server repo's built binary: <repo>/target/release/<name>."""
    return os.path.join(repo, "target", "release", name)
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

SERVER_PORT = 62008                 # wss; control port is +1
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
DE_COMM_PORT_BASE = 62000

# SL-0009: opt-in byte-level link-fault proxy between de_comm and the
# real comm server (server: {type: real, link_proxy: true}). de_comm
# dials <base>+10k instead of the server wss port; <base>+1+10k is the
# fault control port (fake-server line protocol).
LINK_PROXY_PORT_BASE = 61500

# TASK-87: opt-in standalone UDP relay (server: {type: real,
# udp_relay: true}). Its wss control port rides the server block at
# server.port + 12 (control is +1, auth +10/+11); the UDP pair sockets
# come from a per-sim window <base>+40*slot .. +40 - far from every
# module port_base (54000+) and the 5k/7k/8k unit ports.
UDP_RELAY_PORT_BASE = 40000

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
    """A module __doc__ or generated help text (sim_commands.command_help,
    sim_help.render_help) printed as CLI help, colorized on a tty: the
    command in green, its <params>/[flags] in yellow, the description in
    white. Shared by `sim` and `ctl` so all help screens look alike."""
    if not sys.stdout.isatty():
        print(doc)
        return
    green, yellow, white, reset = \
        "\033[32m", "\033[33m", "\033[37m", "\033[0m"
    param_rx = re.compile(r"<[^>]*>|\[[^\]]*\]|\.\.\.|--\S+")
    for line in doc.splitlines():
        # a "usage: <cmd>" line (command_help) keeps the label white and
        # colors the command like a usage row
        mu = re.match(r"^(usage:\s+)(.*)$", line)
        if mu:
            cmd = param_rx.sub(yellow + r"\g<0>" + green, mu.group(2))
            print(white + mu.group(1) + reset + green + cmd + reset)
            continue
        # a usage row is "  <cmd><2+ spaces><description>"; a command
        # filling the column has no description on its own line. Anything
        # else (headers, wrapped descriptions, blanks) prints plain white
        m = re.match(r"^(  \S.*?)(?:( {2,})(\S.*))?$", line)
        if not m:
            print(white + line + reset if line.strip() else line)
            continue
        cmd, pad, desc = m.group(1), m.group(2), m.group(3)
        # flag rows (--x under flags:/shared flags:/Common flags:) and
        # params: rows ("  name  default: ...") are parameters - yellow
        # like <params>/[flags] in a usage line, not command green
        if cmd.lstrip().startswith("-") \
                or (desc or "").startswith("default:"):
            print(yellow + cmd + reset
                  + (pad + white + desc + reset if desc else ""))
            continue
        cmd = param_rx.sub(yellow + r"\g<0>" + green, cmd)
        print(green + cmd + reset
              + (pad + white + desc + reset if desc else ""))


def usage_error_exit(text):
    """A one-line usage error (`<usage> - ./sim <cmd> help` or
    `unknown <flag> - ./sim <cmd> help`) -> stderr, exit 1. On a tty it
    carries the print_usage colors - the command green, its
    <params>/[flags] yellow, the rest white - so the error looks like
    the help it points at. Plain text off a tty."""
    line = text
    if sys.stderr.isatty():
        green, yellow, white, reset = \
            "\033[32m", "\033[33m", "\033[37m", "\033[0m"
        param_rx = re.compile(r"<[^>]*>|\[[^\]]*\]|\.\.\.|--\S+")
        usage, sep, tail = text.partition(" - ")
        base = green if usage.startswith("./sim") else white
        line = base + param_rx.sub(yellow + r"\g<0>" + base, usage) \
            + reset
        if sep:
            # the `- ./sim <cmd> help` pointer keeps command green and
            # any hint after `help` white
            m = re.match(r"^(\./sim \S+ help)(.*)$", tail)
            tail = green + m.group(1) + reset + white + m.group(2) \
                + reset if m else white + tail + reset
            line += white + sep + reset + tail
    print(line, file=sys.stderr)
    sys.exit(1)


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


def kind_of(path):
    """The file's EXECUTE kind (SL-0137), inferred from its place in the
    tree - never stored in the file so it cannot drift: tests/** ->
    test, scenarios/demos/** -> demo, scenarios/examples/** -> example,
    scenarios/challenges/** -> challenge; anything else (sims/, loose
    scenarios/*.yaml, files outside SIM_ROOT) -> None."""
    rel = os.path.relpath(path, SIM_ROOT) if os.path.isabs(path) else path
    parts = rel.split(os.sep)
    if parts[0] == "tests":
        return "test"
    if parts[0] == "scenarios" and len(parts) > 2:
        return {"demos": "demo", "examples": "example",
                "challenges": "challenge"}.get(parts[1])
    return None


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


def module_base(m, catalog=None):
    """Catalog key for an instance proc name: strip a trailing instance
    number only when m itself is not a catalog key and the stripped
    base is (de_camera2 -> de_camera; a catalog key ending in a digit
    stays itself)."""
    catalog = catalog if catalog is not None else load_catalog()
    if m in catalog:
        return m
    b = re.sub(r"\d+$", "", m)
    return b if b in catalog else m


def module_instance(m, catalog=None):
    """Instance number of a module proc name: the trailing digits
    (de_camera3 -> 3) when module_base resolves a different base,
    else 1."""
    b = module_base(m, catalog)
    if b == m:
        return 1
    n = m[len(b):]
    return int(n) if n.isdigit() else 1


def instance_port(port_base, slot, i):
    """s2s port for module instance i (1-based): port_base + 10*slot +
    2*(i-1). The odd offset under each instance stays the SL-0010
    udp_proxy port, so instances stride by 2; a port_base stride is
    1000, so i<=5 keeps the slot's 10-port block."""
    if i > 5:
        raise SimError(f"module instance {i}: max 5 instances per unit")
    return int(port_base) + 10 * slot + 2 * (i - 1)


# --module name=path (./sim run|try): per-run binary overrides - the
# agent-iteration hook ("rebuilt de_mavlink, try it on the warm sim").
# They feed the sim hash, so a different override never silently reuses a
# sim running the old binary; ensure() swaps just the overridden procs
# when the fleet spec itself is unchanged.
_MODULE_BINS = {}

# ./sim up --cam <dev>: pin THE camera of a single-camera sim. Applied
# inside load_sim like the auth/slot overrides so the value is part of
# the resolved spec and feeds sim_hash - a different --cam never
# silently reuses the live sim.
_CAM_OVERRIDE = None


def set_cam_override(dev):
    global _CAM_OVERRIDE
    _CAM_OVERRIDE = None if dev is None else str(dev)


def apply_cam_override(sim, dev):
    """--cam <dev> on the resolved spec: the single de_camera instance
    of the sim's only camera unit binds this device. Numeric ->
    device_num, anything else -> a card label (device_name)."""
    cam_units = []
    for u in sim["units"]:
        insts = [m for m in u["modules"]
                 if module_base(m) == "de_camera"]
        if len(insts) > 1:
            raise SimError(
                f"--cam: unit {u['name']!r} runs {len(insts)} de_camera "
                "instances - use `cameras:`")
        if insts:
            cam_units.append((u, insts[0]))
    if not cam_units:
        raise SimError("--cam: the sim runs no de_camera unit")
    if len(cam_units) > 1:
        raise SimError("--cam: several units run de_camera "
                       f"({[u['name'] for u, _ in cam_units]}) - "
                       "use `cameras:`")
    u, proc = cam_units[0]
    cfg = u["config"].setdefault(proc, {})
    camcfg = cfg.setdefault("camera", {})
    for k in ("camera_start_index", "camera_end_index"):
        camcfg.pop(k, None)
        cfg.pop(k, None)
    e = {"name": "CAM", "key": "cam_0"}
    if dev.isdigit():
        e["device_num"] = int(dev)
    else:
        e["device_name"] = dev
    camcfg["camera_list"] = [e]


def set_module_overrides(overrides):
    """Install {module: abs bin path} for this process. Called by the
    sim CLI before load_sim; {} clears."""
    global _MODULE_BINS
    _MODULE_BINS = dict(overrides or {})


def module_overrides():
    return _MODULE_BINS


# SL-0116: --set name=value (./sim show|up|run|view|suite|validate|studio):
# per-run overrides for the sim yaml's `params:` block. They feed the
# resolved param values (defaults + overrides) that ${name} substitution
# spreads into the spec before block/sim resolution - so a different
# --set changes the server port / unit config and thus the sim hash.
_PARAM_OVERRIDES = {}


def set_param_overrides(overrides):
    """Install {param name: str value} for this process. Called by the
    sim CLI before load_sim; {} clears. Values are strings here - type
    coercion (int/float/bool) and choices validation happen in
    _resolve_params against the sim's own `params:` declarations. The
    CLI also mirrors them into the SIM_PARAMS env var so suite
    subprocesses inherit them (the same env-propagation channel
    --with/--slot-base use)."""
    global _PARAM_OVERRIDES
    _PARAM_OVERRIDES = dict(overrides or {})


def param_overrides():
    return _PARAM_OVERRIDES


def _effective_param_overrides():
    """The --set overrides in force: the SIM_PARAMS env var (inherited by
    suite subprocesses) below the in-process set_param_overrides() (a
    direct ./sim call sets both to the same values)."""
    overrides = {}
    env_set = os.environ.get("SIM_PARAMS")
    if env_set:
        try:
            d = json.loads(env_set)
            if isinstance(d, dict):
                overrides.update({str(k): str(v) for k, v in d.items()})
        except (ValueError, TypeError):
            pass
    overrides.update(_PARAM_OVERRIDES)
    return overrides


_PARAM_REF = re.compile(r"\$\{(\w+)\}")


_PARAM_TYPES = ("str", "int", "float", "bool")

_PARAM_BOOL = {"true": True, "1": True, "yes": True,
               "false": False, "0": False, "no": False}


def _coerce_param(name, raw, p, what):
    """Run one param value through its declaration: `type`
    (str|int|float|bool, default str) coerces it, `choices` validates it.
    `what` labels the error ('default' / '--set name=')."""
    ptype = p.get("type", "str")
    if ptype == "int":
        try:
            val = int(raw)
        except (ValueError, TypeError):
            raise SimError(f"sim param {name}: {what} {raw!r} is not a "
                           "valid int")
    elif ptype == "float":
        try:
            val = float(raw)
        except (ValueError, TypeError):
            raise SimError(f"sim param {name}: {what} {raw!r} is not a "
                           "valid float")
    elif ptype == "bool":
        val = raw if isinstance(raw, bool) \
            else _PARAM_BOOL.get(str(raw).lower())
        if val is None:
            raise SimError(f"sim param {name}: {what} {raw!r} is not a "
                           "bool (true/false/1/0/yes/no)")
    else:
        val = raw
    choices = p.get("choices")
    if choices is not None and val not in choices:
        raise SimError(f"sim param {name}: {what} {raw!r} not in choices "
                       f"{list(choices)}")
    return val


def _resolve_params(decl, src):
    """Build {name: value} from the sim's `params:` declarations plus the
    --set overrides (set_param_overrides). Each declaration needs a
    `default` and a one-line `help`; `type` (str|int|float|bool, default
    str) coerces both the default and the override, `choices` (a list)
    validates them. A --set name not in the declarations is a clear error
    listing the available params; --set on a sim with no params: block at
    all is an error too."""
    if decl is None:
        decl = {}
    if not isinstance(decl, dict):
        raise SimError(f"sim params: must be a mapping, got "
                       f"{type(decl).__name__}")
    values = {}
    for name, p in decl.items():
        if not isinstance(p, dict) or "default" not in p:
            raise SimError(f"sim param {name!r}: needs a 'default'")
        if "help" not in p:
            raise SimError(f"sim param {name!r}: needs a 'help' line")
        ptype = p.get("type", "str")
        if ptype not in _PARAM_TYPES:
            raise SimError(f"sim param {name!r}: bad type {ptype!r} "
                           f"(one of {', '.join(_PARAM_TYPES)})")
        values[name] = _coerce_param(name, p["default"], p, "default")
    for name, raw in _effective_param_overrides().items():
        if name not in decl:
            avail = ", ".join(sorted(decl)) if decl else "(none)"
            raise SimError(f"--set {name!r}: not a param of this sim "
                           f"(params: {avail})")
        values[name] = _coerce_param(name, raw, decl[name],
                                     f"--set {name}=")
    return values


def _substitute_params(obj, values):
    """Recursively replace ${name} in string values with the resolved
    param value. A whole-string ${name} becomes the native type (an int
    for an int param); a ${name} embedded in a larger string is str()-ed.
    A ${name} that matches no declared param is a clear error."""
    if isinstance(obj, dict):
        return {k: _substitute_params(v, values) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_substitute_params(v, values) for v in obj]
    if isinstance(obj, str):
        m = _PARAM_REF.fullmatch(obj)
        if m:
            name = m.group(1)
            if name not in values:
                raise SimError(f"unknown param ${{{name}}}")
            return values[name]

        def _sub(mm):
            name = mm.group(1)
            if name not in values:
                raise SimError(f"unknown param ${{{name}}}")
            return str(values[name])
        return _PARAM_REF.sub(_sub, obj)
    return obj


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
        if m != "de_comm" and module_base(m, catalog) not in catalog:
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
    and a changed overlay set always changes the hash. `exclusive` marks
    the load as the front of an exclusive bring-up (a test run that
    downs every other live sim before it binds) - it only changes how
    --slot-base any treats a live sim with clashing unit names."""
    spec, src = _sim_spec(path_or_spec, base_dir)
    applied = []
    for ov in _spec_overlays(spec, overlays):
        spec = _apply_overlay(spec, ov, applied)
    # SL-0116: params: declarations + --set overrides + ${name}
    # substitution, before block/sim resolution so substituted values
    # flow into the server port, unit config, etc. Popped off the spec
    # so it never reaches the unknown-keys check; the resolved values
    # are stored on the sim dict (params + param_values).
    params_decl = spec.pop("params", None)
    param_values = _resolve_params(params_decl, src)
    spec = _substitute_params(spec, param_values)
    unknown = set(spec) - {"version", "name", "description", "server", "home",
                           "slot_base", "units", "render", "speedup",
                           "fcb_proxy", "fcb_link", "example",
                           "camera_feed"}
    if unknown:
        raise SimError(f"sim: unknown keys {sorted(unknown)}")
    if spec.get("version", 1) != 1:
        raise SimError("sim: unsupported version")
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
    # ./sim --slot-base <N> (SIM_SLOT_BASE_OVERRIDE) tops the file's
    # slot_base: and the SIM_SLOT_BASE fallback; "any" resolves to the
    # lowest fully-free base (below)
    ov = str(setting("SIM_SLOT_BASE_OVERRIDE", ""))
    if ov == "any":
        return _load_sim_free_slot(dict(spec, name=name), base_dir,
                                   exclusive)
    slot_base = int(ov or spec.get("slot_base", SLOT_BASE))
    srv_spec = spec.get("server") or {}
    # a server port derived from the spec's own slot (the scenario loader
    # writes SERVER_PORT + slot_base) follows an overridden slot like
    # every unit port - else `any` moves the units but keeps a busy
    # server port; a port pinned to anything else stays as written
    own_slot = int(spec.get("slot_base", SLOT_BASE))
    if ov and srv_spec.get("type") != "external" and \
            srv_spec.get("port") == SERVER_PORT + own_slot:
        srv_spec = dict(srv_spec, port=SERVER_PORT + slot_base)
    sim = {
        "name": name,
        "description": spec.get("description", "").strip(),
        "server": load_server(srv_spec, slot_base),
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
        # SL-0102: sim-wide `fcb_link: {kind: pty, baud: N}` applies to
        # the FC units only; each resolved fc.fcb_link is what hashes
        "fcb_link": spec.get("fcb_link"),
        "units": [],
        "render": _render_spec(spec.get("render"), slot_base),
        # DE-0050: `camera_feed: <src>` spawns ffmpeg testsrc producers
        # onto each camera unit's /dev/videoN before de_camera scans
        # (exclusive_caps needs a producer for CAPTURE to enumerate)
        "camera_feed": _camera_feed_spec(spec.get("camera_feed")),
        # SL-0116: the params: declarations and their resolved values
        # (defaults + --set overrides) - the declarations are kept for
        # `./sim show`/introspection, the values are what ${name} spread
        "params": params_decl or {},
        "param_values": param_values,
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
                            "fcb_link", "fc_client", "faketime",
                            "udp_proxy", "link_proxy", "android", "env",
                            "cameras"}
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
    if _CAM_OVERRIDE is not None:
        apply_cam_override(sim, _CAM_OVERRIDE)
    sim.pop("_cam_alloc", None)
    sim.pop("_cam_taken", None)
    seen = set()
    for u in sim["units"]:
        if u["name"] in seen:
            raise SimError(f"sim: duplicate unit name {u['name']!r}")
        seen.add(u["name"])
    # a card label is one physical device - two bindings to the same
    # label (across units or across a unit's camera instances) would
    # open the same camera twice
    label_owner = {}
    for u in sim["units"]:
        for l in camera_video_labels(u):
            if l in label_owner:
                raise SimError(
                    f"sim: {label_owner[l]} and {u['name']} both bind "
                    f"camera label {l} - one device, one binding")
            label_owner[l] = u["name"]
    # one host_serial adapter can only front one unit's wire - a
    # sim-level `fcb_link:` fan-out or a duplicated device: must not
    # silently bridge two units onto the same tty. The same goes for a
    # bt_spp RFCOMM channel: it can only serve one unit's bridge.
    dev_owner = {}
    chan_owner = {}
    for u in sim["units"]:
        fl = (u.get("fc") or {}).get("fcb_link") or {}
        if fl.get("kind") == "host_serial":
            if fl["device"] in dev_owner:
                raise SimError(
                    f"sim: units {dev_owner[fl['device']]} and "
                    f"{u['name']} share fcb_link device {fl['device']} "
                    "- each host_serial unit needs its own adapter")
            dev_owner[fl["device"]] = u["name"]
        elif fl.get("kind") == "bt_spp":
            if fl["channel"] in chan_owner:
                raise SimError(
                    f"sim: units {chan_owner[fl['channel']]} and "
                    f"{u['name']} share fcb_link channel "
                    f"{fl['channel']} - each bt_spp unit needs its "
                    "own RFCOMM channel")
            chan_owner[fl["channel"]] = u["name"]
    sim["hash"] = sim_hash(sim, _MODULE_BINS)
    return sim


def sim_hash(sim, overrides):
    """The fleet-identity hash: server + home + resolved units + the
    --module overrides in force. `ensure` rehashes with the manifest's
    stored overrides to tell "same fleet, different binaries" apart from
    a real spec change."""
    return hashlib.sha256(json.dumps(
        [sim["server"], sim["home"], sim["units"], overrides or {}],
        sort_keys=True).encode()).hexdigest()[:16]


def _pick_match(arg, rel, matches):
    """Numbered pick on a tty for a name that resolves to more than one
    file. Anything but a valid number (empty, q, EOF, Ctrl-C) -> the same
    SimError the non-interactive path raises."""
    print(f"ambiguous name {arg!r}:", file=sys.stderr)
    for i, r in enumerate(rel, 1):
        print(f"  {i}) {r}", file=sys.stderr)
    try:
        ans = input(f"which one [1-{len(rel)}]? ").strip()
    except (EOFError, KeyboardInterrupt):
        ans = ""
    if ans.isdigit() and 1 <= int(ans) <= len(matches):
        return matches[int(ans) - 1]
    raise SimError(f"ambiguous name {arg!r} - {', '.join(rel)}")


def resolve_sim_arg(arg, kinds=("sims",), choose=False):
    """Resolve a sim/test argument to a file path. An existing path is
    returned unchanged (so sims/copters3.yaml and tests/a/b.yaml work as
    written). A bare name - with or without the .yaml suffix - resolves
    to sims/<name>.yaml, and when `kinds` includes "tests" also
    tests/**/<name>.yaml, with "scenarios" also scenarios/**/<name>.yaml
    (both recursive - a file may nest under an area subdir). One match
    -> the path; none -> SimError pointing at `./sim list sims`; several
    -> a numbered pick when `choose` and stdin is a tty, else SimError
    listing them. A path with '/', or an unmatched arg that carried a
    .yaml suffix, is returned as-is so the downstream loader reports
    the file-not-found."""
    if os.path.exists(arg):
        return arg
    if "/" in arg:
        return arg
    name = arg[:-5] if arg.endswith(".yaml") else arg
    matches = []
    if "sims" in kinds:
        p = os.path.join(SIM_ROOT, "sims", name + ".yaml")
        if os.path.isfile(p):
            matches.append(p)
    if "tests" in kinds:
        matches.extend(sorted(glob.glob(os.path.join(
            SIM_ROOT, "tests", "**", name + ".yaml"), recursive=True)))
    if "scenarios" in kinds:
        matches.extend(sorted(glob.glob(os.path.join(
            SIM_ROOT, "scenarios", "**", name + ".yaml"), recursive=True)))
    if len(matches) == 1:
        return matches[0]
    if not matches:
        if arg.endswith(".yaml"):
            return arg
        raise SimError(f"no sim or test named {arg!r} - ./sim list sims")
    rel = [os.path.relpath(m, SIM_ROOT) for m in matches]
    if choose and sys.stdin.isatty():
        return _pick_match(arg, rel, matches)
    raise SimError(f"ambiguous name {arg!r} - {', '.join(rel)}")


def _load_sim_free_slot(spec, base_dir, exclusive=False):
    """SIM_SLOT_BASE_OVERRIDE=any: resolve the sim once per candidate
    base, lowest first; the first whose whole port set is free wins.
    "Free" means no clash with a live sim's manifest AND nothing already
    bound on the host - ports_held catches the dev server or stray SITL
    that is not a sim. A same-named live sim's ports are its own (`up`
    restarts it). The ports of a live sim with clashing unit names are
    its own too, but only for an exclusive bring-up: `run` downs it
    first, so they come free whatever the base - a suite's sequential
    scenarios keep the pick they always had. Additive callers (up,
    --shared) leave it running, so every live sim's ports are real
    conflicts - parallel sims may reuse unit names on disjoint server
    namespaces now, but never ports.
    Candidates start at SLOT_BASE (the configured floor) and stop when a
    port would pass 65535. A shifted pick changes the sim hash - `ensure`
    then does a fresh up instead of reusing."""
    live = live_sim_names()
    prev = os.environ.get("SIM_SLOT_BASE_OVERRIDE")
    try:
        for cand in range(SLOT_BASE, SLOT_BASE + 200):
            os.environ["SIM_SLOT_BASE_OVERRIDE"] = str(cand)
            sim = load_sim(spec, base_dir)
            ports = used_ports(sim)
            if ports and max(ports) > 65535:
                break                       # every port grows with it
            names = {u["name"] for u in sim["units"]}
            clash, own = False, set()
            for other in live:
                m = load_manifest(other)
                if other == sim["name"]:
                    own = used_ports(m)
                    continue
                if exclusive and \
                        names & {u["name"] for u in m["units"]}:
                    # slot-independent: an exclusive test run downs it
                    # (a suite's previous scenario) before it binds, so
                    # its ports are this pick's for the taking
                    own |= used_ports(m)
                    continue
                if ports & used_ports(m):
                    clash = True
                    break
            if clash or ports_held(ports - own):
                continue
            return sim
    finally:
        if prev is None:
            os.environ.pop("SIM_SLOT_BASE_OVERRIDE", None)
        else:
            os.environ["SIM_SLOT_BASE_OVERRIDE"] = prev
    raise SimError("no free slot_base - every candidate base has a "
                   "conflicting or already-bound port")


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
    server/home/slot_base/render/speedup/fcb_proxy/fcb_link - deep-merged
    like extends; units - APPENDED; config - deep-merged into every unit
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
                        "speedup", "fcb_proxy", "fcb_link"}
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


def load_server(srv, slot_base=None):
    """server: {type: real|fake|external, port, auth, account{user, code},
    seed, control_port, gcs_party}.

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
      this sim's local ctl port (default 62009 + slot_base); gcs_party
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
                      "udp_relay"}
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
            "control_port": int(srv.get("control_port") or
                                62009 + int(slot_base or SLOT_BASE)),
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
    port = int(srv.get("port", SERVER_PORT))
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
        # the relay's wss control port rides the server port block; the
        # UDP pair window is per-sim, sized at gen time from the fleet
        urelay = dict(urelay)
        urelay.setdefault("port", port + 12)
        urelay.setdefault("gcs_peer_policy", "open")
        urelay.setdefault("ssl", True)
    return {
        "type": t, "ip": "127.0.0.1", "port": port,
        "control_port": int(srv.get("control_port") or port + 1),
        "auth": bool(srv.get("auth", False)),
        "auth_ip": "127.0.0.1", "auth_port": port + 10,
        "auth_s2s_port": port + 11,
        "auth_verify_ssl": False,
        "account": {"user": user, "code": code},
        "gcs_party": srv.get("gcs_party") or GCS_PARTY,
        "seed": int(srv.get("seed", 1337)),
        "clock_offset_ms": int(srv.get("clock_offset_ms", 0)),
        "noclock": list(srv.get("noclock") or []),
        "link_proxy": bool(srv.get("link_proxy", False)),
        "udp_relay": urelay,
    }


RENDER_MODES = ("viewer", "cameras", "physics", "full")

# P5-13: the world stream binds 127.0.0.1 like every sim port; the
# Mavlink3DMap UDP port rides the same slot block so --seeds runs and
# parallel sims never share it. P5-14 adds the bridge's WebSocket port
# (8801+slot = 8811 on the default slot, Mavlink3DMap's own default)
# and the viewer's http port (8070+slot = 8080, again its default).
STREAM_PORT_BASE = 61000
MAV_PORT_BASE = 16450
MAV_WS_PORT_BASE = 8801
WEB_PORT_BASE = 8070
SINK_PORT_BASE = 8090


def _render_spec(raw, slot_base):
    """render: {mode, headless, stream_port, mav_port, mav_ws_port,
    web_port} -> a normalized dict, or None when the scenario did not
    opt in (D1: absent render means no process, port or param changes -
    the sim hash, generated from the unit specs, stays byte-identical
    to before)."""
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
    return {"mode": mode,
            "headless": bool(raw.get("headless", False)),
            "stream_port": int(raw.get("stream_port",
                                       STREAM_PORT_BASE + slot_base)),
            "mav_port": int(raw.get("mav_port",
                                    MAV_PORT_BASE + slot_base)),
            "mav_ws_port": int(raw.get("mav_ws_port",
                                       MAV_WS_PORT_BASE + slot_base)),
            "web_port": int(raw.get("web_port",
                                    WEB_PORT_BASE + slot_base)),
            "sink_port": int(raw.get("sink_port",
                                     SINK_PORT_BASE + slot_base))}


def shared_render_web_port():
    """SL-0154: the page port a live dev stack provides for render: sims
    (a stacks/*.yaml `provides: render_web` whose proc is stack-alive),
    else None - the sim then runs its own slot-derived vite as always.
    devstack is imported lazily and any failure degrades to None: a
    malformed stacks/ file must not take every render: bring-up down."""
    try:
        import devstack
        return devstack.provided_port("render_web")
    except Exception as e:
        print(col_dim(f"render: shared-page check skipped ({e})"))
        return None


def render_page_port(render):
    """The port actually serving the Mavlink3DMap page: a shared dev
    stack's when one was live at gen time (baked into the manifest as
    render.page_port, SL-0154), else this sim's own slot web_port."""
    return render.get("page_port") or render["web_port"]


def render_viewer_url(render, world_stream=True):
    """The printed Mavlink3DMap URL. `world_stream` is only True when a
    de.worldstream/1 feed will serve stream_port - i.e. a scenario with
    a world. The ?world= param selects the DE world scene, whose vehicle
    frame is gated on the stream's hello: advertising it without the
    stream pins vehicles at the origin (attitude still works)."""
    q = (f"world=ws://127.0.0.1:{render['stream_port']}&"
         if world_stream else "")
    return (f"http://127.0.0.1:{render_page_port(render)}/?{q}"
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
         # SL-0095: modules listed in android.remote_modules run on an
         # adb-connected phone (engine/android.py), not as local procs
         "android": None,
         # SL-0014: launch de_comm under libfaketime so `ctl clock_step`
         # can skew its clock (FAKETIME_TIMESTAMP_FILE)
         "faketime": bool(entry.get("faketime")),
         # SL-0067: per-module launch env - {module: {VAR: value}};
         # DE_BOARD_ID defaults to the unit name and may be overridden
         # here. A DE_PROC_STAT_PATH entry also spawns a fake_proc_<m>
         # writer proc (stubs/fake_proc_stat.py) before the module.
         "env": dict(entry.get("env") or {})}
    # P5-07: a unit may add modules on top of its block (scenario fleet
    # entries carry `modules:` for the P5-08 stubs). Unknown modules fail
    # here with the catalog list, before ports are allocated.
    for m in entry.get("add_modules", []):
        if m in u["modules"]:
            continue
        if module_base(m, catalog) not in catalog:
            raise SimError(f"unit {name}: unknown add_modules entry {m!r} "
                           f"(catalog: {', '.join(catalog)})")
        if catalog.get(module_base(m, catalog), {}).get("needs_fc") \
                and not b["fc"]:
            raise SimError(f"unit {name}: module {m} needs an `fc:`")
        u["modules"].append(m)
    for mod, over in (entry.get("config") or {}).items():
        if mod not in u["modules"] and mod != "sitl":
            raise SimError(f"unit {name}: config for {mod!r} but block "
                           f"{b['name']} does not run it")
        u["config"][mod] = deep_merge(u["config"].get(mod, {}), over)
    for mod, e in u["env"].items():
        if mod not in u["modules"]:
            raise SimError(f"unit {name}: env for {mod!r} but block "
                           f"{b['name']} does not run it")
        if not isinstance(e, dict):
            raise SimError(f"unit {name}: env[{mod!r}] must be a "
                           "{VAR: value} mapping")
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
    # SL-0163: `cameras:` -> one de_camera instance per binding. Each
    # instance's generated camera_list carries exactly one entry: str
    # entries bind a pool label (device_name), ints a /dev/videoN
    # (device_num), dicts give {label|num, name, key} explicitly. An int
    # count auto-assigns the next DE-CAM-k labels in declaration order -
    # deterministic (the spec alone decides bindings, sim_hash stays
    # stable). Default: a de_camera unit that selects nothing gets one
    # labeled cam; the other pool devices stay free for tracker/AI.
    cam_mods = [m for m in u["modules"]
                if module_base(m, catalog) == "de_camera"]
    raw_cams = entry.get("cameras")
    if raw_cams is not None and not cam_mods:
        raise SimError(f"unit {name}: `cameras:` but the unit runs no "
                       "de_camera module")

    def _has_cam_selection(m):
        flat = u["config"].get(m) or {}
        c = flat.get("camera") or {}
        return bool(c.get("camera_list")) \
            or c.get("camera_start_index") is not None \
            or c.get("camera_end_index") is not None \
            or flat.get("camera_start_index") is not None \
            or flat.get("camera_end_index") is not None

    if raw_cams is None:
        raw_cams = 1 if cam_mods and not any(
            _has_cam_selection(m) for m in cam_mods) else 0
    if isinstance(raw_cams, int):
        cam_list = [None] * raw_cams
    elif isinstance(raw_cams, list):
        cam_list = list(raw_cams)
    else:
        raise SimError(f"unit {name}: `cameras:` must be a count or a "
                       "list of bindings")
    if len(cam_list) > 5:
        raise SimError(f"unit {name}: `cameras:` - max 5 de_camera "
                       "instances per unit")
    if cam_list and _is_render_camera_unit(u["modules"], sim):
        raise SimError(f"unit {name}: `cameras:` - a rendered-camera "
                       "unit binds its DE-SRC/DE-AI feed, not pool "
                       "labels")
    def _alloc_cam_label():
        # next free DE-CAM label vs bindings made so far in this sim -
        # deterministic: declaration order decides, never live state
        taken = sim.setdefault("_cam_taken", set())
        n = sim.get("_cam_alloc", 0) + 1
        while f"DE-CAM{n}" in taken:
            n += 1
        sim["_cam_alloc"] = n
        taken.add(f"DE-CAM{n}")
        return f"DE-CAM{n}"

    for i, cam in enumerate(cam_list):
        proc = "de_camera" if i == 0 else f"de_camera{i + 1}"
        if proc not in u["modules"]:
            u["modules"].append(proc)
        if i > 0:
            # instances inherit the base de_camera overlay; a
            # config.<proc> entry can still override on top
            u["config"][proc] = deep_merge(
                copy.deepcopy(u["config"].get("de_camera") or {}),
                u["config"].get(proc) or {})
        cfg = u["config"].setdefault(proc, {})
        if cam is None:
            cam = {"device_name": _alloc_cam_label()}
        elif isinstance(cam, str):
            cam = {"device_name": cam}
        elif isinstance(cam, int):
            cam = {"device_num": cam}
        elif isinstance(cam, dict):
            cam = dict(cam)
            if "label" in cam:
                cam["device_name"] = cam.pop("label")
            if "num" in cam:
                cam["device_num"] = cam.pop("num")
            if "device_name" not in cam and "device_num" not in cam:
                cam["device_name"] = _alloc_cam_label()
        else:
            raise SimError(f"unit {name}: cameras[{i}] must be a label, "
                           "an index or {label|num, name, key}")
        if cam.get("device_name"):
            sim.setdefault("_cam_taken", set()).add(cam["device_name"])
        cam.setdefault("name", f"CAM{i}")
        cam.setdefault("key", f"cam_{i}")
        # the instance binds exactly one camera - the generated entry
        # replaces any `camera:` scan selection
        camcfg = cfg.setdefault("camera", {})
        for k in ("camera_start_index", "camera_end_index"):
            camcfg.pop(k, None)
            cfg.pop(k, None)
        camcfg["camera_list"] = [cam]
        if i > 0:
            # s2s module ids must differ on one unit
            cfg["module_id"] = \
                f"{cfg.get('module_id') or 'de_camera'}{i + 1}"
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
    u["ports"]["de_comm"] = DE_COMM_PORT_BASE + 10 * slot
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
    u["link_proxy"] = {"port": LINK_PROXY_PORT_BASE + 10 * slot,
                       "ctrl": LINK_PROXY_PORT_BASE + 1 + 10 * slot} \
        if lp else None
    for m in u["modules"]:
        if m != "de_comm":
            u["ports"][m] = instance_port(
                catalog[module_base(m, catalog)]["port_base"], slot,
                module_instance(m, catalog))
    for m in entry.get("udp_proxy") or []:
        # SL-0010: a chunk-fault proxy on this module's s2s link. It
        # claims the module's slot port +1 (port_base strides are 1000,
        # so +1..+9 of any module's slot decade is free); gen_unit then
        # points the module's s2s_udp_target_port at it and the proxy
        # owns the whole module<->de_comm path - de_comm answers the
        # sender's address, so both directions cross it.
        if m == "de_comm" or m not in u["modules"]:
            raise SimError(f"unit {name}: udp_proxy {m!r} is not one of "
                           f"its modules ({u['modules']})")
        u["ports"]["udp_proxy_" + m] = u["ports"][m] + 1
    if max(u["ports"].values()) > 65535:
        raise SimError(f"unit {name}: slot {slot} pushes ports past 65535")

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
    # SL-0102: `fcb_link: {kind: pty, baud: N}` (unit or sim level)
    # swaps de_mavlink's tcp dial for a real serial device - a socat
    # pty bridged into the fcb proxy and paced at baud/10 B/s by the
    # SL-0101 rate rule. kind tcp is today's wiring and records
    # nothing; sim-level applies to FC units only (a static skips it).
    fl = entry.get("fcb_link")
    if fl is not None and not isinstance(fl, dict):
        raise SimError(f"unit {name}: `fcb_link` wants a mapping "
                       "{kind: ..., baud: ...}")
    if fl is None:
        fl = sim.get("fcb_link") if fc else None
    if fl and not fc:
        raise SimError(f"unit {name}: `fcb_link` but the block has no fc")
    bad = set(fl or ()) - {"kind", "baud", "device", "channel", "peer"}
    if bad:
        raise SimError(f"unit {name}: `fcb_link` unknown keys "
                       f"{sorted(bad)}")
    link_kind = (fl or {}).get("kind", "tcp")
    if link_kind not in ("tcp", "pty", "host_serial", "bt_spp", "udp"):
        raise SimError(f"unit {name}: `fcb_link` kind {link_kind!r} "
                       "(have: tcp, pty, host_serial, bt_spp, udp)")
    if link_kind != "host_serial" and "device" in (fl or {}):
        raise SimError(f"unit {name}: `fcb_link` device only applies "
                       "to kind host_serial")
    if link_kind != "bt_spp" and "channel" in (fl or {}):
        raise SimError(f"unit {name}: `fcb_link` channel only applies "
                       "to kind bt_spp")
    if link_kind != "udp" and "peer" in (fl or {}):
        raise SimError(f"unit {name}: `fcb_link` peer only applies "
                       "to kind udp")
    # SL-0104: `fc_client: external` - the FC client lives off-host (an
    # Android/Pi wired to the host's USB-serial adapter, or paired over
    # Bluetooth SPP - SL-0105): the unit runs SITL + the bridge but no
    # local de_comm/de_mavlink. It only makes sense with a link kind
    # the client can physically attach to.
    fc_client = str(entry.get("fc_client", "local"))
    if fc_client not in ("local", "external"):
        raise SimError(f"unit {name}: `fc_client` {fc_client!r} "
                       "(have: local, external)")
    if fc_client == "external":
        if not fc:
            raise SimError(f"unit {name}: `fc_client: external` but "
                           "the block has no fc")
        if link_kind not in ("tcp", "host_serial", "bt_spp", "udp"):
            raise SimError(f"unit {name}: `fc_client: external` needs "
                           "`fcb_link` kind tcp, host_serial, bt_spp "
                           "or udp - the kinds that wire a client in "
                           "from outside the host")
    if fc:
        if not 1 <= slot <= 250:
            raise SimError(f"unit {name}: slot {slot} is not a valid MAVLink sysid")
        kind = FC_KINDS[fc["vehicle"]]
        u["fc"] = {
            "vehicle": fc["vehicle"],
            "model": fc.get("model", kind["model"]),
            "sysid": slot, "instance": slot,
            "speedup": sim["speedup"],
            "pilot_port": 5760 + 10 * slot,
            "aux_port": 5762 + 10 * slot,
            "fcb_port": 7660 + 10 * slot,
            "arm_mode": fc.get("arm_mode", kind["arm_mode"]),
            "land": fc.get("land", kind["land"]),
            "defaults": fc.get("defaults", kind["defaults"]),
            "params": fc.get("params") or {},
        }
        # a catalog module flagged `pose` reads the vehicle pose on its
        # own SITL serial (serial5, tcp:5 -> 5765+10k) - the P5-08 stubs
        # use it; serial0/1/2 stay with ctl / de_mavlink / telemetry and
        # serial3 stays the SITL GPS device on every unit
        if any(catalog.get(m, {}).get("pose") for m in u["modules"]):
            u["fc"]["pose_port"] = 5765 + 10 * slot
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
        if link_kind == "pty":
            baud = int(fl.get("baud", 57600))
            if baud <= 0:
                raise SimError(f"unit {name}: `fcb_link` baud must be "
                               "positive")
            u["fc"]["fcb_link"] = {
                "kind": "pty", "baud": baud,
                # the serial device de_mavlink opens - socat's link=
                # path under the unit's run dir (stable across
                # bridge restarts)
                "tty": os.path.join(unit_dir(u), "fcb.tty")}
        elif link_kind == "host_serial":
            # SL-0104: the host's USB-serial adapter is the wire - a
            # socat bridge binds it to the fcb proxy while the client
            # device (crossed cable: its TX->our RX) lives off-host.
            device = str(fl.get("device") or "").strip()
            if not device:
                raise SimError(f"unit {name}: `fcb_link` kind "
                               "host_serial needs `device:` (e.g. "
                               "/dev/ttyUSB0 or a /dev/serial/by-id "
                               "path)")
            if fc_client != "external":
                raise SimError(f"unit {name}: `fcb_link` kind "
                               "host_serial needs `fc_client: "
                               "external` - the bridge owns the "
                               "adapter; a local de_mavlink opening "
                               "the same device would fight it")
            baud = int(fl.get("baud", 57600))
            if baud <= 0:
                raise SimError(f"unit {name}: `fcb_link` baud must be "
                               "positive")
            u["fc"]["fcb_link"] = {"kind": "host_serial",
                                   "device": device, "baud": baud}
        elif link_kind == "bt_spp":
            # SL-0105: the host's Bluetooth adapter runs an RFCOMM SPP
            # server (engine/bt_spp_bridge.py, proc `bt_spp`) bridged
            # to the fcb proxy - the client device pairs with the host
            # and dials the channel like a drone's FC serial port.
            if fc_client != "external":
                raise SimError(f"unit {name}: `fcb_link` kind "
                               "bt_spp needs `fc_client: external` - "
                               "the bridge owns the adapter channel; "
                               "a local client cannot dial it")
            try:
                channel = int(fl.get("channel"))
            except (TypeError, ValueError):
                channel = -1
            if not 1 <= channel <= 30:
                raise SimError(f"unit {name}: `fcb_link` kind bt_spp "
                               "needs `channel:` in 1-30 (the RFCOMM "
                               "channel the client dials)")
            link = {"kind": "bt_spp", "channel": channel}
            if "baud" in fl:
                baud = int(fl["baud"])
                if baud <= 0:
                    raise SimError(f"unit {name}: `fcb_link` baud "
                                   "must be positive")
                link["baud"] = baud
            u["fc"]["fcb_link"] = link
        elif link_kind == "tcp" and fc_client == "external":
            # SL-0106: the client dials the host's LAN address at the
            # fcb proxy's listen port - the proxy is forced on below
            # and binds 0.0.0.0 for external units so the off-host
            # dial reaches it. Recorded so status shows the dial point
            # and the link kind lands in the sim hash.
            u["fc"]["fcb_link"] = {"kind": "tcp"}
        elif link_kind == "udp":
            # SL-0106: the client is a UDP listener (Andruav's
            # UdpConnection learns its peer from the first inbound
            # datagram and never sends first), so a socat bridge
            # (proc fcb_udp) pushes the proxy's byte stream to `peer`
            # (phone-ip:port) and pipes the replies back into the
            # proxy. The bridge's UDP source port is ephemeral - no
            # host port is allocated.
            if fc_client != "external":
                raise SimError(f"unit {name}: `fcb_link` kind udp "
                               "needs `fc_client: external` - the "
                               "bridge pushes to a remote listener; "
                               "a local de_mavlink is tcp instead")
            peer = str(fl.get("peer") or "").strip()
            p_host, _, p_port = peer.rpartition(":")
            try:
                peer_port = int(p_port)
            except ValueError:
                peer_port = 0
            if not p_host or not 1 <= peer_port <= 65535:
                raise SimError(f"unit {name}: `fcb_link` kind udp "
                               "needs `peer:` as <phone-ip>:<port> "
                               "(e.g. 192.168.1.20:14550) - the "
                               "client listens, the bridge pushes")
            u["fc"]["fcb_link"] = {"kind": "udp",
                                   "peer": f"{p_host}:{peer_port}"}
        if fc_client == "external":
            u["fc_client"] = "external"
            u["modules"] = [m for m in u["modules"]
                            if m not in ("de_comm", "de_mavlink")]
        px = entry.get("fcb_proxy")
        if px is None:
            px = bool(sim.get("fcb_proxy"))
        if px or link_kind in ("pty", "host_serial", "bt_spp", "udp") \
                or (link_kind == "tcp" and fc_client == "external"):
            # a serial/udp link only fronts the proxy - it cannot
            # exist alone; an external tcp client dials the proxy
            # directly, so it too must exist
            u["fc"]["fcb_proxy"] = {"port": 7760 + 10 * slot,
                                    "ctrl": 7860 + 10 * slot}
        # world.prepare_sim stamps config.<stub>.stub.{sysid,pose_port}
        # from the slot_base it guesses before the fleet resolves -
        # studio/`--slot-base any`/`SIM_SLOT_BASE_OVERRIDE` can resolve
        # a different slot, and the SITL serial5 listener always follows
        # the resolved slot above. A stale stamp leaves the stub dialing
        # a dead port (pose feed refused all run -> level-attitude
        # fallback -> wrong geo projections), so the resolved fc values
        # always win here.
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
    spec = entry.get("android")
    if spec is not None:
        if not isinstance(spec, dict):
            raise SimError(f"unit {name}: android must be a mapping "
                           "({serial, remote_modules, mode})")
        remote = list(spec.get("remote_modules") or [])
        unknown = [m for m in remote if m not in u["modules"]]
        if unknown:
            raise SimError(f"unit {name}: android.remote_modules "
                           f"{unknown} not in the unit's modules "
                           f"({u['modules']})")
        # SL-0112: `mode: app` - the app owns module configuration and
        # starts de_comm/de_mavlink itself; the sim pushes nothing.
        mode = spec.get("mode") or "sim"
        if mode not in ("sim", "app"):
            raise SimError(f"unit {name}: android.mode must be 'sim' "
                           f"or 'app' (got {mode!r})")
        if mode == "app" and set(remote) != {"de_comm", "de_mavlink"}:
            raise SimError(f"unit {name}: android mode: app needs "
                           "remote_modules [de_comm, de_mavlink]")
        # SL-0118: `net` picks how the device reaches the host - auto
        # (emulator serials adb-reverse, everything else LAN probing),
        # adb_reverse (a USB device with no network reaches host TCP
        # ports as 127.0.0.1), lan (host_ip() LAN probing always).
        net = spec.get("net") or "auto"
        if net not in ("auto", "adb_reverse", "lan"):
            raise SimError(f"unit {name}: android.net must be 'auto', "
                           f"'adb_reverse' or 'lan' (got {net!r})")
        u["android"] = {"serial": spec.get("serial"),
                        "remote_modules": remote, "mode": mode,
                        "net": net,
                        "device_ip": None, "clock_offset_ms": 0}
    # AN-0037 test hook: SIM_ANDROID_MODULES="de_comm[,...]" forces the
    # fleet's FIRST unit to run those modules on the adb device - lets
    # the unmodified suite files exercise a phone-side de_comm
    env_mods = os.environ.get("SIM_ANDROID_MODULES")
    if env_mods and u["android"] is None \
            and u["slot"] == sim["slot_base"]:
        remote = [m.strip() for m in env_mods.split(",") if m.strip()]
        unknown = [m for m in remote if m not in u["modules"]]
        if unknown:
            raise SimError(f"unit {name}: SIM_ANDROID_MODULES {unknown} "
                           f"not in the unit's modules ({u['modules']})")
        u["android"] = {"serial": None, "remote_modules": remote,
                        "mode": "sim", "net": "auto",
                        "device_ip": None, "clock_offset_ms": 0}
    # SL-0095: a phone-side de_mavlink always dials through the
    # proxy - SITL's serial TCP listener is single-client and keeps
    # the killed module's socket ESTABLISHED, so a restarted remote
    # module can never re-dial SITL directly; the proxy is a stable
    # host-side hop it can reconnect to freely. Checked on the resolved
    # u["android"] so the SIM_ANDROID_MODULES env hook above - which
    # lands after the fc resolve - is covered as well as a spec-declared
    # android: unit (DE-0035).
    if u["fc"] and not u["fc"].get("fcb_proxy") \
            and "de_mavlink" in (
                (u.get("android") or {}).get("remote_modules") or []):
        u["fc"]["fcb_proxy"] = {"port": 7760 + 10 * slot,
                                "ctrl": 7860 + 10 * slot}
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


def feed_dir(sim):
    return os.path.join(run_dir(sim), "feed")


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


def _udp_relay_range(sim):
    """The relay's resolved [lo, hi] udp_port_range - explicit config,
    else the slot-derived default. Usable on a spec (pre-gen) or a
    manifest (gen_server writes the resolved value back)."""
    ur = sim["server"]["udp_relay"]
    if ur.get("udp_port_range"):
        return ur["udp_port_range"]
    n_pairs = sum(1 for u in sim["units"] if "de_mavlink" in u["modules"])
    return [UDP_RELAY_PORT_BASE + 40 * sim["slot_base"],
            UDP_RELAY_PORT_BASE + 40 * sim["slot_base"]
            + max(4, 2 * n_pairs) - 1]


def used_ports(sim):
    """Every local port a sim binds: unit s2s + SITL ports, and (for
    engine-run servers) the server/auth ports. The external server's own
    port is remote - only its local bridge control_port counts."""
    ports = set()
    srv = sim["server"]
    ports.add(srv["control_port"])
    if srv["type"] != "external":
        ports.add(srv["port"])
        if srv["auth"]:
            ports.update((srv["auth_port"], srv["auth_s2s_port"]))
        ur = srv.get("udp_relay")
        if ur:
            ports.add(ur["port"])
            lo, hi = _udp_relay_range(sim)
            ports.update(range(int(lo), int(hi) + 1))
    if sim.get("render"):
        ports.update((sim["render"]["stream_port"],
                      sim["render"]["mav_port"],
                      sim["render"]["mav_ws_port"],
                      sim["render"]["web_port"],
                      sim["render"]["sink_port"]))
    for u in sim["units"]:
        ports.update(u["ports"].values())
        if u.get("link_proxy"):
            ports.update((u["link_proxy"]["port"], u["link_proxy"]["ctrl"]))
        if u["fc"]:
            ports.update((u["fc"]["pilot_port"], u["fc"]["aux_port"],
                          u["fc"]["fcb_port"]))
            if u["fc"].get("pose_port"):
                ports.add(u["fc"]["pose_port"])
            if u["fc"].get("fcb_proxy"):
                ports.update((u["fc"]["fcb_proxy"]["port"],
                              u["fc"]["fcb_proxy"]["ctrl"]))
    return ports


def port_rows(sim):
    """[(owner, port, proto)] - every local port a sim binds, labeled.
    Same coverage as used_ports(); the relay's udp_port_range is one
    row per port so `sim ports` can show how much of the block is live."""
    rows = []
    srv = sim["server"]
    rows.append(("server.control", srv["control_port"], "tcp"))
    if srv["type"] != "external":
        rows.append(("server.wss", srv["port"], "tcp"))
        if srv["auth"]:
            rows.append(("server.auth", srv["auth_port"], "tcp"))
            rows.append(("server.auth_s2s", srv["auth_s2s_port"], "tcp"))
        ur = srv.get("udp_relay")
        if ur:
            rows.append((f"{UDP_RELAY_LEASE}.wss", ur["port"], "tcp"))
            lo, hi = _udp_relay_range(sim)
            rows += [(f"{UDP_RELAY_LEASE}.range", p, "udp")
                     for p in range(int(lo), int(hi) + 1)]
    if sim.get("render"):
        r = sim["render"]
        rows += [("render.stream", r["stream_port"], "tcp"),
                 ("render.mav", r["mav_port"], "tcp"),
                 ("render.mav_ws", r["mav_ws_port"], "tcp"),
                 ("render.web", r["web_port"], "tcp"),
                 ("render.sink", r["sink_port"], "tcp")]
    for u in sim["units"]:
        rows += [(f"{u['name']}.{m}", p, "udp")
                 for m, p in u["ports"].items()]
        if u.get("link_proxy"):
            rows += [(f"{u['name']}.link_proxy", u["link_proxy"]["port"],
                      "tcp"),
                     (f"{u['name']}.link_proxy.ctrl",
                      u["link_proxy"]["ctrl"], "tcp")]
        if u["fc"]:
            fc = u["fc"]
            rows += [(f"{u['name']}.sitl.pilot", fc["pilot_port"], "tcp"),
                     (f"{u['name']}.sitl.fcb", fc["fcb_port"], "tcp"),
                     (f"{u['name']}.sitl.aux", fc["aux_port"], "tcp")]
            if fc.get("pose_port"):
                rows.append((f"{u['name']}.sitl.pose",
                             fc["pose_port"], "tcp"))
            if fc.get("fcb_proxy"):
                rows += [(f"{u['name']}.fcb_proxy",
                          fc["fcb_proxy"]["port"], "tcp"),
                         (f"{u['name']}.fcb_proxy.ctrl",
                          fc["fcb_proxy"]["ctrl"], "tcp")]
    return rows


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
    (_names_must_be_unique)."""
    ports = used_ports(sim)
    names = {u["name"] for u in sim["units"]}
    labels = set(claimed_camera_labels(sim))
    for other in live_sim_names():
        if other == sim["name"]:
            continue
        m = load_manifest(other)
        clash = sorted(ports & used_ports(m))
        lclash = sorted(labels & set(claimed_camera_labels(m)))
        dup = (sorted(names & {u["name"] for u in m["units"]})
               if _names_must_be_unique(sim, m) else [])
        if clash or dup or lclash:
            raise SimError(
                f"sim {sim['name']!r} conflicts with running sim {other!r}:"
                + (f" ports {clash}" if clash else "")
                + (f" camera labels {lclash}" if lclash else "")
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


def bt_adapters():
    """[{name, up}] of the host's Bluetooth controllers (SL-0105) -
    /sys/class/bluetooth/hciN is the presence test, `hciconfig <hci>`
    the UP/RUNNING state. An absent hciconfig reports up=unknown."""
    import subprocess
    out = []
    hdir = "/sys/class/bluetooth"
    names = sorted(n for n in os.listdir(hdir)
                   if n.startswith("hci")) \
        if os.path.isdir(hdir) else []
    for n in names:
        up = None
        try:
            r = subprocess.run(["hciconfig", n], capture_output=True,
                               text=True, timeout=5)
            up = "UP RUNNING" in r.stdout
        except (OSError, subprocess.SubprocessError):
            pass
        out.append({"name": n, "up": up})
    return out


def bt_adapter_up():
    """A bt_spp bridge can serve when some controller is UP (it binds
    on every local adapter anyway)."""
    return any(a["up"] for a in bt_adapters())


def bt_rfcomm_squatters():
    """[{pid, name, sockets}] of foreign non-root processes holding an
    RFCOMM socket (SL-0105). Our bt_spp bridge binds its channel last -
    a pre-existing foreign listener on the same channel keeps winning
    the dials (the phone connects, the squatter reads MAVLink as
    garbage, resets; kdeconnectd's SPP listener is the known thief).
    Kernel delivers by channel, which /proc does not expose, so any
    foreign RFCOMM socket is reported as a candidate - on a bench they
    are rare enough that naming the process is the useful part.
    Root-owned sockets (bluetoothd's HFP/GATT profile listeners) are
    system services on their own channels and are skipped."""
    try:
        with open("/proc/net/rfcomm") as f:
            rows = [l.split() for l in f.read().splitlines()[1:]
                    if l.split()]
    except OSError:
        return []
    nonroot = {r[5] for r in rows if len(r) >= 6 and r[4] != "0"}
    if not nonroot:
        return []
    owners = {}                                # pid -> {"name", "n"}
    claimed = set()
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        fdd = f"/proc/{pid}/fd"
        try:
            fds = os.listdir(fdd)
        except OSError:
            continue
        for fd in fds:
            try:
                tgt = os.readlink(os.path.join(fdd, fd))
            except OSError:
                continue
            m = re.fullmatch(r"socket:\[(\d+)\]", tgt)
            if not m or m.group(1) not in nonroot:
                continue
            claimed.add(m.group(1))
            e = owners.setdefault(int(pid), {"name": "?", "n": 0})
            e["n"] += 1
    for pid, e in owners.items():
        try:
            with open(f"/proc/{pid}/comm") as c:
                e["name"] = c.read().strip()
        except OSError:
            pass
    out = [{"pid": p, "name": o["name"], "sockets": o["n"]}
           for p, o in sorted(owners.items()) if not pid_is_ours(p)]
    for inode in nonroot - claimed:
        out.append({"pid": 0, "name": f"unattributed (inode {inode})",
                    "sockets": 1})
    return out


def faketime_rc_path(u):
    """Per-unit FAKETIME_TIMESTAMP_FILE - `ctl clock_step` rewrites it
    at runtime, libfaketime re-reads it."""
    return os.path.join(unit_dir(u), "faketime.rc")


def bridge_proc(u):
    """The serial/udp-bridge proc name a unit's `fcb_link` kind runs
    through, or None for the plain tcp link (no bridge exists):
    fcb_tty (socat) fronts pty/host_serial, bt_spp
    (engine/bt_spp_bridge.py) fronts bt_spp, fcb_udp (socat) fronts
    udp - SL-0102/4/5/6. Every
    place that resolves the bridge's process - launch order, ctl
    fcb_unplug/fcb_plug, fault-window pid recording - goes through
    this so a new kind is wired in one spot."""
    kind = ((u.get("fc") or {}).get("fcb_link") or {}).get("kind")
    return {"pty": "fcb_tty", "host_serial": "fcb_tty",
            "bt_spp": "bt_spp", "udp": "fcb_udp"}.get(kind)


def procs_of(u):
    """Process names of a unit in launch order. fcb_proxy (SL-0008) rides
    between sitl and the modules: it must listen before de_mavlink's
    fcb_connection_uri is dialed. The serial bridge (fcb_tty / bt_spp -
    bridge_proc) follows it: the bridge waits
    for the proxy's ctrl port,
    arms the link baud as a rate rule, then execs - and it must exist
    before de_mavlink opens the device. link_proxy (SL-0009) starts
    before the modules too: it is what de_comm dials for the comm server.
    udp_proxy_<m> (SL-0010) goes right before its module so the module's
    s2s dial hits the proxy."""
    out = ["sitl"] if u["fc"] else []
    if u["fc"] and u["fc"].get("fcb_proxy"):
        out.append("fcb_proxy")
    if u["fc"] and bridge_proc(u):
        out.append(bridge_proc(u))
    if u.get("link_proxy"):
        out.append("link_proxy")
    for m in u["modules"]:
        if "udp_proxy_" + m in u["ports"]:
            out.append("udp_proxy_" + m)    # SL-0010: proxy before its module
        if "DE_PROC_STAT_PATH" in ((u.get("env") or {}).get(m) or {}):
            out.append(f"fake_proc_{m}")    # SL-0067: writer before its reader
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
        spec = catalog.get(module_base(m, catalog))
        if m == "de_comm" or spec is None:
            continue
        if spec.get("ready_re"):
            out.append((m, spec["ready_re"], "journal"))
        if spec.get("ready_log_re"):
            out.append((m, spec["ready_log_re"], "log"))
    return out


# ---------------------------------------------------------------- generation

def _fill(v, ctx):
    """Fill "{key}" placeholders in catalog `set`/`wire` values. A string
    that is exactly one placeholder takes the value's own type (ports
    stay ints)."""
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


def _module_env(u, m):
    """Merged launch env for module m of unit u: DE_BOARD_ID defaults to
    the unit name; the unit's `env: {<module>: {VAR: value}}` map
    overrides or adds vars (SL-0067). Values resolve the {unit} and
    {unit_dir} placeholders."""
    merged = {"DE_BOARD_ID": u["name"]}
    for k, v in (u.get("env") or {}).get(m, {}).items():
        merged[str(k)] = str(v).replace("{unit}", u["name"]) \
                               .replace("{unit_dir}", unit_dir(u))
    return sorted(merged.items())


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

    if u.get("android") and not u["android"].get("device_ip"):
        # filled in by launcher.up before generate(); probing here too so
        # `./sim show` / direct gen on a connected device stays correct
        import android as _android
        if _android.adb_ok(u):
            u["android"]["device_ip"] = _android.device_ip(u)

    # ---- de_comm ----
    cfg = load_config(os.path.join(TEMPLATES_DIR, "de_comm.json"))
    cfg = deep_merge(cfg, u["config"].get("de_comm"))
    cfg.update({
        "s2s_udp_listening_port": str(u["ports"]["de_comm"]),
        "use_unix_socket": False,          # fixed /run|/tmp path, not per-unit
        "unitID": u["unitID"],
        "de_plan_file": os.path.join(udir, "de_plan.json"),
        "de_package_file": os.path.join(udir, "de_package.json"),
        "de_run_file": os.path.join(udir, "de_run.json"),
        "de_journal_dir": journal_dir(u) + "/",
    })
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
                        if u.get("link_proxy") else srv["port"],
                    # SL-0159: the template carries the real stack's
                    # auth endpoint (19408). With a live authenticator
                    # on this host an auth-off unit would probe a foreign
                    # server and get rejected - point it at this sim's
                    # reserved auth slot instead (nothing serves it when
                    # auth is off: connection refused, same as before).
                    "auth_ip": srv["auth_ip"], "auth_port": srv["auth_port"]})
    if u.get("android") and "de_comm" in u["android"]["remote_modules"]:
        # SL-0095: de_comm runs on the phone - it dials the host's
        # servers over the LAN, and listens on the phone for any
        # host-side modules of the same unit
        import android as _android
        hip = _android.host_ip()
        if hip:
            if srv["auth"]:
                cfg["auth_ip"] = hip
            else:
                cfg["local_comm_server_ip"] = hip
    if (u.get("android") or {}).get("remote_modules"):
        # DE-0036: any remote module means some s2s leg crosses the
        # LAN. A datagram bigger than the link MTU IP-fragments and
        # Android drops fragments, so de_comm's capability answers
        # (multi-KB de.cap payloads) never reach a host-side module -
        # cap chunks at a WAN-safe size; the bus reassembles them
        cfg["s2s_udp_packet_size"] = "1400"
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
        "auth_ip": cfg.get("auth_ip", srv["auth_ip"]),
        "auth_port": cfg.get("auth_port", srv["auth_port"]),
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
    env = [("DE_BOOT_ID_FILE", boot_id)] + _module_env(u, "de_comm")
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
    for m in u["modules"]:
        if m != "de_comm":
            gen_module(u, sim, m, catalog)
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
        # serial5 (tcp:5 -> 5765+10k) only exists on units with a
        # `pose`-flagged catalog module - a dedicated pose feed for the
        # P5-08 stubs; SERIAL3 stays a GPS on every unit. serial4 is the
        # P5-13 render fan-out (udpclient to the render.mav_port)
        serial5 = " --serial5 tcp:5" if fc.get("pose_port") else ""
        serial4 = (" --serial4 udpclient:127.0.0.1:%d" % fc["mav_port"]
                   if fc.get("mav_port") else "")
        speedup = (" --speedup %g" % fc["speedup"]
                   if fc.get("speedup", 1) != 1 else "")
        write_text(os.path.join(udir, "launch.sitl.sh"),
                   "#!/bin/bash\ncd '%s'\n'%s' -w -M %s -I %d --sysid %d "
                   "--home %.7f,%.7f,%.1f,%.0f --defaults '%s' "
                   "--serial0 tcp:0 --serial1 tcp:%d --serial2 tcp:2%s%s%s &\n"
                   "child=$!\necho $child > '%s'\nwait $child\n"
                   % (sdir, sitl_bin, fc["model"], fc["instance"], fc["sysid"],
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
            # an external client (SL-0104/5/6) may dial the listen port
            # from off-host - bind every interface then; a local
            # de_mavlink only ever needs loopback. The ctrl port stays
            # loopback-only in all cases.
            _launch_script(
                u, "fcb_proxy", pdir,
                "'%s' -u '%s' --unit %s --listen %d --ctrl %d "
                "--target 127.0.0.1:%d --statedir '%s' --seed %d%s"
                % (sys.executable, os.path.join(ENGINE_DIR, "fcb_proxy.py"),
                   u["name"], px["port"], px["ctrl"], fc["fcb_port"],
                   pdir, u["slot"],
                   " --bind 0.0.0.0"
                   if u.get("fc_client") == "external"
                   or "de_mavlink" in ((u.get("android") or {})
                                       .get("remote_modules") or [])
                   else ""))

        # ---- fcb serial link (SL-0102 pty, SL-0104 host_serial) ----
        # `fcb_link: {kind: pty}` gives de_mavlink a real serial device:
        # socat bridges <unit>/fcb.tty <-> the fcb proxy's listen port.
        # `fcb_link: {kind: host_serial, device: /dev/ttyUSBn}` puts a
        # real host adapter on the same spot instead - the wire an
        # `fc_client: external` device attaches to. The wrapper waits
        # for the proxy's ctrl port and arms the link's baud as a
        # proxy `rate` rule (a real UART cannot move bytes faster)
        # before dialing - both repeat per reconnect, so a proxy
        # restart re-presents the same device path.
        fl = fc.get("fcb_link")
        if fl and fl["kind"] in ("pty", "host_serial"):
            if fl["kind"] == "pty":
                peer = "pty,raw,echo=0,link='%s'" % fl["tty"]
            else:
                # ispeed/ospeed, not b<baud>: b encodes the speed into
                # c_cflag which ptys reject with EINVAL - ispeed/ospeed
                # set the termios speed fields and work on both ptys
                # and real USB-serial adapters
                peer = "'%s',raw,echo=0,ispeed=%d,ospeed=%d" \
                    % (fl["device"], fl["baud"], fl["baud"])
            pdir = os.path.join(udir, "fcb_tty")
            os.makedirs(pdir, exist_ok=True)
            # the `: '<pdir>'` marker keeps the exec'd bash -c's cmdline
            # inside RUN_DIR - pid_is_ours/stray-sweep must find this
            # wrapper (pty's link= happens to carry it; host_serial's
            # device path does not)
            _launch_script(
                u, "fcb_tty", pdir,
                "bash -c \": '%s'; while true; do until (exec 3<>/dev/tcp/"
                "127.0.0.1/%d && printf 'rate %d\\n' >&3 && exec 3>&-) "
                "2>/dev/null; do sleep 1; done; socat -d -d "
                "%s tcp:127.0.0.1:%d; sleep 1; "
                "done\""
                % (pdir, px["ctrl"], fl["baud"], peer, px["port"]))
        elif fl and fl["kind"] == "bt_spp":
            # SL-0105: a host-side RFCOMM SPP server bridged to the
            # proxy. The bridge re-execs under a bluetooth-capable
            # python when needed; its --statedir arg also marks the
            # cmdline for pid_is_ours/stray-sweep. The wrapper retries
            # on exit - adapter hiccup or fcb_plug restart both end up
            # listening on the same channel again.
            pdir = os.path.join(udir, "bt_spp")
            os.makedirs(pdir, exist_ok=True)
            _launch_script(
                u, "bt_spp", pdir,
                "bash -c \": '%s'; while true; do '%s' '%s' "
                "--channel %d --target %d --ctrl %d "
                "--statedir '%s'%s; sleep 1; done\""
                % (pdir, sys.executable,
                   os.path.join(ENGINE_DIR, "bt_spp_bridge.py"),
                   fl["channel"], px["port"], px["ctrl"], pdir,
                   " --baud %d" % fl["baud"] if fl.get("baud")
                   else ""))
        elif fl and fl["kind"] == "udp":
            # SL-0106: the client is a UDP listener, so a socat bridge
            # (proc fcb_udp) plays the active side - it pushes the
            # proxy's byte stream to `peer` (phone-ip:port) on a
            # connected UDP socket and pipes the client's replies back
            # into the proxy. Its UDP source port is ephemeral, so a
            # replug just starts it again on the same peer. The
            # wrapper waits for the proxy's ctrl port first - socat
            # must not dial a proxy that is not up yet.
            pdir = os.path.join(udir, "fcb_udp")
            os.makedirs(pdir, exist_ok=True)
            _launch_script(
                u, "fcb_udp", pdir,
                "bash -c \": '%s'; while true; do until (exec 3<>/dev/tcp/"
                "127.0.0.1/%d && exec 3>&-) 2>/dev/null; do sleep 1; "
                "done; socat -d -d UDP4:%s tcp:127.0.0.1:%d; "
                "sleep 1; done\""
                % (pdir, px["ctrl"], fl["peer"], px["port"]))
    # the resolved config stays on the manifest unit - camera bindings,
    # label claims and ctl add_module's inheritance all read it there
    u.pop("parm_files", None)




def gen_module(u, sim, m, catalog):
    """Generate the run-dir artifacts of one generic s2s module: its
    merged <m>.config.json, optional local file and launch.<m>.sh.
    Shared by gen_unit (every module at bring-up) and ctl add_module
    (one proc on a live unit)."""
    udir = unit_dir(u)
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
        # SL-0102: `fcb_link: {kind: pty}` turns the dial into a serial
        # open of the unit's socat-bridged fcb.tty (dynamic: false -
        # the device is promised present at start, like real hardware)
        link = u["fc"].get("fcb_link") or {}
        if link.get("kind") == "pty":
            ctx["fcb_serial_path"] = link["tty"]
            ctx["fcb_connection_uri"] = {"type": "serial",
                                         "port": link["tty"],
                                         "baudrate": link["baud"],
                                         "dynamic": False}
        else:
            ctx["fcb_connection_uri"] = {"ip": "127.0.0.1",
                                         "port": ctx["fcb_connect_port"],
                                         "type": "tcp"}
        # SL-0095: a phone-side de_mavlink dials the host's fcb port over
        # the LAN (SITL already binds INADDR_ANY - HIL.md Q1)
        if u.get("android") and "de_mavlink" in u["android"]["remote_modules"]:
            import android as _android
            if _android.host_ip():
                ctx["fcb_connection_uri"] = {
                    "ip": _android.host_ip(),
                    "port": ctx["fcb_connect_port"], "type": "tcp"}
    spec = catalog[module_base(m, catalog)]
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
    os.makedirs(mdir, exist_ok=True)
    menv = (u.get("env") or {}).get(m) or {}
    if "DE_PROC_STAT_PATH" in menv:
        # SL-0067: a live fake /proc/stat for this module - the
        # writer rewrites it every tick from the load percent in
        # <mdir>/cpu_load.json (a scenario write_file flips it)
        stat_path = str(menv["DE_PROC_STAT_PATH"]) \
            .replace("{unit}", u["name"]) \
            .replace("{unit_dir}", udir)
        os.makedirs(os.path.dirname(stat_path), exist_ok=True)
        _launch_script(
            u, f"fake_proc_{m}", udir,
            f"'{sys.executable}' "
            f"'{os.path.join(SIM_ROOT, 'stubs', 'fake_proc_stat.py')}'"
            f" --out '{stat_path}'"
            f" --ctl '{os.path.join(mdir, 'cpu_load.json')}'")
    mcfg = load_config(module_template(spec))
    mcfg = deep_merge(mcfg, _fill(spec.get("set") or {},
                                  dict(ctx, module=m)))
    mcfg = deep_merge(mcfg, u["config"].get(m))
    # SL-0010: a proxied module aims at its udp_proxy port instead
    # of de_comm's - engine wiring, so block/unit overlays cannot
    # break it either way
    target = u["ports"].get("udp_proxy_" + m, u["ports"]["de_comm"])
    target_ip = "127.0.0.1"
    listen_ip = "127.0.0.1"
    if u.get("android"):
        import android as _android
        remote = u["android"]["remote_modules"]
        if m in remote:
            if "de_comm" in remote:
                # same phone - s2s stays on the device loopback
                target_ip = "127.0.0.1"
            else:
                # phone-side module -> host-side de_comm over LAN
                target_ip = _android.host_ip() or target_ip
            listen_ip = "0.0.0.0"
        elif "de_comm" in remote and u["android"].get("device_ip"):
            # de_comm is on the phone - a host-side module of the
            # same unit must dial the phone, and must bind a real
            # interface or its packets carry src=127.0.0.1 and the
            # replies loop back on the phone itself
            target_ip = u["android"]["device_ip"]
            listen_ip = "0.0.0.0"
    mcfg.update({
        "s2s_udp_target_ip": target_ip,
        "s2s_udp_target_port": str(target),
        "s2s_udp_listening_ip": listen_ip,
        "s2s_udp_listening_port": str(u["ports"][m]),
        "use_unix_socket": False,
    })
    if target_ip != "127.0.0.1":
        # DE-0036: this module's s2s datagrams cross the LAN to
        # reach de_comm - over-MTU datagrams IP-fragment and
        # phones drop fragments, so cap chunks at a WAN-safe size
        mcfg["s2s_udp_packet_size"] = "1400"
    mcfg = deep_merge(mcfg, _fill(spec.get("wire") or {},
                                  dict(ctx, port=u["ports"][m],
                                         module=m)))
    if m == "de_mavlink" and ctx.get("fcb_serial_path"):
        # SL-0102: the serial uri replaces the template's tcp keys
        # outright - deep_merge would keep its ip: 0.0.0.0 behind
        mcfg["fcb_connection_uri"] = ctx["fcb_connection_uri"]
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
    if m == "de_mavlink" and ctx.get("fcb_serial_path"):
        # SL-0102: the pty link file appears when the unit's socat
        # bridge (proc fcb_tty) is running - de_mavlink opens the
        # device once at startup, so hold it until the link exists
        cmd = (f"bash -c \"until [ -e '{ctx['fcb_serial_path']}' ]; "
               f"do sleep 1; done; exec {cmd}\"")
    _launch_script(u, m, mdir, cmd, spec.get("start_delay", 0),
                   env=tuple(_module_env(u, m)))



def _server_script(sim, repo, name, cfg):
    """Launch script for a Rust server: run in its repo (ssl_local/ and
    the --config path are resolved there); the extra --sim_run arg is
    ignored by the binary but marks the pid as ours."""
    rel = os.path.relpath(cfg, repo)
    path = os.path.join(server_dir(sim["name"]), f"launch.{name}.sh")
    write_text(path,
               "#!/bin/bash\ncd '%s'\necho $$ > '%s'\n"
               "exec '%s' --config=%s --sim_run=%s\n"
               % (repo, pid_path(sim["name"], "server", name),
                  server_bin(repo, SERVER_BINS[name]), rel,
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
        # SL-0095: remote android units dial the host over the LAN
        if any(u.get("android") for u in sim["units"]):
            extra += " --bind 0.0.0.0"
        write_text(os.path.join(sdir, "launch.fake_server.sh"),
                   "#!/bin/bash\necho $$ > '%s'\nexec python3 -u '%s' --port %d "
                   "--seed %d --cert '%s' --key '%s' --sim '%s'%s --pretty\n"
                   % (pid_path(sim["name"], "server", "fake_server"),
                      os.path.join(ENGINE_DIR, "fake_fleet_server.py"),
                      srv["port"], srv["seed"], cert_path(sim["name"]),
                      key_path(sim["name"]), sim["name"], extra), 0o755)
        return ["fake_server"]

    ssl_dir = os.path.join(SERVER_REPO, "ssl_local")
    # SL-0095: phone-side units dial the host over the LAN - the spawned
    # servers must bind all interfaces and advertise the host IP
    import android as _android
    has_remote = any(u.get("android") for u in sim["units"])
    host_ip = _android.host_ip() if has_remote else None

    if srv["auth"]:
        acfg = load_config(os.path.join(AUTH_REPO, "server.config"))
        acfg.update({
            "server_ip": "0.0.0.0" if has_remote else "127.0.0.1",
            "server_port": srv["auth_port"],
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
        _server_script(sim, AUTH_REPO, "de_auth", cfg)
        procs.append("de_auth")

    scfg = load_config(os.path.join(SERVER_REPO, "server.config"))
    scfg.update({
        "server_ip": "0.0.0.0" if has_remote else "127.0.0.1",
        "server_port": srv["port"],
        "public_host": host_ip or "127.0.0.1",
        "local_server_enabled": not srv["auth"],
        "s2s_ws_target_ip": "127.0.0.1",
        "s2s_ws_target_port": str(srv["auth_s2s_port"]),
        "ssl_key_file": os.path.join(ssl_dir, "ssl_airgap/domain.key"),
        "ssl_cert_file": os.path.join(ssl_dir, "ssl_airgap/domain.crt"),
        "ca_cert_path": os.path.join(ssl_dir, "ssl_airgap/root.crt"),
        "s2s_my_private_key": os.path.join(ssl_dir, "DE_ServerComm_private.pem"),
        "command_plugin": [],
        "enable_storage_server": False,
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
        if not os.path.isfile(
                server_bin(UDP_RELAY_REPO, SERVER_BINS["udp_relay"])):
            raise SimError(f"{UDP_RELAY_REPO}: run `cargo build "
                           "--release` there first (the relay binary "
                           "is missing)")
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
            "public_host": host_ip or "127.0.0.1",
            "udp_port_range": [int(lo[0]), int(lo[1])],
            "gcs_peer_policy": urelay["gcs_peer_policy"],
            "state_file": os.path.join(sdir, "udp_relay_state.json"),
            "log_directory": os.path.join(sdir, "logs") + "/",
        })
        cfg = os.path.join(sdir, "udp_relay.config")
        write_json(cfg, rcfg)
        _server_script(sim, UDP_RELAY_REPO, "udp_relay", cfg)
        procs.append("udp_relay")
        scfg["udp_relay"] = [{
            "host": "127.0.0.1", "port": urelay["port"],
            "id": rcfg["server_id"], "ssl": bool(urelay.get("ssl", True)),
        }]
        scfg["s2s_shared_secret_file"] = secret_path

    cfg = os.path.join(sdir, "de_server.config")
    write_json(cfg, scfg)
    _server_script(sim, SERVER_REPO, "de_server", cfg)
    procs.append("de_server")

    # one persistent GCS session on the real server, exposing the same
    # control protocol as the fake server (ctl.py talks to either)
    procs += _bridge_script(sim)
    return procs


def binaries_needed(sim, catalog):
    out = set()
    if any("de_comm" not in ((u.get("android") or {}).get("remote_modules")
                             or []) for u in sim["units"]):
        out.add(resolve_bin("de_comm"))
    if sim["server"]["type"] == "real":
        out.add(server_bin(SERVER_REPO, SERVER_BINS["de_server"]))
        if sim["server"]["auth"]:
            out.add(server_bin(AUTH_REPO, SERVER_BINS["de_auth"]))
        if sim["server"].get("udp_relay"):
            out.add(server_bin(UDP_RELAY_REPO,
                              SERVER_BINS["udp_relay"]))
    for u in sim["units"]:
        remote = set((u.get("android") or {}).get("remote_modules") or [])
        for m in u["modules"]:
            if m != "de_comm" and m not in remote:
                out.add(resolve_bin(m, catalog[module_base(m, catalog)]))
        if u["fc"]:
            out.add(os.path.join(ARDUPILOT_DIR, "build", "sitl", "bin",
                                 FC_KINDS[u["fc"]["vehicle"]]["bin"]))
    return sorted(out)


def check_binaries(sim):
    missing = [b for b in binaries_needed(sim, load_catalog())
               if not os.path.isfile(b)]
    if missing:
        # server repos build with cargo, not sim.local.env keys
        lines = [b + ("   # cd %s && cargo build --release"
                      % b.split("/target/release/")[0]
                      if "/target/release/" in b else "")
                 for b in missing]
        raise SimError("missing binaries - build them, or point sim.local.env "
                       "at them (see sim.local.env.example):\n  "
                       + "\n  ".join(lines))


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
           % (render_page_port(render), render["stream_port"],
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
           % (render_page_port(render), render["stream_port"],
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
    other process.

    SL-0154: when a dev stack `provides: render_web` is live (the shared
    Mavlink3DMap vite page, e.g. de_mavlink3dmap), the sim starts no
    `web` proc and bakes render.page_port = the stack's port into the
    manifest instead - the page is slot-agnostic; ?mavlink=/ ?world=
    in the viewer URL keep selecting this sim's own feeds. Stack down
    or absent -> the per-sim vite exactly as before."""
    render = sim.get("render") or {}
    if not render or render["mode"] not in ("viewer", "cameras", "physics",
                                            "full") \
            or (render["headless"] and render["mode"] != "physics"):
        return []
    m3d = MAVLINK3DMAP_DIR
    bridge = os.path.join(m3d, "backend", "src", "udp2websocket.js")
    shared = shared_render_web_port()
    # npm workspaces hoist vite to the repo root; a standalone
    # frontend install keeps it under frontend/node_modules. The shared
    # stack owns the page, so a missing local vite only fails when the
    # sim would launch its own
    vite_js = next((p for p in (
            os.path.join(m3d, "node_modules", "vite", "bin", "vite.js"),
            os.path.join(m3d, "frontend", "node_modules", "vite",
                         "bin", "vite.js"))
            if os.path.isfile(p)), None)
    for p in [bridge] + ([] if shared is not None else [vite_js]):
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
    procs = ["mav_bridge"]
    if shared is None:
        _render_script(sim, "web", os.path.join(m3d, "frontend"),
                       "'%s' --require '%s' '%s' --port %d --strictPort "
                       "--host 127.0.0.1"
                       % (NODE, marker, vite_js, render["web_port"]))
        procs.append("web")
    else:
        render["page_port"] = shared
    if render["mode"] in ("cameras", "full"):
        procs += _camera_render_procs(sim, render, rdir, marker, m3d,
                                      vite_js)
    if render["mode"] == "physics":
        procs += _physics_render_procs(sim, render, rdir)
    return procs


# ------------------------------------------------- virtual cameras (DE-0050)

_FEED_SRC_ALIASES = {"clock": "testsrc2", "colors": "rgbtestsrc"}
V4L2_CAP_VIDEO_CAPTURE = 0x1
_VIDIOC_QUERYCAP = 0x80685600


def _camera_feed_spec(raw):
    """`camera_feed: <src>` or {source, size, rate, pix_fmt} -> a
    normalized dict, or None. `src` aliases clock/colors map to the
    lavfi sources video_clock.sh/video_colors.sh use; anything else is
    passed to ffmpeg's lavfi -i verbatim."""
    if raw is None or raw is False:
        return None
    if isinstance(raw, str):
        raw = {"source": raw}
    if not isinstance(raw, dict):
        raise SimError("camera_feed: must be a source name or a mapping")
    bad = set(raw) - {"source", "size", "rate", "pix_fmt"}
    if bad:
        raise SimError(f"camera_feed: unknown keys {sorted(bad)}")
    src = _FEED_SRC_ALIASES.get(str(raw.get("source", "testsrc2")),
                                str(raw.get("source", "testsrc2")))
    return {"source": src,
            "size": str(raw.get("size", "640x480")),
            "rate": int(raw.get("rate", 30)),
            "pix_fmt": str(raw.get("pix_fmt", "yuyv422"))}


def camera_video_indices(u, label_map=None):
    """Sorted /dev/videoN indices this unit's de_camera instances open:
    camera.camera_list `device_num` entries, `device_name` card labels
    resolved through v4l2_label_map (pass a fixture map in tests;
    unresolved labels surface via camera_missing_labels), plus the
    camera_start_index..camera_end_index scan range (falling back to
    the legacy flat fields, like the module does). [] for non-camera
    units."""
    label_map = label_map if label_map is not None else v4l2_label_map()
    idx = set()
    for m in u.get("modules", []):
        if module_base(m) != "de_camera":
            continue
        dcfg = (u.get("config") or {}).get(m) or {}
        cfg = dcfg.get("camera") or {}
        for e in cfg.get("camera_list") or []:
            if not isinstance(e, dict):
                continue
            if isinstance(e.get("device_num"), int):
                idx.add(e["device_num"])
            elif e.get("device_name") in label_map:
                idx.add(int(
                    label_map[e["device_name"]].rsplit("video", 1)[-1]))
        a = cfg.get("camera_start_index", dcfg.get("camera_start_index"))
        b = cfg.get("camera_end_index", dcfg.get("camera_end_index"))
        if a is not None or b is not None:
            a, b = int(a or 0), int(b if b is not None else a)
            idx.update(range(a, b + 1))
    return sorted(idx)


def camera_video_labels(u):
    """Card labels this unit's de_camera instances bind via camera_list
    `device_name` - the pool labels it claims. [] for non-camera
    units."""
    out = []
    for m in u.get("modules", []):
        if module_base(m) != "de_camera":
            continue
        dcfg = (u.get("config") or {}).get(m) or {}
        for e in (dcfg.get("camera") or {}).get("camera_list") or []:
            if isinstance(e, dict) and e.get("device_name"):
                out.append(e["device_name"])
    return out


def camera_missing_labels(sim, label_map=None):
    """Sorted camera-binding labels the sim claims that no host V4L2
    device carries (v4l2_label_map keys)."""
    label_map = label_map if label_map is not None else v4l2_label_map()
    return sorted({l for u in sim["units"]
                   for l in camera_video_labels(u)
                   if l not in label_map})


def camera_feed_indices(u, label_map=None):
    """device indices a feed producer may write into for this unit:
    camera_video_indices minus real (non-loopback) devices - a real
    webcam/capture card never gets ffmpeg streamed into it."""
    return [i for i in camera_video_indices(u, label_map)
            if not os.path.exists(f"/dev/video{i}")
            or video_is_loopback(i)]


def camera_binding_desc(u, label_map=None):
    """`cam=...` text for `sim show`: every de_camera instance's bound
    device - DE-CAM1(/dev/video2) for a resolved label, /dev/videoN for
    a device_num, the bare label when unresolved, the scan range for
    index-mode units."""
    label_map = label_map if label_map is not None else v4l2_label_map()
    out = []
    for m in u.get("modules", []):
        if module_base(m) != "de_camera":
            continue
        cfg = (u.get("config") or {}).get(m) or {}
        cam = cfg.get("camera") or {}
        for e in cam.get("camera_list") or []:
            if not isinstance(e, dict):
                continue
            if e.get("device_name"):
                l = e["device_name"]
                out.append(l + (f"({label_map[l]})"
                                if l in label_map else ""))
            elif e.get("device_num") is not None:
                out.append(f"/dev/video{e['device_num']}")
        if not out and (cam.get("camera_start_index") is not None
                        or cam.get("camera_end_index") is not None):
            a = cam.get("camera_start_index", 0)
            b = cam.get("camera_end_index", a)
            out.append(f"video{a}-{b}" if a != b else f"video{a}")
    return ",".join(out)


def free_cam_label(exclude=None):
    """first DE-CAM-k label present on the host (v4l2 pool) and not
    claimed by any live manifest's camera bindings - runtime
    allocation for ctl add_module (live state, not spec)."""
    taken = set(exclude or ())
    for n in live_sim_names():
        taken.update(claimed_camera_labels(load_manifest(n)))
    for label in sorted(v4l2_label_map()):
        if re.fullmatch(r"DE-CAM\d+", label) and label not in taken:
            return label
    return None


def claimed_camera_labels(sim):
    """Card labels this sim claims on the machine: every de_camera
    device_name binding plus the render units' DE-SRC/DE-AI pairs -
    machine-global, clashes checked like ports."""
    out = set()
    for u in sim["units"]:
        out.update(camera_video_labels(u))
        out.update(u.get("cam_labels") or ())
    return sorted(out)


def video_is_loopback(idx):
    """True when /dev/video<idx> is a v4l2loopback virtual device
    (loopback nodes live under /sys/devices/virtual/video4linux/)."""
    return os.path.isdir(
        f"/sys/devices/virtual/video4linux/video{idx}")


def video_device_caps(path):
    """The V4L2 device_caps bitmask of a video node
    (VIDIOC_QUERYCAP), or None when it cannot be queried. With
    exclusive_caps a loopback device advertises CAPTURE only while a
    producer streams into it."""
    import fcntl
    import struct
    try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
    except OSError:
        return None
    try:
        buf = fcntl.ioctl(fd, _VIDIOC_QUERYCAP, bytes(104))
        return struct.unpack_from("<I", buf, 84)[0]
    except OSError:
        return None
    finally:
        os.close(fd)


def video_capture_capable(idx):
    c = video_device_caps(f"/dev/video{idx}")
    return c is not None and bool(c & V4L2_CAP_VIDEO_CAPTURE)


def gen_feeds(sim):
    """camera_feed (DE-0050): per camera unit + configured /dev/videoN
    a pid-tracked ffmpeg lavfi producer under var/run/<sim>/feed/, so
    exclusive_caps loopback devices advertise CAPTURE before de_camera
    scans. Procs are `feed.<unit>_<idx>`; the pidfile pid is the launch
    bash (its cmdline names the launch script under var/run, which is
    what pid_is_ours matches) - ffmpeg rides `&`+`wait` like SITL, and
    the TERM trap kills it on `sim down`. [] without a camera_feed."""
    feed = sim.get("camera_feed")
    if not feed:
        return []
    os.makedirs(feed_dir(sim["name"]), exist_ok=True)
    os.makedirs(pids_dir(sim["name"]), exist_ok=True)
    procs = []
    for u in sim["units"]:
        for idx in camera_feed_indices(u):
            proc = f"{u['name']}_{idx}"
            gen_feed_script(sim, proc, idx)
            procs.append(proc)
    return procs


def gen_feed_script(sim, proc, idx):
    """write launch.<proc>.sh for one feed.<unit>_<idx> - the
    single-feed half of gen_feeds, shared with ctl add_module."""
    feed = sim.get("camera_feed") or {}
    rdir = feed_dir(sim["name"])
    os.makedirs(rdir, exist_ok=True)
    cmd = (f"'{FFMPEG_BIN}' -hide_banner -loglevel warning "
           f"-f lavfi -i "
           f"'{feed['source']}=size={feed['size']}"
           f":rate={feed['rate']}' -pix_fmt {feed['pix_fmt']} "
           f"-c:v rawvideo -f v4l2 '/dev/video{idx}'")
    path = os.path.join(rdir, f"launch.{proc}.sh")
    write_text(path,
               "#!/bin/bash\ncd '%s'\necho $$ > '%s'\n%s &\n"
               "trap 'kill %%1 2>/dev/null' TERM\nwait\n"
               % (rdir, pid_path(sim["name"], "feed", proc),
                  cmd), 0o755)


def camera_missing_nodes(sim):
    """Sorted /dev/videoN indices a camera unit will open that do not
    exist on this host."""
    out = set()
    for u in sim["units"]:
        for i in camera_video_indices(u):
            if not os.path.exists(f"/dev/video{i}"):
                out.add(i)
    return sorted(out)


def vcam_plan(sim):
    """DE-0050: the v4l2loopback setup a camera sim needs, as a dict:
    `needed` (every /dev/videoN index its de_camera units open),
    `real`/`loopback` (indices already present, split by driver),
    `missing` (indices to create), `loaded` (module state) and `cmd`
    (the modprobe argv for the missing indices). `real` devices are
    capture devices already - never touched."""
    needed = sorted({i for u in sim["units"]
                     for i in camera_video_indices(u)})
    real, loopback = [], []
    for i in needed:
        if not os.path.exists(f"/dev/video{i}"):
            continue
        (loopback if video_is_loopback(i) else real).append(i)
    missing = [i for i in needed
               if not os.path.exists(f"/dev/video{i}")]
    labels = [f"DE-VCAM-{i}" for i in missing]
    cmd = (["modprobe", "v4l2loopback",
            f"devices={len(missing)}",
            "video_nr=" + ",".join(str(i) for i in missing),
            "card_label=" + ",".join(labels),
            "exclusive_caps=" + ",".join("1" for _ in missing)]
           if missing else None)
    return {"needed": needed, "real": real, "loopback": loopback,
            "missing": missing, "loaded": _module_loaded("v4l2loopback"),
            "cmd": cmd}


def _module_loaded(name):
    import subprocess
    try:
        out = subprocess.run(["lsmod"], capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return False
    return any(ln.split() and ln.split()[0] == name
               for ln in out.splitlines())


def vcam_run(sim, print_only=False, reload=False):
    """`./sim vcam <sim>`: print the plan, then sudo modprobe the
    missing loopback devices. reload=True unloads the module first
    (needed when the module is already loaded without the indices the
    sim wants - reload drops every existing loopback device)."""
    import subprocess
    plan = vcam_plan(sim)
    if not plan["needed"]:
        print(f"sim {sim['name']}: no camera units - nothing to do")
        return 0
    print(f"sim {sim['name']}: camera indices {plan['needed']}"
          + (f"  (loopback {plan['loopback']})"
             if plan["loopback"] else "")
          + (f"  (real device(s) {plan['real']} - left alone)"
             if plan["real"] else ""))
    if not plan["missing"]:
        print("every configured index already exists - nothing to do")
        return 0
    cmdline = "sudo " + " ".join(plan["cmd"])
    print(f"missing: {plan['missing']} -> {cmdline}")
    if plan["loaded"] and not reload:
        print("v4l2loopback is already loaded - its existing devices "
              "are kept; pass --reload to replace them")
        return 0
    if print_only:
        return 0
    if plan["loaded"]:
        if subprocess.call(["sudo", "modprobe", "-r", "v4l2loopback"]):
            sys.exit("could not unload v4l2loopback (a device is open)")
    return subprocess.call(["sudo"] + plan["cmd"])


def vcam_list():
    """SL-0170 `./sim vcam --list`: every /dev/videoN with its card
    name, kind (real vs loopback), the live-sim unit.proc bound to it
    and the feed.<unit>_<idx> producing into it - the answer to "which
    cam number or label is free" before add_module. Read-only and
    unprivileged; owners/feeds come from live manifests only, so a dead
    sim's stale bindings never claim a device."""
    label_map = v4l2_label_map()
    owners, feeds = {}, {}
    for n in live_sim_names():
        m = load_manifest(n)
        for u in m["units"]:
            for mod in u.get("modules", []):
                if module_base(mod) != "de_camera":
                    continue
                cam = ((u.get("config") or {}).get(mod) or {}) \
                    .get("camera") or {}
                for e in cam.get("camera_list") or []:
                    if not isinstance(e, dict):
                        continue
                    idx = e.get("device_num") \
                        if isinstance(e.get("device_num"), int) else None
                    if idx is None and e.get("device_name") in label_map:
                        idx = int(label_map[e["device_name"]]
                                  .rsplit("video", 1)[-1])
                    if idx is not None:
                        owners.setdefault(idx, set()) \
                            .add(f"{n}:{u['name']}.{mod}")
        live = read_pids(n)
        for p in m.get("feed_procs") or []:
            idx = p.rsplit("_", 1)[-1]
            if idx.isdigit():
                feeds.setdefault(int(idx), set()).add(
                    f"{n}:feed.{p}"
                    + ("" if f"feed.{p}" in live else "(dead)"))
    v4l = "/sys/class/video4linux"
    rows, free = [], []
    if os.path.isdir(v4l):
        for name in sorted((x for x in os.listdir(v4l)
                            if x.startswith("video")
                            and x[5:].isdigit()),
                           key=lambda x: int(x[5:])):
            idx = int(name[5:])
            try:
                label = open(os.path.join(v4l, name, "name")) \
                    .read().strip()
            except OSError:
                label = "?"
            loop = video_is_loopback(idx)
            owner = "+".join(sorted(owners.get(idx) or ()))
            feed = "+".join(sorted(feeds.get(idx) or ()))
            status = owner or ("free" if loop else "-")
            if not loop or owner:
                pass
            else:
                free.append(f"{label}(/dev/video{idx})")
            rows.append((f"/dev/video{idx}", label,
                         "loopback" if loop else "real", status, feed))
    if not rows:
        print("no video devices found")
        return 0
    w = max(len(r[1]) for r in rows)
    for dev, label, kind, status, feed in rows:
        line = f"{dev:<12} {label:<{w}}  {kind:<9} {status}"
        if feed:
            line += f"  feed={feed}"
        print(line)
    if free:
        print("free: " + " ".join(free))
    return 0


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
    feed_procs = gen_feeds(sim)
    manifest = {"version": 1, "name": sim["name"], "sim_file": sim_file,
                "sim_hash": sim["hash"], "owner": owner_record(),
                "server": sim["server"],
                "server_procs": server_procs,
                "render_procs": render_procs,
                "feed_procs": feed_procs,
                "camera_feed": sim.get("camera_feed"),
                "render": sim.get("render"),
                "module_overrides": dict(_MODULE_BINS),
                "home": sim["home"], "speedup": sim.get("speedup", 1),
                "units": units}
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


def _journal_shift(u, lines):
    """SL-0095: a phone-side de_comm stamps journal lines with the device
    clock - subtract the offset measured at `up` so cross-unit timing
    asserts stay honest (HIL.md Q4)."""
    off = int((u.get("android") or {}).get("clock_offset_ms") or 0)
    if not off:
        return lines
    return [(ln[0] - off,) + ln[1:] for ln in lines]


def journal_lines(u, since=None):
    """[(unix_ms, engine, mission, seq, kind, msg, file)] in event-time
    order across every journal file of the unit."""
    per_file = []
    for f in _journal_files(u):
        off = since.get(f, 0) if since else 0
        with open(f, errors="replace") as fh:
            fh.seek(off)
            per_file.append([_parse_journal_line(ln, f) for ln in fh])
    return _journal_shift(u, _merge_by_time(per_file))


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
    return _journal_shift(u, _merge_by_time(per_file))


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
