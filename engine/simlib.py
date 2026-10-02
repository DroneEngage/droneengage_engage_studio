#!/usr/bin/env python3
"""simlib.py - core of droneengage_simulation_land.

Three layers, all data:

  blocks/<name>/       a reusable unit type: which modules it runs, its
                       flight controller (if any) and config overlays per
                       module. Blocks can `extends:` another block.
  sims/<name>.yaml     a simulation: blocks composed into a fleet
                       (count, names, placement, per-unit overrides).
  tests/<name>.yaml    a scenario: a sim + steps + assertions (runner.py).

This module resolves blocks and sims, allocates ports, generates every
config + launch script under run/, writes the manifest (run/sim.json) and
holds the journal / pid helpers used by ctl.py and runner.py.

Port scheme (slot k = slot_base + unit index, default slot_base 10 so a
Phase-4A fleet on slots 1..9 can run at the same time):
    SITL -I k:  pilot 5760+10k (serial0)  aux 5762+10k (serial2)
                fcb   7660+10k (serial1, de_mavlink)   sysid k
    de_comm     62000 + 10k     other modules: catalog port_base + 10k
    server      wss 62008, control 62009 (sim `server.port`)
"""

import copy
import hashlib
import json
import os
import re
import sys

import yaml

SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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
RUN_DIR = os.path.abspath(setting("SIM_RUN_DIR", os.path.join(SIM_ROOT, "run")))
PIDS_DIR = os.path.join(RUN_DIR, "pids")
MANIFEST = os.path.join(RUN_DIR, "sim.json")
RESULTS_DIR = os.path.abspath(setting("SIM_RESULTS_DIR", os.path.join(SIM_ROOT, "results")))

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

SERVER_PORT = 62008                 # wss; control port is +1
SLOT_BASE = 10
HOME = (-35.363262, 149.165237, 584.0, 353.0)   # SITL default home
SPACING_M = 20.0                    # default: units 20 m apart, northward

CERT = os.path.join(RUN_DIR, "server", "server.crt")
KEY = os.path.join(RUN_DIR, "server", "server.key")

GCS_PARTY = "GCS_FLEET"
FULL_PERM = 0xFFFFFFFF

DE_COMM_BIN = setting("DE_COMM_BIN", os.path.join(COMM_REPO, "bin", "de_comm"))
DE_COMM_PORT_BASE = 62000

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
        return json.loads(strip_comments(open(path).read()))
    except ValueError as e:
        raise SimError(f"{path}: bad JSON: {e}")


def load_yaml(path):
    try:
        return yaml.safe_load(open(path)) or {}
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
    cat = load_yaml(os.path.join(MODULES_DIR, "catalog.yaml"))
    for name in ("de_comm", "sitl"):
        if name in cat:
            raise SimError(f"catalog.yaml: '{name}' is built in, remove it")
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
    return sorted(d for d in os.listdir(BLOCKS_DIR)
                  if os.path.isfile(os.path.join(BLOCKS_DIR, d, "block.yaml")))


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

SERVER_TYPES = ("real", "fake")


def load_sim(path_or_spec, base_dir=None):
    """Load a sim file (or an inline dict) and expand it into concrete units.
    Returns {server{}, home, slot_base, units[], hash}."""
    spec = _sim_spec(path_or_spec, base_dir)
    unknown = set(spec) - {"version", "description", "server", "home",
                           "slot_base", "units"}
    if unknown:
        raise SimError(f"sim: unknown keys {sorted(unknown)}")
    if spec.get("version", 1) != 1:
        raise SimError("sim: unsupported version")
    home = tuple(spec.get("home", HOME))
    if len(home) == 3:
        home = home + (HOME[3],)
    sim = {
        "description": spec.get("description", "").strip(),
        "server": load_server(spec.get("server") or {}),
        "home": list(home),
        "slot_base": int(spec.get("slot_base", SLOT_BASE)),
        "units": [],
    }
    catalog = load_catalog()
    blocks = {}
    if not spec.get("units"):
        raise SimError("sim: no units")
    for entry in spec["units"]:
        if isinstance(entry, str):
            entry = {"block": entry}
        bad = set(entry) - {"block", "name", "count", "at", "config", "fc"}
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
        for i, name in enumerate(names):
            at = dict(entry.get("at") or {})
            if count is not None and "at" in entry:
                at["north_m"] = at.get("north_m", 0) + SPACING_M * i
            sim["units"].append(_expand_unit(sim, b, name, entry, at, catalog))
    seen = set()
    for u in sim["units"]:
        if u["name"] in seen:
            raise SimError(f"sim: duplicate unit name {u['name']!r}")
        seen.add(u["name"])
    sim["hash"] = hashlib.sha256(json.dumps(
        [sim["server"], sim["home"], sim["units"]],
        sort_keys=True).encode()).hexdigest()[:16]
    return sim


def _sim_spec(path_or_spec, base_dir=None, _chain=()):
    """Raw sim dict with `extends: <other sim file>` applied: the parent's
    keys, deep-merged with this file's (a `units` list replaces the
    parent's, so a child can re-use a fleet and only swap the server)."""
    if isinstance(path_or_spec, dict):
        spec, here = dict(path_or_spec), base_dir or os.getcwd()
    else:
        path = os.path.abspath(path_or_spec)
        if path in _chain:
            raise SimError(f"sim extends loop at {path}")
        _chain += (path,)
        spec, here = load_yaml(path), os.path.dirname(path)
    parent = spec.pop("extends", None)
    if not parent:
        return spec
    return deep_merge(_sim_spec(os.path.join(here, parent), None, _chain), spec)


def load_server(srv):
    """server: {type: real|fake, port, auth, account{user, code}, seed}.
    real (default): the actual droneengage_server from its repo, on the
      sim's own port with a generated config. auth: false runs it in
      local_server_enabled mode (units connect directly, like
      use_local_comm_server); auth: true also starts droneengage_authenticator
      and every unit logs in with the sim account.
    fake: engine/fake_fleet_server.py - same routing plus seeded per-link
      loss / delay / cut. Use it for link-fault tests."""
    bad = set(srv) - {"type", "port", "auth", "account", "seed"}
    if bad:
        raise SimError(f"sim server: unknown keys {sorted(bad)}")
    t = srv.get("type", "real")
    if t not in SERVER_TYPES:
        raise SimError(f"sim server.type must be one of {SERVER_TYPES}")
    if t == "fake" and srv.get("auth"):
        raise SimError("sim server: auth needs type real")
    port = int(srv.get("port", SERVER_PORT))
    acc = srv.get("account") or {}
    user = acc.get("user", "sim@sim.droneengage.com")
    code = str(acc.get("code", "sim1234"))
    # droneengage_authenticator rule (hlp_strings.fn_isValidAccountName /
    # fn_isAlphanumeric): a bad one is refused as "API endpoint not found"
    if not (re.fullmatch(r"[A-Za-z0-9]+", user)
            or re.fullmatch(r"[^@\s]+@[^@\s]+\.[A-Za-z]{2,}", user)):
        raise SimError(f"server.account.user {user!r}: alphanumeric or an "
                       "e-mail with a TLD (the authenticator refuses others)")
    if not re.fullmatch(r"[A-Za-z0-9]+", code):
        raise SimError("server.account.code must be alphanumeric")
    return {
        "type": t, "port": port, "control_port": port + 1,
        "auth": bool(srv.get("auth", False)),
        "auth_port": port + 10, "auth_s2s_port": port + 11,
        "account": {"user": user, "code": code},
        "seed": int(srv.get("seed", 1337)),
    }


def _expand_unit(sim, b, name, entry, at, catalog):
    if not NAME_RX.match(name):
        raise SimError(f"unit name {name!r}: letters, digits, _ only")
    slot = sim["slot_base"] + len(sim["units"])
    u = {"name": name, "block": b["name"], "chain": b["chain"], "slot": slot,
         "party_id": name, "unitID": name,
         "modules": list(b["modules"]), "ports": {}, "fc": None,
         "config": copy.deepcopy(b["config"]),
         "parm_files": list(b["parm_files"]),
         "location": None}
    for mod, over in (entry.get("config") or {}).items():
        if mod not in u["modules"] and mod != "sitl":
            raise SimError(f"unit {name}: config for {mod!r} but block "
                           f"{b['name']} does not run it")
        u["config"][mod] = deep_merge(u["config"].get(mod, {}), over)
    u["ports"]["de_comm"] = DE_COMM_PORT_BASE + 10 * slot
    for m in u["modules"]:
        if m != "de_comm":
            u["ports"][m] = int(catalog[m]["port_base"]) + 10 * slot
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
    if fc:
        if not 1 <= slot <= 250:
            raise SimError(f"unit {name}: slot {slot} is not a valid MAVLink sysid")
        kind = FC_KINDS[fc["vehicle"]]
        u["fc"] = {
            "vehicle": fc["vehicle"],
            "model": fc.get("model", kind["model"]),
            "sysid": slot, "instance": slot,
            "pilot_port": 5760 + 10 * slot,
            "aux_port": 5762 + 10 * slot,
            "fcb_port": 7660 + 10 * slot,
            "arm_mode": fc.get("arm_mode", kind["arm_mode"]),
            "land": fc.get("land", kind["land"]),
            "defaults": fc.get("defaults", kind["defaults"]),
            "params": fc.get("params") or {},
        }
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

def load_manifest(required=True):
    if not os.path.isfile(MANIFEST):
        if required:
            sys.exit(f"no sim manifest at {MANIFEST} - run `./sim up <sim.yaml>` first")
        return None
    return json.load(open(MANIFEST))


def units_by_name(manifest=None):
    m = manifest or load_manifest()
    return {u["name"]: u for u in m["units"]}


def unit_dir(u):
    return os.path.join(RUN_DIR, u["name"])


def journal_dir(u):
    return os.path.join(unit_dir(u), "journal")


def procs_of(u):
    """Process names of a unit in launch order."""
    return (["sitl"] if u["fc"] else []) + list(u["modules"])


def ready_checks(u, catalog=None):
    """[(module, regex)] that `sim up` waits for on this unit's journal."""
    catalog = catalog or load_catalog()
    return [(m, catalog[m]["ready_re"]) for m in u["modules"]
            if m != "de_comm" and catalog.get(m, {}).get("ready_re")]


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
               % (cwd, pid_path(u["name"], proc),
                  f"sleep {delay}\n" if delay else "",
                  "".join(f"export {k}='{v}'\n" for k, v in env), cmd), 0o755)
    return path


def gen_unit(u, sim, catalog):
    udir = unit_dir(u)
    for d in ("journal", "logs"):
        os.makedirs(os.path.join(udir, d), exist_ok=True)
    srv = sim["server"]

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
        # full login: authenticator -> comm server address + temp key
        cfg.update({"use_local_comm_server": False,
                    "auth_ip": "127.0.0.1", "auth_port": srv["auth_port"],
                    "auth_verify_ssl": False,
                    "userName": srv["account"]["user"],
                    "accessCode": srv["account"]["code"]})
    else:
        cfg.update({"use_local_comm_server": True,
                    "local_comm_server_ip": "127.0.0.1",
                    "local_comm_server_port": srv["port"]})
    if not u["fc"]:
        p = u["position"]
        cfg["location"] = {
            "lat": int(round(p["lat"] * 1e7)), "lng": int(round(p["lng"] * 1e7)),
            "alt_abs": int(p["alt"] * 1000), "alt_rel": 0,
            "heading": p["yaw"], "acc_mm": 1000,
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
    _launch_script(u, "de_comm", cdir,
                   f"'{DE_COMM_BIN}' -c '{comm_cfg}' -b '{comm_local}'",
                   env=(("DE_BOOT_ID_FILE", boot_id),))

    # ---- generic s2s modules ----
    ctx = {"unit": u["name"], "comm_port": u["ports"]["de_comm"]}
    if u["fc"]:
        ctx.update({k: u["fc"][k] for k in ("fcb_port", "pilot_port", "sysid")})
    for m in u["modules"]:
        if m == "de_comm":
            continue
        spec = catalog[m]
        mdir = os.path.join(udir, m)
        mcfg = load_config(module_template(spec))
        mcfg = deep_merge(mcfg, spec.get("set"))
        mcfg = deep_merge(mcfg, u["config"].get(m))
        mcfg.update({
            "s2s_udp_target_ip": "127.0.0.1",
            "s2s_udp_target_port": str(u["ports"]["de_comm"]),
            "s2s_udp_listening_ip": "127.0.0.1",
            "s2s_udp_listening_port": str(u["ports"][m]),
            "use_unix_socket": False,
        })
        mcfg = deep_merge(mcfg, _fill(spec.get("wire") or {},
                                      dict(ctx, port=u["ports"][m])))
        cfg_path = os.path.join(mdir, f"{m}.config.json")
        write_json(cfg_path, mcfg)
        local = spec.get("local") or {}
        cmd = f"'{module_bin(m, spec)}' -c '{cfg_path}'"
        if local:
            lpath = os.path.join(mdir, local["file"])
            write_json(lpath, {"module_key": f"{u['name']}-{m}"}, indent=1)
            if local.get("arg"):
                cmd += f" {local['arg']} '{lpath}'"
        _launch_script(u, m, mdir, cmd, spec.get("start_delay", 0))

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
        write_text(os.path.join(udir, "launch.sitl.sh"),
                   "#!/bin/bash\ncd '%s'\n'%s' -w -M %s -I %d --sysid %d "
                   "--home %.7f,%.7f,%.1f,%.0f --defaults '%s' "
                   "--serial0 tcp:0 --serial1 tcp:%d --serial2 tcp:2 &\n"
                   "child=$!\necho $child > '%s'\nwait $child\n"
                   % (sdir, sitl_bin, fc["model"], fc["instance"], fc["sysid"],
                      p["lat"], p["lng"], p["alt"], p["yaw"], defaults,
                      fc["fcb_port"], pid_path(u["name"], "sitl")), 0o755)
    u.pop("config", None)
    u.pop("parm_files", None)


def _node_script(repo, name, cfg):
    """Launch script for a node server: run in its repo (node_modules,
    plugins and the --config path are resolved there); the extra
    --sim_run arg is ignored by the server but marks the pid as ours."""
    rel = os.path.relpath(cfg, repo)
    path = os.path.join(RUN_DIR, "server", f"launch.{name}.sh")
    write_text(path,
               "#!/bin/bash\ncd '%s'\necho $$ > '%s'\n"
               "exec '%s' ./src/server.js --config=%s --sim_run=%s\n"
               % (repo, pid_path("server", name), NODE, rel, RUN_DIR), 0o755)
    return path


def gen_server(sim):
    """Write the server side: configs + launch scripts in run/server/.
    Returns the server process names in launch order."""
    srv = sim["server"]
    sdir = os.path.join(RUN_DIR, "server")
    os.makedirs(sdir, exist_ok=True)
    procs = []
    if srv["type"] == "fake":
        write_text(os.path.join(sdir, "launch.fake_server.sh"),
                   "#!/bin/bash\necho $$ > '%s'\nexec python3 -u '%s' --port %d "
                   "--seed %d --cert '%s' --key '%s' --pretty\n"
                   % (pid_path("server", "fake_server"),
                      os.path.join(ENGINE_DIR, "fake_fleet_server.py"),
                      srv["port"], srv["seed"], CERT, KEY), 0o755)
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
        _node_script(AUTH_REPO, "de_auth", cfg)
        procs.append("de_auth")

    scfg = load_config(os.path.join(SERVER_REPO, "server.config"))
    scfg.update({
        "server_ip": "127.0.0.1", "server_port": srv["port"],
        "public_host": "127.0.0.1",
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
    cfg = os.path.join(sdir, "de_server.config")
    write_json(cfg, scfg)
    _node_script(SERVER_REPO, "de_server", cfg)
    procs.append("de_server")

    # one persistent GCS session on the real server, exposing the same
    # control protocol as the fake server (ctl.py talks to either)
    write_text(os.path.join(sdir, "launch.gcs_bridge.sh"),
               "#!/bin/bash\necho $$ > '%s'\nsleep 2\nexec python3 -u '%s'\n"
               % (pid_path("server", "gcs_bridge"),
                  os.path.join(ENGINE_DIR, "gcs_bridge.py")), 0o755)
    procs.append("gcs_bridge")
    return procs


def binaries_needed(sim, catalog):
    out = {DE_COMM_BIN}
    if sim["server"]["type"] == "real":
        out.add(os.path.join(SERVER_REPO, "src", "server.js"))
        if sim["server"]["auth"]:
            out.add(os.path.join(AUTH_REPO, "src", "server.js"))
    for u in sim["units"]:
        for m in u["modules"]:
            if m != "de_comm":
                out.add(module_bin(m, catalog[m]))
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


def generate(sim, sim_file=None):
    """Write run/<unit>/... for every unit and the manifest."""
    catalog = load_catalog()
    check_binaries(sim)
    os.makedirs(RUN_DIR, exist_ok=True)
    units = copy.deepcopy(sim["units"])
    for u in units:
        gen_unit(u, sim, catalog)
    server_procs = gen_server(sim)
    manifest = {"version": 1, "sim_file": sim_file, "sim_hash": sim["hash"],
                "server": sim["server"], "server_procs": server_procs,
                "home": sim["home"], "units": units}
    write_json(MANIFEST, manifest, indent=2)
    return manifest


# ---------------------------------------------------------------- journal

_LINE = re.compile(r"^(\d+)\s+(\S+)\s+(\S+)\s+(\S+)\s+([A-Z_]+)\s+(.*)$")


def journal_mark(u):
    """Snapshot of a unit's journal file offsets - pass as `since` to only
    see lines appended after this point."""
    jdir = journal_dir(u)
    if not os.path.isdir(jdir):
        return {}
    return {os.path.join(jdir, f): os.path.getsize(os.path.join(jdir, f))
            for f in os.listdir(jdir) if f.endswith(".log")}


def journal_lines(u, since=None):
    """[(unix_ms, engine, mission, seq, kind, msg, file)], oldest file first."""
    out = []
    jdir = journal_dir(u)
    files = sorted(
        (os.path.join(jdir, f) for f in os.listdir(jdir) if f.endswith(".log")),
        key=os.path.getmtime) if os.path.isdir(jdir) else []
    for f in files:
        off = since.get(f, 0) if since else 0
        with open(f, errors="replace") as fh:
            fh.seek(off)
            for ln in fh:
                m = _LINE.match(ln.strip())
                if m:
                    out.append((int(m.group(1)), m.group(2), m.group(3),
                                m.group(4), m.group(5), m.group(6), f))
                else:
                    out.append((0, "-", "-", "-", "RAW", ln.strip(), f))
    return out


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

def pid_path(name, proc):
    return os.path.join(PIDS_DIR, f"{name}.{proc}.pid")


def pid_is_ours(pid):
    """True only if the live pid's command line points into this repo
    (every launch script, config and parm file lives under SIM_ROOT/RUN_DIR).
    Never matches another harness's de_comm/SITL with the same binary."""
    try:
        cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
    except OSError:
        return False
    return RUN_DIR in cmd or ENGINE_DIR in cmd


def read_pids():
    """{pidfile_stem: pid} for entries whose process is alive and ours."""
    out = {}
    if not os.path.isdir(PIDS_DIR):
        return out
    for f in os.listdir(PIDS_DIR):
        if not f.endswith(".pid"):
            continue
        try:
            pid = int(open(os.path.join(PIDS_DIR, f)).read().strip())
            os.kill(pid, 0)
        except (ValueError, OSError):
            continue
        if pid_is_ours(pid):
            out[f[:-4]] = pid
    return out
