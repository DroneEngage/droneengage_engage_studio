#!/usr/bin/env python3
"""scenario.py - the de.scenario/1 loader (TASK-P5-07).

One versioned scenario file describes a whole rehearsal: operation +
fleet + world + timeline + operator policy + assertions. The format is
validated against engine/de_scenario.schema.json on load - unknown keys
fail fast.

    version: 1
    name: sar_nominal
    seed: 42
    realtime: true                  # timing asserts need SITL speed-up 1
    transport: fake                 # fake | real | {server: block}
    world: ../worlds/forest_a.yaml  # P5-08 - accepted, not consumed yet
    operation: ../ops/sar.deop      # or a deploy spec / package
    verify: {expect_errors: 0}      # P5-06 CLI before deploy
    home: [lat, lng, alt, yaw]      # fleet home (absolute)
    fleet:
      - {id: cam1, cls: copter, role: cam_scout, home: [..], modules: [..]}
      - {id: mast, cls: static, role: sdr_mast,  base: [..]}
    operator:                       # the bot's standing policy
      approve: {launch: {after_s: 5}, action: {after_s: 3}}
      confirm: {person: {after_s: 10}}
    timeline:
      - {t: 0,   do: deploy}
      - {t: 5,   do: start, start: {searcher: standby}}
      - {t: 300, fault: link_cut, unit: s2, dur_s: 60}
      - {on: {unit: amb, re: "APPROVAL_REQ .* launch"}, do: operator,
         act: deny, once: true}
    assert:
      - journal_count: {unit: all, re: "BUS_IN phone.signal", eq: 1}
      - distance_to: {unit: amb, re: "ACTION .*release.* ok",
                      truth: person_1, max_m: 15}

`t:` is seconds from the scenario start (the moment the timeline engine
begins, after bring-up + verify); negative `t` runs at once, before the
nominal start. `on:` arms a journal watcher; its regex match groups are
available to the step's string args as {M1}..{M9} (e.g. req=(\\S+) ->
req: "{M1}").

A legacy tests/*.yaml file (version 1, `sim:` + `steps:`) loads as
{legacy: True} unchanged - the scenario runner executes the same steps
through the P4A-03 machinery and adds the new results layout.
"""

import json
import os
import re

import yaml

import simlib

SCHEMA_PATH = os.path.join(simlib.ENGINE_DIR, "de_scenario.schema.json")

# de.scenario/1 fleet cls -> blocks/ name (a blocks/ name itself also
# works: `cls: copter_lowbat`)
CLS_BLOCK = {"copter": "copter", "plane": "plane", "rover": "rover",
             "static": "comm_only", "comm_only": "comm_only"}


# every kind scenario_runner._fault implements
FAULT_KINDS = ("link_cut", "loss", "delay", "gcs_disconnect", "server_down",
               "kill", "restart", "module_kill", "freeze", "battery",
               "gps_off", "server_clock", "no_time_sync", "rc_override",
               "mode", "udp_chunks")


def is_legacy_spec(raw):
    """A P4A-03 test file: version 1 with sim:/steps: and no scenario
    keys. Runs unchanged through the new runner."""
    return isinstance(raw, dict) and "steps" in raw and \
        "timeline" not in raw and "fleet" not in raw


def is_sim_spec(raw):
    """A sims/*.yaml fleet file (version + units:, no steps/timeline/
    fleet): `./sim run|view` on one just ensures the sim is up - there
    is nothing to execute."""
    return isinstance(raw, dict) and "units" in raw and \
        "steps" not in raw and "timeline" not in raw and "fleet" not in raw


def _validate(raw, path):
    try:
        import jsonschema
    except ImportError:
        jsonschema = None
    schema = json.load(open(SCHEMA_PATH))
    if jsonschema:
        try:
            jsonschema.validate(raw, schema)
        except jsonschema.ValidationError as e:
            where = ".".join(str(p) for p in e.absolute_path) or "<root>"
            raise simlib.SimError(
                f"{path}: de.scenario/1 violation at {where}: {e.message}")
    else:
        # jsonschema not installed: still enforce the fail-fast rules
        allowed = set(schema["properties"])
        bad = set(raw) - allowed
        if bad:
            raise simlib.SimError(f"{path}: unknown keys {sorted(bad)}")
        for req in schema["required"]:
            if req not in raw:
                raise simlib.SimError(f"{path}: missing required key {req!r}")
        for ent in raw.get("fleet", []):
            bad = set(ent) - set(schema["properties"]["fleet"]["items"]
                                 ["properties"])
            if bad:
                raise simlib.SimError(
                    f"{path}: fleet {ent.get('id')}: unknown keys {sorted(bad)}")
    # a typo in an assertion name or fault kind must fail HERE, not after
    # a ten-minute fleet run ("unknown assertion" is only seen at the end)
    import checks
    for i, asr in enumerate(raw.get("assert", [])):
        if not isinstance(asr, dict) or len(asr) != 1:
            raise simlib.SimError(
                f"{path}: assert[{i}] must be a single {{name: args}} mapping")
        nm = next(iter(asr))
        if nm not in checks.DISPATCH:
            raise simlib.SimError(
                f"{path}: assert[{i}] unknown assertion {nm!r} "
                f"(known: {', '.join(sorted(checks.DISPATCH))})")
    for i, step in enumerate(raw.get("timeline", [])):
        if "fault" in step and step["fault"] not in FAULT_KINDS:
            raise simlib.SimError(
                f"{path}: timeline[{i}] unknown fault {step['fault']!r} "
                f"(known: {', '.join(FAULT_KINDS)})")
    for i, step in enumerate(raw.get("timeline", [])):
        if ("t" in step) == ("on" in step):
            raise simlib.SimError(
                f"{path}: timeline[{i}] needs exactly one of t:/on:")
        if ("do" in step) == ("fault" in step):
            raise simlib.SimError(
                f"{path}: timeline[{i}] needs exactly one of do:/fault:")


def _absolute_to_at(home, pos):
    """[lat,lng,alt,yaw] absolute -> sim `at` meters-from-home."""
    import math
    lat, lng, alt = pos[0], pos[1], pos[2]
    yaw = pos[3] if len(pos) > 3 else home[3]
    return {
        "north_m": (lat - home[0]) * 111320.0,
        "east_m": (lng - home[1]) * 111320.0 * math.cos(math.radians(home[0])),
        "alt_m": alt - home[2],
        "yaw": yaw,
    }


def _server_spec(scn, slot_base):
    """transport: fake|real|{...} -> a sim `server:` block. The scenario
    seed drives the fake server's link RNG when no explicit seed is set;
    the server port rides the same slot block so --seeds runs can never
    share ports (parallel `sim up` still refuses a real clash)."""
    t = scn.get("transport", "real")
    if isinstance(t, str):
        srv = {"type": t}
    else:
        srv = dict(t)
    srv.setdefault("port", simlib.SERVER_PORT + slot_base)
    if srv.get("type") == "fake" and "seed" not in srv:
        srv["seed"] = scn["seed"]
    return srv


def _fleet_units(scn, home):
    """fleet: entries -> sim `units:` entries. `cls` picks the block;
    `modules` lands as add_modules; `params` as fc.params; `home`/`base`
    are absolute positions converted to meters offsets. A tlog is enabled
    on every de_mavlink unit - the position assertions read them."""
    units = []
    for ent in scn["fleet"]:
        block = CLS_BLOCK.get(ent["cls"], ent["cls"])
        if block not in simlib.list_blocks():
            raise simlib.SimError(
                f"scenario {scn['name']}: unknown cls/block {ent['cls']!r} "
                f"for unit {ent['id']} (cls: {sorted(CLS_BLOCK)} or a "
                f"block from {', '.join(simlib.list_blocks())})")
        u = {"block": block, "name": ent["id"]}
        pos = ent.get("home") or ent.get("base")
        if pos is not None:
            u["at"] = _absolute_to_at(home, pos)
        elif ent.get("at"):
            u["at"] = dict(ent["at"])
        if ent.get("role"):
            u["role"] = ent["role"]
        if ent.get("modules"):
            u["add_modules"] = list(ent["modules"])
        if ent.get("params"):
            u["fc"] = {"params": dict(ent["params"])}
        cfg = dict(ent.get("config") or {})
        # SITL tlogs feed distance_to / no_sep_violation / the realtime
        # speed-up check - enable on units that run de_mavlink
        if "de_mavlink" in simlib.load_block(block)["modules"]:
            cfg = simlib.deep_merge(
                {"de_mavlink": {"tlog_enabled": True}}, cfg)
        if cfg:
            u["config"] = cfg
        if ent.get("account"):
            u["account"] = ent["account"]
        if ent.get("udp_proxy"):
            u["udp_proxy"] = list(ent["udp_proxy"])
        units.append(u)
    return units


def _fault_units(units, tok):
    """The `unit:` selector over resolved sim unit dicts - all | name |
    1-based index | block:<b> | comma list (the ctl.resolve subset a
    scenario file can name)."""
    if tok in (None, "all"):
        return list(units)
    out = []
    for t in str(tok).split(","):
        if t.startswith("block:"):
            out += [u for u in units if u["block"] == t[6:]]
        elif t.isdigit() and 1 <= int(t) <= len(units):
            out.append(units[int(t) - 1])
        else:
            out += [u for u in units if u["name"] == t]
    return out


def _inject_udp_proxies(units, timeline, path):
    """SL-0010: a `fault: udp_chunks` step needs the module's s2s link
    proxied from sim-up (the module's s2s_udp_target_port is generated
    once). Inject the unit's udp_proxy opt-in here so the scenario file
    stays self-contained like every other fault - an explicit
    `udp_proxy:` fleet key does the same by hand."""
    blocks = {}
    for i, s in enumerate(timeline):
        if s.get("fault") != "udp_chunks":
            continue
        mod = s.get("module")
        if not mod:
            raise simlib.SimError(
                f"{path}: timeline[{i}] udp_chunks needs module:")
        hit = 0
        for u in _fault_units(units, s.get("unit")):
            b = blocks.setdefault(u["block"], simlib.load_block(u["block"]))
            if mod not in b["modules"] + u.get("add_modules", []):
                continue
            u.setdefault("udp_proxy", [])
            if mod not in u["udp_proxy"]:
                u["udp_proxy"].append(mod)
            hit += 1
        if not hit:
            raise simlib.SimError(
                f"{path}: timeline[{i}] udp_chunks "
                f"{s.get('unit', 'all')}.{mod}: no unit in the fleet "
                "runs that module")


def _yaml_bool_keys(raw):
    """YAML 1.1 parses the bare key `on:` as True (yes/no/on/off are
    booleans). The format says `on:` - rename it back before
    validation so the spec's syntax works unquoted."""
    for step in raw.get("timeline") or []:
        if isinstance(step, dict) and True in step:
            step["on"] = step.pop(True)


def load_scenario(path, seed=None):
    """-> a normalized run spec. Raises simlib.SimError on any format
    violation (unknown keys, bad timeline steps, unknown cls)."""
    path = os.path.abspath(path)
    raw = yaml.safe_load(open(path)) or {}
    base = os.path.dirname(path)
    _yaml_bool_keys(raw)
    if is_legacy_spec(raw):
        return {"legacy": True, "file": path, "test": raw,
                "name": os.path.splitext(os.path.basename(path))[0],
                "seed": seed or 1}
    _validate(raw, path)
    home = tuple(raw.get("home", simlib.HOME))
    if len(home) == 3:
        home = home + (simlib.HOME[3],)
    seed = int(seed if seed is not None else raw.get("seed", 1))
    scn = {
        "legacy": False,
        "file": path,
        "name": raw["name"],
        "description": raw.get("description", ""),
        "seed": seed,
        "realtime": bool(raw.get("realtime", False)),
        "transport": raw.get("transport", "real"),
        "world": (os.path.abspath(os.path.join(base, raw["world"]))
                  if isinstance(raw.get("world"), str) else None),
        "operation": (os.path.join(base, raw["operation"])
                      if raw.get("operation") else None),
        "verify": raw.get("verify"),
        "fleet": raw["fleet"],
        "operator": raw.get("operator") or {},
        "timeline": raw.get("timeline", []),
        "assert": raw.get("assert", []),
        "teardown": raw.get("teardown", "land"),
        "settle_s": float(raw.get("settle_s", 30)),
        "render": raw.get("render"),
        "raw": raw,
    }
    units = _fleet_units(scn, home)
    _inject_udp_proxies(units, scn["timeline"], path)
    # one port block per run: the scenario may pin slot_base, else it is
    # derived from the seed so --seeds runs never share ports (a parallel
    # `sim up` would still refuse a clash)
    def sim_at(sb):
        return {
            "version": 1,
            "name": re.sub(r"[^\w-]", "_", scn["name"]),
            "server": _server_spec(scn, sb),
            "home": list(home),
            "slot_base": sb,
            "units": units,
            "render": raw.get("render"),
            **({"speedup": 1} if scn["realtime"] else {}),
        }

    if raw.get("slot_base"):
        scn["sim"] = sim_at(int(raw["slot_base"]))
    else:
        scn["sim"] = _host_free_sim(sim_at, _seed_slot_bases(seed, units))
    return scn


def _host_free_sim(sim_at, bases):
    """The first seed-ordered slot block whose ports no foreign process
    holds (live sims' own ports don't count - the run downs or refuses
    them). Falls back to the seed's block when every one is held."""
    live = set()
    for n in simlib.live_sim_names():
        live |= simlib.used_ports(simlib.load_manifest(n))
    for sb in bases:
        spec = sim_at(sb)
        held = simlib.ports_held(
            simlib.used_ports(simlib.load_sim(spec)) - live)
        if not held:
            return spec
        print(f"slot_base {sb}: port(s) {held} held by another process "
              "on this host - trying the next block", flush=True)
    return sim_at(bases[0])


def _seed_slot_base(seed, units):
    """10 + 25*(seed % 9), unless that block pushes this fleet's highest
    module port past 65535 or an FC sysid past 250 (de_camera 64000 +
    10*k overflows from k=154) - then the seed picks among the blocks that
    fit, so a camera fleet runs on every seed."""
    cat = simlib.load_catalog()
    top = simlib.DE_COMM_PORT_BASE
    has_fc = False
    for u in units:
        b = simlib.load_block(u["block"])
        has_fc |= bool(b["fc"])
        for m in b["modules"] + list(u.get("add_modules", [])):
            if m in cat:
                top = max(top, int(cat[m]["port_base"]))
    last = len(units) - 1

    def fits(sb):
        return top + 10 * (sb + last) <= 65535 and \
            (not has_fc or sb + last <= 250)

    cands = [10 + 25 * i for i in range(9)]
    pick = cands[seed % 9]
    if fits(pick):
        return pick
    ok = [c for c in cands if fits(c)]
    if not ok:
        raise simlib.SimError(
            f"{len(units)} units with module ports up to {top} fit no "
            "slot block - pin a lower slot_base or split the fleet")
    return ok[seed % len(ok)]


def _seed_slot_bases(seed, units):
    """Every fitting slot block, the seed's own first - the fallbacks
    when a host process holds a port of the seed's block."""
    first = _seed_slot_base(seed, units)
    cands = [10 + 25 * i for i in range(9)]
    i = cands.index(first)
    rest = cands[i + 1:] + cands[:i]
    out = [first]
    for c in rest:
        try:
            if _seed_slot_base(cands.index(c), units) == c:
                out.append(c)
        except simlib.SimError:
            pass
    return out
