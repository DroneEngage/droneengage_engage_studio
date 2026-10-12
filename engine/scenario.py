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
    chaos:                          # TASK-10 seeded fault schedule
      faults: [loss, {kind: freeze, proc: de_comm}]
      units: [c1, c2]               # draw pool (default: the fleet)
      every_s: [60, 180]            # gap between draws
      dur_s: [10, 60]               # fault window
      from_t: 60                    # window; until_t defaults to the
      until_t: 1800                 # scripted timeline's last t
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

`chaos:` expands at LOAD time into concrete timeline fault steps on the
run's seed - the same seed always draws the same schedule. Only kinds
the transport serves are allowed (link faults need transport: fake),
same unit+kind windows never overlap, and a chaos scenario must carry
at least one run-wide invariant assertion (validate enforces all three).
The expansion is printed, written to var/results/<run>/expanded.yaml
(`./sim expand` prints it without running) and reruns as an ordinary
scenario - a failing seed becomes a fixed regression file.
"""

import copy
import json
import os
import random
import re

import yaml

import simlib

SCHEMA_PATH = os.path.join(simlib.ENGINE_DIR, "de_scenario.schema.json")

# de.scenario/1 fleet cls -> blocks/ name (a blocks/ name itself also
# works: `cls: copter_lowbat`)
CLS_BLOCK = {"copter": "copter", "plane": "plane", "rover": "rover",
             "static": "comm_only", "comm_only": "comm_only"}


# Named FC fault presets (SL-0005): fault kind -> {SITL param:
# (step arg, default)}. A None default means the param is written only
# when the step arg is given; arg None means "always the default".
# scenario_runner._fault applies these through the generic `param`
# read-before-set path, so dur_s restores the values the FC reported.
# Param names verified against the pinned ArduPilot
# (libraries/SITL/SITL.cpp, SIM_GPS.cpp) - note SIM_GPS1_POS_* is the
# antenna offset, NOT a position fault.
FC_PRESETS = {
    "battery":     {"SIM_BATT_VOLTAGE": ("volts", 10.5),
                    "BATT_CAPACITY": ("cap_mah", None),
                    "SIM_BATT_RES_OHM": ("res_ohm", None)},
    "gps_off":     {"SIM_GPS1_FIXTYPE": ("fixtype", 0)},
    "gps_jam":     {"SIM_GPS1_JAM": ("jam", 1)},
    "gps_glitch":  {"SIM_GPS1_GLTCH_X": ("x", 40),
                    "SIM_GPS1_GLTCH_Y": ("y", 0),
                    "SIM_GPS1_GLTCH_Z": ("z", 0)},
    "gps_degrade": {"SIM_GPS1_NUMSATS": ("sats", 4),
                    "SIM_GPS1_NOISE": ("noise", 10)},
    "motor_out":   {"SIM_ENGINE_FAIL": ("motors", 1),
                    "SIM_ENGINE_MUL": ("mul", 0)},
    "rc_fail":     {"SIM_RC_FAIL": ("how", 1)},
    "compass_fail": {"SIM_MAG1_FAIL": ("how", 1),
                     "SIM_MAG1_OFS_X": ("ofs_x", None),
                     "SIM_MAG1_OFS_Y": ("ofs_y", None),
                     "SIM_MAG1_OFS_Z": ("ofs_z", None),
                     "SIM_MAG_RND": ("noise", None)},
    "vibration":   {"SIM_VIB_FREQ_X": ("freq", 60),
                    "SIM_VIB_FREQ_Y": ("freq", 60),
                    "SIM_VIB_FREQ_Z": ("freq", 60),
                    "SIM_VIB_MOT_MAX": ("amp", 40),
                    "SIM_ACC1_RND": ("noise", 20)},
}

# every kind scenario_runner._fault implements
FAULT_KINDS = ("link_cut", "loss", "delay", "gcs_disconnect", "server_down",
               "kill", "restart", "module_kill", "freeze",
               "server_clock", "no_time_sync", "rc_override",
               "mode", "fcb_cut", "fcb_stall", "fcb_corrupt",
               "link_stall", "link_profile",
               # SL-0014 OS faults
               "cpu_starve", "journal_unwritable", "clock_step",
               "param", "udp_chunks") + tuple(FC_PRESETS)

# ------------------------------------------------------- chaos: (TASK-10)
#
# A `chaos:` block is a seeded fault schedule expanded at LOAD time into
# ordinary {t, fault, ...} timeline steps - no RNG exists at run time,
# and the same seed always draws the same schedule. The expanded file
# (var/results/<run>/expanded.yaml, `./sim expand`) is a valid
# de.scenario/1: a failing seed reruns as a fixed regression.

# kinds that need the fake server's control port - drawn only on
# transport: fake (TASK-07 lifts link faults onto real transports;
# update this set when that lands)
CHAOS_FAKE_ONLY = {"link_cut", "loss", "delay", "server_clock",
                   "no_time_sync"}

# kinds that need a flight controller (pilot/param commands) - they
# draw only from chaos.units entries with an fc
CHAOS_FC_ONLY = {"battery", "gps_off", "rc_override", "mode"}

# kinds that take a unit: (server_down / gcs_disconnect / server_clock
# are server- and operator-scoped)
CHAOS_UNIT_KINDS = frozenset(FAULT_KINDS) - {"gcs_disconnect",
                                             "server_down",
                                             "server_clock"}

# kinds whose dur_s ends a fault window (restart already brings the
# unit back; a mode switch has no window)
CHAOS_DUR_KINDS = frozenset(FAULT_KINDS) - {"restart", "mode"}

CHAOS_EVERY_S = (60.0, 180.0)
CHAOS_DUR_S = (10.0, 60.0)


def _chaos_entry(ent):
    """one chaos.faults entry -> (kind, args): a bare kind string,
    {kind: <name>, ...step args} (an `args:` mapping merges in), or the
    single-key {kind: {step args}} form."""
    if isinstance(ent, str):
        return ent, {}
    if isinstance(ent, dict):
        if "kind" not in ent and len(ent) == 1:
            (k, v), = ent.items()
            return k, (dict(v) if isinstance(v, dict) else {})
        args = {k: v for k, v in ent.items() if k not in ("kind", "args")}
        args.update(ent.get("args") or {})
        return ent.get("kind"), args
    return None, {}


def _fleet_draw_info(raw):
    """fleet -> {id: {fc, modules}} for the chaos draw: which units a
    kind may hit and which proc names module_kill/freeze can pick."""
    info = {}
    for ent in raw.get("fleet") or []:
        b = simlib.load_block(CLS_BLOCK.get(ent["cls"], ent["cls"]))
        mods = list(b["modules"]) + [
            m for m in (ent.get("modules") or []) if m not in b["modules"]]
        info[ent["id"]] = {"fc": bool(b["fc"]), "modules": mods}
    return info


def _last_t(raw):
    """The last scheduled t: of the scripted timeline, or None - the
    default chaos.until_t."""
    ts = [s["t"] for s in raw.get("timeline") or []
          if isinstance(s, dict) and isinstance(s.get("t"), (int, float))]
    return max(ts) if ts else None


def _chaos_window(ch, key, default, path):
    """A [min, max] range arg - schema-checked when jsonschema is
    installed, rechecked here so the loader alone never draws from a
    malformed range."""
    v = ch.get(key) or default
    if not isinstance(v, (list, tuple)) or len(v) != 2:
        raise simlib.SimError(f"{path}: chaos.{key} must be a [min, max] pair")
    lo, hi = float(v[0]), float(v[1])
    if lo < 0 or hi < lo:
        raise simlib.SimError(
            f"{path}: chaos.{key} must be [min, max] with 0 <= min <= max")
    return lo, hi


def _validate_chaos(raw, path):
    """chaos: semantics the schema cannot express - known kinds the
    transport can serve, at least one drawable unit, a usable window,
    and (TASK-10) a run-wide invariant assertion: an unscripted run is
    only evidence when an invariant watched it."""
    import checks
    ch = raw["chaos"]
    if not isinstance(ch, dict):
        raise simlib.SimError(f"{path}: chaos: must be a mapping")
    fleet_ids = [e["id"] for e in raw.get("fleet") or []]
    info = _fleet_draw_info(raw)
    if not ch.get("faults"):
        raise simlib.SimError(
            f"{path}: chaos.faults must list at least one fault kind")
    units = [str(u) for u in (ch.get("units") or fleet_ids)]
    unknown = [u for u in units if u not in fleet_ids]
    if unknown:
        raise simlib.SimError(
            f"{path}: chaos.units {unknown} not in the fleet "
            f"({', '.join(fleet_ids)})")
    t = raw.get("transport", "real")
    ttype = t.get("type") if isinstance(t, dict) else t
    fc_units = [u for u in units if info[u]["fc"]]
    for i, ent in enumerate(ch.get("faults") or []):
        kind, args = _chaos_entry(ent)
        if kind not in FAULT_KINDS:
            raise simlib.SimError(
                f"{path}: chaos.faults[{i}] unknown fault {kind!r} "
                f"(known: {', '.join(FAULT_KINDS)})")
        if kind in CHAOS_FAKE_ONLY and ttype != "fake":
            raise simlib.SimError(
                f"{path}: chaos.faults[{i}] {kind!r} needs the fake "
                f"server's control port - transport is {ttype!r} "
                "(link faults ride a real transport only once "
                "TASK-07 lands)")
        if kind in CHAOS_FC_ONLY and not fc_units:
            raise simlib.SimError(
                f"{path}: chaos.faults[{i}] {kind!r} needs a flight "
                "controller - no FC unit in chaos.units")
        if kind == "mode" and not args.get("mode"):
            raise simlib.SimError(
                f"{path}: chaos.faults[{i}] 'mode' needs a mode: "
                "(e.g. {kind: mode, mode: RTL})")
    every_lo, _ = _chaos_window(ch, "every_s", CHAOS_EVERY_S, path)
    if every_lo < 1:
        raise simlib.SimError(
            f"{path}: chaos.every_s min must be >= 1")
    _chaos_window(ch, "dur_s", CHAOS_DUR_S, path)
    until = ch.get("until_t", _last_t(raw))
    if until is None or float(until) <= float(ch.get("from_t", 0)):
        raise simlib.SimError(
            f"{path}: chaos needs until_t (or a timeline) beyond "
            f"from_t {ch.get('from_t', 0)}")
    names = {next(iter(a)) for a in raw.get("assert") or []
             if isinstance(a, dict) and len(a) == 1}
    if not names & checks.INVARIANT_ASSERTS:
        raise simlib.SimError(
            f"{path}: a chaos scenario must carry at least one run-wide "
            f"invariant assertion ({', '.join(sorted(checks.INVARIANT_ASSERTS))})"
            " - an unscripted run proves nothing without one")


def expand_chaos(raw, seed):
    """chaos: -> concrete timeline fault steps, drawn on a private
    random.Random(seed) stream at load time. Overlapping windows of the
    same unit+kind are never drawn (scripted fault: steps pre-mark
    theirs too)."""
    ch = raw["chaos"]
    rng = random.Random(seed)
    pool = [_chaos_entry(ent) for ent in ch["faults"]]
    info = _fleet_draw_info(raw)
    fleet_ids = [e["id"] for e in raw["fleet"]]
    units = [str(u) for u in (ch.get("units") or fleet_ids)]
    every_lo, every_hi = _chaos_window(ch, "every_s", CHAOS_EVERY_S,
                                       "<expansion>")
    dur_lo, dur_hi = _chaos_window(ch, "dur_s", CHAOS_DUR_S, "<expansion>")
    until_t = float(ch.get("until_t", _last_t(raw)))

    # (unit, kind) -> the t the fault window ends; None = server-scoped
    busy = {}
    for s in raw.get("timeline") or []:
        if not isinstance(s, dict) or "fault" not in s or "t" not in s:
            continue            # on:-triggered faults mark no window
        dur = float(s.get("dur_s") or s.get("seconds") or 0)
        tok = str(s.get("unit", "all"))
        hit = units if tok == "all" else \
            [u for u in tok.split(",") if u in units]
        for u in hit:
            busy[(u, s["fault"])] = float(s["t"]) + dur

    steps = []
    t = float(ch.get("from_t", 0))
    while True:
        t += rng.uniform(every_lo, every_hi)
        if t > until_t:
            break
        kind, args = rng.choice(pool)
        if kind in CHAOS_FC_ONLY:
            cands = [u for u in units if info[u]["fc"]]
        elif kind in CHAOS_UNIT_KINDS:
            cands = list(units)
        else:
            cands = [None]
        free = [u for u in cands
                if busy.get((u, kind), float("-inf")) <= t]
        if not free:
            continue                # this (unit, kind) is inside a window
        u = rng.choice(free)
        step = {"t": round(t, 1), "fault": kind}
        step.update(args)
        if u is not None:
            step["unit"] = u
        if kind == "module_kill" and "module" not in step:
            step["module"] = rng.choice(info[u]["modules"])
        if kind == "freeze" \
                and "proc" not in step and "module" not in step:
            step["proc"] = rng.choice(info[u]["modules"])
        if kind in CHAOS_DUR_KINDS and "dur_s" not in step:
            d = round(rng.uniform(dur_lo, dur_hi), 1)
            if d > 0:
                step["dur_s"] = d
        busy[(u, kind)] = t + float(step.get("dur_s") or 0)
        steps.append(step)
    return steps


def _merge_timeline(timeline, chaos_steps):
    """scripted + chaos steps, t-sorted (stable: a scripted step wins a
    tie); on: watchers keep their order at the end - position means
    nothing for a watcher."""
    timed = [s for s in timeline if "t" in s] + chaos_steps
    timed.sort(key=lambda s: s["t"])
    return timed + [s for s in timeline if "t" not in s]


def expanded_text(scn):
    """The chaos-expanded scenario as a standalone de.scenario/1 file:
    chaos: removed, the drawn steps inlined into timeline:, file-relative
    paths absolutized (expanded.yaml lives under var/results/). A failing
    seed's expansion reruns as a fixed regression."""
    exp = copy.deepcopy(scn["raw"])
    exp.pop("chaos", None)
    exp["seed"] = scn["seed"]
    exp["timeline"] = scn["timeline"]
    if scn.get("operation"):
        exp["operation"] = scn["operation"]
    if scn.get("world"):
        exp["world"] = scn["world"]
    verify = exp.get("verify")
    if isinstance(verify, dict) and isinstance(verify.get("cli"), str) \
            and not os.path.isabs(verify["cli"]):
        verify["cli"] = os.path.abspath(
            os.path.join(os.path.dirname(scn["file"]), verify["cli"]))
    return (f"# {scn['name']} seed {scn['seed']} - the chaos: schedule "
            "expanded into a fixed de.scenario/1 file\n"
            f"# source: {scn['file']}\n"
            + yaml.safe_dump(exp, sort_keys=False))



def is_legacy_spec(raw):
    """The removed `sim:` + `steps:` test format (P4A-03; SL-0020). Kept
    only so callers can give a clear refusal instead of a schema error:
    every repo test is de.scenario/1 now."""
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
        if step.get("fault") == "param" and not step.get("set"):
            raise simlib.SimError(
                f"{path}: timeline[{i}] fault param needs a non-empty "
                "set: {NAME: value, ...}")
    for i, step in enumerate(raw.get("timeline", [])):
        if ("t" in step) == ("on" in step):
            raise simlib.SimError(
                f"{path}: timeline[{i}] needs exactly one of t:/on:")
        if ("do" in step) == ("fault" in step):
            raise simlib.SimError(
                f"{path}: timeline[{i}] needs exactly one of do:/fault:")
    # SL-0014: clock_step needs libfaketime on this machine AND the
    # target unit booted under it (`faketime: true` - there is no
    # retroactive LD_PRELOAD). Fail at validate, not mid-run.
    fleet_by = {str(ent.get("id")): ent for ent in raw.get("fleet", [])}
    clock_steps = [s for s in raw.get("timeline", [])
                   if s.get("fault") == "clock_step"]
    if clock_steps or any(ent.get("faketime") for ent in fleet_by.values()):
        if not simlib.faketime_lib():
            raise simlib.SimError(
                f"{path}: clock_step / `faketime: true` needs libfaketime "
                "- apt install libfaketime, or set SIM_FAKETIME_LIB in "
                "sim.local.env")
    for i, step in enumerate(clock_steps):
        target = str(step.get("unit", "all"))
        ids = list(fleet_by) if target == "all" else [target]
        for uid in ids:
            ent = fleet_by.get(uid)
            if ent is not None and not ent.get("faketime"):
                raise simlib.SimError(
                    f"{path}: clock_step on {uid} needs `faketime: true` "
                    "in its fleet entry - de_comm must launch under "
                    "libfaketime")
    if "chaos" in raw:
        _validate_chaos(raw, path)


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
    share ports (parallel `sim up` still refuses a real clash). A real
    transport with `link_proxy: true` (SL-0009) seeds the proxies the
    same way - their link RNG must not drift between runs either."""
    t = scn.get("transport", "real")
    if isinstance(t, str):
        srv = {"type": t}
    else:
        srv = dict(t)
    srv.setdefault("port", simlib.SERVER_PORT + slot_base)
    if "seed" not in srv and \
            (srv.get("type") == "fake" or srv.get("link_proxy")):
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
        if "link_proxy" in ent:
            u["link_proxy"] = ent["link_proxy"]
        if ent.get("modules"):
            u["add_modules"] = list(ent["modules"])
        if ent.get("params"):
            u["fc"] = {"params": dict(ent["params"])}
        if ent.get("fcb_proxy") is not None:
            u["fcb_proxy"] = bool(ent["fcb_proxy"])
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
        if ent.get("faketime"):
            u["faketime"] = True     # SL-0014: de_comm under libfaketime
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


def load_scenario(path, seed=None, probe_ports=True):
    """-> a normalized run spec. Raises simlib.SimError on any format
    violation (unknown keys, bad timeline steps, unknown cls).
    probe_ports=False skips the free-slot search (expand/validate only
    need the spec, not a live port check - and stay silent)."""
    path = os.path.abspath(path)
    raw = yaml.safe_load(open(path)) or {}
    base = os.path.dirname(path)
    _yaml_bool_keys(raw)
    if is_legacy_spec(raw):
        raise simlib.SimError(
            f"{path}: `sim:` + `steps:` is the removed legacy test format "
            "(SL-0020) - rewrite the file as de.scenario/1 "
            "(docs/SCENARIOS.md)")
    _validate(raw, path)
    home = tuple(raw.get("home", simlib.HOME))
    if len(home) == 3:
        home = home + (simlib.HOME[3],)
    seed = int(seed if seed is not None else raw.get("seed", 1))
    scn = {
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
    # TASK-10: a chaos: block expands here - at load time, on the run's
    # effective seed - into ordinary timeline steps
    if raw.get("chaos"):
        scn["chaos_steps"] = expand_chaos(raw, seed)
        scn["timeline"] = _merge_timeline(scn["timeline"],
                                          scn["chaos_steps"])
    else:
        scn["chaos_steps"] = []
    scn["expanded"] = bool(raw.get("chaos"))
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
    elif probe_ports:
        scn["sim"] = _host_free_sim(sim_at, _seed_slot_bases(seed, units))
    else:
        scn["sim"] = sim_at(_seed_slot_bases(seed, units)[0])
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


def _slot_fits(units):
    """fits(sb) for _seed_slot_base/_seed_slot_bases: the fleet's highest
    module port must stay <= 65535 and its FC sysids <= 250 (de_camera
    64000 + 10*k overflows from k=154)."""
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
    return lambda sb: (top + 10 * (sb + last) <= 65535
                       and (not has_fc or sb + last <= 250))


def _seed_slot_base(seed, units):
    """10 + 25*(seed % 9), unless that block pushes this fleet's highest
    module port past 65535 or an FC sysid past 250 - then the seed picks
    among the blocks that fit, so a camera fleet runs on every seed."""
    fits = _slot_fits(units)
    cands = [10 + 25 * i for i in range(9)]
    pick = cands[seed % 9]
    if fits(pick):
        return pick
    ok = [c for c in cands if fits(c)]
    if not ok:
        raise simlib.SimError(
            f"{len(units)} units fit no slot block - pin a lower "
            "slot_base or split the fleet")
    return ok[seed % len(ok)]


def _seed_slot_bases(seed, units):
    """Every fitting slot block, the seed's own first - the fallbacks
    when a host process holds a port of the seed's block. After the
    canonical 25-port blocks come the in-between bases: a host crowded
    with other worktrees' sims can hold a port in EVERY canonical block
    (an FC fleet needs ~7 ports and any hit disqualifies the block),
    and a fine base is a valid base - the port derivation is the same."""
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
    fits = _slot_fits(units)
    out += [b for blk in list(out) for b in range(blk + 1, blk + 25)
            if fits(b)]
    return out
