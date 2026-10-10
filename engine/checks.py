#!/usr/bin/env python3
"""checks.py - de.scenario/1 assertions (TASK-P5-07).

Every assertion produces a structured result:

    {"name": <key>, "status": "PASS" | "FAIL" | "SKIP",
     "value": <number or None>, "detail": "..."}

SKIP is a verdict, not an absence: missing evidence (no truth file, no
telemetry, no operator bot) and unsupported-realtime assertions report
SKIP visibly and never count against the exit code. The runner counts a
scenario green only when every non-SKIP assertion is PASS.

Journal assertions keep the runner.py semantics (same helpers, same
`unit:` resolution). The new ones read:

  - module stdout logs:  module_log_count - var/run/<unit>/logs/<module>.log
                         lines since the test mark (PL-07: module-printed
                         facts like de_precland's EVENT_OUT never reach
                         the journal)
  - journals:            proc_alive/proc_dead, operator_log_count,
                         no_fc_command_to_static
  - telemetry.jsonl:     distance_to, no_sep_violation, fc_mode,
                         telemetry_value (the SITL "tlog"; pick: latest|
                         min|max over the recorded series)
  - truth JSONL:         distance_to, coverage_complete - rows
                         {tid|id|target, t_ms|t|ms, lat, lng, alt|alt_m}
                         and coverage rows {type:"coverage", tid, covered};
                         P5-08 adds detection rows {type:"detection",
                         src, tid|"clutter", cls, t_ms, sent} read by
                         detection_precision and signals_in_range, and
                         distance_to's detected_cls: (bot-seen pos);
                         P5-10 adds the detection rows' vis / why /
                         blockers (ai) and loss_db (sdr) fields, read by
                         detection_recall (who was found, who correctly
                         stayed hidden) and time_to_detect (seconds from
                         the `start` step - the world epoch when none
                         ran - to the first find)
  - stub determinism:    stub_deterministic - two same-seed --selftest
                         replays must produce byte-identical stub logs
  - 1087 status (bot):   final_state
  - MAVLink mission:     fc_items_unchanged_without_cause, mission_items
                         (count on the FC), mission_pull (the GCS 1027
                         chunk read-back the WebClient draws)
"""

import hashlib
import json
import math
import os
import re
import time

import simlib
import ctl
import coverage as coverage_mod
import geo_projection as gp

# assertions that are only meaningful when SITL really runs at ~1x; when
# the scenario asks realtime: true and the measured speed-up is off,
# these report SKIP instead of a misleading number
REALTIME_SENSITIVE = {"max_latency", "journal_time_delta", "time_to_detect"}

# only a WRITE can change FC mission items - a download/read never does
CAUSE_RE = r"MISSION_(WRITE|UPLOAD|ACK)|fc mission|mission.*upload"
# word-bounded: bare "arm" also matched alarm/warm/swarm/harm
FC_LINE_RE = re.compile(
    r"MISSION_|MAVLink|FC_(?!items)|fc_cmd|guided_to|set_mode|"
    r"\b(dis)?arm(ed|ing)?\b", re.I)

# assertions that resolve `unit:`; an empty resolution can never pass
UNIT_ASSERTS = {"journal_count", "module_log_count", "journal_consistent",
                "journal_order", "no_journal", "final_state",
                "fc_items_unchanged_without_cause", "mission_current",
                "mission_items", "mission_pull",
                "proc_alive", "proc_dead", "roster_value", "fc_mode",
                "telemetry_value", "link_offline", "signals_in_range",
                "detection_recall", "status_field"}

DEFAULT_TRUTH_TOL_MS = 10000      # truth row must be this close in time


def _r(name, status, value=None, detail=""):
    return {"name": name, "status": status, "value": value,
            "detail": detail}


def _units(arg, ctx):
    return ctl.resolve(str(arg.get("unit", arg.get("vehicle", "all"))),
                       ctx["manifest"])


def _haversine(lat1, lng1, lat2, lng2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lng2 - lng1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * \
        math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


# -------------------------------------------------------------- truth

def _truth_rows(ctx, truth_file=None):
    """Truth JSONL rows: arg truth_file (relative to the scenario file) >
    <run>/<sim>/truth.jsonl (P5-08's output) > <results>/truth.jsonl."""
    cands = []
    if truth_file:
        p = truth_file if os.path.isabs(truth_file) else \
            os.path.join(os.path.dirname(ctx["file"]), truth_file)
        cands.append(p)
    cands.append(os.path.join(simlib.run_dir(ctx["manifest"]["name"]),
                              "truth.jsonl"))
    if ctx.get("results_dir"):
        cands.append(os.path.join(ctx["results_dir"], "truth.jsonl"))
    for p in cands:
        if not os.path.isfile(p):
            continue
        rows = []
        for ln in open(p, errors="replace"):
            try:
                r = json.loads(ln)
            except ValueError:
                continue
            rows.append(r)
        # --hold leaves the recorders and the world stream writing for
        # the parked viewer; only rows up to the hold boundary (unix ms,
        # the row clock) are scenario evidence - the shared cap is here
        # so every truth/detection reader gets it
        hold_ms = ctx.get("hold_ms")
        if hold_ms is not None:
            rows = [r for r in rows if _row_t(r) <= hold_ms]
        return rows, p
    return None, None


def _row_tid(r):
    return r.get("tid") or r.get("id") or r.get("target")


def _first(r, *keys):
    """First key present and not None - 0 is a value, not 'missing'."""
    for k in keys:
        if r.get(k) is not None:
            return r[k]
    return None


def _row_t(r):
    t = _first(r, "t_ms", "t", "ms")
    return 0 if t is None else t


def _lng(r):
    """lng/lon of a row; 0.0 (the prime meridian) is a valid value."""
    return _first(r, "lng", "lon")


def _truth_at(rows, tid, t_ms, tol_ms=DEFAULT_TRUTH_TOL_MS):
    """Truth row for tid nearest in time to t_ms, no further than tol_ms
    (a row from another phase of the run is not evidence)."""
    best = None
    for r in rows:
        if _row_tid(r) != tid or r.get("lat") is None:
            continue
        # detection / coverage rows reuse the tid but carry the REPORTED
        # position, not the truth
        if r.get("type") in ("detection", "coverage"):
            continue
        d = abs(_row_t(r) - t_ms)
        if best is None or d < best[0]:
            best = (d, r)
    if best is None or (tol_ms is not None and best[0] > tol_ms):
        return None
    return best[1]


def _unit_pos_at(ctx, unit_name, t_ms):
    """Telemetry row for the unit nearest in time to t_ms (<= 5 s off)."""
    tele = ctx.get("telemetry")
    if not tele:
        return None
    best = None
    for r in tele.rows(unit_name):
        if r.get("lat") is None:
            continue
        d = abs(r["t_ms"] - t_ms)
        if d <= 5000 and (best is None or d < best[0]):
            best = (d, r)
    return best[1] if best else None


# ------------------------------------------------------ mission baseline

def mission_download(u, timeout=15):
    """The FC's mission item list [(seq, cmd, x, y, z)] or None when the
    download fails - the serial0 read-back that proves a plan actually
    landed. mission_signature hashes it; mission_items counts it."""
    m = ctl.mav(u, timeout=15)
    try:
        m.mav.mission_request_list_send(m.target_system,
                                        m.target_component)
        cnt = m.recv_match(type="MISSION_COUNT", blocking=True,
                           timeout=timeout)
        if cnt is None:
            return None
        items = []
        for i in range(cnt.count):
            m.mav.mission_request_int_send(m.target_system,
                                           m.target_component, i)
            it = m.recv_match(type=["MISSION_ITEM_INT", "MISSION_ITEM"],
                              blocking=True, timeout=timeout)
            if it is None:
                return None
            items.append([it.seq, it.command, round(float(it.x), 4),
                          round(float(it.y), 4), round(float(it.z), 2)])
        return items
    except Exception:
        return None
    finally:
        m.close()


def mission_signature(u, timeout=15):
    """sha256 over the FC's mission items (seq,cmd,x,y,z), or None when
    the download fails. The baseline is taken at scenario start;
    fc_items_unchanged_without_cause compares at assert time."""
    items = mission_download(u, timeout)
    if items is None:
        return None
    return hashlib.sha256(json.dumps(items).encode()).hexdigest()


def capture_baseline(ctx):
    """Snapshot mission signatures + proc pids before the timeline runs."""
    base = {"missions": {}}
    for u in ctx["manifest"]["units"]:
        if u["fc"]:
            base["missions"][u["name"]] = mission_signature(u)
    return base


# ---------------------------------------------------------------- checks

_CMP_OPS = (("eq", "==", lambda x, w: x == w),
            ("equals", "==", lambda x, w: x == w),
            ("at_least", ">=", lambda x, w: x >= w),
            ("at_most", "<=", lambda x, w: x <= w),
            ("lt", "<", lambda x, w: x < w), ("le", "<=", lambda x, w: x <= w),
            ("gt", ">", lambda x, w: x > w), ("ge", ">=", lambda x, w: x >= w))


def _cmp(a, v, detail=""):
    """{eq|equals, at_least, at_most, lt, gt, le, ge} on a number. Every
    bound given must hold ({at_least: 1, at_most: 3} is a range); none
    given means >= 1."""
    good, txt = True, []
    for k, sym, op in _CMP_OPS:
        if k in a:
            good &= op(v, a[k])
            txt.append(f"{v} {sym} {a[k]}")
    if not txt:
        return v >= 1, f"{v} >= 1"
    return good, " and ".join(txt)


def _journal_count(name, a, ctx):
    marks = ctx["marks"]
    ok, bits, vals = True, [], []
    for u in _units(a, ctx):
        n = len(simlib.journal_match(u, a["re"], marks.get(u["name"])))
        vals.append(n)
        good, txt = _cmp(a, n)
        ok &= good
        bits.append(f"{u['name']}: {txt}")
    if a.get("total"):
        # total: true - one fleet-wide count (the sum over the units),
        # for a line exactly one runtime-chosen unit journals
        good, txt = _cmp(a, sum(vals))
        return _r(name, "PASS" if good else "FAIL", sum(vals),
                  f"total {txt} (" + "; ".join(
                      f"{u['name']}: {n}"
                      for u, n in zip(_units(a, ctx), vals)) + ")")
    return _r(name, "PASS" if ok else "FAIL",
              sum(vals) / len(vals) if vals else 0, "; ".join(bits))


def _module_log_count(name, a, ctx):
    """module_log_count {unit, module, re, equals|at_least|at_most}: the
    journal_count sibling over var/run/<unit>/logs/<module>.log (the module's
    stdout), lines appended since the test mark. Facts a module prints
    itself - de_precland's EVENT_OUT lines - never reach the de_comm
    journal, so standalone module behaviour is asserted here. `total:
    true` (B4-01) sums the count over the units like journal_count's -
    for a line exactly one runtime-chosen unit prints."""
    marks = ctx["marks"]
    rx = re.compile(a["re"])
    ok, bits, vals = True, [], []
    for u in _units(a, ctx):
        n = len(simlib.log_match(u, a["module"], rx,
                                 marks.get(u["name"])))
        vals.append(n)
        good, txt = _cmp(a, n)
        ok &= good
        bits.append(f"{u['name']}: {txt}")
    if a.get("total"):
        good, txt = _cmp(a, sum(vals))
        return _r(name, "PASS" if good else "FAIL", sum(vals),
                  f"total {txt} (" + "; ".join(
                      f"{u['name']}: {n}"
                      for u, n in zip(_units(a, ctx), vals)) + ")")
    return _r(name, "PASS" if ok else "FAIL",
              sum(vals) / len(vals) if vals else 0, "; ".join(bits))


def _no_journal(name, a, ctx):
    """no_journal {unit, re}: the negative sibling of journal_count -
    PASS only when the pattern has zero hits since the test mark (P4F-07:
    'the request was never raised' proofs like no APPROVAL_REQ launch)."""
    marks = ctx["marks"]
    rx = re.compile(a["re"])
    ok, bits, total = True, [], 0
    for u in _units(a, ctx):
        hits = simlib.journal_match(u, rx, marks.get(u["name"]))
        total += len(hits)
        ok &= not hits
        first = f" (first: {hits[0][4]} {hits[0][5][:60]})" if hits else ""
        bits.append(f"{u['name']}: {len(hits)}{first}")
    return _r(name, "PASS" if ok else "FAIL", total, "; ".join(bits))


def _status_field(name, a, ctx):
    """status_field {unit, path, equals|re|bounds}: one field of the
    unit's latest 1087 status as the operator bot records it (P4F-07:
    `launch.grant`, `launch.uses_left`, `launch.ready`, `launch.why`,
    `launch.last_refusal`, ...). `path` is dotted into nested objects;
    `equals`/`re` compare the resolved value, the numeric bounds apply
    _cmp."""
    bot = ctx.get("bot")
    if not bot:
        return _r(name, "SKIP", None, "no operator session")
    path = str(a.get("path", ""))
    ok, bits = True, []
    for u in _units(a, ctx):
        node = bot.status_of(u["party_id"])
        for part in path.split("."):
            node = node.get(part) if isinstance(node, dict) else None
        if node is None:
            ok = False
            bits.append(f"{u['name']}: no status path {path!r}")
            continue
        good, txt = True, []
        if any(k in a for k, _s, _op in _CMP_OPS):
            good, cmp_txt = _cmp(a, node)
            txt.append(cmp_txt)
        if a.get("re"):
            good &= bool(re.search(a["re"], str(node)))
            txt.append(f"{node!r} ~ /{a['re']}/")
        if not txt:
            txt.append(f"present: {node!r}")
        ok &= good
        bits.append(f"{u['name']}: {path}={node!r} ({'; '.join(txt)})")
    return _r(name, "PASS" if ok else "FAIL", None, "; ".join(bits))


def _journal_order(name, a, ctx):
    """journal_order {unit, first, then, proc?}: `first`'s earliest match
    must precede a `then` match. With `proc` the ordered lines come from
    logs/<proc>.log instead of the journal - a module's stdout (de_comm's
    own "Module X is not responding") never reaches the journal."""
    r1, r2 = re.compile(a["first"]), re.compile(a["then"])
    ok, bits = True, []
    marks = ctx["marks"]
    for u in _units(a, ctx):
        if a.get("proc"):
            lines = [(0, "-", "-", "-", "RAW", ln, "log")
                     for ln in simlib.log_match(u, a["proc"], r"",
                                                marks.get(u["name"]))]
        else:
            lines = simlib.journal_lines(u, marks.get(u["name"]))
        i1 = next((i for i, l in enumerate(lines)
                   if simlib.line_hit(l, r1)), None)
        i2 = next((i for i, l in enumerate(lines)
                   if i1 is not None and i > i1 and simlib.line_hit(l, r2)),
                  None)
        ok &= i1 is not None and i2 is not None
        bits.append(f"{u['name']}: first@{i1} then@{i2}")
    return _r(name, "PASS" if ok else "FAIL", None, "; ".join(bits))


def _journal_delta(name, a, ctx, ms=False):
    marks = ctx["marks"]
    u1s = ctl.resolve(str(a.get("u1", a.get("v1"))), ctx["manifest"])
    u2s = ctl.resolve(str(a.get("u2", a.get("v2"))), ctx["manifest"])
    # any: true -> pass when ANY (u1,u2) pair satisfies the delta; the
    # pair lists are comma names for runtime-decided actors (e.g. the
    # takeover adopter is whichever survivor frees its posture first)
    if a.get("any") and (len(u1s) > 1 or len(u2s) > 1):
        bits = []
        for c1 in u1s:
            for c2 in u2s:
                r = _journal_delta_pair(name, a, ctx, c1, c2, marks, ms)
                bits.append(r["detail"])
                if r["status"] == "PASS":
                    return r
        return _r(name, "FAIL", None, "; ".join(bits))
    return _journal_delta_pair(name, a, ctx, u1s[0], u2s[0], marks, ms)


def _journal_delta_pair(name, a, ctx, u1, u2, marks, ms=False):
    # RAW (unparsed) journal lines carry ts 0 - never a time reference
    h1 = [h for h in simlib.journal_match(u1, a["re1"],
                                          marks.get(u1["name"])) if h[0]]
    h2 = [h for h in simlib.journal_match(u2, a["re2"],
                                          marks.get(u2["name"])) if h[0]]
    if not h1 or not h2:
        return _r(name, "FAIL", None,
                  f"missing: {u1['name']}={len(h1)} {u2['name']}={len(h2)}")
    t1 = h1[0][0]
    if u1["name"] == u2["name"]:
        h2 = [h for h in h2 if h[0] >= t1]
        if not h2:
            return _r(name, "FAIL", None,
                      f"no {a['re2']!r} after {a['re1']!r} on {u1['name']}")
    if ms:
        # latency: signed, re2 must follow re1 (skew_ms tolerates
        # cross-unit clock skew); a missing limit is an error, not a pass
        dt = h2[0][0] - t1
        lim = a.get("max_ms")
        if lim is None:
            return _r(name, "FAIL", dt, "needs max_ms")
        skew = a.get("skew_ms", 0)
        good = -skew <= dt <= lim
        return _r(name, "PASS" if good else "FAIL", dt,
                  f"{u1['name']}:{a['re1']!r} -> {u2['name']}:{a['re2']!r}"
                  f" = {dt:.0f} ms (max {lim}, skew {skew})")
    # ordered: true -> signed (re2 must come AFTER re1); default keeps
    # the unsigned "within N s of each other" spread across units
    signed = h2[0][0] - t1
    dt = (signed if a.get("ordered") else abs(signed)) / 1000.0
    lo, hi = a.get("min_s"), a.get("max_s")
    if lo is None and hi is None:
        return _r(name, "FAIL", dt, "needs max_s and/or min_s")
    good = (lo is None or dt >= lo) and (hi is None or dt <= hi)
    return _r(name, "PASS" if good else "FAIL", dt,
              f"|{u1['name']}:{a['re1']!r} - {u2['name']}:{a['re2']!r}|"
              f" = {dt:.1f}s (min {lo}, max {hi})")


def _clock_spread(name, a, ctx):
    rx = re.compile(r"fleet_minus_unix_ms=(-?\d+)")
    vals = {}
    marks = ctx["marks"]
    for u in _units(a, ctx):
        hits = [ln for ln in simlib.journal_lines(u, marks.get(u["name"]))
                if ln[4] == "CLOCK" and rx.search(ln[5])]
        if hits:
            vals[u["name"]] = int(rx.search(hits[-1][5]).group(1))
    if not vals:
        return _r(name, "FAIL", None, "no CLOCK lines")
    if len(vals) < 2:
        return _r(name, "FAIL", None,
                  f"CLOCK lines from only {list(vals)} - a spread needs "
                  "at least two units")
    spread = max(vals.values()) - min(vals.values())
    good = spread <= a.get("max_ms", 100)
    return _r(name, "PASS" if good else "FAIL", spread,
              f"spread {spread} ms over {vals} (max {a.get('max_ms')})")


def _mission_current(name, a, ctx):
    ok, bits, seqs = True, [], []
    for u in ctl.with_fc(_units(a, ctx)):
        seq = ctl.mission_current(u)
        seqs.append(seq)
        good = seq == a["seq"]
        ok &= good
        bits.append(f"{u['name']}: seq={seq} (want {a['seq']})")
    if not seqs:
        return _r(name, "SKIP", None, "no FC units resolved")
    return _r(name, "PASS" if ok else "FAIL",
              seqs[0] if seqs else None, "; ".join(bits))


def _distance_to(name, a, ctx):
    marks = ctx["marks"]
    u = ctl.resolve(str(a["unit"]), ctx["manifest"])[0]
    hits = [h for h in simlib.journal_match(u, a["re"],
                                            marks.get(u["name"])) if h[0]]
    if not hits:
        return _r(name, "FAIL", None,
                  f"{u['name']}: no timestamped journal line matching "
                  f"{a['re']!r}")
    t = hits[0][0]
    rows, src = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return _r(name, "SKIP", None,
                  "no truth source (truth_file / run truth.jsonl - "
                  "P5-08 pending)")
    tids = a["truth"] if isinstance(a["truth"], list) else [a["truth"]]
    tol = a.get("truth_tol_ms", DEFAULT_TRUTH_TOL_MS)
    t0 = t
    tr = None
    for tid in tids:
        c = _truth_at(rows, tid, t, tol)
        if c is not None:
            tr = c
            break
    pos = _unit_pos_at(ctx, u["name"], t)
    pos_src = "telemetry"
    if a.get("detected_cls"):
        # P5-08: measure the world-model detection the operator bot saw
        # (the detect.detected envelope's pos), not the unit's own pose -
        # "the reported detection is within max_m of the truth"
        cands = [tg for tg in _bot_targets(ctx).values()
                 if tg.get("cls") == a["detected_cls"] and tg.get("pos")]
        if not cands:
            return _r(name, "SKIP", None,
                      f"operator saw no {a['detected_cls']!r} detection")
        tg = min(cands, key=lambda g: abs(g.get("t_ms", t) - t))
        pos = {"lat": tg["pos"][0], "lng": tg["pos"][1]}
        pos_src = f"detected {a['detected_cls']}"
        t = tg.get("t_ms", t)
    if tr is None:
        return _r(name, "SKIP", None,
                  f"no truth row for {tids!r} near t={t0} in {src}")
    if pos is None:
        return _r(name, "SKIP", None,
                  f"no telemetry for {u['name']} near t={t}")
    # several tids: the closest truth at t (a 'person' detection may be
    # any of the world's people)
    tr, tid_hit = min(
        ((c, tid) for tid in tids
         for c in [_truth_at(rows, tid, t, tol)] if c is not None),
        key=lambda p: _haversine(pos["lat"], pos["lng"],
                                 p[0]["lat"], _lng(p[0]) or 0),
        default=(tr, tids[0]))
    d = _haversine(pos["lat"], pos["lng"], tr["lat"], _lng(tr))
    lo, hi = a.get("min_m"), a.get("max_m")
    good = (lo is None or d >= lo) and (hi is None or d <= hi)
    return _r(name, "PASS" if good else "FAIL", round(d, 1),
              f"{u['name']} {pos_src} -> {tid_hit} = {d:.1f} m "
              f"(min {lo}, max {hi}, src {os.path.basename(src)})")


def _journal_consistent(name, a, ctx):
    """Every unit must journal the SAME set of values for capture group 1
    of `re` (e.g. the target ids of "WM_STATE (\\S+) confirmed ..."), and
    at least `min_count` (default 1) of them - agreement on nothing is
    not agreement. Catches units that act on different targets, which a
    per-unit journal_count (each unit sees 1) cannot."""
    rx = re.compile(a["re"])
    if rx.groups < 1:
        return _r(name, "FAIL", None, "re needs a capture group")
    sets, bits = {}, []
    for u in _units(a, ctx):
        vals = set()
        for ln in simlib.journal_match(u, a["re"],
                                       ctx["marks"].get(u["name"])):
            m = rx.search(ln[4] + " " + ln[5]) or rx.search(ln[5])
            if m:
                vals.add(m.group(1))
        sets[u["name"]] = vals
        bits.append(f"{u['name']}: {sorted(vals)}")
    need = int(a.get("min_count", 1))
    ref = next(iter(sets.values()), set())
    same = all(v == ref for v in sets.values())
    enough = all(len(v) >= need for v in sets.values())
    return _r(name, "PASS" if same and enough and sets else "FAIL",
              len(ref), "; ".join(bits) +
              ("" if same else " - units DISAGREE") +
              ("" if enough else f" - fewer than {need} value(s)"))


def _signals_in_range(name, a, ctx):
    """SDR-style gate: every journal line matching `re` must be stamped
    while the truth entity was within `max_m` of the unit (the stub only
    reports inside the modelled range - this proves it). Also requires
    `min_count` (default 1) hits so "never out of range" can't pass on
    silence. Static units read their fixed position; FC units their
    telemetry."""
    marks = ctx["marks"]
    ok, bits, total, checked = True, [], 0, 0
    rows, src = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return _r(name, "SKIP", None, "no truth source")
    tol = a.get("truth_tol_ms", DEFAULT_TRUTH_TOL_MS)
    for u in _units(a, ctx):
        hits = [h for h in simlib.journal_match(u, a["re"],
                                                marks.get(u["name"]))
                if h[0]]
        total += len(hits)
        p0 = u.get("position") or {}
        bad = []
        for ln in hits:
            tr = _truth_at(rows, a["truth"], ln[0], tol)
            if tr is None:
                continue
            if u["fc"]:
                up = _unit_pos_at(ctx, u["name"], ln[0])
                lat, lng = ((up or {}).get("lat"),
                            (up or {}).get("lng"))
            else:
                lat, lng = p0.get("lat"), p0.get("lng")
            if lat is None:
                continue
            checked += 1
            d = _haversine(lat, lng, tr["lat"], _lng(tr))
            if d > a["max_m"]:
                bad.append(round(d, 1))
        good = not bad
        ok &= good
        bits.append(f"{u['name']}: {len(hits)} signals, "
                    f"{len(bad)} out-of-range{bad[:3]}")
    need = int(a.get("min_count", 1))
    if total < need:
        ok = False
        bits.append(f"only {total} signals (need {need})")
    elif total and checked == 0:
        # every signal lacked a truth row or a unit position: nothing
        # was actually compared, so "none out of range" proves nothing
        return _r(name, "SKIP", total,
                  f"{total} signals but none could be compared with "
                  "truth + position")
    return _r(name, "PASS" if ok else "FAIL", total, "; ".join(bits))


# no_sep_violation: the planner layers units exactly min_vsep_m apart, and
# AMSL readings of units on different home spots differ by a cm or so
# (82 m over one home vs 76 m over another read 5.99 m) - still the layer
SEP_V_TOL_M = 0.05


def _no_sep_violation(name, a, ctx):
    units = [u["name"] for u in ctl.with_fc(_units(a, ctx))]
    if len(units) < 2:
        return _r(name, "SKIP", None,
                  "fewer than two FC units - nothing to separate")
    # a separation that was never measured is not a pass: the runner
    # always records telemetry, so a missing recorder/overlap is a defect
    tele = ctx.get("telemetry")
    if not tele:
        return _r(name, "FAIL", None,
                  "no telemetry recorder - separation not measured")
    # h_m (or min_m): required horizontal separation; v_m: a pair too
    # close horizontally still passes when it keeps v_m vertical gap
    h_min = a.get("h_m", a.get("min_m", 10))
    v_min = a.get("v_m")
    worst = (None, None)
    separated_only = False
    for i, n1 in enumerate(units):
        rows1 = tele.rows(n1)
        if not rows1:
            continue
        for n2 in units[i + 1:]:
            rows2 = tele.rows(n2)
            for r1 in rows1:
                # P4B-10: a pre-fix row reports (0,0) - pairing two of
                # them reads as a 0 m co-location; only a real 3D fix is
                # a position (fix_type absent = recorder predates GPS)
                if r1.get("lat") is None or \
                        (r1.get("fix_type") is not None and
                         r1["fix_type"] < 3):
                    continue
                r2 = min(rows2, key=lambda r:
                         abs(r["t_ms"] - r1["t_ms"]), default=None)
                if r2 is None or r2.get("lat") is None \
                        or (r2.get("fix_type") is not None and
                            r2["fix_type"] < 3) \
                        or abs(r2["t_ms"] - r1["t_ms"]) > 2000:
                    continue
                d = _haversine(r1["lat"], r1["lng"], r2["lat"], r2["lng"])
                if v_min is not None \
                        and r1.get("alt_m") is not None \
                        and r2.get("alt_m") is not None \
                        and abs(r1["alt_m"] - r2["alt_m"]) >= v_min - SEP_V_TOL_M:
                    separated_only = True
                    continue          # vertically separated - no violation
                if worst[0] is None or d < worst[0]:
                    worst = (d, f"{n1}<->{n2} @t={r1['t_ms']}")
    if worst[0] is None:
        if separated_only:
            return _r(name, "PASS", None,
                      "every overlapping pair kept the v_m vertical gap")
        return _r(name, "FAIL", None,
                  "no overlapping 3D-fix telemetry between FC units - "
                  "separation not measured")
    good = worst[0] >= h_min
    return _r(name, "PASS" if good else "FAIL", round(worst[0], 1),
              f"min pairwise distance {worst[0]:.1f} m ({worst[1]}), "
              f"h_m={h_min} v_m={v_min}")


def _coverage_grid(name, a, ctx):
    """The shared P5-12 preamble: resolve the area, pick the camera
    units, build or reuse the computed grid. -> (grid, result) where
    result is set (and grid None) on SKIP/FAIL. The grid is cached on
    ctx["coverage_grids"] keyed by (area, poly, cell_m, units) so sibling
    asserts share it and _collect can write the artefacts."""
    wld = ctx.get("world")
    if wld is None:
        return None, _r(name, "SKIP", None,
                        "no world - coverage needs a P5-08 world run")
    cell_m = float(a.get("cell_m", coverage_mod.DEFAULT_CELL_M))
    if a.get("poly"):
        poly, area_id = [list(p) for p in a["poly"]], "inline"
    elif a.get("area"):
        area_id = str(a["area"])
        spec_a = wld.areas.get(area_id)
        if spec_a is None:
            return None, _r(name, "FAIL", None,
                            f"unknown world area {area_id!r} "
                            f"(areas: {sorted(wld.areas)})")
        poly = spec_a["poly"]
    else:
        return None, _r(name, "FAIL", None, "needs area: or poly:")
    if a.get("unit") is not None:
        units = [u["name"] for u in _units(a, ctx)]
        if not units:
            return None, _r(name, "SKIP", None,
                            f"unit {a['unit']!r} resolves to none")
    else:
        # every fleet unit running the camera stub (D1 - the cameras
        # that really looked)
        units = [u["name"] for u in ctx["manifest"]["units"]
                 if "stub_ai_box" in u.get("modules", [])]
    if not units:
        return None, _r(name, "SKIP", None,
                        "no camera units in this run")
    key = (area_id, tuple(tuple(p) for p in poly), cell_m,
           tuple(sorted(units)))
    grids = ctx.setdefault("coverage_grids", {})
    if key in grids:
        return grids[key], None
    tele = ctx.get("telemetry")
    if tele is None:
        return None, _r(name, "SKIP", None, "no telemetry recorder")
    cam = wld.sensors["camera"]
    by_name = {u["name"]: u for u in ctx["manifest"]["units"]}
    looks, n_rows, missing_att = [], 0, set()
    for uname in units:
        u = by_name.get(uname) or {}
        # the stub's own camera_kind override decides thermal vs rgb
        kind = str(((u.get("config") or {}).get("stub_ai_box") or {})
                   .get("camera_kind") or cam.get("kind", "rgb"))
        env_f = wld.sensor_factors(kind)
        for r in coverage_mod.thin_rows(tele.rows(uname),
                                        cam["fps_boxes"]):
            n_rows += 1
            if r.get("lat") is None or r.get("rel_alt_m") is None:
                continue
            if r.get("roll") is None or r.get("pitch") is None \
                    or r.get("yaw") is None:
                missing_att.add(uname)
                continue
            looks.append((gp.GeoAttitude(r["roll"], r["pitch"],
                                        r["yaw"]),
                          gp.GeoPose(r["lat"], r["lng"],
                                     r["rel_alt_m"]),
                          env_f, kind))
    if not looks:
        return None, _r(
            name, "SKIP", None,
            "telemetry lacks attitude (old tlog?)" if missing_att
            else f"no usable telemetry rows for {units}")
    grid = {"grid": coverage_mod.compute(poly, wld.origin, cell_m, wld,
                                         cam, looks),
            "area": area_id, "cell_m": cell_m, "units": units}
    grids[key] = grid
    det = []
    if missing_att:
        det.append("attitude missing for "
                   + ",".join(sorted(missing_att)))
    grid["note"] = "; ".join(det)
    return grid, None


def _coverage_complete(name, a, ctx):
    if a.get("area") or a.get("poly"):
        grid, early = _coverage_grid(name, a, ctx)
        if early is not None:
            return early
        s = coverage_mod.summary(grid["grid"])
        want = float(a.get("min", 1.0))
        good = s["coverage"] >= want
        return _r(name, "PASS" if good else "FAIL",
                  round(s["coverage"], 3),
                  f"{s['covered']}/{s['cells']} cells covered "
                  f"({s['looks']} looks, {grid['units']})"
                  + (f" [{grid['note']}]" if grid.get("note") else ""))
    if a.get("re"):
        ok, bits = True, []
        for u in _units(a, ctx):
            n = len(simlib.journal_match(
                u, a["re"], ctx["marks"].get(u["name"])))
            ok &= n >= 1
            bits.append(f"{u['name']}: {n} match(es)")
        return _r(name, "PASS" if ok else "FAIL", None, "; ".join(bits))
    rows, src = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return _r(name, "SKIP", None,
                  "no coverage source (re: or truth rows - P5-08 pending)")
    cov = [r for r in rows if r.get("type") == "coverage"]
    if not cov:
        return _r(name, "SKIP", None,
                  f"no coverage rows in {src}")
    done = [r for r in cov if r.get("covered") or r.get("visited")]
    frac = len(done) / len(cov)
    return _r(name, "PASS" if frac >= a.get("min", 1.0) else "FAIL",
              round(frac, 3),
              f"{len(done)}/{len(cov)} coverage cells visited "
              f"(src {os.path.basename(src)})")


def _pod_mean(name, a, ctx):
    """P5-12: the mean cumulative POD over the whole search area -
    "would a person ANYWHERE in sector_a likely have been found?"
    Needs min (an unbounded POD assert is vacuous)."""
    if a.get("min") is None:
        return _r(name, "FAIL", None, "needs min")
    grid, early = _coverage_grid(name, a, ctx)
    if early is not None:
        return early
    s = coverage_mod.summary(grid["grid"])
    good = s["pod_mean"] >= float(a["min"])
    return _r(name, "PASS" if good else "FAIL", round(s["pod_mean"], 3),
              f"mean POD {s['pod_mean']:.3f} over {s['cells']} cells "
              f"(coverage {s['coverage']:.3f})")


def _pod_at(name, a, ctx):
    """P5-12: the POD of the cell holding one truth entity - the
    planning/sensor split: high pod_at + missed detection = a sensor
    chain failure; low pod_at = a planning failure. The position is
    the entity's truth at t_s (seconds since the world epoch) or the
    last truth row when t_s is omitted. min and max both accepted
    (max: 0 asserts an occluded cell)."""
    if a.get("min") is None and a.get("max") is None:
        return _r(name, "FAIL", None, "needs min and/or max")
    tid = a.get("truth")
    if not tid:
        return _r(name, "FAIL", None, "needs truth:")
    wld = ctx.get("world")
    if wld is None:
        return _r(name, "SKIP", None,
                  "no world - pod_at needs a P5-08 world run")
    e = wld.entity(str(tid))
    if e is None:
        return _r(name, "FAIL", None,
                  f"unknown truth entity {tid!r}")
    if a.get("t_s") is not None:
        lat, lng, _alt = e.pos_at(float(a["t_s"]), wld)
        where = f"t_s {a['t_s']}"
    else:
        rows, src = _truth_rows(ctx, a.get("truth_file"))
        last = None
        for r in _entity_rows(rows or []):
            if _row_tid(r) == str(tid) and r.get("lat") is not None:
                last = r
        if last is None:
            return _r(name, "SKIP", None,
                      f"no truth rows for {tid} - pass t_s to locate "
                      "it without a recorder")
        lat, lng = last["lat"], _lng(last)
        where = f"truth row t {_row_t(last)}"
    if a.get("area") or a.get("poly"):
        grid, early = _coverage_grid(name, a, ctx)
        if early is not None:
            return early
    else:
        grids = ctx.get("coverage_grids") or {}
        if len(grids) != 1:
            return _r(name, "FAIL", None,
                      "pod_at needs area:/poly: (or exactly one "
                      "coverage_complete/pod_mean grid before it)")
        grid = next(iter(grids.values()))
    cell = coverage_mod.cell_at(grid["grid"], lat, lng)
    if cell is None:
        return _r(name, "FAIL", None,
                  f"{tid} at ({lat:.6f},{lng:.6f}) lies outside the "
                  f"{grid['area']} area")
    pod = cell["pod"]
    good = True
    if a.get("min") is not None:
        good &= pod >= float(a["min"])
    if a.get("max") is not None:
        good &= pod <= float(a["max"])
    return _r(name, "PASS" if good else "FAIL", round(pod, 3),
              f"{tid} ({where}) sits in a cell with POD {pod:.3f} "
              f"({cell['looks']} looks)")


def _detection_precision(name, a, ctx):
    """Truth detection rows ({"type":"detection", tid, cls, sent} rows
    the P5-08 stubs append): precision = emitted detections that name a
    real world entity / all emitted detections of the class - clutter
    rows (tid "clutter") are the false positives. `min` is the required
    fraction; `unit` narrows the rows to one stub."""
    rows, src = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return _r(name, "SKIP", None,
                  "no truth source (truth_file / run truth.jsonl)")
    det = [r for r in rows if r.get("type") == "detection"
           and r.get("sent")
           and (a.get("cls") is None or r.get("cls") == a["cls"])
           and (a.get("unit") is None or r.get("unit") == a["unit"])]
    if not det:
        return _r(name, "SKIP", None,
                  f"no sent detection rows for cls {a.get('cls')!r} in "
                  f"{os.path.basename(src)}")
    good = [r for r in det if r.get("tid") not in (None, "clutter")]
    prec = len(good) / len(det)
    want = a.get("min", 1.0)
    return _r(name, "PASS" if prec >= want else "FAIL",
              round(prec, 3),
              f"{len(good)}/{len(det)} detections hit a truth entity "
              f"({len(det) - len(good)} clutter) >= {want} "
              f"(src {os.path.basename(src)})")


def _entity_rows(rows):
    """The recorder's 1 Hz entity rows - a detection/coverage row reuses
    the tid but reports the REPORTED position, not the truth."""
    return [r for r in rows
            if r.get("type") not in ("detection", "coverage")]


def _entity_cls(rows, tid):
    for r in _entity_rows(rows):
        if r.get("tid") == tid:
            return r.get("cls")
    return None


def _recall_targets(rows, spec):
    """detection_recall's truth: id | [ids] | cls -> the entity ids it
    names. A string that matches an entity row's tid is an id; anything
    else is read as a class name (every entity of that class)."""
    if isinstance(spec, list):
        return [str(t) for t in spec]
    spec = str(spec)
    if any(r.get("tid") == spec for r in _entity_rows(rows)):
        return [spec]
    return [str(t) for t in sorted({r.get("tid") for r in _entity_rows(rows)
                                    if r.get("cls") == spec
                                    and r.get("tid")})]


def _bot_targets(ctx):
    """The operator bot's world-model targets, capped at the --hold
    boundary when one was stamped (targets carry t_ms on the same clock
    as the truth rows): a detection the parked viewer session produced
    must not grade the scenario - it would inflate recalls and trip
    `max: 0` negative controls."""
    bot = ctx.get("bot")
    tgs = (bot.targets() if bot else {}) or {}
    hold_ms = ctx.get("hold_ms")
    if hold_ms is None:
        return tgs
    return {tid: tg for tid, tg in tgs.items()
            if tg.get("t_ms", 0) <= hold_ms}


def _first_find(a, ctx, rows, tid, cls):
    """(found, t_ms of the first find or None) for one truth entity -
    the shared finder of detection_recall and time_to_detect. src ai |
    sdr: the earliest sent detection row with that tid (unit: narrows
    the rows to one stub). src bot (the default): the earliest operator
    target of the entity's class within match_m of the entity's truth
    at the target's t_ms - the whole product chain."""
    src = str(a.get("src", "bot"))
    match_m = float(a.get("match_m", 25))
    tol = a.get("truth_tol_ms", DEFAULT_TRUTH_TOL_MS)
    if src in ("ai", "sdr"):
        best = None
        for r in rows:
            if r.get("type") != "detection" or not r.get("sent") \
                    or r.get("src") != src or _row_tid(r) != tid:
                continue
            if a.get("unit") is not None and r.get("unit") != a["unit"]:
                continue
            t = _row_t(r)
            if best is None or t < best:
                best = t
        return (best is not None, best)
    best = None
    for tg in _bot_targets(ctx).values():
        if tg.get("cls") != cls or not tg.get("pos"):
            continue
        tr = _truth_at(rows, tid, tg.get("t_ms", 0), tol)
        if tr is None:
            continue
        if _haversine(tg["pos"][0], tg["pos"][1],
                      tr["lat"], _lng(tr)) > match_m:
            continue
        t = tg.get("t_ms", 0)
        if best is None or t < best:
            best = t
    return (best is not None, best)


def _dominant_why(rows, a, tid):
    """The most frequent why over the entity's unsent ai rows - the
    reason it stayed hidden (no row at all -> it was never in frame)."""
    counts = {}
    for r in rows:
        if r.get("type") != "detection" or r.get("sent") \
                or r.get("src") != "ai" or _row_tid(r) != tid:
            continue
        if a.get("unit") is not None and r.get("unit") != a["unit"]:
            continue
        counts[r.get("why") or "?"] = counts.get(r.get("why") or "?", 0) + 1
    if not counts:
        return "no in-frame row"
    return max(sorted(counts), key=counts.get)


def _recall_common(name, a, ctx):
    """The shared preamble: rows + src + targets, or the result to
    return instead."""
    rows, src_file = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return None, _r(name, "SKIP", None,
                        "no truth source (truth_file / run truth.jsonl)")
    src = str(a.get("src", "bot"))
    if src not in ("ai", "sdr", "bot", "render_vis"):
        return None, _r(name, "FAIL", None,
                        f"src must be ai|sdr|bot|render_vis, got {src!r}")
    if src in ("bot", "render_vis") and not ctx.get("bot"):
        return None, _r(name, "SKIP", None, "no operator session")
    tids = _recall_targets(rows, a["truth"])
    if not tids:
        return None, _r(name, "FAIL", None,
                        f"no truth entities match {a['truth']!r} in "
                        f"{os.path.basename(src_file)}")
    return (rows, tids), None


def _detection_recall(name, a, ctx):
    """Recall over the named truth entities: the fraction that were
    *found*. Found means: src ai|sdr - at least one sent: true detection
    row with that tid; src bot (the default) - an operator target of
    the entity's class within match_m (default 25 m) of the entity's
    truth at the target's t_ms, i.e. the whole product chain. `max: 0`
    is the negative control ("must stay hidden"); the detail line lists
    the found and missed ids with each miss's dominant why."""
    common, early = _recall_common(name, a, ctx)
    if early is not None:
        return early
    rows, tids = common
    # P5-15 D5/D8: src render_vis narrows the denominator to entities
    # the ID pass actually rendered >= min_px for >= min_frames frames -
    # the recall is over what the pixels could show, not what the model
    # guessed. "Found" stays the whole product chain (bot target).
    src_label = str(a.get("src", "bot"))
    if a.get("src") == "render_vis":
        min_px = float(a.get("min_px", 16))
        min_frames = int(a.get("min_frames", 1))
        seen = {}
        for r in rows:
            if r.get("type") != "event" or r.get("src") != "render_vis" \
                    or r.get("kind") != "visibility":
                continue
            if a.get("unit") is not None and r.get("unit") != a["unit"]:
                continue
            if float(r.get("px") or 0) >= min_px:
                seen.setdefault(r.get("tid"), set()).add(r.get("frame_no"))
        tids = [t for t in tids
                if len(seen.get(t) or ()) >= min_frames]
        if not tids:
            return _r(name, "FAIL", None,
                      "no truth entity reached the render_vis "
                      f"visibility bar (min_px {min_px:g}, "
                      f"min_frames {min_frames})")
        a = dict(a, src="bot")
    found, missed = [], []
    for tid in tids:
        cls = _entity_cls(rows, tid)
        hit, _t = _first_find(a, ctx, rows, tid, cls) if cls is not None \
            else (False, None)
        (found if hit else missed).append(tid)
    frac = len(found) / len(tids)
    lo, hi = a.get("min"), a.get("max")
    if lo is None and hi is None:
        lo = 1.0                    # recall defaults to "found them all"
    good = (lo is None or frac >= lo) and (hi is None or frac <= hi)
    bits = [f"found {found}"] if found else ["found none"]
    if missed:
        bits.append("missed " + ", ".join(
            f"{t} ({_dominant_why(rows, a, t)})" for t in missed))
    return _r(name, "PASS" if good else "FAIL", round(frac, 3),
              "; ".join(bits) + f" (src {src_label})")


def _time_to_detect(name, a, ctx):
    """Seconds from the scenario's `start` step to the first find of
    one truth entity (the same found rule as detection_recall;
    match_m applies to the bot src) - falling back to the world epoch
    (world_epoch.json) when the timeline never ran a start step.
    FAIL when the entity is never found or the time exceeds max_s;
    realtime: true scenarios SKIP this when SITL did not hold
    speed-up ~1."""
    if a.get("max_s") is None:
        return _r(name, "FAIL", None, "needs max_s")
    common, early = _recall_common(name, a, ctx)
    if early is not None:
        return early
    rows, tids = common
    if len(tids) != 1:
        return _r(name, "FAIL", None,
                  f"truth must name exactly one entity, got {a['truth']!r}")
    # measuring from the world epoch would charge the arm/EKF warm-up
    # between fleet-up and `start` against max_s
    epoch, basis = ctx.get("start_ms"), "the start step"
    if epoch is None:
        epoch, basis = ctx.get("world_epoch_ms"), "the epoch"
    if epoch is None:
        return _r(name, "SKIP", None,
                  "no start step or world epoch - time_to_detect needs "
                  "a P5-08 world run")
    tid = tids[0]
    hit, t_ms = _first_find(a, ctx, rows, tid, _entity_cls(rows, tid))
    if not hit:
        return _r(name, "FAIL", None,
                  f"{tid} never found (src {a.get('src', 'bot')})")
    dt = (t_ms - epoch) / 1000.0
    good = dt <= float(a["max_s"])
    return _r(name, "PASS" if good else "FAIL", round(dt, 1),
              f"{tid} found {dt:.1f} s after {basis} "
              f"(src {a.get('src', 'bot')}, max {a['max_s']})")


def _stub_deterministic(name, a, ctx):
    """Same-seed acceptance: run <module> --selftest <poses> twice in a
    scratch run dir and byte-compare stubs/<unit>.log. Uses the unit's
    generated module config (the world + signal_detect blocks live run
    sees), so the compared output is the real model path on a virtual
    clock."""
    import subprocess
    module = a["module"]
    unit = str(a["unit"])
    poses = a["poses"]
    if not os.path.isabs(poses):
        poses = os.path.join(os.path.dirname(ctx["file"]), poses)
    if not os.path.isfile(poses):
        return _r(name, "FAIL", None, f"poses file missing: {poses}")
    gen = os.path.join(simlib.run_dir(ctx["sim"]), unit, module,
                       f"{module}.config.json")
    if not os.path.isfile(gen):
        return _r(name, "SKIP", None,
                  f"{unit}.{module} not in this sim's config")
    scr = os.path.join(simlib.run_dir(ctx["sim"]), "det", unit, module)
    cfg = os.path.join(scr, f"{module}.config.json")
    try:
        os.makedirs(scr, exist_ok=True)
        with open(gen) as f, open(cfg, "w") as g:
            g.write(f.read())
    except OSError as e:
        return _r(name, "FAIL", None, f"config copy: {e}")
    script = os.path.join(simlib.SIM_ROOT, "stubs", f"{module}.py")
    logs = []
    det_run = os.path.join(simlib.run_dir(ctx["sim"]), "det")
    log_path = os.path.join(det_run, "stubs", f"{unit}.log")
    for i in (1, 2):
        try:
            if os.path.isfile(log_path):
                os.unlink(log_path)
            r = subprocess.run(
                ["python3", script, "-c", cfg, "--selftest", poses,
                 "--dur", str(a.get("dur_s", 60))],
                capture_output=True, text=True,
                timeout=float(a.get("dur_s", 60)) + 60)
        except Exception as e:
            return _r(name, "FAIL", None, f"replay {i}: {e}")
        if r.returncode != 0:
            return _r(name, "FAIL", None,
                      f"replay {i} exit {r.returncode}: "
                      f"{r.stderr[-300:]}")
        try:
            logs.append(open(log_path, "rb").read())
        except OSError as e:
            return _r(name, "FAIL", None, f"replay {i} log: {e}")
    if not logs[0] or not logs[1]:
        return _r(name, "FAIL", 0,
                  "stub produced an empty log - two empty logs are not "
                  "a determinism proof")
    same = logs[0] == logs[1]
    return _r(name, "PASS" if same else "FAIL", len(logs[0]),
              f"two same-seed selftests, stub log {len(logs[0])} B "
              f"({'identical' if same else 'DIFFERS'})")


def _final_state(name, a, ctx):
    bot = ctx.get("bot")
    tele = ctx.get("telemetry")
    if not bot:
        return _r(name, "SKIP", None, "no operator session")
    want_states = a.get("in", a.get("state"))
    if want_states is not None and not isinstance(want_states, list):
        want_states = [want_states]
    want_modes = a.get("fc_mode")
    if want_modes is not None and not isinstance(want_modes, list):
        want_modes = [want_modes]
    ok, bits = True, []
    for u in _units(a, ctx):
        st = bot.status_of(u["party_id"])
        cur = st.get("st") or st.get("state")
        mis = st.get("mis")
        good = cur is not None
        if want_states is not None:
            good &= cur in want_states
        if a.get("mis") is not None:
            good &= mis == a["mis"]
        if a.get("re"):
            good &= bool(re.search(a["re"], json.dumps(st)))
        mode = None
        if want_modes is not None:
            mode = st.get("fc_mode") or st.get("mode") or \
                (tele.latest(u["name"]).get("mode") if tele else None)
            good &= mode is not None and str(mode).upper() in \
                [str(m).upper() for m in want_modes]
        ok &= good
        bits.append(f"{u['name']}: st={cur} mis={mis}"
                    + (f" mode={mode}" if want_modes is not None else ""))
    return _r(name, "PASS" if ok else "FAIL", None, "; ".join(bits))


def _fc_items_unchanged(name, a, ctx):
    base = (ctx.get("baseline") or {}).get("missions", {})
    cause = re.compile(a.get("cause_re", CAUSE_RE))
    ok, bits, skipped, compared = True, [], False, 0
    for u in _units(a, ctx):
        if not u["fc"]:
            bits.append(f"{u['name']}: no fc - skipped")
            continue
        now = mission_signature(u)
        was = base.get(u["name"])
        if was is None:
            # no baseline (the download failed at start): cannot tell
            # "unchanged" from "changed" - say so instead of guessing
            skipped = True
            bits.append(f"{u['name']}: no baseline - cannot compare")
            continue
        if now is None:
            ok = False
            bits.append(f"{u['name']}: mission unreadable now")
            continue
        compared += 1
        if now == was:
            bits.append(f"{u['name']}: unchanged")
            continue
        caused = any(simlib.line_hit(ln, cause)
                     for ln in simlib.journal_lines(
                         u, ctx["marks"].get(u["name"])))
        ok &= caused
        bits.append(f"{u['name']}: CHANGED "
                    f"({'caused' if caused else 'NO CAUSE JOURNALED'})")
    if ok and compared == 0 and skipped:
        return _r(name, "SKIP", None, "; ".join(bits))
    return _r(name, "PASS" if ok else "FAIL", None, "; ".join(bits))


def _mission_items(name, a, ctx):
    """mission_items {unit, equals|at_least|at_most}: the FC's mission
    item count from a full MAVLink read-back over serial0 - the "did the
    plan actually land on the FC" number the capacity tests
    (tests/capacity/) measure."""
    ok, bits, vals, any_fc = True, [], [], False
    for u in _units(a, ctx):
        if not u["fc"]:
            bits.append(f"{u['name']}: no fc - skipped")
            continue
        any_fc = True
        items = mission_download(u)
        if items is None:
            ok = False
            bits.append(f"{u['name']}: mission unreadable")
            continue
        vals.append(len(items))
        good, txt = _cmp(a, len(items))
        ok &= good
        bits.append(f"{u['name']}: {txt}")
    if ok and not any_fc:
        return _r(name, "SKIP", None, "; ".join(bits))
    return _r(name, "PASS" if ok else "FAIL",
              sum(vals) / len(vals) if vals else None, "; ".join(bits))


def _mission_pull(name, a, ctx):
    """mission_pull {unit, secs, equals|at_least|at_most}: the plan
    read-back the WebClient mission display uses - a RemoteExecute
    {C:501} reload makes de_mavlink stream the FC mission as mt 1027
    WPS chunks (ms.n items each, the last chunk carries ms.i=999
    WAYPOINT_LAST_CHUNK). Counts the items a GCS would draw and
    requires the terminating chunk, so a truncated stream fails the
    bound AND the marker. 501 is ignored while is_gcs_blocked or a
    waypoint transfer is in flight - give secs room after a big
    upload."""
    secs = float(a.get("secs", 90))
    ok, bits, vals, any_fc = True, [], [], False
    for u in _units(a, ctx):
        if not u["fc"]:
            bits.append(f"{u['name']}: no fc - skipped")
            continue
        any_fc = True
        party = u["party_id"]
        frame = {"ty": "i", "mt": 1005, "tg": party, "ms": {"C": 501}}
        if ctl.server()["type"] == "fake":
            got = ctl.gcs_roundtrip(
                lambda _f, fr=frame: [(party, dict(fr))], secs)
            chunks = [m for m in got.get(party, [])
                      if m.get("mt") == 1027]
        else:
            chunks = ctl.ctrl_stream(
                "reqm %g 1027 %s" % (secs, json.dumps([frame])), secs)
        n = sum(int((m.get("ms") or {}).get("n", 0)) for m in chunks)
        # an empty mission answers one bare {"n": 0} with no "i" at all
        last = bool(chunks) and \
            ((chunks[-1].get("ms") or {}).get("i") == 999 or
             (len(chunks) == 1 and n == 0))
        vals.append(n)
        good, txt = _cmp(a, n)
        ok &= good and last
        bits.append(f"{u['name']}: {txt}; {len(chunks)} chunk(s), "
                    f"last marker {'999' if last else 'MISSING'}")
    if ok and not any_fc:
        return _r(name, "SKIP", None, "; ".join(bits))
    return _r(name, "PASS" if ok else "FAIL",
              sum(vals) / len(vals) if vals else None, "; ".join(bits))


def _no_fc_command_to_static(name, a, ctx):
    ok, bits, n_all = True, [], 0
    for u in _units(a, ctx):
        if u["fc"]:
            bits.append(f"{u['name']}: has an fc - skipped")
            continue
        n = sum(1 for ln in simlib.journal_lines(
            u, ctx["marks"].get(u["name"])) if FC_LINE_RE.search(
            ln[4] + " " + ln[5]))
        n_all += n
        ok &= n == 0
        bits.append(f"{u['name']}: {n} FC-facing line(s)")
    return _r(name, "PASS" if ok else "FAIL", n_all, "; ".join(bits))


def _pid_running(pid):
    """/proc/<pid> exists and the process is not a zombie (a killed
    child nobody reaped still has a /proc entry)."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            # state is the field after the ")" that closes comm
            return f.read().rsplit(")", 1)[1].split()[0] not in ("Z", "X")
    except (OSError, IndexError):
        return False


def _proc_state(name, a, ctx, want_alive):
    sim = ctx["manifest"]["name"]
    ok, bits, pids = True, [], []
    if str(a.get("unit", "")) == "server":
        owners = [("server", a.get("proc", "all"))]
        if a.get("proc", "all") == "all":
            owners = [("server", p)
                      for p in ctx["manifest"]["server_procs"]]
    else:
        owners = [(u["name"], a.get("proc", "de_comm"))
                  for u in _units(a, ctx)]
    live = simlib.read_pids(sim)
    for owner, proc in owners:
        pid = live.get(f"{owner}.{proc}")
        alive = pid is not None and _pid_running(pid)
        ok &= alive == want_alive
        pids.append(pid or 0)
        bits.append(f"{owner}.{proc}: {'alive' if alive else 'down'}"
                    f"({pid or '-'})")
    return _r(name, "PASS" if ok else "FAIL",
              pids[0] if pids else None, "; ".join(bits))


def _operator_log_count(name, a, ctx):
    path = ctx.get("operator_log")
    if not path or not os.path.isfile(path):
        return _r(name, "SKIP", None, "no operator.log")
    rx = re.compile(a["re"])
    n = sum(1 for ln in open(path, errors="replace") if rx.search(ln))
    good, txt = _cmp(a, n)
    return _r(name, "PASS" if good else "FAIL", n, txt)


def _link_offline(name, a, ctx):
    """Count offline intervals seen by the link monitor (links.jsonl);
    `min_s` keeps only intervals that lasted at least that long. An
    interval still open at assert time counts once."""
    events = ctx.get("link_events") or {}
    min_s = float(a.get("min_s", 0))
    now_ms = int(time.time() * 1000)
    # --hold: transitions after the boundary are viewer traffic; an
    # interval still open at the boundary closes there
    hold_ms = ctx.get("hold_ms")
    if hold_ms is not None:
        now_ms = min(now_ms, hold_ms)
    ok, bits, total = True, [], 0
    for u in _units(a, ctx):
        ev = events.get(u["party_id"], [])
        if hold_ms is not None:
            ev = [(t, on) for t, on in ev if t <= hold_ms]
        n = 0
        t_off = None
        # the sentinel closes an interval still open now: it counts only
        # if it has already lasted min_s
        for t, online in ev + [(now_ms, True)]:
            if not online and t_off is None:
                t_off = t
            elif online and t_off is not None:
                dur = (t - t_off) / 1000.0
                if dur >= min_s:
                    n += 1
                t_off = None
        total += n
        good, txt = _cmp(a, n)
        ok &= good
        bits.append(f"{u['name']}: {n} offline interval(s) "
                    f"(min_s {min_s:g}; want {txt.split(' ', 1)[-1]})")
    return _r(name, "PASS" if ok else "FAIL", total, "; ".join(bits))


# ------------------------------------------------------------- invariants

def _sample_rows(ctx):
    """samples.jsonl rows (sampler thread output), capped at hold_ms
    like every other reader. ctx["samples"] (a pre-parsed list) wins
    over ctx["samples_file"] so tests can feed rows directly."""
    rows = ctx.get("samples")
    if rows is None:
        path = ctx.get("samples_file")
        rows = []
        if path and os.path.isfile(path):
            with open(path, errors="replace") as f:
                for ln in f:
                    try:
                        rows.append(json.loads(ln))
                    except ValueError:
                        continue
    hold_ms = ctx.get("hold_ms")
    if hold_ms is not None:
        rows = [r for r in rows if r.get("t_ms", 0) <= hold_ms]
    return rows


def _sample_keys(rows, a, ctx):
    """pidfile stems the args select: proc matches the stem's suffix,
    unit its prefix; either absent = all sampled processes."""
    proc = a.get("proc")
    keys = sorted({r["key"] for r in rows if r.get("key")})
    if proc:
        keys = [k for k in keys if k.split(".", 1)[1] == proc]
    if str(a.get("unit", "all")) != "all":
        sel = {u["name"] for u in _units(a, ctx)}
        keys = [k for k in keys if k.split(".", 1)[0] in sel]
    return keys


def _no_unexpected_restart(name, a, ctx):
    """Every pid change a tracked process made must sit inside a fault
    window (kill/restart/module_kill/server_down/reset/server_up) - a
    process that dies or is reborn with no such step is a restart nobody
    asked for. Events come from ctx["proc_events"]; `tol_s` is how far
    the observed change may drift from the ctl call."""
    rows = _sample_rows(ctx)
    events = ctx.get("proc_events") or []
    tol_ms = float(a.get("tol_s", 10)) * 1000

    def allowed(key, t_ms):
        ku, _, kp = key.partition(".")
        for e in events:
            if abs(t_ms - e["t_ms"]) > tol_ms:
                continue
            if e.get("unit") not in (ku, "all"):
                continue
            if e.get("proc") not in (kp, "all"):
                continue
            return e
        return None

    ok, bits, worst = True, [], 0
    keys = _sample_keys(rows, a, ctx)
    if not keys:
        return _r(name, "SKIP", None, "no sampled processes")
    last_t = max(r["t_ms"] for r in rows)
    for key in keys:
        seq = [r for r in rows if r["key"] == key]
        prev = None
        for r in seq:
            if prev is not None and r["pid"] != prev["pid"]:
                ev = allowed(key, r["t_ms"])
                if ev is None:
                    ok = False
                    worst += 1
                    bits.append(
                        f"{key}: pid {prev['pid']} -> {r['pid']} "
                        f"at +{(r['t_ms'] - seq[0]['t_ms']) / 1000:.0f}s "
                        "with no fault window")
            prev = r
        # a key that stopped being sampled well before the run ended
        # died mid-run - the last sighting stamps the change
        if seq and last_t - seq[-1]["t_ms"] > 2500:
            ev = allowed(key, seq[-1]["t_ms"])
            if ev is None:
                ok = False
                worst += 1
                bits.append(f"{key}: pid {seq[-1]['pid']} vanished "
                            "mid-run with no fault window")
    detail = "; ".join(bits) if bits else \
        f"{len(keys)} process(es): every pid change inside a fault window"
    return _r(name, "PASS" if ok else "FAIL", worst, detail)


def _rss_growth(name, a, ctx):
    """Least-squares slope of VmRSS vs time per process, after warmup_s
    of samples; must stay <= max_mb_per_h."""
    rows = _sample_rows(ctx)
    limit = a.get("max_mb_per_h")
    if limit is None:
        return _r(name, "FAIL", None, "rss_growth needs max_mb_per_h")
    warmup_ms = float(a.get("warmup_s", 30)) * 1000
    ok, bits, worst = True, [], 0.0
    keys = _sample_keys(rows, a, ctx)
    if not keys:
        return _r(name, "SKIP", None, "no sampled processes")
    for key in keys:
        seq = [r for r in rows if r["key"] == key]
        if not seq:
            continue
        t0 = seq[0]["t_ms"]
        seq = [r for r in seq if r["t_ms"] - t0 >= warmup_ms]
        if len(seq) < 2:
            bits.append(f"{key}: {len(seq)} post-warmup sample(s) - SKIP")
            continue
        n = len(seq)
        sx = sum(r["t_ms"] for r in seq)
        sy = sum(r["rss_kb"] for r in seq)
        sxx = sum(r["t_ms"] ** 2 for r in seq)
        sxy = sum(r["t_ms"] * r["rss_kb"] for r in seq)
        den = n * sxx - sx * sx
        slope = (n * sxy - sx * sy) / den if den else 0.0
        mb_h = slope * 3600 * 1000 / 1024
        good = mb_h <= float(limit)
        ok &= good
        worst = max(worst, mb_h)
        bits.append(f"{key}: {mb_h:+.2f} MB/h (max {limit:g})")
    return _r(name, "PASS" if ok else "FAIL",
              round(worst, 2), "; ".join(bits))


def _fd_growth(name, a, ctx):
    """Open-fd count growth over the run per process <= max."""
    rows = _sample_rows(ctx)
    limit = a.get("max")
    if limit is None:
        return _r(name, "FAIL", None, "fd_growth needs max")
    ok, bits, worst = True, [], 0
    keys = _sample_keys(rows, a, ctx)
    if not keys:
        return _r(name, "SKIP", None, "no sampled processes")
    for key in keys:
        seq = [r for r in rows if r["key"] == key]
        if not seq:
            continue
        growth = seq[-1]["fd"] - seq[0]["fd"]
        good = growth <= float(limit)
        ok &= good
        worst = max(worst, growth)
        bits.append(f"{key}: {seq[0]['fd']} -> {seq[-1]['fd']} "
                    f"({growth:+d}; max {limit:g})")
    return _r(name, "PASS" if ok else "FAIL", worst, "; ".join(bits))


def _journal_size(name, a, ctx):
    """Total bytes of a unit's journal dir <= max_mb."""
    limit = a.get("max_mb")
    if limit is None:
        return _r(name, "FAIL", None, "journal_size needs max_mb")
    ok, bits, worst = True, [], 0.0
    for u in _units(a, ctx):
        d = simlib.journal_dir(u)
        total = 0
        try:
            for f in os.listdir(d):
                total += os.path.getsize(os.path.join(d, f))
        except OSError:
            pass
        mb = total / 1048576.0
        good = mb <= float(limit)
        ok &= good
        worst = max(worst, mb)
        bits.append(f"{u['name']}: {mb:.2f} MB (max {limit:g})")
    return _r(name, "PASS" if ok else "FAIL",
              round(worst, 3), "; ".join(bits))


def _recovers_after_fault(name, a, ctx):
    """Every connectivity restore the faults scheduled (link_cut dur_s,
    server_down dur_s, de_comm freeze dur_s, restart) must be followed
    by the unit back online - and, for FC units, fresh telemetry -
    within within_s of the expected restore instant."""
    events = ctx.get("restore_events") or []
    if not events:
        return _r(name, "SKIP", None, "no fault restores recorded")
    within_ms = float(a.get("within_s", 60)) * 1000
    link_events = ctx.get("link_events") or {}
    tele = ctx.get("telemetry")
    ok, bits, n = True, [], 0
    for e in events:
        t0, horizon = e["t_ms"], e["t_ms"] + within_ms
        for u in ctl.resolve(str(e.get("unit", "all")),
                             ctx["manifest"]):
            n += 1
            ev = link_events.get(u["party_id"], [])
            state = True     # no transitions at all = still online
            for t, on in sorted(ev):
                if t <= horizon:
                    state = on
            link_ok = state
            tele_ok = True
            if u.get("fc"):
                if not tele:
                    tele_ok = False
                else:
                    tele_ok = any(
                        t0 < r.get("t_ms", 0) <= horizon
                        for r in tele.rows(u["name"]))
            good = link_ok and tele_ok
            ok &= good
            detail = f"link {'up' if link_ok else 'down'}"
            if u.get("fc"):
                detail += f", tele {'fresh' if tele_ok else 'stale'}"
            bits.append(f"{u['name']}@{e['kind']}: "
                        f"{'ok' if good else 'NOT recovered'} ({detail})")
    return _r(name, "PASS" if ok else "FAIL", n, "; ".join(bits))


def _no_stuck_state(name, a, ctx):
    """A state-reporting journal line (re) must change value at least
    every max_s: a run of identical matching msgs spanning longer means
    the reporter is stuck."""
    rx = re.compile(a["re"])
    max_ms = float(a.get("max_s", 60)) * 1000
    marks = ctx["marks"]
    ok, bits, worst = True, [], 0.0
    for u in _units(a, ctx):
        seq = [ln for ln in simlib.journal_lines(u, marks.get(u["name"]))
               if simlib.line_hit(ln, rx)]
        stuck_s = 0.0
        run_msg, run_start, run_end = None, 0, 0
        for ln in seq:
            msg = f"{ln[4]} {ln[5]}"
            if msg != run_msg:
                stuck_s = max(stuck_s, (run_end - run_start) / 1000.0)
                run_msg, run_start = msg, ln[0]
            run_end = ln[0]
        stuck_s = max(stuck_s, (run_end - run_start) / 1000.0)
        good = stuck_s <= float(a.get("max_s", 60))
        ok &= good
        worst = max(worst, stuck_s)
        bits.append(f"{u['name']}: longest identical {stuck_s:.0f}s "
                    f"(max {a.get('max_s', 60):g})")
    return _r(name, "PASS" if ok else "FAIL",
              round(worst, 1), "; ".join(bits))


def _roster_value(name, a, ctx):
    """A field of the unit's last 1092 roster frame via the bot
    (tq, bat, st, ...)."""
    bot = ctx.get("bot")
    if not bot:
        return _r(name, "SKIP", None, "no operator session")
    ok, bits = True, []
    for u in _units(a, ctx):
        v = bot.roster_of(u["party_id"]).get(a["field"])
        if v is None:
            return _r(name, "SKIP", None,
                      f"{u['name']}: no roster field {a['field']!r} yet")
        good, txt = _cmp(a, v)
        ok &= good
        bits.append(f"{u['name']}: {a['field']}={v} (want {txt})")
    return _r(name, "PASS" if ok else "FAIL", None, "; ".join(bits))


def _telemetry_value(name, a, ctx):
    tele = ctx.get("telemetry")
    if not tele:
        return _r(name, "SKIP", None, "no telemetry recorder")
    pick = a.get("pick", "latest")
    ok, bits, vals = True, [], []
    for u in _units(a, ctx):
        lbl = a["field"]
        if pick == "latest":
            old = _stale(tele, u["name"], a)
            if old:
                return _r(name, "SKIP", None, old)
            v = tele.latest(u["name"]).get(a["field"])
        elif pick in ("min", "max"):
            # a sag/spike that recovers before the assert window (SITL
            # resets consumed_mah on disarm) still lives in the series
            series = [r.get(a["field"]) for r in tele.rows(u["name"])]
            series = [x for x in series if isinstance(x, (int, float))
                      and not isinstance(x, bool)]
            v = (min if pick == "min" else max)(series) if series else None
            lbl = f"{pick}({a['field']})"
        else:
            return _r(name, "FAIL", None,
                      f"bad pick {pick!r} (latest|min|max)")
        if v is None:
            bits.append(f"{u['name']}: no {lbl} - SKIP")
            return _r(name, "SKIP", None, "; ".join(bits))
        vals.append(v)
        good, txt = _cmp(a, v)
        ok &= good
        bits.append(f"{u['name']}: {lbl} {txt}")
    return _r(name, "PASS" if ok else "FAIL",
              vals[0] if len(vals) == 1 else None, "; ".join(bits))


TELEMETRY_MAX_AGE_MS = 10000


def _stale(tele, unit_name, a):
    """Detail string when the unit's newest telemetry row is too old to
    describe its state now (a dead stream keeps its last row forever)."""
    age = tele.age_ms(unit_name) if hasattr(tele, "age_ms") else None
    lim = a.get("max_age_ms", TELEMETRY_MAX_AGE_MS)
    if age is not None and age > lim:
        return f"{unit_name}: newest telemetry is {age / 1000:.0f} s old"
    return None


def _fc_mode(name, a, ctx):
    tele = ctx.get("telemetry")
    if not tele:
        return _r(name, "SKIP", None, "no telemetry recorder")
    ok, bits = True, []
    for u in _units(a, ctx):
        old = _stale(tele, u["name"], a)
        if old:
            return _r(name, "SKIP", None, old)
        cur = tele.latest(u["name"]).get("mode")
        if cur is None:
            return _r(name, "SKIP", None, f"{u['name']}: no FC telemetry")
        ok &= str(cur).upper() == str(a["mode"]).upper()
        bits.append(f"{u['name']}: mode={cur} (want {a['mode']})")
    return _r(name, "PASS" if ok else "FAIL", None, "; ".join(bits))


def _render_latency(name, a, ctx):
    """P5-15: p95 pose->V4L2 latency over the render chain. The render
    farm's frame_meta events (src render, t_ms = pose sample) join the
    probe's frame_seen rows (src render_probe, t_ms = device readback)
    on (unit, frame_no); latency = seen - pose. `unit:` narrows to one
    camera unit; p95_max_ms bounds the 95th percentile."""
    if a.get("p95_max_ms") is None:
        return _r(name, "FAIL", None, "needs p95_max_ms")
    rows, src = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return _r(name, "SKIP", None,
                  "no truth source (truth_file / run truth.jsonl)")
    metas = {}
    for r in rows:
        if r.get("type") == "event" and r.get("src") == "render" \
                and r.get("kind") == "frame_meta":
            if a.get("unit") is not None and r.get("unit") != a["unit"]:
                continue
            metas[(r.get("unit"), r.get("frame_no"))] = _row_t(r)
    lat = []
    for r in rows:
        if r.get("type") == "event" and r.get("src") == "render_probe" \
                and r.get("kind") == "frame_seen":
            if a.get("unit") is not None and r.get("unit") != a["unit"]:
                continue
            t0 = metas.get((r.get("unit"), r.get("frame_no")))
            if t0 is not None:
                lat.append(_row_t(r) - t0)
    if not lat:
        return _r(name, "SKIP", None,
                  f"no joined frame_meta/frame_seen pairs in "
                  f"{os.path.basename(src)}")
    lat.sort()
    p95 = lat[min(len(lat) - 1, int(0.95 * (len(lat) - 1)))]
    good = p95 <= float(a["p95_max_ms"])
    return _r(name, "PASS" if good else "FAIL", p95,
              f"p95 {p95} ms over {len(lat)} joined frames "
              f"(max {a['p95_max_ms']})")


def _visible_px(name, a, ctx):
    """P5-15 ID-pass visibility of a truth entity: how many ID-pass
    frames painted it >= min_px (default 1). Bounds: `min_frames` /
    `max_frames` on that count, and `max_px` on the biggest single-frame
    paint - `max_px: 0` is the occluded control ("never visible in any
    pass"). `unit:` narrows to one camera's passes."""
    rows, src = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return _r(name, "SKIP", None,
                  "no truth source (truth_file / run truth.jsonl)")
    tids = _recall_targets(rows, a["truth"])
    if not tids:
        return _r(name, "FAIL", None,
                  f"no truth entities match {a['truth']!r}")
    min_px = float(a.get("min_px", 1))
    stats = {}
    any_rows = False
    for r in rows:
        if r.get("type") != "event" or r.get("src") != "render_vis" \
                or r.get("kind") != "visibility":
            continue
        if a.get("unit") is not None and r.get("unit") != a["unit"]:
            continue
        any_rows = True
        tid = r.get("tid")
        if tid not in tids:
            continue
        s = stats.setdefault(tid, {"frames": 0, "max_px": 0})
        px = float(r.get("px") or 0)
        s["max_px"] = max(s["max_px"], px)
        if px >= min_px:
            s["frames"] += 1
    if not any_rows:
        return _r(name, "SKIP", None,
                  "no render_vis rows - the ID pass ran nowhere")
    bits, good = [], True
    for tid in tids:
        s = stats.get(tid) or {"frames": 0, "max_px": 0}
        if a.get("min_frames") is not None \
                and s["frames"] < int(a["min_frames"]):
            good = False
        if a.get("max_frames") is not None \
                and s["frames"] > int(a["max_frames"]):
            good = False
        if a.get("max_px") is not None \
                and s["max_px"] > float(a["max_px"]):
            good = False
        bits.append(f"{tid}: {s['frames']} frame(s) >= {min_px:g}px, "
                    f"peak {s['max_px']:g}px")
    return _r(name, "PASS" if good else "FAIL",
              max((stats.get(t) or {}).get("frames", 0) for t in tids),
              "; ".join(bits))


def _iou(b1, b2):
    """IoU of two normalized {x,y,w,h} boxes (top-left origin)."""
    x0 = max(b1["x"], b2["x"]); y0 = max(b1["y"], b2["y"])
    x1 = min(b1["x"] + b1["w"], b2["x"] + b2["w"])
    y1 = min(b1["y"] + b1["h"], b2["y"] + b2["h"])
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    if inter <= 0:
        return 0.0
    a1 = b1["w"] * b1["h"]; a2 = b2["w"] * b2["h"]
    return inter / (a1 + a2 - inter) if (a1 + a2 - inter) > 0 else 0.0


def _detection_precision_render(name, a, ctx):
    """P5-15: of the 1078 boxes the real chain reported (bot world-model
    targets carrying `box`), the fraction matching an ID-pass box of the
    same unit's temporally-nearest visibility frame at IoU >= `iou`
    (default 0.3). `unit:` narrows to one camera unit's reports."""
    iou_min = float(a.get("iou", 0.3))
    rows, src = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return _r(name, "SKIP", None,
                  "no truth source (truth_file / run truth.jsonl)")
    bot = ctx.get("bot")
    if not bot:
        return _r(name, "SKIP", None, "no operator session")
    # visibility boxes per unit: frame_no -> [(tid, box)]
    vis = {}
    for r in rows:
        if r.get("type") == "event" and r.get("src") == "render_vis" \
                and r.get("kind") == "visibility" and r.get("box"):
            vis.setdefault(r.get("unit"), []).append(r)
    if not vis:
        return _r(name, "SKIP", None,
                  "no render_vis box rows - the ID pass ran nowhere")
    targets = [tg for tg in _bot_targets(ctx).values()
               if isinstance(tg.get("box"), dict)]
    if a.get("unit") is not None:
        targets = [tg for tg in targets if tg.get("src") == a["unit"]]
    if not targets:
        return _r(name, "SKIP", None,
                  "no bot targets carry a 1078 box")
    matched = 0
    for tg in targets:
        vrows = vis.get(tg.get("src")) or []
        if not vrows:
            continue
        # nearest-ID-pass rows for that unit (same frame_no group)
        t = tg.get("t_ms", 0)
        best_frame = min(vrows, key=lambda r: abs(_row_t(r) - t))
        group = [r["box"] for r in vrows
                 if r.get("frame_no") == best_frame.get("frame_no")]
        if group and max(_iou(tg["box"], b) for b in group) >= iou_min:
            matched += 1
    prec = matched / len(targets)
    lo = a.get("min")
    good = True if lo is None else prec >= float(lo)
    return _r(name, "PASS" if good else "FAIL", round(prec, 3),
              f"{matched}/{len(targets)} 1078 boxes matched the ID pass "
              f"(iou {iou_min:g}, min {lo})")


DISPATCH = {
    "journal_count": _journal_count,
    "module_log_count": _module_log_count,
    "no_journal": _no_journal,
    "journal_order": _journal_order,
    "journal_time_delta": _journal_delta,
    "clock_spread": _clock_spread,
    "mission_current": _mission_current,
    "distance_to": _distance_to,
    "no_sep_violation": _no_sep_violation,
    "coverage_complete": _coverage_complete,
    "pod_mean": _pod_mean,
    "pod_at": _pod_at,
    "detection_precision": _detection_precision,
    "detection_precision_render": _detection_precision_render,
    "detection_recall": _detection_recall,
    "render_latency": _render_latency,
    "visible_px": _visible_px,
    "time_to_detect": _time_to_detect,
    "stub_deterministic": _stub_deterministic,
    "signals_in_range": _signals_in_range,
    "journal_consistent": _journal_consistent,
    "final_state": _final_state,
    "fc_items_unchanged_without_cause": _fc_items_unchanged,
    "mission_items": _mission_items,
    "mission_pull": _mission_pull,
    "no_fc_command_to_static": _no_fc_command_to_static,
    "max_latency": lambda n, a, c: _journal_delta(n, a, c, ms=True),
    "proc_alive": lambda n, a, c: _proc_state(n, a, c, True),
    "proc_dead": lambda n, a, c: _proc_state(n, a, c, False),
    "operator_log_count": _operator_log_count,
    "link_offline": _link_offline,
    "no_unexpected_restart": _no_unexpected_restart,
    "rss_growth": _rss_growth,
    "fd_growth": _fd_growth,
    "journal_size": _journal_size,
    "recovers_after_fault": _recovers_after_fault,
    "no_stuck_state": _no_stuck_state,
    "roster_value": _roster_value,
    "telemetry_value": _telemetry_value,
    "fc_mode": _fc_mode,
    "status_field": _status_field,
}

# the SL-0011 run-wide invariants: they read run evidence (samples.jsonl,
# links.jsonl, journals) rather than a scripted step, so they are the
# only assertions meaningful on an unscripted chaos/soak run - a
# scenario with a chaos: block must carry at least one (validate enforces)
INVARIANT_ASSERTS = frozenset({
    "no_unexpected_restart", "rss_growth", "fd_growth", "journal_size",
    "recovers_after_fault", "no_stuck_state"})


def check_assertion(a, ctx):
    """One {name: args} assertion -> the structured result dict."""
    if not isinstance(a, dict) or len(a) != 1:
        return _r("?", "FAIL", None,
                  f"an assertion is a single {{name: args}} mapping, got {a!r}")
    (name, arg), = a.items()
    arg = arg or {}
    if not isinstance(arg, dict):
        return _r(name, "FAIL", None,
                  f"{name}: arguments must be a mapping, got {arg!r}")
    fn = DISPATCH.get(name)
    if fn is None:
        return _r(name, "FAIL", None, f"unknown assertion {name!r}")
    if (name in REALTIME_SENSITIVE or arg.get("realtime")) \
            and ctx.get("realtime_required") \
            and not ctx.get("realtime_ok"):
        return _r(name, "SKIP", None,
                  "scenario needs realtime but SITL did not hold "
                  f"speed-up 1 ({ctx.get('realtime_note', '?')})")
    try:
        if name in UNIT_ASSERTS and str(arg.get("unit")) != "server" \
                and not _units(arg, ctx):
            return _r(name, "FAIL", None, "no units resolved - an "
                      "assertion over zero units must not pass")
        return fn(name, arg, ctx)
    except SystemExit as e:
        return _r(name, "FAIL", None, f"resolution error: {e}")
    except Exception as e:
        return _r(name, "FAIL", None, f"{type(e).__name__}: {e}")
