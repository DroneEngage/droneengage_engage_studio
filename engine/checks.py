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

  - journals:            proc_alive/proc_dead, operator_log_count,
                         no_fc_command_to_static
  - telemetry.jsonl:     distance_to, no_sep_violation, fc_mode,
                         telemetry_value (the SITL "tlog")
  - truth JSONL:         distance_to, coverage_complete - rows
                         {tid|id|target, t_ms|t|ms, lat, lng, alt|alt_m}
                         and coverage rows {type:"coverage", tid, covered};
                         P5-08 adds detection rows {type:"detection",
                         src, tid|"clutter", cls, t_ms, sent} read by
                         detection_precision and signals_in_range, and
                         distance_to's detected_cls: (bot-seen pos)
  - stub determinism:    stub_deterministic - two same-seed --selftest
                         replays must produce byte-identical stub logs
  - 1087 status (bot):   final_state
  - MAVLink mission:     fc_items_unchanged_without_cause
"""

import hashlib
import json
import math
import os
import re

import simlib
import ctl

# assertions that are only meaningful when SITL really runs at ~1x; when
# the scenario asks realtime: true and the measured speed-up is off,
# these report SKIP instead of a misleading number
REALTIME_SENSITIVE = {"max_latency", "journal_time_delta"}

CAUSE_RE = r"MISSION_(WRITE|UPLOAD|ACK)|fc mission|mission.*(upload|download)"
FC_LINE_RE = re.compile(
    r"MISSION_|MAVLink|FC_(?!items)|fc_cmd|guided_to|set_mode|arm", re.I)


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
        return rows, p
    return None, None


def _row_tid(r):
    return r.get("tid") or r.get("id") or r.get("target")


def _row_t(r):
    return r.get("t_ms") or r.get("t") or r.get("ms") or 0


def _truth_at(rows, tid, t_ms):
    """Truth row for tid nearest in time to t_ms."""
    best = None
    for r in rows:
        if _row_tid(r) != tid or r.get("lat") is None:
            continue
        d = abs(_row_t(r) - t_ms)
        if best is None or d < best[0]:
            best = (d, r)
    return best[1] if best else None


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

def mission_signature(u, timeout=15):
    """sha256 over the FC's mission items (seq,cmd,x,y,z), or None when
    the download fails. The baseline is taken at scenario start;
    fc_items_unchanged_without_cause compares at assert time."""
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
        return hashlib.sha256(json.dumps(items).encode()).hexdigest()
    except Exception:
        return None
    finally:
        m.close()


def capture_baseline(ctx):
    """Snapshot mission signatures + proc pids before the timeline runs."""
    base = {"missions": {}}
    for u in ctx["manifest"]["units"]:
        if u["fc"]:
            base["missions"][u["name"]] = mission_signature(u)
    return base


# ---------------------------------------------------------------- checks

def _cmp(a, v, detail=""):
    """{eq|equals, at_least, at_most, lt, gt, le, ge} on a number."""
    if "eq" in a or "equals" in a:
        want = a.get("eq", a.get("equals"))
        return v == want, f"{v} == {want}"
    if "at_least" in a:
        return v >= a["at_least"], f"{v} >= {a['at_least']}"
    if "at_most" in a:
        return v <= a["at_most"], f"{v} <= {a['at_most']}"
    for k, op in (("lt", lambda x, w: x < w), ("le", lambda x, w: x <= w),
                  ("gt", lambda x, w: x > w), ("ge", lambda x, w: x >= w)):
        if k in a:
            return op(v, a[k]), f"{v} {k} {a[k]}"
    return v >= 1, f"{v} >= 1"


def _journal_count(name, a, ctx):
    marks = ctx["marks"]
    ok, bits, vals = True, [], []
    for u in _units(a, ctx):
        n = len(simlib.journal_match(u, a["re"], marks.get(u["name"])))
        vals.append(n)
        good, txt = _cmp(a, n)
        ok &= good
        bits.append(f"{u['name']}: {txt}")
    return _r(name, "PASS" if ok else "FAIL",
              sum(vals) / len(vals) if vals else 0, "; ".join(bits))


def _journal_order(name, a, ctx):
    r1, r2 = re.compile(a["first"]), re.compile(a["then"])
    ok, bits = True, []
    marks = ctx["marks"]
    for u in _units(a, ctx):
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
    u1 = ctl.resolve(str(a.get("u1", a.get("v1"))), ctx["manifest"])[0]
    u2 = ctl.resolve(str(a.get("u2", a.get("v2"))), ctx["manifest"])[0]
    h1 = simlib.journal_match(u1, a["re1"], marks.get(u1["name"]))
    h2 = simlib.journal_match(u2, a["re2"], marks.get(u2["name"]))
    if not h1 or not h2:
        return _r(name, "FAIL", None,
                  f"missing: {u1['name']}={len(h1)} {u2['name']}={len(h2)}")
    t1 = h1[0][0]
    if u1["name"] == u2["name"]:
        h2 = [h for h in h2 if h[0] >= t1]
        if not h2:
            return _r(name, "FAIL", None,
                      f"no {a['re2']!r} after {a['re1']!r} on {u1['name']}")
    dt = (h2[0][0] - t1) if ms else abs(t1 - h2[0][0]) / 1000.0
    if ms:
        lim = a.get("max_ms")
        good = lim is None or dt <= lim
        return _r(name, "PASS" if good else "FAIL", dt,
                  f"{u1['name']}:{a['re1']!r} -> {u2['name']}:{a['re2']!r}"
                  f" = {dt:.0f} ms (max {lim})")
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
    hits = simlib.journal_match(u, a["re"], marks.get(u["name"]))
    if not hits:
        return _r(name, "FAIL", None,
                  f"{u['name']}: no journal line matching {a['re']!r}")
    t = hits[0][0]
    rows, src = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return _r(name, "SKIP", None,
                  "no truth source (truth_file / run truth.jsonl - "
                  "P5-08 pending)")
    tids = a["truth"] if isinstance(a["truth"], list) else [a["truth"]]
    t0 = t
    tr = None
    for tid in tids:
        c = _truth_at(rows, tid, t)
        if c is not None:
            tr = c
            break
    pos = _unit_pos_at(ctx, u["name"], t)
    pos_src = "telemetry"
    if a.get("detected_cls"):
        # P5-08: measure the world-model detection the operator bot saw
        # (the detect.detected envelope's pos), not the unit's own pose -
        # "the reported detection is within max_m of the truth"
        bot = ctx.get("bot")
        cands = [tg for tg in (bot.targets() if bot else {}).values()
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
         for c in [_truth_at(rows, tid, t)] if c is not None),
        key=lambda p: _haversine(pos["lat"], pos["lng"],
                                 p[0]["lat"], p[0]["lng"] or 0),
        default=(tr, tids[0]))
    d = _haversine(pos["lat"], pos["lng"], tr["lat"],
                   tr["lng"] or tr.get("lon"))
    lo, hi = a.get("min_m"), a.get("max_m")
    good = (lo is None or d >= lo) and (hi is None or d <= hi)
    return _r(name, "PASS" if good else "FAIL", round(d, 1),
              f"{u['name']} {pos_src} -> {tid_hit} = {d:.1f} m "
              f"(min {lo}, max {hi}, src {os.path.basename(src)})")


def _signals_in_range(name, a, ctx):
    """SDR-style gate: every journal line matching `re` must be stamped
    while the truth entity was within `max_m` of the unit (the stub only
    reports inside the modelled range - this proves it). Also requires
    `min_count` (default 1) hits so "never out of range" can't pass on
    silence. Static units read their fixed position; FC units their
    telemetry."""
    marks = ctx["marks"]
    ok, bits, total = True, [], 0
    rows, src = _truth_rows(ctx, a.get("truth_file"))
    if rows is None:
        return _r(name, "SKIP", None, "no truth source")
    for u in _units(a, ctx):
        hits = simlib.journal_match(u, a["re"], marks.get(u["name"]))
        total += len(hits)
        p0 = u.get("position") or {}
        bad = []
        for ln in hits:
            tr = _truth_at(rows, a["truth"], ln[0])
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
            d = _haversine(lat, lng, tr["lat"],
                           tr["lng"] or tr.get("lon"))
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
    return _r(name, "PASS" if ok else "FAIL", total, "; ".join(bits))


def _no_sep_violation(name, a, ctx):
    tele = ctx.get("telemetry")
    if not tele:
        return _r(name, "SKIP", None, "no telemetry recorder")
    units = [u["name"] for u in ctl.with_fc(_units(a, ctx))]
    if len(units) < 2:
        return _r(name, "SKIP", None,
                  "fewer than two FC units - nothing to separate")
    # h_m (or min_m): required horizontal separation; v_m: a pair too
    # close horizontally still passes when it keeps v_m vertical gap
    h_min = a.get("h_m", a.get("min_m", 10))
    v_min = a.get("v_m")
    worst = (None, None)
    for i, n1 in enumerate(units):
        rows1 = tele.rows(n1)
        if not rows1:
            continue
        for n2 in units[i + 1:]:
            rows2 = tele.rows(n2)
            for r1 in rows1:
                if r1.get("lat") is None:
                    continue
                r2 = min(rows2, key=lambda r:
                         abs(r["t_ms"] - r1["t_ms"]), default=None)
                if r2 is None or r2.get("lat") is None \
                        or abs(r2["t_ms"] - r1["t_ms"]) > 2000:
                    continue
                d = _haversine(r1["lat"], r1["lng"], r2["lat"], r2["lng"])
                if v_min is not None \
                        and r1.get("alt_m") is not None \
                        and r2.get("alt_m") is not None \
                        and abs(r1["alt_m"] - r2["alt_m"]) >= v_min:
                    continue          # vertically separated - no violation
                if worst[0] is None or d < worst[0]:
                    worst = (d, f"{n1}<->{n2} @t={r1['t_ms']}")
    if worst[0] is None:
        return _r(name, "SKIP", None, "no overlapping telemetry")
    good = worst[0] >= h_min
    return _r(name, "PASS" if good else "FAIL", round(worst[0], 1),
              f"min pairwise distance {worst[0]:.1f} m ({worst[1]}), "
              f"h_m={h_min} v_m={v_min}")


def _coverage_complete(name, a, ctx):
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
        elif a.get("mis") is not None:
            good &= mis == a["mis"]
        elif a.get("re"):
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
    ok, bits = True, []
    for u in _units(a, ctx):
        if not u["fc"]:
            bits.append(f"{u['name']}: no fc - skipped")
            continue
        now = mission_signature(u)
        was = base.get(u["name"])
        if now is None:
            ok = False
            bits.append(f"{u['name']}: mission unreadable now")
            continue
        if now == was:
            bits.append(f"{u['name']}: unchanged")
            continue
        caused = any(simlib.line_hit(ln, cause)
                     for ln in simlib.journal_lines(
                         u, ctx["marks"].get(u["name"])))
        ok &= caused
        bits.append(f"{u['name']}: CHANGED "
                    f"({'caused' if caused else 'NO CAUSE JOURNALED'})")
    return _r(name, "PASS" if ok else "FAIL", None, "; ".join(bits))


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
        alive = pid is not None and os.path.isdir(f"/proc/{pid}")
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
    ok, bits, total = True, [], 0
    for u in _units(a, ctx):
        ev = events.get(u["party_id"], [])
        n = 0
        t_off = None
        for t, online in ev + [(float("inf"), True)]:
            if not online and t_off is None:
                t_off = t
            elif online and t_off is not None:
                dur = (t - t_off) / 1000.0 if t != float("inf") else min_s
                if dur >= min_s:
                    n += 1
                t_off = None
        total += n
        good, txt = _cmp(a, n)
        ok &= good
        bits.append(f"{u['name']}: {n} offline interval(s) "
                    f"(min_s {min_s:g}; want {txt.split(' ', 1)[-1]})")
    return _r(name, "PASS" if ok else "FAIL", total, "; ".join(bits))


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
    ok, bits, vals = True, [], []
    for u in _units(a, ctx):
        row = tele.latest(u["name"])
        v = row.get(a["field"])
        if v is None:
            bits.append(f"{u['name']}: no {a['field']} - SKIP")
            return _r(name, "SKIP", None, "; ".join(bits))
        vals.append(v)
        good, txt = _cmp(a, v)
        ok &= good
        bits.append(f"{u['name']}: {a['field']} {txt}")
    return _r(name, "PASS" if ok else "FAIL",
              vals[0] if len(vals) == 1 else None, "; ".join(bits))


def _fc_mode(name, a, ctx):
    tele = ctx.get("telemetry")
    if not tele:
        return _r(name, "SKIP", None, "no telemetry recorder")
    ok, bits = True, []
    for u in _units(a, ctx):
        cur = tele.latest(u["name"]).get("mode")
        if cur is None:
            return _r(name, "SKIP", None, f"{u['name']}: no FC telemetry")
        ok &= cur == a["mode"]
        bits.append(f"{u['name']}: mode={cur} (want {a['mode']})")
    return _r(name, "PASS" if ok else "FAIL", None, "; ".join(bits))


DISPATCH = {
    "journal_count": _journal_count,
    "journal_order": _journal_order,
    "journal_time_delta": _journal_delta,
    "clock_spread": _clock_spread,
    "mission_current": _mission_current,
    "distance_to": _distance_to,
    "no_sep_violation": _no_sep_violation,
    "coverage_complete": _coverage_complete,
    "detection_precision": _detection_precision,
    "stub_deterministic": _stub_deterministic,
    "signals_in_range": _signals_in_range,
    "final_state": _final_state,
    "fc_items_unchanged_without_cause": _fc_items_unchanged,
    "no_fc_command_to_static": _no_fc_command_to_static,
    "max_latency": lambda n, a, c: _journal_delta(n, a, c, ms=True),
    "proc_alive": lambda n, a, c: _proc_state(n, a, c, True),
    "proc_dead": lambda n, a, c: _proc_state(n, a, c, False),
    "operator_log_count": _operator_log_count,
    "link_offline": _link_offline,
    "roster_value": _roster_value,
    "telemetry_value": _telemetry_value,
    "fc_mode": _fc_mode,
}


def check_assertion(a, ctx):
    """One {name: args} assertion -> the structured result dict."""
    (name, arg), = a.items()
    arg = arg or {}
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
        return fn(name, arg, ctx)
    except SystemExit as e:
        return _r(name, "FAIL", None, f"resolution error: {e}")
    except Exception as e:
        return _r(name, "FAIL", None, f"{type(e).__name__}: {e}")
