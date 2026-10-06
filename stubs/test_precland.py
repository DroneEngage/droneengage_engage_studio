#!/usr/bin/env python3
"""test_precland.py - the PL-08 stub + event-model checks (no SITL, no
de_comm - stubs/model_import.py shims the databus ids):

    python3 stubs/test_precland.py

- golden contract: replay the real module's
  src/precland/test/golden/precland_events.jsonl through
  stubs/precland_events.py and require identical facts (path via
  DE_PRECLAND_DIR; a missing checkout/file SKIPs visibly).
- advert: stubs/precland_advert.json equals de_precland
  --print-advert (same SKIP rule).
- model: pad centre in/out/at the FOV edge, tag count vs altitude,
  occlusion windows, stale -> searching, enable/disable ordering,
  legacy 6536 + capability + event command paths, and a byte-identical
  same-seed selftest replay.
"""
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "engine"))

import simlib                      # noqa: E402
import model_import               # noqa: E402

mods = model_import.import_stubs()
stub_precland = mods["stub_precland"]
import precland_events as pe      # noqa: E402
import geo_projection as gp       # noqa: E402

PAD_FIELD = os.path.join(ROOT, "worlds", "pad_field.yaml")
PAD_NONE = os.path.join(ROOT, "worlds", "pad_none.yaml")
POSE_DESCENT = os.path.join(ROOT, "worlds", "pose_pad_descent.jsonl")
GOLDEN = os.path.join(
    simlib.repo_dir("DE_PRECLAND_DIR", "droneengage_precision_landing"),
    "src", "precland", "test", "golden", "precland_events.jsonl")
ADVERT_BIN = os.path.join(
    simlib.repo_dir("DE_PRECLAND_DIR", "droneengage_precision_landing"),
    "bin", "de_precland")


def make(world_path=PAD_FIELD, d=None, extra=None):
    """A StubPrecland on a generated-looking config under d/run/sim/u1/."""
    if d is None:
        d = tempfile.mkdtemp()
    mdir = os.path.join(d, "run", "sim", "u1", "stub_precland")
    os.makedirs(mdir, exist_ok=True)
    cfg = {"stub": {"world": world_path, "seed": 7, "unit": "u1",
                    "sysid": 1, "pose_port": 0},
           "s2s_udp_target_port": "1"}
    cfg.update(extra or {})
    p = os.path.join(mdir, "stub_precland.config.json")
    with open(p, "w") as f:
        json.dump(cfg, f)
    s = stub_precland.StubPrecland(["-c", p])
    s._vt_ms = 0          # drive the virtual clock from the start
    return s


def set_pose(stub, lat, lng, rel_alt, t_ms=0):
    f = stub.pose_feed
    f._pos = {"lat": lat, "lng": lng, "alt_m": 584 + rel_alt,
              "rel_alt_m": rel_alt, "t_ms": t_ms}
    f._att = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "t_ms": t_ms}


def over_pad(stub, north_m=0.0, east_m=0.0, alt_m=10.0, t_ms=0):
    pad = stub.world.pads()[0]
    lat, lng, _alt = pad.pos_at(0.0, stub.world)
    lat += north_m / 111320.0
    lng += east_m / (111320.0 * math.cos(math.radians(lat)))
    set_pose(stub, lat, lng, alt_m, t_ms)


def run_ticks(stub, t0_s, secs):
    """Drive tick() at the 20 Hz loop cadence from t0_s for secs."""
    for frame in range(int(t0_s * 20), int((t0_s + secs) * 20)):
        stub._vt_ms = int(frame * 50)
        stub.tick(frame * 0.05, frame)


class GoldenReplay(unittest.TestCase):
    """src/precland/test/golden/precland_events.jsonl row-for-row -
    the same replay the C++ test_precland_events runs, through the
    Python port (PL-08 contract: identical output)."""

    def test_golden_replay(self):
        if not os.path.isfile(GOLDEN):
            self.skipTest(f"SKIP: no golden file {GOLDEN} "
                          "(DE_PRECLAND_DIR unset?)")
        model = pe.PreclandEventModel(pe.EventsConfig())
        line_no = 0
        for line in open(GOLDEN):
            if not line.strip():
                continue
            line_no += 1
            row = json.loads(line)
            inp, out = row["in"], row["out"]
            op = inp.get("op", "")
            if op == "payload":
                self.assertEqual(pe.pl_event_payload(inp), out,
                                 f"line {line_no}: payload pick differs")
                continue
            if op == "set_target":
                target_num, why = pe.pl_validate_set_target(
                    inp.get("payload", {}), inp.get("targets"))
                got = {"ok": target_num >= 0}
                if target_num >= 0:
                    got["target"] = target_num
                else:
                    got["why"] = why
                self.assertEqual(got, out,
                                 f"line {line_no}: set_target differs")
                continue
            if op == "transition":
                facts = model.on_transition(
                    inp.get("prev_state", 0), inp.get("new_state", 0),
                    inp.get("reason", 0), inp.get("now_us", 0),
                    range_m=inp.get("range_m", 0.0),
                    rmse_px=inp.get("rmse_px", 0.0),
                    tags=inp.get("tags", 0),
                    target=inp.get("target", 0),
                    fault=inp.get("fault", ""))
            elif op == "tick":
                facts = model.on_tick(
                    inp.get("state", 0), inp.get("now_us", 0),
                    inp.get("range_m", 0.0), inp.get("rmse_px", 0.0),
                    inp.get("target", 0))
            elif op == "enabled":
                # module order: the enabled edge first, then the setState
                # transition the flip causes
                facts = model.on_enabled_changed(
                    inp.get("was", False), inp.get("now", True),
                    pe.cause_from_name(inp.get("cause", "loop")),
                    inp.get("now_us", 0))
                if "prev_state" in inp:
                    facts += model.on_transition(
                        inp.get("prev_state", 0), inp.get("new_state", 0),
                        inp.get("reason", 0), inp.get("now_us", 0),
                        range_m=inp.get("range_m", 0.0),
                        rmse_px=inp.get("rmse_px", 0.0),
                        tags=inp.get("tags", 0),
                        target=inp.get("target", 0),
                        fault=inp.get("fault", ""))
            elif op == "target":
                facts = model.on_target_changed(
                    inp.get("was", 0), inp.get("now", 0),
                    pe.cause_from_name(inp.get("cause", "loop")))
            else:
                self.fail(f"golden line {line_no}: unknown op {op!r}")
                continue
            self.assertEqual(pe.facts_jsonable(facts), out,
                             f"golden line {line_no} ({op}) differs")
        self.assertEqual(line_no, 42,
                         f"golden replay covered {line_no} rows, not 42")


class Advert(unittest.TestCase):
    def test_stored_advert_matches_module(self):
        stored = json.load(open(os.path.join(
            HERE, "precland_advert.json")))
        if not os.path.isfile(ADVERT_BIN):
            self.skipTest(f"SKIP: no de_precland binary {ADVERT_BIN}")
        live = subprocess.run([ADVERT_BIN, "--print-advert"],
                              capture_output=True, text=True, timeout=10)
        self.assertEqual(live.returncode, 0, live.stderr)
        self.assertEqual(json.loads(live.stdout), stored,
                         "stubs/precland_advert.json drifted from "
                         "de_precland --print-advert")

    def test_advert_shape(self):
        a = json.load(open(os.path.join(HERE, "precland_advert.json")))
        self.assertEqual(a["schema"], "de.cap/1")
        self.assertEqual(a["ns"], "precision_landing")
        self.assertEqual(a["module"], "droneengage_precision_landing")
        self.assertEqual(sorted(a["actions"]),
                         ["disable", "enable", "set_target"])
        self.assertEqual(sorted(a["events"]),
                         ["degraded", "disabled", "enabled", "faulted",
                          "locked", "lost", "stabilized",
                          "target_changed"])
        self.assertEqual(sorted(a["state"]),
                         ["range_m", "reason", "status"])


class Model(unittest.TestCase):
    """stub_precland model behaviour - all in-process on pad_field."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def _events(self, stub, d=None):
        log = os.path.join(d or self.d, "run", "sim", "stubs", "u1.log")
        if not os.path.isfile(log):
            return []
        return [json.loads(ln.split(" ", 2)[2]) for ln in open(log)
                if " EVENT_OUT " in ln]

    def test_centre_over_pad_locks(self):
        s = make(d=self.d)
        s.enable(pe.CAUSE_LEGACY)
        over_pad(s, alt_m=10.0)
        # start past t=2 s so the PL-02 lock/lost pair hysteresis does
        # not suppress the first "locked" (virtual clock starts at 0;
        # live monotonic_us is always past it)
        run_ticks(s, 2.5, 2.0)
        self.assertEqual(s._state, pe.PRECLAND_STATUS_LOCKED)
        verbs = [e["d"] for e in self._events(s)]
        self.assertIn("precision_landing.enabled", verbs)
        self.assertIn("precision_landing.locked", verbs)
        # enabled precedes locked (PL-02 ordering)
        self.assertLess(verbs.index("precision_landing.enabled"),
                        verbs.index("precision_landing.locked"))

    def test_no_pad_world_searches(self):
        s = make(PAD_NONE, self.d)
        s.enable(pe.CAUSE_LEGACY)
        set_pose(s, -35.363262, 149.165237, 10.0)
        run_ticks(s, 0.0, 2.0)
        self.assertEqual(s._state, pe.PRECLAND_STATUS_SEARCHING)

    def test_outside_fov_searches(self):
        s = make(d=self.d)
        s.enable(pe.CAUSE_LEGACY)
        # 60 deg hfov at 10 m: half-frame ~5.8 m; 30 m off-axis is out
        over_pad(s, north_m=30.0, alt_m=10.0)
        run_ticks(s, 0.0, 2.0)
        self.assertEqual(s._state, pe.PRECLAND_STATUS_SEARCHING)

    def test_tag_count_vs_altitude(self):
        s = make(d=self.d)
        over_pad(s, alt_m=10.0)
        vis = s._frame_geometry(s.world.pads()[0],
                                s.pose_feed.pose(), 0.0)
        self.assertIsNotNone(vis)
        self.assertEqual(vis[4], 2)      # 0.68 + 0.17 resolve at 10 m
        over_pad(s, alt_m=1.8)
        vis = s._frame_geometry(s.world.pads()[0],
                                s.pose_feed.pose(), 0.0)
        self.assertEqual(vis[4], 3)      # the 0.0425 tag resolves too
        over_pad(s, alt_m=25.0)
        vis = s._frame_geometry(s.world.pads()[0],
                                s.pose_feed.pose(), 0.0)
        self.assertEqual(vis[4], 1)      # only the largest tag left

    def test_occlusion_window_blinds(self):
        s = make(d=self.d)
        s.enable(pe.CAUSE_LEGACY)
        pad = s.world.pads()[0]
        pad.occlude = [[5.0, 10.0]]
        over_pad(s, alt_m=10.0)
        run_ticks(s, 2.5, 2.0)
        self.assertEqual(s._state, pe.PRECLAND_STATUS_LOCKED)
        run_ticks(s, 5.0, 0.5)         # first occluded frame drops lock
        self.assertEqual(s._state, pe.PRECLAND_STATUS_SEARCHING)
        verbs = [e["d"] for e in self._events(s)]
        self.assertIn("precision_landing.lost", verbs)
        run_ticks(s, 10.0, 3.0)        # visible again past the window
        self.assertEqual(s._state, pe.PRECLAND_STATUS_LOCKED)

    def test_stale_timeout_drops_lock(self):
        # like the C++ loop: the first undetected frame drops LOCKED ->
        # SEARCHING(no_target); stale_timeout_ms decides when the
        # reported reason flips to STALE and the EMA resets
        s = make(d=self.d)
        s.enable(pe.CAUSE_LEGACY)
        over_pad(s, alt_m=10.0)
        run_ticks(s, 2.5, 1.0)
        self.assertEqual(s._state, pe.PRECLAND_STATUS_LOCKED)
        over_pad(s, north_m=30.0, alt_m=10.0)
        run_ticks(s, 3.5, 0.1)         # one missed frame -> searching
        self.assertEqual(s._state, pe.PRECLAND_STATUS_SEARCHING)
        self.assertEqual(s._gate_reason, pe.PRECLAND_REASON_NO_TARGET)
        run_ticks(s, 3.6, 1.0)         # past stale_timeout_ms
        self.assertEqual(s._gate_reason, pe.PRECLAND_REASON_STALE)

    def test_degraded_on_min_tags(self):
        s = make(d=self.d, extra={"detection": {"min_tags": 4}})
        s.enable(pe.CAUSE_LEGACY)
        over_pad(s, alt_m=10.0)        # at most 3 tags resolve
        run_ticks(s, 0.0, 2.0)
        self.assertEqual(s._state, pe.PRECLAND_STATUS_DEGRADED)
        self.assertEqual(s._gate_reason, pe.PRECLAND_REASON_MIN_TAGS)

    def test_disabled_publishes_no_target(self):
        s = make(d=self.d)
        over_pad(s, alt_m=10.0)
        run_ticks(s, 0.0, 1.0)
        self.assertEqual(s._state, pe.PRECLAND_STATUS_DISABLED)

    def test_disable_while_locked(self):
        s = make(d=self.d)
        s.enable(pe.CAUSE_LEGACY)
        over_pad(s, alt_m=10.0)
        run_ticks(s, 2.5, 2.0)
        s.disable(pe.CAUSE_LEGACY)
        self.assertEqual(s._state, pe.PRECLAND_STATUS_DISABLED)
        events = self._events(s)
        verbs = [e["d"] for e in events]
        # disabled fact precedes the lost its transition causes
        self.assertLess(verbs.index("precision_landing.disabled"),
                        verbs.index("precision_landing.lost"))
        self.assertEqual(
            events[verbs.index("precision_landing.disabled")]
            ["ed"]["src"], "legacy")

    def test_command_paths_agree(self):
        # capability invoke, legacy 6536 and the event command all run
        # the same choke points with their cause
        for act_path, cause_src in (("invoke", "invoke"),
                                    ("legacy", "legacy"),
                                    ("event", "event")):
            with self.subTest(path=act_path):
                d = tempfile.mkdtemp()
                s = make(d=d)
                if act_path == "invoke":
                    data, err = s.on_invoke("precision_landing",
                                            "enable", {})
                    self.assertIsNone(err)
                    self.assertEqual(data["status"], "searching")
                elif act_path == "legacy":
                    s._on_precland_action({"a": 1})
                else:
                    s._on_event_command({"d": "precision_landing.enable",
                                         "n": "nonce-1"})
                self.assertTrue(s.enabled)
                self.assertEqual(s._state, pe.PRECLAND_STATUS_SEARCHING)
                events = self._events(s, d)
                self.assertEqual(events[0]["d"],
                                 "precision_landing.enabled")
                self.assertEqual(events[0]["ed"]["src"], cause_src)
                # set_target through the same path
                if act_path == "invoke":
                    s.on_invoke("precision_landing", "set_target",
                                {"target": 0})
                elif act_path == "legacy":
                    s._on_precland_action({"a": 2, "b": 0})
                else:
                    s._on_event_command(
                        {"d": "precision_landing.set_target",
                         "ed": {"target": 0}, "n": "nonce-2"})

    def test_event_nonce_dedupe(self):
        s = make(d=self.d)
        s._on_event_command({"d": "precision_landing.enable",
                             "n": "abc"})
        s._on_event_command({"d": "precision_landing.enable",
                             "n": "abc"})
        s._on_event_command({"d": "precision_landing.disable",
                             "n": "abc"})           # same nonce again
        self.assertTrue(s.enabled)                 # redelivery dropped
        self.assertFalse(s._inbound_nonces.seen("zzz"))

    def test_event_set_target_validation(self):
        s = make(d=self.d,
                 extra={"target": {"targets": [0, 1, 3]}})
        s._on_event_command({"d": "precision_landing.set_target",
                             "ed": {"target": 3}, "n": "n1"})
        self.assertEqual(s.target_num, 3)
        s._on_event_command({"d": "precision_landing.set_target",
                             "ed": {"target": 2}, "n": "n2"})
        self.assertEqual(s.target_num, 3)          # not in allow-list
        s._on_event_command({"d": "precision_landing.set_target",
                             "ed": {"target": -1}, "n": "n3"})
        self.assertEqual(s.target_num, 3)
        verbs = [e["d"] for e in self._events(s)]
        self.assertIn("precision_landing.target_changed", verbs)

    def test_stabilized_fires(self):
        s = make(d=self.d)
        s.enable(pe.CAUSE_LEGACY)
        over_pad(s, alt_m=10.0)
        run_ticks(s, 2.5, 3.0)
        verbs = [e["d"] for e in self._events(s)]
        self.assertIn("precision_landing.stabilized", verbs)

    def _selftest_log(self):
        """--selftest on the descent file -> stub log bytes."""
        mdir = os.path.join(self.d, "run", "sim", "u1", "stub_precland")
        os.makedirs(mdir, exist_ok=True)
        cfg = {"stub": {"world": PAD_FIELD, "seed": 7, "unit": "u1",
                        "sysid": 1, "pose_port": 0},
               "s2s_udp_target_port": "1"}
        p = os.path.join(mdir, "stub_precland.config.json")
        with open(p, "w") as f:
            json.dump(cfg, f)
        s = stub_precland.StubPrecland(
            ["-c", p, "--selftest", POSE_DESCENT, "--dur", "30"])
        s._vt_ms = 0
        rows = sorted((json.loads(ln) for ln in open(POSE_DESCENT)),
                      key=lambda r: r.get("t_ms", 0))
        frame = 0
        t_s = 0.0
        pose_i = 0
        cause = {"legacy": pe.CAUSE_LEGACY, "invoke": pe.CAUSE_INVOKE,
                 "event": pe.CAUSE_EVENT, "config": pe.CAUSE_CONFIG}
        while t_s <= 30.0:
            s._vt_ms = int(round(t_s * 1000))
            while pose_i < len(rows) and \
                    rows[pose_i].get("t_ms", 0) <= t_s * 1000:
                row = rows[pose_i]
                if "lat" in row:
                    s._selftest_pose(row)
                cmd = row.get("cmd")
                src = cause.get(row.get("src", "event"),
                                pe.CAUSE_EVENT)
                if cmd == "enable":
                    s.enable(src)
                elif cmd == "disable":
                    s.disable(src)
                elif cmd == "set_target":
                    s.set_target_num(row.get("target", 0), src)
                pose_i += 1
            s.tick(t_s, frame)
            frame += 1
            t_s = frame * 0.05
        s.log.log("END", {"frames": frame}, ms=s.clock_ms())
        log = os.path.join(self.d, "run", "sim", "stubs", "u1.log")
        return open(log, "rb").read()

    def test_selftest_deterministic(self):
        """Two same-seed replays of the descent produce byte-identical
        stub logs."""
        a = self._selftest_log()
        self.tmp2 = tempfile.TemporaryDirectory()
        self.d = self.tmp2.name
        b = self._selftest_log()
        self.assertEqual(a, b)
        self.assertIn(b'"mt": 6537', a)      # PRECLAND_TARGET flowed
        self.assertIn(b"precision_landing.locked", a)


if __name__ == "__main__":
    unittest.main()
