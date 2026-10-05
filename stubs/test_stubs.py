#!/usr/bin/env python3
"""Offline tests for the stub sensor models (no SITL, no de_comm):

    python3 stubs/test_stubs.py

Needs droneengage_databus/python (the stubs import its message ids); the
whole file SKIPs when it is not checked out.
"""
import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "engine"))

try:
    import stub_sdr
    import stub_ai_box
    import stub_base
    HAVE_DATABUS = True
except ImportError:
    HAVE_DATABUS = False

FOREST = os.path.join(ROOT, "worlds", "forest_a.yaml")
sys.path.insert(0, os.path.join(ROOT, "engine"))
import geo_projection as gp  # noqa: E402


def make(mod_cls, module, d, extra=None):
    """A stub instance on a generated-looking config under d/run/sim/u1/."""
    mdir = os.path.join(d, "run", "sim", "u1", module)
    os.makedirs(mdir)
    cfg = {"stub": {"world": FOREST, "seed": 7, "unit": "u1"},
           "s2s_udp_target_port": "1"}
    cfg.update(extra or {})
    p = os.path.join(mdir, f"{module}.config.json")
    with open(p, "w") as f:
        json.dump(cfg, f)
    return mod_cls(["-c", p])


def set_pose(stub, lat, lng, rel_alt, t_ms=0):
    f = stub.pose_feed
    f._pos = {"lat": lat, "lng": lng, "alt_m": 584 + rel_alt,
              "rel_alt_m": rel_alt, "t_ms": t_ms}
    f._att = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "t_ms": t_ms}


@unittest.skipUnless(HAVE_DATABUS, "droneengage_databus not checked out")
class T(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def _sdr_events(self, s, secs=12):
        s._vt_ms = 0
        for k in range(int(secs / 0.05)):
            s.tick(k * 0.05, k)
        log = os.path.join(self.d, "run", "sim", "stubs", "u1.log")
        if not os.path.isfile(log):
            return []
        with open(log) as f:
            return [ln for ln in f if " EVENT " in ln]

    def test_sdr_wide_scan_still_hears_the_phone(self):
        s = make(stub_sdr.StubSDR, "stub_sdr", self.d,
                 {"signal_detect": {"ref_power_dbm": -40,
                                    "path_loss_n": 2.7,
                                    "debounce_sweeps": 1}})
        e = s.world.entity("phone_1")
        lat, lng, _ = e.pos_at(0, s.world)
        set_pose(s, lat, lng, 0.0)
        # 200 MHz span: > 4096 bins of 25 kHz - the phone sits at 869 MHz,
        # past f_start + 4096 * 25 kHz
        s.on_invoke("sdr", "scan", {"f_start": 700e6, "f_end": 900e6,
                                     "threshold_dbm": -70})
        self.assertTrue(self._sdr_events(s), "wide scan lost the emitter")

    def test_sdr_uses_slant_range(self):
        s = make(stub_sdr.StubSDR, "stub_sdr", self.d,
                 {"signal_detect": {"ref_power_dbm": -40,
                                    "path_loss_n": 2.7,
                                    "debounce_sweeps": 1}})
        lat, lng, _ = s.world.entity("phone_1").pos_at(0, s.world)
        # 60 m straight above: -10 dBm tx, -30 ref, n 2.7 -> ~-88 dBm,
        # under the -70 gate (the horizontal-only model said ~-40)
        set_pose(s, lat, lng, 60.0)
        s.on_invoke("sdr", "scan", {"f_start": 868.5e6, "f_end": 869.5e6,
                                     "threshold_dbm": -70})
        self.assertEqual(self._sdr_events(s), [])

    def test_sdr_reset_run_drops_the_scan(self):
        s = make(stub_sdr.StubSDR, "stub_sdr", self.d)
        s.on_invoke("sdr", "scan", {"f_start": 868.5e6, "f_end": 869.5e6})
        s.reset_run()
        self.assertIsNone(s.scan)
        data, err = s.on_invoke("sdr", "scan",
                                {"f_start": 868.5e6, "f_end": 869.5e6})
        self.assertIsNone(err)

    def test_stale_pose_is_no_pose(self):
        f = stub_base.PoseFeed(0)
        now = stub_base.now_ms()
        f._pos = {"lat": 1.0, "lng": 2.0, "rel_alt_m": 30, "t_ms": now - 9000}
        f._att = {"roll": 0, "pitch": 0, "yaw": 0, "t_ms": now - 9000}
        self.assertIsNotNone(f.pose())      # selftest: no age limit
        f.max_age_ms = 3000
        self.assertIsNone(f.pose())
        f._loc9102 = {"lat": 1.0, "lng": 2.0, "rel_alt_m": 0, "t_ms": now}
        self.assertEqual(f.pose()["src"], "9102")

    def test_static_mast_gets_clutter(self):
        s = make(stub_ai_box.StubAIBox, "stub_ai_box", self.d)
        s.world.clutter["rate_per_km2_min"] = 5e5   # force spawns
        s._vt_ms = 0
        s._clutter_tick(12.0, 1, 0.5, gp.GeoAttitude(),
                        gp.GeoPose(rel_alt_m=12.0))
        self.assertTrue(s._clutter, "a 12 m mast camera saw no clutter")
        with open(os.path.join(self.d, "run", "sim", "truth.jsonl")) as f:
            rows = [json.loads(ln) for ln in f]
        self.assertTrue(all(r["tid"] == "clutter" for r in rows))
        s.reset_run()
        self.assertEqual(s._clutter, [])


if __name__ == "__main__":
    unittest.main()
