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
    import stub_payload
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

    # ------------------------------------------------------ TASK-EV-PAY

    def _payload_log(self, kind):
        log = os.path.join(self.d, "run", "sim", "stubs", "u1.log")
        if not os.path.isfile(log):
            return []
        with open(log) as f:
            return [ln for ln in f if f" {kind} " in ln]

    def test_payload_default_is_single_main_bay(self):
        s = make(stub_payload.StubPayload, "stub_payload", self.d,
                 {"payload": {"pin": "drop_relay"}})
        self.assertEqual(list(s.bays), ["main"])
        self.assertEqual(s.armed_pin, "drop_relay")
        advert = stub_payload.build_advert(
            [{"key": k, "pin": b["pin"]} for k, b in s.bays.items()])
        # the single-bay advert gains only payload.main + optional bay
        self.assertIn("main", advert["actions"])
        self.assertEqual(advert["actions"]["release"]["params"]["bay"]
                         ["values"], ["main"])
        data, err = s.on_invoke("payload", "release", {})
        self.assertIsNone(err)
        self.assertEqual(data["bay"], "main")
        self.assertTrue(data["released"])
        ev = self._payload_log("EVENT")
        self.assertTrue(any("payload.released" in ln for ln in ev))
        self.assertTrue(any("payload.main.released" in ln for ln in ev))

    def test_payload_bays_release_only_the_selected(self):
        s = make(stub_payload.StubPayload, "stub_payload", self.d,
                 {"payload": {"bays": [{"key": "left", "pin": "drop_l"},
                                       {"key": "right"}]}})
        self.assertEqual(list(s.bays), ["left", "right"])
        self.assertEqual(s.bays["right"]["pin"], "stub_right")
        self.assertEqual(s.armed_pin, "drop_l")
        set_pose(s, 24.7, 46.6, 30.0)

        data, err = s.on_invoke("payload", "release", {"bay": "right"})
        self.assertIsNone(err)
        self.assertEqual(data["bay"], "right")
        ev = self._payload_log("EVENT")
        self.assertTrue(any("payload.right.released" in ln
                            for ln in ev), "right bay fact")
        self.assertFalse(any("payload.left.released" in ln
                             for ln in ev), "no left bay fact")
        rel = self._payload_log("RELEASE")
        self.assertEqual(len(rel), 1)
        self.assertIn('"bay": "right"', rel[0])

        # the per-bay action releases its own bay
        data, err = s.on_invoke("payload", "left", {})
        self.assertIsNone(err)
        self.assertEqual(data["bay"], "left")
        self.assertTrue(any("payload.left.released" in ln
                            for ln in self._payload_log("EVENT")))

        # unknown bays and actions are errors, never releases
        data, err = s.on_invoke("payload", "release", {"bay": "middle"})
        self.assertIsNone(data)
        self.assertIn("unknown bay", err)
        data, err = s.on_invoke("payload", "release", {"bay": 3})
        self.assertIsNone(data)
        self.assertIn("must be a string", err)
        data, err = s.on_invoke("payload", "nope", {})
        self.assertIsNone(data)
        self.assertIn("unknown action", err)
        self.assertEqual(len(self._payload_log("RELEASE")), 2,
                         "errors never released")

        # truth rows carry the bay
        with open(os.path.join(self.d, "run", "sim", "truth.jsonl")) as f:
            rows = [json.loads(ln) for ln in f]
        self.assertEqual(sorted(r["bay"] for r in rows
                                if r.get("cls") == "payload_drop"),
                         ["left", "right"])

    def test_payload_command_event_ignored(self):
        s = make(stub_payload.StubPayload, "stub_payload", self.d,
                 {"payload": {"bays": [{"key": "left"},
                                       {"key": "right"}]}})
        # a package/operator "emit payload.left.release" arrives as a
        # Sync_EventFire (1061): the namespace has no command events, so
        # the stub logs it ignored and releases nothing
        s.on_message(None, 0, {"mt": 1061,
                               "ms": {"d": "payload.left.release"}})
        ign = self._payload_log("IGNORED")
        self.assertEqual(len(ign), 1)
        self.assertIn("payload.left.release", ign[0])
        self.assertEqual(self._payload_log("RELEASE"), [])

        # an echoed own fact is not a command event - not logged ignored
        s.on_message(None, 0, {"mt": 1061,
                               "ms": {"d": "payload.left.released"}})
        self.assertEqual(len(self._payload_log("IGNORED")), 1)

    def test_payload_bad_bays_fall_back_to_main(self):
        s = make(stub_payload.StubPayload, "stub_payload", self.d,
                 {"payload": {"bays": [{"key": "release"},
                                       {"key": "Left"},
                                       {"key": "ok"},
                                       {"key": "ok"},
                                       {"nope": 1}]}})
        self.assertEqual(list(s.bays), ["ok"])
        cfg_warns = [ln for ln in self._payload_log("CONFIG")
                     if "payload_bays" in ln]
        self.assertEqual(len(cfg_warns), 4)

        s2 = make(stub_payload.StubPayload, "stub_payload2", self.d,
                  {"payload": {"bays": [{"key": "release"}]}})
        self.assertEqual(list(s2.bays), ["main"])

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
