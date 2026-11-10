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

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "engine"))

try:
    import stub_sdr
    import stub_ai_box
    import stub_payload
    import stub_gnss
    import stub_rtcm
    import stub_base
    HAVE_DATABUS = True
except ImportError:
    HAVE_DATABUS = False

sys.path.insert(0, os.path.join(ROOT, "tests", "gnss", "fixtures"))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import rtcm_fixture  # noqa: E402
import fake_ntrip_caster  # noqa: E402

FOREST = os.path.join(ROOT, "worlds", "forest_a.yaml")
sys.path.insert(0, os.path.join(ROOT, "engine"))
import geo_projection as gp  # noqa: E402


def make(mod_cls, module, d, extra=None):
    """A stub instance on a generated-looking config under d/var/run/sim/u1/."""
    mdir = os.path.join(d, "var", "run", "sim", "u1", module)
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


class _RowSink:
    """TruthAppender stand-in: capture the appended rows in a list."""

    def __init__(self, rows):
        self.rows = rows

    def append(self, row, ms=None):
        self.rows.append(dict(row))


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
        log = os.path.join(self.d, "var", "run", "sim", "stubs", "u1.log")
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
        log = os.path.join(self.d, "var", "run", "sim", "stubs", "u1.log")
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
        with open(os.path.join(self.d, "var", "run", "sim", "truth.jsonl")) as f:
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
        with open(os.path.join(self.d, "var", "run", "sim", "truth.jsonl")) as f:
            rows = [json.loads(ln) for ln in f]
        self.assertTrue(all(r["tid"] == "clutter" for r in rows))
        s.reset_run()
        self.assertEqual(s._clutter, [])

    # ------------------------------------------------------- TASK-13

    HOME_LL = (-35.363262, 149.165237)          # the SITL home

    def _thermal_world(self, ambient_c):
        """A one-person world under the camera whose only difference
        is the environment's ambient temperature."""
        import world as world_mod
        return world_mod.World(
            {"version": 1, "origin": list(self.HOME_LL),
             "entities": [{"id": "p1", "cls": "person",
                           "pos": list(self.HOME_LL)}],
             "environment": {"ambient_c": ambient_c}},
            7, "test")

    def test_thermal_crossover_ambient_drops_p_detect(self):
        """Same person dead under a thermal camera at 40 m: the night
        ambient (8 C) keeps a 34 C body at full contrast, a warmer
        dusk partial, and the crossover ambient (== body temp) drops
        p_detect to 0 - sent rows count the contrast ramp."""
        sent = {}
        factors = {}
        for label, ambient in (("night", 8.0), ("dusk", 30.0),
                               ("crossover", 34.0)):
            s = make(stub_ai_box.StubAIBox, f"stub_ai_box_{label}",
                     self.d, {"camera_kind": "thermal"})
            s.world = self._thermal_world(ambient)
            s.env_f = s.world.sensor_factors("thermal")
            s.classes = [c for c in s.world.classes() if c != "pad"]
            s._vt_ms = 0
            rows = []
            s.truth = _RowSink(rows)
            set_pose(s, self.HOME_LL[0], self.HOME_LL[1], 40.0)
            pose = s.pose_feed.pose()
            att = gp.GeoAttitude()
            gpose = gp.GeoPose(lat_deg=pose["lat"], lng_deg=pose["lng"],
                               rel_alt_m=40.0)
            ent = s.world.entity("p1")
            factors[label] = s.world.thermal_factor(ent, 0.0)
            for f in range(60):
                s._maybe_detect("p1", "person", self.HOME_LL[0],
                                self.HOME_LL[1], 0.0,
                                pose, att, gpose, f, f * 0.5)
            sent[label] = sum(1 for r in rows if r.get("sent"))
        # the contrast ESTIMATE ramps 1 -> ~0.27 -> 0 over the ambients
        self.assertEqual(factors["crossover"], 0.0)
        self.assertLess(factors["dusk"], factors["night"])
        # and the stub emits accordingly: blind at crossover, sees at
        # night, somewhere between at the dusk ambient
        self.assertEqual(sent["crossover"], 0)
        self.assertGreater(sent["night"], sent["dusk"])
        self.assertGreaterEqual(sent["dusk"], sent["crossover"])

    # ----------------------------------------------------------- SL-0233

    def _stub_log(self, kind):
        log = os.path.join(self.d, "var", "run", "sim", "stubs", "u1.log")
        if not os.path.isfile(log):
            return []
        rows = []
        with open(log) as f:
            for ln in f:
                ms, k, js = ln.rstrip("\n").split(" ", 2)
                if k == kind:
                    rows.append((int(ms), json.loads(js)))
        return rows

    def test_gnss_sends_9103_at_1hz_from_the_truth_pose(self):
        s = make(stub_gnss.StubGnss, "stub_gnss", self.d)
        lat, lng = 24.7136, 46.6753
        set_pose(s, lat, lng, 30.0)
        s._vt_ms = 0
        for k in range(int(5.0 / 0.05)):         # 5 s at the 20 Hz tick
            s._vt_ms = k * 50
            s.tick(k * 0.05, k)
        sent = self._stub_log("SEND")
        self.assertEqual(len(sent), 5, "1 Hz for 5 s")
        times = [ms for ms, _ in sent]
        self.assertEqual([b - a for a, b in zip(times, times[1:])],
                         [1000] * 4)
        for _ms, row in sent:
            self.assertEqual(row["mt"], 9103)
            fix = row["ms"]
            # within 1 m of the truth: 1e-7 deg is ~1.1 cm
            dlat = (fix["la"] / 1e7 - lat) * 111320.0
            dlng = (fix["ln"] / 1e7 - lng) * 111320.0 * \
                math.cos(math.radians(lat))
            self.assertLess(math.hypot(dlat, dlng), 1.0)
            self.assertEqual(fix["fix"], 3)
            self.assertEqual(fix["ns"], 12)
            for key in ("hd", "vd", "ha", "va", "src", "gid", "a"):
                self.assertIn(key, fix)

    def test_gnss_model_from_world_sensors_and_stop_start(self):
        s = make(stub_gnss.StubGnss, "stub_gnss", self.d)
        s.model["fix"] = 4              # a test override of the model
        self.assertEqual(s.sensor_params()["fix_type"], 3)
        set_pose(s, 24.7, 46.6, 10.0)
        data, err = s.on_invoke("gnss", "stop", {})
        self.assertEqual((data, err), ({"enabled": False}, None))
        for k in range(60):
            s.tick(k * 0.05, k)
        self.assertEqual(self._stub_log("SEND"), [], "stopped = silent")
        s.on_invoke("gnss", "start", {})
        for k in range(60):
            s.tick(k * 0.05, k)
        sent = self._stub_log("SEND")
        self.assertEqual(len(sent), 3, "60 ticks = 3 s = 3 fixes")
        self.assertEqual(sent[0][1]["ms"]["fix"], 4)
        data, err = s.on_invoke("gnss", "nope", {})
        self.assertIsNone(data)
        self.assertIn("unknown action", err)

    def test_gnss_prefers_simstate_truth_when_fc_has_no_gps(self):
        s = make(stub_gnss.StubGnss, "stub_gnss", self.d)
        set_pose(s, 0.0, 0.0, 0.0)               # FC position: 0,0
        s.pose_feed._truth = {"lat": -35.3632620, "lng": 149.1652370,
                              "t_ms": stub_base.now_ms()}
        p = s.pose_feed.pose()
        self.assertEqual((p["lat"], p["lng"]), (-35.363262, 149.165237))
        fix = s.build_fix(p)
        self.assertEqual((fix["la"], fix["ln"]), (-353632620, 1491652370))
        self.assertEqual(fix["a"], 584000, "home alt + relative alt")

    def test_gnss_no_pose_sends_nothing(self):
        s = make(stub_gnss.StubGnss, "stub_gnss", self.d)
        for k in range(60):
            s.tick(k * 0.05, k)
        self.assertEqual(self._stub_log("SEND"), [])

    def test_rtcm_sends_the_fixture_bytes_in_order(self):
        s = make(stub_rtcm.StubRtcm, "stub_rtcm", self.d,
                 {"rtcm": {"chunk_bytes": 360, "rate_hz": 4}})
        fixture = rtcm_fixture.build_stream()
        for k in range(20 * 20):                  # 20 s is plenty
            s._vt_ms = k * 50
            s.tick(k * 0.05, k)
        sent = self._stub_log("SEND")
        self.assertGreater(len(sent), 1)
        self.assertEqual(sum(r["n"] for _, r in sent), len(fixture))
        offs = [r["off"] for _, r in sent]
        self.assertEqual(offs, sorted(offs))
        self.assertEqual([r["seq"] for _, r in sent],
                         list(range(len(sent))))
        done = self._stub_log("DONE")
        self.assertEqual(len(done), 1)
        self.assertEqual(done[0][1], {"sent": len(fixture),
                                      "fixture": len(fixture)})

    def test_rtcm_chunks_keep_whole_frames_or_split_raw(self):
        data = rtcm_fixture.build_stream()
        whole = stub_rtcm.make_chunks(data, 360, False)
        self.assertEqual(b"".join(c for _, c in whole), data)
        for _off, chunk in whole:
            rtcm_fixture.split_frames(chunk)       # each is whole frames
        # the 720 and 1000 byte frames exceed 360 and go alone
        self.assertIn(720, [len(c) for _, c in whole])
        self.assertIn(1000, [len(c) for _, c in whole])
        raw = stub_rtcm.make_chunks(data, 100, True)
        self.assertEqual(b"".join(c for _, c in raw), data)
        self.assertTrue(all(len(c) <= 100 for _, c in raw))

    def test_rtcm_loop_and_stop(self):
        s = make(stub_rtcm.StubRtcm, "stub_rtcm", self.d,
                 {"rtcm": {"chunk_bytes": 4000, "rate_hz": 20,
                           "loop": True}})
        self.assertEqual(len(s.chunks), 1)
        s.on_invoke("rtcm", "stop", {})
        for k in range(20):
            s.tick(k * 0.05, k)
        self.assertEqual(self._stub_log("SEND"), [])
        s.on_invoke("rtcm", "start", {})
        for k in range(20):
            s.tick(k * 0.05, k)
        self.assertEqual(len(self._stub_log("SEND")), 20)
        self.assertEqual(self._stub_log("DONE"), [], "loop never ends")

    # --------------------------------------------------- fixture + caster

    def test_fixture_frames_have_valid_crc24q(self):
        data = rtcm_fixture.build_stream()
        frames = rtcm_fixture.split_frames(data)
        self.assertEqual([len(f) for f in frames],
                         [20, 180, 360, 720, 1000])
        for f in frames:
            self.assertEqual(f[0], 0xD3)
            self.assertEqual(rtcm_fixture.crc24q(f[:-3]),
                             int.from_bytes(f[-3:], "big"))
        # a flipped bit is caught
        bad = bytearray(data)
        bad[30] ^= 0x10
        with self.assertRaises(ValueError):
            rtcm_fixture.split_frames(bytes(bad))
        # the generator is seeded: the same bytes every time
        self.assertEqual(data, rtcm_fixture.build_stream())
        # CRC24Q check value of "123456789" is 0xCDE703 (Qualcomm)
        self.assertEqual(rtcm_fixture.crc24q(b"123456789"), 0xCDE703)

    @staticmethod
    def _ntrip_get(port, mount, user, pw, send=b"", read_s=0.0):
        import base64
        import socket
        import time
        s = socket.create_connection(("127.0.0.1", port), 3)
        auth = base64.b64encode(f"{user}:{pw}".encode()).decode()
        s.sendall((f"GET /{mount} HTTP/1.0\r\nUser-Agent: NTRIP t\r\n"
                   f"Authorization: Basic {auth}\r\n\r\n").encode())
        s.settimeout(3)
        data = b""
        try:
            data = s.recv(4096)
        except OSError:
            pass
        if send:
            s.sendall(send)
        end = time.time() + read_s
        s.settimeout(0.2)
        while time.time() < end:
            try:
                chunk = s.recv(65536)
            except OSError:
                continue
            if not chunk:
                break
            data += chunk
        return s, data

    def test_ntrip_caster_auth_stream_and_gga_log(self):
        import time
        logp = os.path.join(self.d, "caster.log")
        c = fake_ntrip_caster.FakeCaster(
            port=0, mount="STUDIO", user="u", password="p", chunk=500,
            interval=0.05, log_path=logp)
        port = c.start()
        try:
            s, data = self._ntrip_get(port, "STUDIO", "u", "wrong")
            s.close()
            self.assertTrue(data.startswith(b"HTTP/1.0 401"), data)
            s, data = self._ntrip_get(port, "NOPE", "u", "p")
            s.close()
            self.assertTrue(data.startswith(b"HTTP/1.0 404"), data)
            gga = [b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,"
                   b"545.4,M,46.9,M,,*47\r\n",
                   b"$GNGGA,123520,4807.039,N,01131.001,E,1,08,0.9,"
                   b"545.5,M,46.9,M,,*4A\r\n"]
            s, data = self._ntrip_get(port, "STUDIO", "u", "p",
                                      send=gga[0] + b"$GPRMC,junk\r\n"
                                      + gga[1], read_s=1.5)
            s.close()
            self.assertTrue(data.startswith(b"ICY 200 OK\r\n\r\n"), data)
            body = data[len(b"ICY 200 OK\r\n\r\n"):]
            self.assertEqual(body, rtcm_fixture.build_stream(),
                             "good login streams the fixture")
            time.sleep(0.3)
        finally:
            c.stop()
        self.assertEqual(c.gga, [g.decode().strip() for g in gga],
                         "each GGA line logged, other lines not")
        text = open(logp).read()
        self.assertIn("AUTH_FAIL", text)
        self.assertEqual(text.count(" GGA "), 2)


if __name__ == "__main__":
    unittest.main()
