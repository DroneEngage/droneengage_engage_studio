"""config_test.py - each config fault gives FAIL and names the file."""

import json
import os
import shutil
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import prelaunch  # noqa: E402
from prelaunch_checks import config  # noqa: E402

GOOD_ICE = [{"urls": "stun:turn.example:3478"},
            {"urls": "turn:turn.example:3478", "username": "u1",
             "password": "s3cret-pw"}]


class Fixture(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.d)
        self.web = self.write("config.json", json.dumps(
            {"CONST_ICE_SERVERS": GOOD_ICE}))
        self.cam = self.write("cam.json", '// comment\n{"iceServers": '
                              + json.dumps(GOOD_ICE) + ",}")
        self.and_ = self.write("Pn.java", (
            'new PeerConnection.IceServer("stun:turn.example:3478"));\n'
            'new PeerConnection.IceServer("turn:turn.example:3478", '
            '"u1", "s3cret-pw"));'))
        self.coturn = self.write("turn.conf", (
            "listening-port=3478\nmin-port=20000\nmax-port=39999\n"
            "user=u1:s3cret-pw\n"))
        self.comm = self.write("comm.config", (
            '{"public_host": "srv.example", // note\n'
            ' "udp_relay": [{"host": "127.0.0.1", "port": 9100}]}'))
        self.relay = self.write("relay.config", (
            '{"public_host": "srv.example", "udp_port_range": [40000, 40999]}'))

    def write(self, name, text):
        p = os.path.join(self.d, name)
        with open(p, "w") as fh:
            fh.write(text)
        return p

    def ctx(self, **over):
        t = {"name": "t", "role": "mirror",
             "services": {"turn": {"host": "turn.example"},
                          "comm": {"host": "srv.example"},
                          "relay": {"host": "srv.example",
                                    "control_port": 9100}},
             "config": {"webclient": [self.web, self.web + ".local"],
                        "de_camera": self.cam, "andruav": self.and_,
                        "coturn": self.coturn, "comm": self.comm,
                        "relay": self.relay}}
        t.update(over)
        return prelaunch.Ctx(t, self.d, "v")


class Good(Fixture):
    def test_correct_config_passes_every_check_fast(self):
        t0 = time.time()
        for fn in (config.ice, config.public_host, config.udp_relay,
                   config.port_overlap):
            st, detail = fn(self.ctx())
            self.assertEqual(st, "PASS", detail)
        self.assertLess(time.time() - t0, 30)


class IceFaults(Fixture):
    def test_empty_webclient_list(self):
        self.write("config.json", '{"CONST_ICE_SERVERS": []}')
        st, d = config.ice(self.ctx())
        self.assertEqual(st, "FAIL")
        self.assertIn("config.json", d)
        self.assertIn("CONST_ICE_SERVERS", d)

    def test_missing_webclient_field(self):
        self.write("config.json", "{}")
        st, d = config.ice(self.ctx())
        self.assertEqual(st, "FAIL")
        self.assertIn("CONST_ICE_SERVERS", d)

    def test_local_file_overrides(self):
        self.write("config.json.local", '{"CONST_ICE_SERVERS": []}')
        st, _ = config.ice(self.ctx())
        self.assertEqual(st, "FAIL")

    def test_host_the_target_cannot_reach(self):
        bad = GOOD_ICE + [{"urls": "turn:cloud.ardupilot.org:3478",
                           "username": "x", "password": "y"}]
        self.write("cam.json", json.dumps({"iceServers": bad}))
        st, d = config.ice(self.ctx(role="airgap"))
        self.assertEqual(st, "FAIL")
        self.assertIn("cam.json", d)
        self.assertIn("iceServers", d)
        self.assertIn("cloud.ardupilot.org", d)

    def test_turn_user_differs_from_coturn(self):
        bad = [{"urls": "turn:turn.example:3478", "username": "other",
                "password": "s3cret-pw"}]
        self.write("cam.json", json.dumps({"iceServers": bad}))
        st, d = config.ice(self.ctx())
        self.assertEqual(st, "FAIL")
        self.assertIn("cam.json", d)
        self.assertIn("turn.conf", d)

    def test_turn_password_differs_and_stays_out_of_text(self):
        bad = [{"urls": "turn:turn.example:3478", "username": "u1",
                "password": "wrong-pw-value"}]
        self.write("cam.json", json.dumps({"iceServers": bad}))
        st, d = config.ice(self.ctx())
        self.assertEqual(st, "FAIL")
        self.assertNotIn("wrong-pw-value", d)
        self.assertNotIn("s3cret-pw", d)
        self.assertIn("password", d)

    def test_andruav_list_is_parsed(self):
        e = config.parse_andruav_ice(open(self.and_).read())
        self.assertEqual(e[1]["username"], "u1")
        self.assertEqual(config.ice_host("turn:h.x:3478?transport=tcp"), "h.x")
        self.assertEqual(config.ice_host("turns:h.y"), "h.y")

    def test_no_webclient_config_is_skip(self):
        st, _ = config.ice(self.ctx(config={}))
        self.assertEqual(st, "SKIP")

    def test_missing_declared_file_is_fail(self):
        os.remove(self.cam)
        st, d = config.ice(self.ctx())
        self.assertEqual(st, "FAIL")
        self.assertIn("does not exist", d)


class HostAndRelayFaults(Fixture):
    def test_loopback_public_host(self):
        self.write("comm.config", '{"public_host": "127.0.0.1"}')
        st, d = config.public_host(self.ctx())
        self.assertEqual(st, "FAIL")
        self.assertIn("comm.config", d)
        self.assertIn("public_host", d)

    def test_stale_public_host(self):
        self.write("relay.config", '{"public_host": "10.9.9.9"}')
        st, d = config.public_host(self.ctx())
        self.assertEqual(st, "FAIL")
        self.assertIn("stale", d)
        self.assertIn("relay.config", d)

    def test_no_udp_relay(self):
        self.write("comm.config", '{"public_host": "srv.example"}')
        st, d = config.udp_relay(self.ctx())
        self.assertEqual(st, "FAIL")
        self.assertIn("comm.config", d)
        self.assertIn("udp_relay", d)

    def test_coturn_overlaps_relay(self):
        self.write("turn.conf", "min-port=20000\nmax-port=40000\n")
        st, d = config.port_overlap(self.ctx())
        self.assertEqual(st, "FAIL")
        self.assertIn("overlaps", d)
        self.assertIn("40000", d)

    def test_other_slot_overlap(self):
        st, d = config.port_overlap(
            self.ctx(other_slot={"relay_range": [40500, 41000]}))
        self.assertEqual(st, "FAIL")
        self.assertIn("other slot", d)


if __name__ == "__main__":
    unittest.main()
