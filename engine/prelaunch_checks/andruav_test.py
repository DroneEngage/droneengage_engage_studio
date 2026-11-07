"""andruav_test.py - phone detection and routing to the shared probes."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import prelaunch  # noqa: E402
from prelaunch_checks import andruav  # noqa: E402


def _ctx(**over):
    t = {"name": "t", "services": {}, "test_unit": {"name": "cam1"}}
    t.update(over)
    return prelaunch.Ctx(t, "/tmp", "v")


class Skip(unittest.TestCase):
    def test_no_phone_is_skip(self):
        with mock.patch.object(andruav, "phones_attached", return_value=[]):
            st, d = andruav.video_normal(_ctx(test_phone={"name": "p"}))
        self.assertEqual(st, "SKIP")
        self.assertIn("no phone", d)

    def test_no_phone_setting_is_skip(self):
        st, d = andruav.relay(_ctx())
        self.assertEqual(st, "SKIP")
        self.assertIn("test_phone", d)

    def test_wrong_serial_is_skip(self):
        with mock.patch.object(andruav, "phones_attached", return_value=["a"]):
            st, _ = andruav.video_relay(
                _ctx(test_phone={"name": "p", "adb_serial": "b"}))
        self.assertEqual(st, "SKIP")


class Routing(unittest.TestCase):
    def test_probes_see_the_phone_and_the_target_stays(self):
        seen = {}

        def fake(c):
            seen["unit"] = c.target["test_unit"]
            return "PASS", "ok"
        ctx = _ctx(test_phone={"name": "phone1", "party": "P9"})
        with mock.patch.object(andruav, "phones_attached", return_value=["s"]):
            with mock.patch.object(andruav.video, "camera_relay", fake):
                self.assertEqual(andruav.video_relay(ctx)[0], "PASS")
            self.assertEqual(seen["unit"]["name"], "phone1")
            with mock.patch.object(andruav.relay_probe, "mavlink_both_ways",
                                   fake):
                self.assertEqual(andruav.relay(ctx)[0], "PASS")
            self.assertEqual(seen["unit"]["party"], "P9")
        self.assertEqual(ctx.target["test_unit"], {"name": "cam1"})


class Adb(unittest.TestCase):
    def test_unauthorized_phone_does_not_count(self):
        out = ("List of devices attached\nAAA\tdevice\nBBB\tunauthorized\n\n")
        with mock.patch("shutil.which", return_value="/x/adb"), \
                mock.patch("subprocess.run",
                           return_value=mock.Mock(stdout=out)):
            self.assertEqual(andruav.phones_attached(), ["AAA"])


class StaleIce(unittest.TestCase):
    def _run(self, entries, target=None):
        ctx = _ctx(test_phone={"name": "p"}, role="airgap",
                   services={"turn": {"host": "10.0.0.5"}})
        with mock.patch.object(andruav, "phones_attached", return_value=["s"]), \
                mock.patch.object(andruav, "phone_ice",
                                  return_value=(entries, None)):
            return andruav.stale_ice(ctx)

    def test_stale_host_is_fail_naming_ice(self):
        st, d = self._run([{"urls": "turn:old.example.com:3478"}])
        self.assertEqual(st, "FAIL")
        self.assertIn("ICE", d)
        self.assertIn("old.example.com", d)

    def test_empty_list_is_fail(self):
        st, d = self._run([])
        self.assertEqual((st, "ICE" in d), ("FAIL", True))

    def test_good_list_passes(self):
        self.assertEqual(self._run([{"urls": "turn:10.0.0.5:3478"}])[0], "PASS")

    def test_no_phone_is_skip(self):
        with mock.patch.object(andruav, "phones_attached", return_value=[]):
            self.assertEqual(andruav.stale_ice(
                _ctx(test_phone={"name": "p"}))[0], "SKIP")

    def test_fake_adb_reads_file(self):
        out = mock.Mock(returncode=0, stdout='{"iceServers":[{"urls":"stun:a:1"}]}',
                        stderr="")
        with mock.patch("subprocess.run", return_value=out):
            self.assertEqual(andruav.phone_ice("s")[0], [{"urls": "stun:a:1"}])


if __name__ == "__main__":
    unittest.main()
