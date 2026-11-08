#!/usr/bin/env python3
"""test_capabilities.py - the SL-0215 capability probes (no hardware).

Each probe is mocked into its ok and its missing shape: probe() maps
capability -> {"ok", "reason"} and missing() lists only the failed
entries. Nothing here touches a real device - subprocess, filesystem
and simlib device helpers are all patched.
"""

import os
import sys
import unittest
from unittest import mock

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import capabilities  # noqa: E402
import simlib  # noqa: E402


def _proc(rc=0, out=""):
    class R:
        returncode = rc
        stdout = out
        stderr = ""
    return R()


class SitlProbeTest(unittest.TestCase):
    def test_ok(self):
        with mock.patch.object(os.path, "isfile",
                               side_effect=lambda p: p.endswith(
                                   "arducopter")):
            ok, reason = capabilities.probe_sitl()
        self.assertTrue(ok)
        self.assertIn("arducopter", reason)

    def test_missing(self):
        with mock.patch.object(os.path, "isfile", return_value=False):
            ok, reason = capabilities.probe_sitl()
        self.assertFalse(ok)
        self.assertIn("no SITL binary", reason)


class CameraProbeTest(unittest.TestCase):
    def test_ok(self):
        with mock.patch.object(capabilities, "_video_indices",
                               return_value=[0, 2]), \
             mock.patch.object(simlib, "video_capture_capable",
                               side_effect=lambda i: i == 2):
            ok, reason = capabilities.probe_camera()
        self.assertTrue(ok)
        self.assertIn("/dev/video2", reason)

    def test_no_devices(self):
        with mock.patch.object(capabilities, "_video_indices",
                               return_value=[]):
            ok, reason = capabilities.probe_camera()
        self.assertFalse(ok)
        self.assertIn("no /dev/video*", reason)

    def test_no_capture(self):
        with mock.patch.object(capabilities, "_video_indices",
                               return_value=[0]), \
             mock.patch.object(simlib, "video_capture_capable",
                               return_value=False):
            ok, reason = capabilities.probe_camera()
        self.assertFalse(ok)
        self.assertIn("CAPTURE", reason)


class LoopbackProbeTest(unittest.TestCase):
    def test_ok(self):
        with mock.patch.object(capabilities, "_video_indices",
                               return_value=[0, 4]), \
             mock.patch.object(simlib, "video_is_loopback",
                               side_effect=lambda i: i == 4):
            ok, reason = capabilities.probe_v4l2loopback()
        self.assertTrue(ok)
        self.assertIn("/dev/video4", reason)

    def test_missing(self):
        with mock.patch.object(capabilities, "_video_indices",
                               return_value=[0]), \
             mock.patch.object(simlib, "video_is_loopback",
                               return_value=False):
            ok, reason = capabilities.probe_v4l2loopback()
        self.assertFalse(ok)
        self.assertIn("no v4l2loopback", reason)


class AdbProbeTest(unittest.TestCase):
    DEVICES = "List of devices attached\n66d0200f\tdevice\n\n"

    def test_no_adb_binary(self):
        with mock.patch("shutil.which", return_value=None):
            ok, reason = capabilities.probe_android_adb()
        self.assertFalse(ok)
        self.assertIn("adb not on PATH", reason)

    def test_ok(self):
        with mock.patch("shutil.which", return_value="/usr/bin/adb"), \
             mock.patch("subprocess.run",
                        return_value=_proc(0, self.DEVICES)):
            ok, reason = capabilities.probe_android_adb()
        self.assertTrue(ok)
        self.assertIn("66d0200f", reason)

    def test_unauthorized_only(self):
        out = "List of devices attached\nabc123\tunauthorized\n"
        with mock.patch("shutil.which", return_value="/usr/bin/adb"), \
             mock.patch("subprocess.run", return_value=_proc(0, out)):
            ok, reason = capabilities.probe_android_adb()
        self.assertFalse(ok)
        self.assertIn("no live adb device", reason)


class TargetProbeTest(unittest.TestCase):
    def test_no_target(self):
        with mock.patch.object(simlib, "setting", return_value=None):
            ok, reason = capabilities.probe_rpi_target()
        self.assertFalse(ok)
        self.assertIn("no target", reason)

    def test_ok(self):
        with mock.patch("shutil.which", return_value="/usr/bin/ssh"), \
             mock.patch("subprocess.run", return_value=_proc(0)):
            ok, reason = capabilities.probe_rpi_target("pi@lab")
        self.assertTrue(ok)
        self.assertIn("pi@lab", reason)

    def test_unreachable(self):
        with mock.patch("shutil.which", return_value="/usr/bin/ssh"), \
             mock.patch("subprocess.run",
                        return_value=_proc(255, "Connection refused")):
            ok, reason = capabilities.probe_rpi_target("pi@lab")
        self.assertFalse(ok)
        self.assertIn("not reachable", reason)

    def test_setting_fallback(self):
        with mock.patch.object(simlib, "setting",
                               return_value="pi@lab"), \
             mock.patch("shutil.which", return_value="/usr/bin/ssh"), \
             mock.patch("subprocess.run", return_value=_proc(0)):
            ok, _r = capabilities.probe_rpi_target()
        self.assertTrue(ok)


class M3dProbeTest(unittest.TestCase):
    def _m3d(self, bridge, vite):
        def _isfile(p):
            return (bridge and p.endswith("udp2websocket.js")) or \
                   (vite and p.endswith("vite.js"))
        return mock.patch.object(os.path, "isfile",
                                 side_effect=_isfile)

    def test_ok(self):
        with self._m3d(True, True), \
             mock.patch.object(simlib, "m3d_version_mismatch",
                               return_value=None), \
             mock.patch.object(simlib, "m3d_version",
                               return_value="1.4.0"):
            ok, reason = capabilities.probe_mavlink3dmap()
        self.assertTrue(ok)
        self.assertIn("1.4.0", reason)

    def test_missing(self):
        with self._m3d(False, False):
            ok, reason = capabilities.probe_mavlink3dmap()
        self.assertFalse(ok)
        self.assertIn("incomplete", reason)

    def test_version_mismatch_is_missing(self):
        with self._m3d(True, True), \
             mock.patch.object(simlib, "m3d_version_mismatch",
                               return_value="expected 1.4.0, found 9.9"):
            ok, reason = capabilities.probe_mavlink3dmap()
        self.assertFalse(ok)
        self.assertIn("9.9", reason)


class RealServersProbeTest(unittest.TestCase):
    def test_ok(self):
        with mock.patch.object(os.path, "isfile", return_value=True), \
             mock.patch.object(os, "access", return_value=True):
            ok, reason = capabilities.probe_real_servers()
        self.assertTrue(ok)
        self.assertIn("andruavmainserver", reason)

    def test_missing(self):
        with mock.patch.object(os.path, "isfile", return_value=False):
            ok, reason = capabilities.probe_real_servers()
        self.assertFalse(ok)
        self.assertIn("cargo build", reason)


class Esp32ProbeTest(unittest.TestCase):
    def test_ok(self):
        import esp32
        with mock.patch.object(esp32, "default_port",
                               return_value="/dev/ttyUSB0"), \
             mock.patch.object(esp32, "port_usable", return_value=True):
            ok, reason = capabilities.probe_esp32()
        self.assertTrue(ok)
        self.assertIn("ttyUSB", reason)

    def test_missing(self):
        import esp32
        with mock.patch.object(esp32, "default_port",
                               return_value=None):
            ok, reason = capabilities.probe_esp32()
        self.assertFalse(ok)
        self.assertIn("no usable ESP32", reason)


class ProbeMapTest(unittest.TestCase):
    def test_every_capability_has_a_probe(self):
        self.assertEqual(set(simlib.CAPABILITIES), set(capabilities.PROBES))

    def test_probe_never_raises(self):
        with mock.patch.dict(capabilities.PROBES, {
                "sitl": mock.Mock(side_effect=RuntimeError("boom"))}):
            res = capabilities.probe(only=["sitl"])
        self.assertFalse(res["sitl"]["ok"])
        self.assertIn("boom", res["sitl"]["reason"])

    def test_unknown_name_reports(self):
        res = capabilities.probe(only=["warp_drive"])
        self.assertFalse(res["warp_drive"]["ok"])

    def test_missing_lists_only_failures(self):
        res = {"camera": {"ok": True, "reason": "ok"},
               "sitl": {"ok": False, "reason": "no sitl"}}
        with mock.patch.object(capabilities, "probe", return_value=res):
            miss = capabilities.missing(["camera", "sitl"])
        self.assertEqual(miss, [("sitl", "no sitl")])


if __name__ == "__main__":
    unittest.main()
