#!/usr/bin/env python3
"""test_android_emulator.py - SL-0112: Android `mode: app` + emulator
adb-reverse networking helpers.

Covers (no device needed - android._adb is monkeypatched):
  - is_emulator: 'emulator-*' serials true, real serials / unset false;
  - reverse_ports issues one `adb reverse tcp:P tcp:P` per port and
    raises RuntimeError on a non-zero returncode; clear_reverse issues
    `adb reverse --remove-all`;
  - android mode validation: 'app' needs remote_modules
    [de_comm, de_mavlink]; an unknown mode is a SimError; a missing
    mode resolves to 'sim';
  - ctl.start_proc in mode app does NOT push_module_config (the app
    owns its configuration) but still start_module's; mode sim pushes;
  - ANDROID_HOST_IP=127.0.0.1 + an android unit -> the generated comm
    server config's public_host is 127.0.0.1;
  - SL-0118 `android: {net}` - auto|adb_reverse|lan: uses_adb_reverse
    for every serial x net combination, 'auto' default, bad net ->
    SimError.
"""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

import simlib  # noqa: E402
import android  # noqa: E402
import ctl  # noqa: E402


def _adb_ok(*args, **kwargs):
    return subprocess.CompletedProcess(args, 0, "", "")


def _emu_spec(mode=None, remote=("de_comm", "de_mavlink")):
    """A minimal one-unit copter sim whose android spec can be tweaked.
    type fake -> generate() never reads the real server repos."""
    spec = {"version": 1, "name": "emu_test", "slot_base": 10,
            "server": {"type": "fake", "port": 62900},
            "units": [{"block": "copter", "name": "emu",
                       "android": {"serial": "emulator-5554",
                                   "remote_modules": list(remote)}}]}
    if mode is not None:
        spec["units"][0]["android"]["mode"] = mode
    return spec


class EmulatorHelpersTest(unittest.TestCase):
    def test_is_emulator(self):
        self.assertTrue(android.is_emulator(
            {"android": {"serial": "emulator-5554"}}))
        self.assertFalse(android.is_emulator(
            {"android": {"serial": "66d0200f"}}))
        # no serial in spec -> ANDROID_SERIAL fallback (a real phone's
        # serial on this box, or unset) is not an emulator either
        self.assertFalse(android.is_emulator({"android": {}}))
        self.assertFalse(android.is_emulator({}))

    def test_reverse_ports(self):
        calls = []

        def fake_adb(u, *args, **kw):
            calls.append(args)
            return _adb_ok()
        with mock.patch.object(android, "_adb", fake_adb):
            android.reverse_ports({"android": {"serial": "emulator-5554"}},
                                  [62900, 62910, 8160])
        self.assertEqual(calls,
                         [("reverse", "tcp:62900", "tcp:62900"),
                          ("reverse", "tcp:62910", "tcp:62910"),
                          ("reverse", "tcp:8160", "tcp:8160")])

    def test_reverse_ports_raises(self):
        def fake_adb(u, *args, **kw):
            return subprocess.CompletedProcess(
                args, 1, "", "error: closed")
        with mock.patch.object(android, "_adb", fake_adb):
            with self.assertRaises(RuntimeError) as cm:
                android.reverse_ports({"android": {}}, [62900])
        self.assertIn("tcp:62900", str(cm.exception))

    def test_clear_reverse(self):
        calls = []

        def fake_adb(u, *args, **kw):
            calls.append(args)
            return _adb_ok()
        with mock.patch.object(android, "_adb", fake_adb):
            android.clear_reverse({"android": {}})
        self.assertEqual(calls, [("reverse", "--remove-all")])


class AdbReverseNetTest(unittest.TestCase):
    """SL-0118: `android: {net}` - auto|adb_reverse|lan picks how the
    device reaches the host."""

    def _u(self, serial, net=None):
        u = {"android": {"serial": serial}}
        if net is not None:
            u["android"]["net"] = net
        return u

    def test_uses_adb_reverse_matrix(self):
        # emulator serials adb-reverse in auto; anything else LANs -
        # explicit net: beats the serial either way
        cases = [
            ("emulator-5554", None, True),      # auto + emulator
            ("emulator-5554", "auto", True),
            ("emulator-5554", "adb_reverse", True),
            ("emulator-5554", "lan", False),
            ("66d0200f", None, False),          # auto + real phone
            ("66d0200f", "auto", False),
            ("66d0200f", "adb_reverse", True),  # USB phone, no network
            ("66d0200f", "lan", False),
        ]
        for serial, net, want in cases:
            with self.subTest(serial=serial, net=net):
                self.assertIs(
                    android.uses_adb_reverse(self._u(serial, net)), want)

    def test_net_defaults_to_auto(self):
        sim = simlib.load_sim(_emu_spec())
        self.assertEqual(sim["units"][0]["android"]["net"], "auto")

    def test_net_values_accepted(self):
        for net in ("auto", "adb_reverse", "lan"):
            spec = _emu_spec()
            spec["units"][0]["android"]["net"] = net
            sim = simlib.load_sim(spec)
            self.assertEqual(sim["units"][0]["android"]["net"], net)

    def test_bad_net_rejected(self):
        spec = _emu_spec()
        spec["units"][0]["android"]["net"] = "wifi"
        with self.assertRaises(simlib.SimError) as cm:
            simlib.load_sim(spec)
        self.assertIn("net", str(cm.exception))


class AndroidModeTest(unittest.TestCase):
    def test_default_mode_is_sim(self):
        sim = simlib.load_sim(_emu_spec())
        self.assertEqual(sim["units"][0]["android"]["mode"], "sim")

    def test_app_mode_accepted(self):
        sim = simlib.load_sim(_emu_spec(mode="app"))
        self.assertEqual(sim["units"][0]["android"]["mode"], "app")

    def test_unknown_mode_rejected(self):
        with self.assertRaises(simlib.SimError):
            simlib.load_sim(_emu_spec(mode="x"))

    def test_app_needs_both_modules(self):
        with self.assertRaises(simlib.SimError) as cm:
            simlib.load_sim(_emu_spec(mode="app", remote=("de_comm",)))
        self.assertIn("de_mavlink", str(cm.exception))

    def test_start_proc_app_mode_skips_push(self):
        """mode app: ctl start still start_module's the device service
        but never pushes de_override config (the app owns it)."""
        sim = simlib.load_sim(_emu_spec(mode="app"))
        u = sim["units"][0]
        calls = {"push": 0, "start": 0}
        with mock.patch.object(ctl, "_android_unit", lambda o: u), \
                mock.patch.object(ctl, "owner_dir", lambda o: "/tmp"), \
                mock.patch.object(android, "push_module_config",
                                  lambda *a, **k: calls.__setitem__(
                                      "push", calls["push"] + 1)), \
                mock.patch.object(android, "start_module",
                                  lambda *a, **k: calls.__setitem__(
                                      "start", calls["start"] + 1)):
            ctl.start_proc("emu", "de_comm")
        self.assertEqual(calls["push"], 0)
        self.assertEqual(calls["start"], 1)

    def test_start_proc_sim_mode_pushes(self):
        """control case: mode sim keeps the push + start sequence."""
        sim = simlib.load_sim(_emu_spec())
        u = sim["units"][0]
        calls = {"push": 0, "start": 0}
        with mock.patch.object(ctl, "_android_unit", lambda o: u), \
                mock.patch.object(ctl, "owner_dir", lambda o: "/tmp"), \
                mock.patch.object(android, "push_module_config",
                                  lambda *a, **k: calls.__setitem__(
                                      "push", calls["push"] + 1)), \
                mock.patch.object(android, "start_module",
                                  lambda *a, **k: calls.__setitem__(
                                      "start", calls["start"] + 1)):
            ctl.start_proc("emu", "de_comm")
        self.assertEqual(calls["push"], 1)
        self.assertEqual(calls["start"], 1)


class EmulatorNetworkingTest(unittest.TestCase):
    def test_public_host_is_loopback(self):
        """ANDROID_HOST_IP=127.0.0.1 (what launcher.up sets for an
        emulator unit) -> the generated comm server config advertises
        public_host 127.0.0.1 so the device reaches it via adb
        reverse."""
        spec = _emu_spec(mode="app")
        spec["name"] = "emu_gen"
        spec["server"] = {"type": "real", "auth": True, "port": 62900}
        sim = simlib.load_sim(spec)
        old = os.environ.get("ANDROID_HOST_IP")
        os.environ["ANDROID_HOST_IP"] = "127.0.0.1"
        try:
            with tempfile.TemporaryDirectory() as td, \
                    mock.patch.object(simlib, "RUN_DIR", td), \
                    mock.patch.object(simlib, "check_binaries",
                                      lambda *a, **k: None), \
                    mock.patch.object(android, "adb_ok",
                                      lambda u: False):
                simlib.generate(sim)
                cfg = json.load(open(os.path.join(
                    td, "emu_gen", "server", "de_server.config")))
        finally:
            if old is None:
                os.environ.pop("ANDROID_HOST_IP", None)
            else:
                os.environ["ANDROID_HOST_IP"] = old
        self.assertEqual(cfg["public_host"], "127.0.0.1")
        self.assertEqual(cfg["server_ip"], "0.0.0.0")


if __name__ == "__main__":
    unittest.main()
