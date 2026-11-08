#!/usr/bin/env python3
"""test_ctl_start_android.py - start_proc never starts an android module
over an old process that did not stop (DE-0131); kill_proc never claims
a stop the device did not do (DE-0132)."""

import contextlib
import io
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ctl
import android
import simlib

UNIT = {"name": "u1", "android": {"remote_modules": ["de_comm"]}}


class StartProcAndroid(unittest.TestCase):
    def run_start(self, down_results):
        with mock.patch.object(ctl, "_android_unit", return_value=UNIT), \
             mock.patch.object(ctl, "owner_dir", return_value="/tmp/x"), \
             mock.patch.object(android, "android_mode", return_value="sim"), \
             mock.patch.object(android, "push_module_config"), \
             mock.patch.object(android, "module_alive", return_value=True), \
             mock.patch.object(android, "stop_module") as stop, \
             mock.patch.object(android, "wait_module_down",
                               side_effect=down_results) as wait, \
             mock.patch.object(android, "start_module") as start:
            try:
                ctl.start_proc("u1", "de_comm")
                err = None
            except SystemExit as e:
                err = e
            return err, stop, wait, start

    def test_never_stops_aborts_without_start(self):
        err, stop, wait, start = self.run_start([False, False])
        self.assertIsNotNone(err)
        self.assertEqual(stop.call_count, 2)
        start.assert_not_called()

    def test_stops_on_retry_starts(self):
        err, stop, wait, start = self.run_start([False, True])
        self.assertIsNone(err)
        self.assertEqual(stop.call_count, 2)
        start.assert_called_once()


class KillProcAndroid(unittest.TestCase):
    """kill_proc must not print "stopped on device" nor drop the pidfile
    when wait_module_down says the module still runs (DE-0132)."""

    def run_kill(self, down_result):
        with tempfile.TemporaryDirectory() as td:
            pf = os.path.join(td, "de_comm.pid")
            with open(pf, "w") as f:
                f.write("123")
            buf = io.StringIO()
            with mock.patch.object(ctl, "_android_unit", return_value=UNIT), \
                 mock.patch.object(ctl, "owner_sim", return_value="s1"), \
                 mock.patch.object(simlib, "pid_path", return_value=pf), \
                 mock.patch.object(android, "stop_module") as stop, \
                 mock.patch.object(android, "wait_module_down",
                                   return_value=down_result), \
                 mock.patch.object(android, "serial_of",
                                   return_value="serial1"), \
                 contextlib.redirect_stdout(buf):
                try:
                    ctl.kill_proc("u1", "de_comm")
                    err = None
                except SystemExit as e:
                    err = e
            return err, buf.getvalue(), os.path.isfile(pf), stop

    def test_still_running_aborts_no_stopped_message(self):
        err, out, pidfile_kept, stop = self.run_kill(False)
        self.assertIsNotNone(err)
        stop.assert_called_once()
        self.assertNotIn("stopped on device", out)
        self.assertIn("still running on device", str(err))
        self.assertTrue(pidfile_kept)

    def test_module_down_stops_and_drops_pidfile(self):
        err, out, pidfile_kept, stop = self.run_kill(True)
        self.assertIsNone(err)
        stop.assert_called_once()
        self.assertIn("stopped on device", out)
        self.assertFalse(pidfile_kept)


if __name__ == "__main__":
    unittest.main()
