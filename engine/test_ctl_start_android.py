#!/usr/bin/env python3
"""test_ctl_start_android.py - start_proc never starts an android module
over an old process that did not stop (DE-0131)."""

import os
import sys
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


if __name__ == "__main__":
    unittest.main()
