#!/usr/bin/env python3
"""test_ctl_bridge_error.py - a refused GCS bridge login is reported, not
waited on as if every unit were offline."""

import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ctl
import simlib

REFUSED = ("[bridge] session down: RuntimeError('login refused: Account Not "
           "Found.') - retrying in 5 s\n")


class BridgeLoginError(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.d)
        os.makedirs(os.path.join(self.d, "server", "logs"))
        self.log = os.path.join(self.d, "server", "logs", "gcs_bridge.log")
        for p in (mock.patch.object(simlib, "run_dir", lambda _n: self.d),
                  mock.patch.object(simlib, "load_manifest", lambda _n: {
                      "server": {"account": {"user": "sim@x"}}})):
            p.start()
            self.addCleanup(p.stop)

    def write(self, text):
        with open(self.log, "w") as fh:
            fh.write(text)

    def test_no_log_is_no_error(self):
        self.assertIsNone(ctl.bridge_login_error("s"))

    def test_refusal_names_account_and_reason(self):
        self.write(REFUSED + "[ctrl] 17:20:09 list\n")
        err = ctl.bridge_login_error("s")
        self.assertIn("sim@x", err)
        self.assertIn("Account Not Found.", err)
        self.assertIn("SIM_AUTH_USER", err)

    def test_later_success_clears_it(self):
        self.write(REFUSED + "[bridge] GCS G connected to 127.0.0.1\n")
        self.assertIsNone(ctl.bridge_login_error("s"))

    def test_an_older_run_is_ignored(self):
        self.write(REFUSED)
        self.assertIsNone(ctl.bridge_login_error(
            "s", os.path.getmtime(self.log) + 60))

    def test_rate_limit_keeps_no_false_refusal(self):
        self.write("[bridge] session down: <HTTPError 429: 'Too Many "
                   "Requests'> - retrying in 65 s\n")
        self.assertIsNone(ctl.bridge_login_error("s"))


if __name__ == "__main__":
    unittest.main()
