"""prelaunch_canary_test.py - diagnosis, notices and read-only behaviour."""

import json
import os
import socket
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import prelaunch  # noqa: E402
import prelaunch_canary as C  # noqa: E402


def _closed():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class Diagnose(unittest.TestCase):
    def ctx(self, port):
        return prelaunch.Ctx({"name": "t", "role": "live", "services": {
            "comm": {"host": "127.0.0.1", "port": port, "machine": "b"}}},
            "/tmp", "v")

    def test_listening_but_unreachable_is_a_firewall_fault(self):
        p = _closed()
        st, d = C.diagnose(self.ctx(p), {"b": {p}})
        self.assertEqual(st, "FAIL")
        self.assertIn("firewall fault", d)

    def test_not_listening_is_a_service_fault(self):
        p = _closed()
        st, d = C.diagnose(self.ctx(p), {"b": {22}})
        self.assertEqual(st, "FAIL")
        self.assertIn("service fault", d)


class Notices(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        p = mock.patch.object(C.simlib, "RESULTS_DIR", self.tmp.name)
        p.start()
        self.addCleanup(p.stop)
        self.sent = []
        n = mock.patch.object(C, "notify",
                              lambda ctx, lines: self.sent.append(lines))
        n.start()
        self.addCleanup(n.stop)

    def run_with(self, status, detail="x"):
        t = {"name": "live", "role": "live", "services": {}}
        with mock.patch("sys.stdout"):
            return C.run(t, checks={"c": lambda ctx: (status, detail)})

    def test_change_to_fail_notifies_once(self):
        self.assertEqual(self.run_with("PASS"), 0)
        self.assertEqual(self.sent, [])
        self.assertEqual(self.run_with("FAIL"), 1)
        self.assertEqual(len(self.sent), 1)
        self.run_with("FAIL")
        self.assertEqual(len(self.sent), 1)

    def test_fail_after_recovery_notifies_again(self):
        self.run_with("FAIL")
        self.run_with("PASS")
        self.run_with("FAIL")
        self.assertEqual(len(self.sent), 2)

    def test_certificate_warning_notifies_once(self):
        self.run_with("PASS", "WARNING a: 5 days left")
        self.run_with("PASS", "WARNING a: 5 days left")
        self.assertEqual(len(self.sent), 1)


class ReadOnly(unittest.TestCase):
    def test_module_has_no_write_commands(self):
        src = open(C.__file__).read()
        for bad in ("systemctl", "pm2 ", "ufw", "iptables", "reboot",
                    "ARM_DISARM", "rm -"):
            code = src.split('"""', 2)[2]
            self.assertNotIn(bad, code)

    def test_only_two_remote_commands_exist(self):
        src = open(C.__file__).read()
        self.assertEqual(sorted(set(
            __import__("re").findall(r'run_on\(host, f?"([a-z]+)', src))),
            ["cat", "ss"])


if __name__ == "__main__":
    unittest.main()
