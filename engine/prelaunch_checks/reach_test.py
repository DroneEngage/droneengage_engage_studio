"""reach_test.py - pure tests of the reach probes on loopback sockets."""

import os
import socket
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import prelaunch  # noqa: E402
from prelaunch_checks import reach  # noqa: E402


def _listener():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    s.listen(5)
    return s, s.getsockname()[1]


def _closed_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _ctx(services, role="local", account=True):
    t = {"name": "t", "role": role, "services": services}
    if account:
        t["account"] = {"user_env": "PL_U", "code_env": "PL_C"}
    return prelaunch.Ctx(t, "/tmp", "v")


class Ports(unittest.TestCase):
    def test_open(self):
        s, p = _listener()
        self.addCleanup(s.close)
        st, d = reach.ports(_ctx({"comm": {"host": "127.0.0.1", "port": p}}))
        self.assertEqual(st, "PASS", d)

    def test_blocked_port_is_fail_and_names_the_port(self):
        s, p = _listener()
        self.addCleanup(s.close)
        bad = _closed_port()
        st, d = reach.ports(_ctx({"auth": {"host": "127.0.0.1", "port": p},
                                  "comm": {"host": "127.0.0.1", "port": bad}}))
        self.assertEqual(st, "FAIL")
        self.assertIn(str(bad), d)
        self.assertIn("comm", d)

    def test_probe_on_the_server_is_refused(self):
        s, p = _listener()
        self.addCleanup(s.close)
        st, d = reach.ports(_ctx({"comm": {"host": "127.0.0.1", "port": p}},
                                 role="airgap"))
        self.assertEqual(st, "FAIL")
        self.assertIn("another host", d)


class PublicHost(unittest.TestCase):
    def test_stale_public_host_names_public_host(self):
        ctx = _ctx({"auth": {"host": "127.0.0.1", "port": 1}})
        bad = _closed_port()
        with mock.patch.object(reach, "_gcs_login",
                               return_value=("127.0.0.1", bad, "k")), \
                mock.patch.object(ctx, "account", return_value=("u", "c")):
            st, d = reach.public_host(ctx)
        self.assertEqual(st, "FAIL")
        self.assertIn("public_host", d)
        self.assertIn(str(bad), d)


class Login(unittest.TestCase):
    def test_missing_account_setting_is_skip_with_the_key_name(self):
        ctx = _ctx({"auth": {"host": "127.0.0.1", "port": 1}})
        with self.assertRaises(prelaunch.Skip) as e:
            reach.login(ctx)
        self.assertIn("PL_U", str(e.exception))

    def test_refused_account_is_a_clear_error_and_not_a_hang(self):
        s, p = _listener()
        self.addCleanup(s.close)
        ctx = _ctx({"auth": {"host": "127.0.0.1", "port": p}})
        with mock.patch.object(ctx, "account", return_value=("u", "c")), \
                mock.patch.object(reach, "_gcs_login",
                                  side_effect=RuntimeError("login refused")):
            st, d = reach.login(ctx)
        self.assertEqual(st, "FAIL")
        self.assertIn("test account", d)


class RelayControl(unittest.TestCase):
    def test_open_control_port_is_fail(self):
        s, p = _listener()
        self.addCleanup(s.close)
        ctx = _ctx({"relay": {"host": "h", "control_port": p}}, role="mirror")
        with mock.patch.object(reach, "tcp_state", return_value="open"), \
                mock.patch.object(reach, "is_this_host", return_value=False):
            st, d = reach.relay_control_closed(ctx)
        self.assertEqual(st, "FAIL")
        self.assertIn("comm servers", d)


if __name__ == "__main__":
    unittest.main()
