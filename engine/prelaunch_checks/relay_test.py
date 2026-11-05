"""relay_test.py - the relay probes on real loopback sockets. A bound socket
that never answers acts the same as a firewall that drops packets, because
the probe sees silence in both cases."""

import os
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import prelaunch  # noqa: E402
from prelaunch_checks import relay  # noqa: E402


def _free_udp_block(n):
    """A start port with n free UDP ports in a row."""
    for base in range(47000, 49000, n + 1):
        held = []
        try:
            for p in range(base, base + n):
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.bind(("127.0.0.1", p))
                held.append(s)
            return base
        except OSError:
            continue
        finally:
            for s in held:
                s.close()
    raise RuntimeError("no free block")


def _ctx(lo, hi):
    return prelaunch.Ctx({"name": "t", "role": "local", "services": {
        "relay": {"host": "127.0.0.1", "udp_range": [lo, hi]}}}, "/tmp", "v")


class Edges(unittest.TestCase):
    def blackhole(self, ports):
        """Bind these ports silently: the same result as a firewall drop."""
        for p in ports:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.bind(("127.0.0.1", p))
            self.addCleanup(s.close)

    def test_open_range_passes(self):
        lo = _free_udp_block(20)
        st, d = relay.range_edges(_ctx(lo, lo + 19))
        self.assertEqual(st, "PASS", d)

    def test_whole_range_blocked_fails(self):
        lo = _free_udp_block(8)
        self.blackhole(range(lo, lo + 8))
        st, d = relay.range_edges(_ctx(lo, lo + 7))
        self.assertEqual(st, "FAIL")
        self.assertIn("firewall", d)

    def test_upper_part_blocked_fails_and_names_the_high_end(self):
        lo = _free_udp_block(20)
        self.blackhole(range(lo + 10, lo + 20))
        st, d = relay.range_edges(_ctx(lo, lo + 19))
        self.assertEqual(st, "FAIL")
        self.assertIn("high end", d)
        self.assertNotIn("low end", d)

    def test_one_busy_port_at_an_end_is_not_a_fault(self):
        lo = _free_udp_block(20)
        self.blackhole([lo])
        st, d = relay.range_edges(_ctx(lo, lo + 19))
        self.assertEqual(st, "PASS", d)


class Judge(unittest.TestCase):
    def test_no_heartbeat_says_blocked(self):
        st, d = relay.judge_exchange("1.2.3.4", 40000, {"heartbeats": 0,
                                                        "ack": None}, "pi")
        self.assertEqual(st, "FAIL")
        self.assertIn("not reachable", d)
        self.assertIn("pi", d)
        self.assertIn("1.2.3.4:40000", d)

    def test_no_ack_says_gcs_to_fc_is_broken(self):
        st, d = relay.judge_exchange("h", 1, {"heartbeats": 20, "ack": None},
                                     "pi")
        self.assertEqual(st, "FAIL")
        self.assertIn("COMMAND_ACK", d)

    def test_good(self):
        st, _ = relay.judge_exchange("h", 1, {"heartbeats": 20, "ack": 0}, "pi")
        self.assertEqual(st, "PASS")


class BlockedRelay(unittest.TestCase):
    def test_silent_relay_port_gives_no_heartbeat(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        self.addCleanup(s.close)
        res = relay.gcs_exchange("127.0.0.1", s.getsockname()[1], seconds=1.5)
        self.assertEqual(res["heartbeats"], 0)


if __name__ == "__main__":
    unittest.main()
