#!/usr/bin/env python3
"""Offline unit tests for the fcb_proxy `rate` rule - the SL-0101
UART-style token bucket that paces the de_mavlink<->SITL link at
baud/10 bytes a second per direction. Pure python, a virtual clock,
no SITL:

    python3 engine/test_fcb_rate.py

Covered (the task's acceptance):
  * 10 000 bytes at rate 9600 take ~10.4 s (+- 10 %)
  * frames are never split: every sendall carries exactly one frame
  * a frame larger than one 10 ms quantum still leaves whole (its debt
    is its airtime on a real UART)
  * `rate 0` / `clear` remove the rule; stats reports rate + delayed
"""
import os
import sys
import unittest

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "var", "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fcb_proxy


class VClock:
    """Virtual clock - the tests advance it, nothing sleeps for real."""
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class _Sink:
    """A socket stand-in that records each sendall chunk."""
    def __init__(self):
        self.chunks = []

    def sendall(self, b):
        self.chunks.append(bytes(b))


def _v2_frame(payload_len):
    """A well-formed MAVLink v2 frame: 12 + payload_len bytes total."""
    return bytes([0xFD, payload_len, 0x00]) + bytes(9 + payload_len)


def _proxy(clock):
    p = fcb_proxy.FcbProxy(0, "127.0.0.1", 0, 0, "t1", 1)
    for d in ("c2s", "s2c"):
        p.buckets[d] = fcb_proxy.RateBucket(clock)
    return p


class TestRateBucket(unittest.TestCase):

    def test_10k_at_9600_takes_about_10_4s(self):
        # 9600 baud -> 960 B/s; 250 x 40 B frames -> 10000/960 = 10.42 s
        clock = VClock()
        p = _proxy(clock)
        p.buckets["s2c"].set_bps(9600 / 10.0)
        pending = bytearray().join(
            [_v2_frame(28) for _ in range(250)])
        sink = _Sink()
        while pending:
            self.assertTrue(p._drain("s2c", pending, sink))
            if pending:
                clock.t += 0.001
        self.assertAlmostEqual(clock.t, 10.417, delta=1.04)
        self.assertEqual(sum(len(c) for c in sink.chunks), 10000)

    def test_frames_never_split(self):
        clock = VClock()
        p = _proxy(clock)
        p.buckets["s2c"].set_bps(960.0)      # quantum = 9.6 B < frame
        pending = bytearray().join(
            [_v2_frame(28) for _ in range(20)])
        sink = _Sink()
        guard = 0
        while pending and guard < 100000:
            p._drain("s2c", pending, sink)
            if pending:
                clock.t += 0.001
            guard += 1
        self.assertFalse(pending)
        # every sendall carried exactly one whole 40 B frame
        self.assertTrue(all(len(c) == 40 for c in sink.chunks))
        self.assertEqual(len(sink.chunks), 20)

    def test_big_frame_goes_whole_on_debt(self):
        # a 267 B frame at 9600 baud dwarfs the 9.6 B quantum - it must
        # still leave in ONE sendall, its cost recovered over ~0.27 s
        clock = VClock()
        p = _proxy(clock)
        p.buckets["c2s"].set_bps(960.0)
        pending = bytearray(_v2_frame(255))
        sink = _Sink()
        self.assertTrue(p._drain("c2s", pending, sink))
        self.assertEqual([len(c) for c in sink.chunks], [267])
        # the next frame waits for the debt to refill
        pending += _v2_frame(28)
        self.assertTrue(p._drain("c2s", pending, sink))
        self.assertEqual(len(sink.chunks), 1)   # still waiting
        clock.t += 0.5
        self.assertTrue(p._drain("c2s", pending, sink))
        self.assertFalse(pending)

    def test_unlimited_passes_everything_at_once(self):
        p = _proxy(VClock())
        pending = bytearray().join(
            [_v2_frame(28) for _ in range(10)])
        sink = _Sink()
        self.assertTrue(p._drain("s2c", pending, sink))
        self.assertFalse(pending)
        self.assertEqual(sum(len(c) for c in sink.chunks), 400)

    def test_delayed_counts_bytes_that_waited(self):
        clock = VClock()
        p = _proxy(clock)
        p.buckets["s2c"].set_bps(960.0)
        pending = bytearray().join(
            [_v2_frame(28) for _ in range(3)])
        p._drain("s2c", pending, _Sink())
        # first 9.6 B quantum buys nothing (frame = 40 B): the head
        # frame's 40 bytes were delayed
        self.assertEqual(p.stats["s2c"]["delayed"], 40)


class TestControlCommands(unittest.TestCase):

    def test_rate_clear_and_stats(self):
        p = _proxy(VClock())
        self.assertEqual(p._cmd("rate 9600"), "rate 9600")
        self.assertEqual(p.buckets["c2s"].bps, 960.0)
        self.assertEqual(p.buckets["s2c"].bps, 960.0)
        stats = p._cmd("stats")
        self.assertIn("rate=9600baud", stats)
        self.assertIn("delayed=", stats)
        self.assertEqual(p._cmd("rate 0"), "rate 0")
        self.assertEqual(p.buckets["c2s"].bps, 0.0)
        p._cmd("rate 57600")
        self.assertEqual(p._cmd("clear"), "clear")
        self.assertEqual(p.buckets["s2c"].bps, 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
