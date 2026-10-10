#!/usr/bin/env python3
"""Offline unit tests for link_model.py - the fake-server link model
(SL-0007). Pure python, a virtual clock, no SITL:

    python3 engine/test_link_model.py

Covered (the task's acceptance):
  * Gilbert-Elliot occupancy over 100k ticks lands within 2% of
    p_gb / (p_gb + p_bg)
  * stall mode (GE bad state = hold) delivers EVERY message, in order
  * the bandwidth cap shapes throughput within 10% of the set rate
  * two same-seed runs produce identical counters
Plus the asymmetric knobs (loss_tx/loss_rx), the directed `stall <s>`
hold, profile presets, and clear().
"""
import os
import sys
import unittest

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "var", "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import link_model


def drain(ln, direction, now):
    """Release every due frame of a direction at `now`."""
    return ln.pop_ready(direction, now)


class TestGilbertElliot(unittest.TestCase):

    def test_occupancy(self):
        # 100k ticks: bad-state share ~ p_gb/(p_gb+p_bg) within 2 %
        for p_gb, p_bg in ((0.02, 0.30), (0.10, 0.10), (0.30, 0.05)):
            ln = link_model.Link("u", 1)
            ln.set_ge(p_gb, p_bg, 0.0, 100.0)
            ge = ln.dir["tx"].ge
            for _ in range(100_000):
                ge.tick()
            expected = p_gb / (p_gb + p_bg)
            got = ge.bad_ticks / ge.ticks
            self.assertAlmostEqual(got, expected, delta=0.02,
                                   msg=f"p_gb={p_gb} p_bg={p_bg}")

    def test_bad_state_drops(self):
        ln = link_model.Link("u", 1)
        ln.set_ge(1.0, 0.0, 0.0, 100.0)   # stuck in bad, drop mode
        for i in range(10):
            self.assertEqual(ln.offer("tx", 10, float(i), i), "drop")
        self.assertEqual(ln.dir["tx"].dropped, 10)

    def test_bad_state_stall_holds_then_flushes_in_order(self):
        ln = link_model.Link("u", 1)
        ln.set_ge(1.0, 0.0, 0.0, 0.0, "stall")   # stuck bad: hold all
        for i in range(20):
            self.assertEqual(ln.offer("tx", 10, float(i), i), "queued")
        # nothing releases while the chain is bad
        self.assertEqual(drain(ln, "tx", 100.0), [])
        self.assertEqual(ln.dir["tx"].held, 20)
        self.assertEqual(ln.dir["tx"].dropped, 0)
        # flip good: the next offer's tick unstalls and the backlog
        # drains in offer order - every message, none lost
        ln.set_ge(0.0, 1.0, 0.0, 0.0, "stall")   # stays good
        ln.offer("tx", 10, 100.0, 20)
        out = []
        t = 100.0
        for _ in range(100):
            out += drain(ln, "tx", t)
            t += 0.5
        self.assertEqual(out, list(range(21)))

    def test_directed_stall_holds_everything_in_order(self):
        ln = link_model.Link("u", 1)
        ln.stall(5.0, 0.0)
        for i in range(8):
            ln.offer("rx", 10, float(i) * 0.01, i)
        self.assertEqual(drain(ln, "rx", 4.9), [])
        self.assertEqual(drain(ln, "rx", 5.1), list(range(8)))
        self.assertEqual(ln.dir["rx"].held, 8)


class TestDirections(unittest.TestCase):

    def test_loss_rx_only(self):
        # uplink_dead: the unit's rx dies, its tx still flows
        ln = link_model.Link("u", 1)
        ln.apply_profile("uplink_dead", 0.0)
        self.assertEqual(ln.offer("tx", 10, 0.0, "up"), "queued")
        self.assertEqual(ln.offer("rx", 10, 0.0, "down"), "drop")
        self.assertEqual(ln.dir["rx"].dropped, 1)
        self.assertEqual(ln.dir["tx"].dropped, 0)

    def test_flat_loss_both_directions(self):
        ln = link_model.Link("u", 1)
        ln.loss = 100.0
        self.assertEqual(ln.offer("tx", 10, 0.0, 1), "drop")
        self.assertEqual(ln.offer("rx", 10, 0.0, 2), "drop")


class TestBandwidth(unittest.TestCase):

    def test_throughput_within_10_percent(self):
        # 1 kB/s cap, 100-byte frames at 10 ms intervals: releases ride
        # the serialization watermark - ~1 kB/s over the run
        ln = link_model.Link("u", 1)
        ln.bw_bps = 1000
        t = 0.0
        sent = []
        for i in range(200):
            ln.offer("rx", 100, t, i)
            sent += [(t_r, tag) for t_r in [t] for tag in
                     drain(ln, "rx", t)]
            t += 0.01
        # drain the tail
        end = t + 200 * 100 / 1000.0 + 1.0
        while t < end:
            sent += [(t, tag) for tag in drain(ln, "rx", t)]
            t += 0.01
        got = [tag for _, tag in sent]
        self.assertEqual(got, list(range(200)))       # order kept
        last_rel = ln.dir["rx"].last_rel
        # 200 frames x 100 B through a 1 kB/s pipe ~= 20 s of serialization
        self.assertAlmostEqual(last_rel, 200 * 100 / 1000.0,
                               delta=0.1 * 200 * 100 / 1000.0)

    def test_bw_delays_but_never_drops(self):
        ln = link_model.Link("u", 1)
        ln.bw_bps = 500
        for i in range(10):
            self.assertEqual(ln.offer("tx", 250, 0.0, i), "queued")
        t, out = 0.0, []
        while len(out) < 10 and t < 60:
            out += drain(ln, "tx", t)
            t += 0.05
        self.assertEqual(out, list(range(10)))
        self.assertEqual(ln.dir["tx"].dropped, 0)


class TestReproducibility(unittest.TestCase):

    def _run(self, seed):
        ln = link_model.Link("u", seed)
        ln.loss = 15.0
        ln.dir["rx"].loss = 40.0
        ln.delay_ms, ln.jitter_ms = 30, 90
        ln.set_ge(0.10, 0.30, 2.0, 60.0, "drop")
        t = 0.0
        released = []
        for i in range(500):
            ln.offer("tx", 64 + i % 37, t, i)
            ln.offer("rx", 64 + i % 37, t, i)
            t += 0.02
        while t < 200:
            released += [("tx", x) for x in drain(ln, "tx", t)]
            released += [("rx", x) for x in drain(ln, "rx", t)]
            t += 0.02
        return ({d: {k: getattr(ln.dir[d], k)
                     for k in ("dropped", "delayed", "held", "delivered")}
                 for d in link_model.DIRECTIONS}, released)

    def test_same_seed_same_counters(self):
        a, ra = self._run(1337)
        b, rb = self._run(1337)
        self.assertEqual(a, b)
        self.assertEqual(ra, rb)

    def test_different_seed_differs(self):
        a, _ = self._run(1337)
        b, _ = self._run(7331)
        self.assertNotEqual(a, b)


class TestKnobs(unittest.TestCase):

    def test_clear_purges_and_resets(self):
        ln = link_model.Link("u", 1)
        ln.apply_profile("lte_edge", 0.0)
        ln.stall(10.0, 0.0)
        for i in range(5):
            ln.offer("tx", 10, 0.0, i)
        ln.clear()
        self.assertEqual(ln.dir["tx"].purged, 5)
        self.assertEqual(ln.loss, 0.0)
        self.assertIsNone(ln.dir["tx"].ge)
        self.assertEqual(ln.offer("tx", 10, 0.0, 99), "queued")
        self.assertEqual(drain(ln, "tx", 0.01), [99])

    def test_profiles(self):
        for name in link_model.PROFILES:
            ln = link_model.Link("u", 1)
            ln.apply_profile(name, 0.0)     # must not raise
        with self.assertRaises(KeyError):
            link_model.Link("u", 1).apply_profile("nope", 0.0)

    def test_cut_drops_and_refuses_queue(self):
        ln = link_model.Link("u", 1)
        ln.cut(2.0, 0.0)
        self.assertEqual(ln.offer("tx", 10, 0.5, 1), "drop")
        self.assertEqual(ln.offer("tx", 10, 2.5, 2), "queued")
        self.assertEqual(drain(ln, "tx", 2.6), [2])

    def test_jitter_delays_within_bounds(self):
        ln = link_model.Link("u", 1)
        ln.delay_ms, ln.jitter_ms = 100, 200
        rels = []
        for i in range(50):
            ln.offer("rx", 10, 0.0, i)
            rels.append(ln.dir["rx"].queue[-1]["rel"])
        for r in rels:
            self.assertGreaterEqual(r, 0.1)
            self.assertLessEqual(r, 0.3 + 1e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
