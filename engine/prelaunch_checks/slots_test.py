"""slots_test.py - collisions, a mirror wired to live, and unit config drift."""

import json
import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import prelaunch  # noqa: E402
from prelaunch_checks import reach, slots  # noqa: E402


def _ctx(**over):
    t = {"name": "m", "role": "mirror",
         "services": {"auth": {"host": "h", "port": 19500},
                      "comm": {"host": "h", "port": 9970},
                      "relay": {"host": "h", "control_port": 9101,
                                "udp_range": [41000, 41999]}},
         "other_slot": {"ports": [19408, 9966, 9100],
                        "relay_range": [40000, 40999]}}
    t.update(over)
    return prelaunch.Ctx(t, "/tmp", "v")


class Ports(unittest.TestCase):
    def test_clear_slots_pass(self):
        st, d = slots.ports(_ctx())
        self.assertEqual(st, "PASS", d)

    def test_shared_port_fails_and_names_it(self):
        st, d = slots.ports(_ctx(other_slot={"ports": [9970]}))
        self.assertEqual(st, "FAIL")
        self.assertIn("9970", d)
        self.assertIn("comm.port", d)

    def test_overlapping_relay_range_fails(self):
        st, d = slots.ports(_ctx(other_slot={
            "ports": [1], "relay_range": [41500, 42000]}))
        self.assertEqual(st, "FAIL")
        self.assertIn("overlaps", d)


class CommAuth(unittest.TestCase):
    def run_with(self, port):
        ctx = _ctx()
        with mock.patch.object(ctx, "account", return_value=("u", "c")), \
                mock.patch.object(reach, "_gcs_login",
                                  return_value=("h", port, "k")):
            return slots.comm_auth(ctx)

    def test_mirror_comm_passes(self):
        self.assertEqual(self.run_with(9970)[0], "PASS")

    def test_mirror_wired_to_live_fails(self):
        st, d = self.run_with(9966)
        self.assertEqual(st, "FAIL")
        self.assertIn("live comm", d)


class UnitConfig(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.d)

    def files(self, a, b):
        pa, pb = os.path.join(self.d, "a"), os.path.join(self.d, "b")
        json.dump(a, open(pa, "w"))
        json.dump(b, open(pb, "w"))
        return _ctx(test_unit={"config": pa, "production_config": pb})

    def test_only_auth_port_differs(self):
        st, d = slots.unit_config(self.files({"auth_port": 1, "x": 2},
                                             {"auth_port": 9, "x": 2}))
        self.assertEqual(st, "PASS", d)
        self.assertIn("only in auth_port", d)

    def test_other_difference_fails_and_names_it(self):
        st, d = slots.unit_config(self.files({"auth_port": 1, "x": 2},
                                             {"auth_port": 9, "x": 3}))
        self.assertEqual(st, "FAIL")
        self.assertIn("x", d)


if __name__ == "__main__":
    unittest.main()
