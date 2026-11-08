"""relay_check_registered_test.py - DE-0128: the UDP proxy relay checks
from SL-0194 stay registered in prelaunch/features.yaml.

A probe in engine/prelaunch_checks that no check row references gives a
false feeling of cover. This test fails when relay.mavlink_both_ways or
relay.range_edges is removed from the register, from the udp_proxy_qgc
feature, or loses its working impl.

Run: python3 tests/prelaunch/relay_check_registered_test.py
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "prelaunch"))
sys.path.insert(0, os.path.join(ROOT, "engine"))

import check_features  # noqa: E402
from prelaunch_checks import relay  # noqa: E402

FEATURE = "udp_proxy_qgc"
IDS = ("relay.mavlink_both_ways", "relay.range_edges")
IMPLS = {"relay.mavlink_both_ways": "mavlink_both_ways",
         "relay.range_edges": "range_edges"}


class Registered(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.reg = check_features.load()

    def test_register_is_valid(self):
        problems = check_features.validate(self.reg, ROOT)
        self.assertEqual(problems, [], "features.yaml problems: " + "; ".join(problems))

    def test_relay_checks_in_checks_map(self):
        checks = self.reg.get("checks") or {}
        for cid in IDS:
            self.assertIn(cid, checks, f"{cid} is not registered in checks:")
            self.assertEqual(checks[cid].get("kind"), "probe")

    def test_relay_checks_in_feature(self):
        feats = {f.get("id"): f for f in self.reg.get("features") or []}
        self.assertIn(FEATURE, feats)
        for cid in IDS:
            self.assertIn(cid, feats[FEATURE].get("checks") or [],
                          f"{cid} is not wired into feature {FEATURE}")

    def test_impl_resolves_to_a_function(self):
        checks = self.reg.get("checks") or {}
        for cid, func in IMPLS.items():
            impl = (checks.get(cid) or {}).get("impl", "")
            self.assertEqual(impl, f"prelaunch_checks.relay:{func}",
                             f"{cid} impl is {impl!r}, not the SL-0194 probe")
            self.assertTrue(callable(getattr(relay, func, None)),
                            f"prelaunch_checks.relay has no function {func}")


if __name__ == "__main__":
    unittest.main()
