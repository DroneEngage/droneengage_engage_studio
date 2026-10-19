"""
de.cap/1 advert validator tests (TASK-PF-02 resources/event_patterns,
TASK-PF-01 action "claim").
Run: python3 -m unittest test_cap_schema -v   (from de_databus/python/)
"""
import copy
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from cap_schema import cap_match_event_pattern, cap_validate_advert

PF02_ADVERT = {
    "schema": "de.cap/1",
    "ns": "gpio",
    "module": "droneengage_rpi_gpio", "ver": "1.4.1",
    "actions": {
        "write": {"params": {"pin": {"type": ["string", "int"]},
                             "value": {"type": "int", "min": 0}}},
        "read":  {"params": {"pin": {"type": ["string", "int"]}}},
    },
    "resources": {"led": "out", "servo": "pwm", "btn": "in"},
    "event_patterns": {
        "*.went_on":  {"kind": "fact", "payload": {"v": {"type": "int"},
                                                  "src": {"type": "string"}}},
        "*.went_off": {"kind": "fact", "payload": {"v": {"type": "int"},
                                                   "src": {"type": "string"}}},
        "*.changed":  {"kind": "fact", "res": ["pwm"],
                       "payload": {"v": {"type": "int"},
                                   "src": {"type": "string"}}},
        "*.on":       {"kind": "command", "res": ["out", "pwm"]},
        "*.off":      {"kind": "command", "res": ["out", "pwm"]},
        "*.pulse":    {"kind": "command", "res": ["out"],
                       "payload": {"ms": {"type": "int", "optional": True,
                                          "min": 1, "max": 4000}}},
        "*.set":      {"kind": "command", "res": ["pwm"],
                       "payload": {"v": {"type": "int", "min": 0, "max": 1024}}},
    },
    "state": {"led": {"type": "int"}, "servo": {"type": "int"},
              "btn": {"type": "int"}},
}

PRE_PF02_ADVERT = {
    "schema": "de.cap/1",
    "ns": "camera",
    "module": "droneengage_camera", "ver": "3.2.1",
    "actions": {"capture": {"params": {"count": {"type": "int"}}}},
    "events": {"captured": {"payload": {"file": {"type": "string"}}}},
    "state": {"recording": {"type": "bool"}},
}


class TestEventPatternsValid(unittest.TestCase):

    def test_pf02_advert_validates(self):
        self.assertEqual(cap_validate_advert(copy.deepcopy(PF02_ADVERT)), "")

    def test_advert_with_only_new_keys_validates(self):
        advert = {"schema": "de.cap/1", "ns": "gpio", "module": "m",
                  "ver": "1", "resources": {"led": "out"},
                  "event_patterns": {"*.went_on": {"kind": "fact"}}}
        self.assertEqual(cap_validate_advert(advert), "")

    def test_pre_pf02_advert_unchanged(self):
        self.assertEqual(cap_validate_advert(copy.deepcopy(PRE_PF02_ADVERT)), "")

    def test_resources_shape(self):
        a = copy.deepcopy(PF02_ADVERT)
        a["resources"] = ["led"]
        self.assertNotEqual(cap_validate_advert(a), "")
        for bad in ({"Bad Key": "out"}, {"led": 7}, {"led": "Out"}):
            a = copy.deepcopy(PF02_ADVERT)
            a["resources"].update(bad)
            self.assertNotEqual(cap_validate_advert(a), "", f"{bad!r}")

    def test_event_pattern_keys(self):
        for bad_key in ("went_on", "*.*.*", "*.Went_on", "*." + "a" * 33):
            a = copy.deepcopy(PF02_ADVERT)
            a["event_patterns"][bad_key] = {}
            self.assertNotEqual(cap_validate_advert(a), "", bad_key)
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"] = ["*.went_on"]
        self.assertNotEqual(cap_validate_advert(a), "")
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"]["*.x"] = "fact"
        self.assertNotEqual(cap_validate_advert(a), "")

    def test_pattern_fields(self):
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"]["*.on"]["kind"] = "query"
        self.assertNotEqual(cap_validate_advert(a), "")
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"]["*.on"]["kind"] = 3
        self.assertNotEqual(cap_validate_advert(a), "")
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"]["*.on"]["res"] = "out"
        self.assertNotEqual(cap_validate_advert(a), "")
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"]["*.on"]["res"] = ["Out"]
        self.assertNotEqual(cap_validate_advert(a), "")
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"]["*.on"]["res"] = [3]
        self.assertNotEqual(cap_validate_advert(a), "")
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"]["*.went_on"]["desc"] = 1
        self.assertNotEqual(cap_validate_advert(a), "")
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"]["*.pulse"]["payload"]["ms"]["type"] = "frobnicate"
        self.assertNotEqual(cap_validate_advert(a), "")

    def test_dotted_events_key_still_rejected(self):
        # the pre-PF-02 fake stays rejected - this is why the keys had to
        # be top-level
        a = copy.deepcopy(PF02_ADVERT)
        a["events"] = {"*.went_on": {}}
        self.assertNotEqual(cap_validate_advert(a), "")
        a = copy.deepcopy(PF02_ADVERT)
        a["events"] = {"led.went_on": {}}
        self.assertNotEqual(cap_validate_advert(a), "")


class TestMatchEventPattern(unittest.TestCase):

    def setUp(self):
        self.advert = copy.deepcopy(PF02_ADVERT)

    def match(self, key, verb, kind, advert=None):
        return cap_match_event_pattern(advert or self.advert, key, verb, kind)

    def test_facts(self):
        self.assertIsNotNone(self.match("led", "went_on", "fact"))
        self.assertIsNotNone(self.match("btn", "went_on", "fact"))
        # "res" class whitelist: changed is pwm-only
        self.assertIsNotNone(self.match("servo", "changed", "fact"))
        self.assertIsNone(self.match("led", "changed", "fact"))

    def test_commands(self):
        self.assertIsNotNone(self.match("led", "on", "command"))
        self.assertIsNotNone(self.match("servo", "set", "command"))
        # an input resource takes no commands
        self.assertIsNone(self.match("btn", "on", "command"))
        self.assertIsNone(self.match("btn", "pulse", "command"))

    def test_kind_never_crosses(self):
        self.assertIsNone(self.match("led", "on", "fact"))
        self.assertIsNone(self.match("led", "went_on", "command"))

    def test_unknown_verb_and_key(self):
        self.assertIsNone(self.match("led", "went_onn", "fact"))
        self.assertIsNone(self.match("nosuch", "went_on", "fact"))

    def test_kind_defaults_to_fact(self):
        self.advert["event_patterns"]["*.flipped"] = {}
        self.assertIsNotNone(self.match("led", "flipped", "fact"))
        self.assertIsNone(self.match("led", "flipped", "command"))

    def test_no_patterns_no_match(self):
        self.assertIsNone(self.match("led", "went_on", "fact",
                                     copy.deepcopy(PRE_PF02_ADVERT)))

    def test_no_resources_key_unchecked(self):
        del self.advert["resources"]
        self.assertIsNotNone(self.match("anykey", "went_on", "fact"))
        self.assertIsNotNone(self.match("anykey", "on", "command"))
        self.assertIsNotNone(self.match("anykey", "set", "command"))

    def test_malformed_never_crashes(self):
        a = copy.deepcopy(PF02_ADVERT)
        a["event_patterns"] = 5
        self.assertIsNone(cap_match_event_pattern(a, "led", "went_on", "fact"))
        a = copy.deepcopy(PF02_ADVERT)
        a["resources"] = 5
        self.assertIsNone(cap_match_event_pattern(a, "led", "went_on", "fact"))


class TestActionClaim(unittest.TestCase):
    """TASK-PF-01: an action may carry "claim": "ns"|"action"."""

    def test_claim_spellings_accepted(self):
        a = copy.deepcopy(PF02_ADVERT)
        a["actions"]["write"]["claim"] = "ns"
        a["actions"]["read"]["claim"] = "action"
        self.assertEqual(cap_validate_advert(a), "")

    def test_bad_claim_rejected(self):
        for bad in ("pin", 1, True, None, ["action"], {"c": "action"}):
            a = copy.deepcopy(PF02_ADVERT)
            a["actions"]["read"]["claim"] = bad
            self.assertNotEqual(cap_validate_advert(a), "", repr(bad))

    def test_old_advert_unchanged(self):
        self.assertEqual(cap_validate_advert(copy.deepcopy(PRE_PF02_ADVERT)), "")


if __name__ == "__main__":
    unittest.main()
