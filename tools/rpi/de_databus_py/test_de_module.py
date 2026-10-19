"""
de_databus python helper tests - no bus needed (sendJMSG is faked).
Run: python3 -m unittest test_de_module -v   (from de_databus/python/)
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from de_module import CModule
from messages import TYPE_AndruavMessage_Sync_EventFire


class TestFireEvent(unittest.TestCase):

    def _capture(self):
        """Swap CModule.sendJMSG for a recorder; returns (module, sent)."""
        module = CModule()
        sent = []
        original = module.sendJMSG
        module.sendJMSG = lambda target, jmsg, mid, internal: \
            sent.append((target, jmsg, mid, internal))
        self.addCleanup(setattr, module, "sendJMSG", original)
        return module, sent

    def test_fire_event_returns_the_nonce_it_sent(self):
        module, sent = self._capture()
        nonce = module.fire_event("sound", "beep.started",
                                  {"src": "event", "loop": False})
        self.assertEqual(len(sent), 1)
        _, jmsg, mid, internal = sent[0]
        self.assertEqual(mid, TYPE_AndruavMessage_Sync_EventFire)
        self.assertTrue(internal)
        self.assertEqual(jmsg["d"], "sound.beep.started")
        self.assertEqual(jmsg["ed"], {"src": "event", "loop": False})
        self.assertIsInstance(nonce, str)
        self.assertEqual(nonce, jmsg["n"])

    def test_fire_event_nonces_are_unique(self):
        module, sent = self._capture()
        first = module.fire_event("sound", "a.finished", {})
        second = module.fire_event("sound", "b.finished", {})
        self.assertNotEqual(first, second)
        self.assertEqual([s[1]["n"] for s in sent], [first, second])


if __name__ == "__main__":
    unittest.main()
