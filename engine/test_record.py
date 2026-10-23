#!/usr/bin/env python3
"""Offline unit tests for record.py - the session.jsonl -> de.scenario/1
mapping. No sim needed: _step_for/_group_deploys are pure, finalize is
fed a synthetic session file through a patched session_path.

    python3 engine/test_record.py
"""
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import record

MANIFEST = {
    "name": "t_sim",
    "sim_file": "/x/sims/t_sim.yaml",
    "units": [{"name": "c1", "party_id": "copter1", "block": "copter"},
              {"name": "c2", "party_id": "copter2", "block": "copter"}],
    # record() only reads manifest fields - a pool port (a prefer hint
    # that moved, SL-0076) and its flat lease map, not a formula value
    "server": {"type": "real", "port": 20431, "gcs_party": "GCS_FLEET"},
    "ports": {"server.wss": {"port": 20431, "proto": "tcp"},
              "server.ctrl": {"port": 20432, "proto": "tcp"}},
}

T0 = 1_000_000


def row(t, mt, sd="GCSWEB_x", ty="i", tg="copter1", ms=None):
    return {"t_ms": T0 + int(t * 1000), "sd": sd, "ty": ty, "tg": tg,
            "mt": mt, "ms": ms or {}}


class TestStepFor(unittest.TestCase):

    def st(self, mt, ms=None, tg="copter1", ty="i"):
        return record._step_for(
            MANIFEST, {"mt": mt, "ty": ty, "tg": tg, "ms": ms or {}})

    def test_arm_disarm(self):
        self.assertEqual(self.st(1030, {"A": True, "D": False}),
                         {"do": "arm", "unit": "c1"})
        self.assertEqual(self.st(1030, {"A": False, "D": True}),
                         {"do": "disarm", "unit": "c1"})

    def test_takeoff_and_land(self):
        # 1031 ChangeAltitude {a: alt} is the takeoff/climb command
        self.assertEqual(self.st(1031, {"a": 10}),
                         {"do": "takeoff", "alt": 10, "unit": "c1"})
        self.assertEqual(self.st(1032),
                         {"do": "land", "unit": "c1"})

    def test_flight_control_modes(self):
        self.assertEqual(self.st(1010, {"F": 9}),
                         {"do": "mode", "mode": "GUIDED", "unit": "c1"})
        self.assertEqual(self.st(1010, {"F": 22}),
                         {"do": "takeoff", "unit": "c1"})
        self.assertEqual(self.st(1010, {"F": 12}),
                         {"do": "land", "unit": "c1"})
        self.assertIsNone(self.st(1010, {"F": 9999}))

    def test_supervisor_actions(self):
        self.assertEqual(self.st(1086, {"a": "start"}), {"do": "start"})
        self.assertEqual(self.st(1086, {"a": "stop", "mission": "m1"}),
                         {"do": "stop", "mission": "m1"})
        self.assertEqual(
            self.st(1086, {"a": "pause", "hold": True}),
            {"do": "operator", "act": "pause", "hold": True})
        self.assertEqual(self.st(1086, {"a": "status"}), "noise")
        self.assertIsNone(self.st(1086, {"a": "bogus"}))

    def test_op_control(self):
        self.assertEqual(self.st(1094, {"a": "prepare"}), "prepare")
        st = self.st(1094, {"a": "start", "run": "r1",
                            "t_start": 123, "in_ms": 500})
        self.assertEqual(st, {"do": "start", "in_ms": 500})
        self.assertEqual(self.st(1094, {"a": "abort", "why": "x"}),
                         {"do": "abort", "why": "x"})
        self.assertEqual(self.st(1094, {"a": "status"}), "noise")

    def test_events(self):
        self.assertEqual(self.st(1061, {"d": "op.go"}),
                         {"do": "event", "event": "op.go"})
        self.assertEqual(self.st(1086, {"a": "fire", "event": "e1"}),
                         {"do": "fire", "event": "e1", "unit": "c1"})

    def test_polls_are_noise(self):
        for mt in (1004, 9005, 9015):
            self.assertEqual(self.st(mt), "noise")
        # 1005 reads + the message-pull convention (C >= 600 requests
        # message type C - seen live: C:1004/1022/1029/1071)
        for c in (500, 501, 505, 507, 508, 1004, 1022, 1029, 1071):
            self.assertEqual(self.st(1005, {"C": c}), "noise")
        self.assertEqual(self.st(1089, {"r": True}), "noise")

    def test_actionable_send(self):
        st = self.st(1040, {"s": 3.5})
        self.assertEqual(st["do"], "send")
        self.assertEqual(st["mt"], 1040)
        self.assertEqual(st["tg"], "copter1")
        # a RemoteExecute command (not a read) replays verbatim
        st = self.st(1005, {"C": 102})
        self.assertEqual(st["do"], "send")
        self.assertEqual(st["ms"], {"C": 102})

    def test_unmapped(self):
        self.assertIsNone(self.st(1076, {"a": 4}))
        self.assertIsNone(self.st(9999, {"x": 1}))


class TestGroupDeploys(unittest.TestCase):

    def test_prepares_share_run(self):
        rows = [
            row(1, 1094, tg="copter1",
                ms={"a": "prepare", "op": "demo", "run": "r9",
                    "oh": "h1", "pkg": "PKG1",
                    "bind": {"copter1": "lead", "copter2": "wing"}}),
            row(2, 1094, tg="copter2",
                ms={"a": "prepare", "op": "demo", "run": "r9",
                    "oh": "h1", "pkg": "PKG1",
                    "bind": {"copter1": "lead", "copter2": "wing"}}),
            row(3, 1030, ms={"A": True}),
        ]
        deploys, rest = record._group_deploys(rows)
        self.assertEqual(len(deploys), 1)
        d = deploys[0]
        self.assertEqual((d["kind"], d["op"], d["run"]), ("op", "demo", "r9"))
        self.assertEqual(d["roles"]["lead"]["units"], ["copter1"])
        self.assertEqual(d["roles"]["wing"]["units"], ["copter2"])
        self.assertEqual(len(rest), 1)

    def test_pkg_uploads_share_hash(self):
        rows = [row(1, 1085, tg="copter1",
                    ms={"hash": "h", "pkg": "P"}),
                row(2, 1085, tg="copter2",
                    ms={"hash": "h", "pkg": "P"})]
        deploys, rest = record._group_deploys(rows)
        self.assertEqual(len(deploys), 1)
        self.assertEqual(deploys[0]["units"], ["copter1", "copter2"])
        self.assertEqual(rest, [])


class TestFinalize(unittest.TestCase):

    def _run(self, rows):
        tmp = tempfile.mkdtemp()
        sess = os.path.join(tmp, "session.jsonl")
        with open(sess, "w") as fh:
            for r in rows:
                fh.write(json.dumps(r) + "\n")
        out = os.path.join(tmp, "out.yaml")
        with mock.patch.object(record, "session_path",
                               return_value=sess):
            n = record.finalize(MANIFEST, T0, out)
        with open(out) as fh:
            text = fh.read()
        return n, text, tmp

    def test_human_session_maps(self):
        n, text, tmp = self._run([
            row(0, 9015, ty="s", tg=None),          # server ping - not gcs
            row(1, 1030, ms={"A": True}),
            row(2, 1010, ms={"F": 9}),
            row(3, 1031, ms={"a": 10}),
            row(4, 1032),
            row(5, 1004, ty="g", tg=None),          # ping - noise
            row(6, 1076, ms={"a": 4}),              # no verb - comment
            row(7, 6502, sd="copter1"),             # unit row - filtered
        ])
        self.assertIn("do: arm", text)
        self.assertIn("do: mode, mode: GUIDED", text)
        self.assertIn("do: takeoff, alt: 10", text)
        self.assertIn("do: land", text)
        self.assertIn("# unmapped: 1076", text)
        self.assertNotIn("1005", text)
        self.assertNotIn("6502", text)
        self.assertEqual(n[1], 1)                  # one unmapped
        self.assertTrue(os.path.isfile(
            os.path.join(tmp, "out.md")))          # runbook stub

    def test_filters_non_gcs_senders(self):
        # the bridge, the operator bot and the units are not the human
        n, text, _ = self._run([
            row(1, 1030, sd="GCS_FLEET", ms={"A": True}),
            row(1, 1030, sd="GCS_OPERATOR", ms={"A": True}),
            row(1, 1030, sd="copter1", ms={"A": True}),
            row(1, 1030, sd="GCSWEB_x", ms={"A": True}),
        ])
        self.assertEqual(text.count("do: arm"), 1)

    def test_op_deploy_writes_sidecars(self):
        n, text, tmp = self._run([
            row(1, 1094, tg="copter1",
                ms={"a": "prepare", "op": "demo", "run": "r",
                    "oh": "h", "pkg": "PKGTXT",
                    "bind": {"copter1": "lead"}}),
            row(2, 1094, ms={"a": "start", "run": "r"}),
        ])
        self.assertIn("do: deploy", text)
        self.assertIn("do: start", text)
        self.assertIn("operation: out.deploy.json", text)
        with open(os.path.join(tmp, "out.deploy.json")) as fh:
            spec = json.load(fh)
        self.assertEqual(spec["roles"]["lead"]["units"], ["c1"])
        self.assertTrue(os.path.isfile(
            os.path.join(tmp, "out.lead.pkg.json")))


if __name__ == "__main__":
    unittest.main(verbosity=2)
