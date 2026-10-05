#!/usr/bin/env python3
"""Offline unit tests for checks.py / scenario_runner.run_verify - the
assertion engine itself must be trustworthy. No SITL needed:

    python3 engine/test_checks.py
"""
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks
import simlib
import scenario_runner

U1 = {"name": "u1", "party_id": "p1", "fc": True}
U2 = {"name": "u2", "party_id": "p2", "fc": True}


def line(ts, kind, msg):
    return (ts, "e", "m", "0", kind, msg, "f")


def ctx(journals, **kw):
    """journals: {unit name: [lines]} served through simlib.journal_*"""
    c = {"manifest": {"name": "t", "units": [U1, U2], "server_procs": []},
         "marks": {}, "file": "/x/s.yaml", "results_dir": None}
    c.update(kw)
    return c, journals


def run(a, journals, **kw):
    c, j = ctx(journals, **kw)
    import re

    def jl(u, since=None):
        return list(j.get(u["name"], []))

    def jm(u, pat, since=None):
        rx = re.compile(pat)
        return [l for l in jl(u) if simlib.line_hit(l, rx)]
    with mock.patch.object(simlib, "journal_lines", jl), \
            mock.patch.object(simlib, "journal_match", jm), \
            mock.patch.object(checks.ctl, "resolve",
                              lambda tok, m=None: _resolve(tok, c)):
        return checks.check_assertion(a, c)


def _resolve(tok, c):
    units = c["manifest"]["units"]
    if tok == "all":
        return units
    return [u for u in units if u["name"] == tok]


class T(unittest.TestCase):
    def test_max_latency_needs_limit(self):
        j = {"u1": [line(1000, "A", "a")], "u2": [line(1500, "B", "b")]}
        r = run({"max_latency": {"u1": "u1", "re1": "a", "u2": "u2",
                                 "re2": "b"}}, j)
        self.assertEqual(r["status"], "FAIL")      # was a silent PASS

    def test_max_latency_is_signed(self):
        j = {"u1": [line(2000, "A", "a")], "u2": [line(1000, "B", "b")]}
        r = run({"max_latency": {"u1": "u1", "re1": "a", "u2": "u2",
                                 "re2": "b", "max_ms": 5000}}, j)
        self.assertEqual(r["status"], "FAIL")      # b came BEFORE a
        j = {"u1": [line(1000, "A", "a")], "u2": [line(1900, "B", "b")]}
        r = run({"max_latency": {"u1": "u1", "re1": "a", "u2": "u2",
                                 "re2": "b", "max_ms": 1000}}, j)
        self.assertEqual(r["status"], "PASS")

    def test_time_delta_ordered_and_raw_lines(self):
        j = {"u1": [line(5000, "A", "a")], "u2": [line(1000, "B", "b")]}
        spec = {"u1": "u1", "re1": "a", "u2": "u2", "re2": "b", "max_s": 10}
        self.assertEqual(run({"journal_time_delta": spec}, j)["status"],
                         "PASS")                  # spread, unsigned
        spec["ordered"] = True
        # ordered: dt is -4 s; max_s alone cannot catch a wrong order, so
        # an ordered check pairs with min_s: 0
        spec["min_s"] = 0
        self.assertEqual(run({"journal_time_delta": spec}, j)["status"],
                         "FAIL")
        # a RAW line (ts 0) is never a time reference
        j = {"u1": [(0, "-", "-", "-", "RAW", "a", "f")],
             "u2": [line(1000, "B", "b")]}
        self.assertEqual(run({"journal_time_delta": {
            "u1": "u1", "re1": "a", "u2": "u2", "re2": "b", "max_s": 5}},
            j)["status"], "FAIL")

    def test_clock_spread_needs_two_units(self):
        j = {"u1": [line(1, "CLOCK", "fleet_minus_unix_ms=5")]}
        r = run({"clock_spread": {"unit": "all"}}, j)
        self.assertEqual(r["status"], "FAIL")

    def test_journal_consistent(self):
        j = {"u1": [line(1, "WM", "WM_STATE t1 confirmed")],
             "u2": [line(1, "WM", "WM_STATE t2 confirmed")]}
        a = {"journal_consistent": {"unit": "all",
                                    "re": r"WM_STATE (\S+) confirmed"}}
        self.assertEqual(run(a, j)["status"], "FAIL")   # disagree
        j["u2"] = [line(1, "WM", "WM_STATE t1 confirmed")]
        self.assertEqual(run(a, j)["status"], "PASS")
        self.assertEqual(run(a, {})["status"], "FAIL")   # nothing at all

    def test_no_units_never_passes(self):
        c, j = ctx({})
        c["manifest"]["units"] = []
        with mock.patch.object(checks.ctl, "resolve",
                               lambda tok, m=None: []):
            r = checks.check_assertion(
                {"journal_count": {"unit": "all", "re": "x",
                                   "at_most": 0}}, c)
        self.assertEqual(r["status"], "FAIL")

    def test_bad_assertion_shapes_do_not_crash(self):
        c, _ = ctx({})
        self.assertEqual(checks.check_assertion(
            {"a": {}, "b": {}}, c)["status"], "FAIL")
        self.assertEqual(checks.check_assertion(
            {"journal_count": "oops"}, c)["status"], "FAIL")

    def test_truth_at_zero_longitude_and_tolerance(self):
        rows = [{"tid": "p", "t_ms": 1000, "lat": 0.0, "lng": 0.0}]
        self.assertIsNotNone(checks._truth_at(rows, "p", 1500))
        self.assertIsNone(checks._truth_at(rows, "p", 1000 + 60000))
        self.assertEqual(checks._lng(rows[0]), 0.0)

    def test_fc_mode_case_insensitive(self):
        class Tele:
            def latest(self, n):
                return {"mode": "GUIDED"}
        c, _ = ctx({}, telemetry=Tele())
        with mock.patch.object(checks.ctl, "resolve",
                               lambda tok, m=None: _resolve(tok, c)):
            r = checks.check_assertion(
                {"fc_mode": {"unit": "u1", "mode": "guided"}}, c)
        self.assertEqual(r["status"], "PASS")

    def test_stale_telemetry_skips(self):
        class Tele:
            def latest(self, n):
                return {"mode": "GUIDED"}

            def age_ms(self, n):
                return 60000
        c, _ = ctx({}, telemetry=Tele())
        with mock.patch.object(checks.ctl, "resolve",
                               lambda tok, m=None: _resolve(tok, c)):
            r = checks.check_assertion(
                {"fc_mode": {"unit": "u1", "mode": "GUIDED"}}, c)
        self.assertEqual(r["status"], "SKIP")

    def test_telemetry_value_pick(self):
        class Tele:
            def latest(self, n):
                return {"bat_v": 12.6}

            def rows(self, n):
                return [{"unit": n, "bat_v": 12.6},
                        {"unit": n, "bat_v": 10.4},
                        {"unit": n, "bat_v": 12.3}]

            def age_ms(self, n):
                return 0
        c, _ = ctx({}, telemetry=Tele())
        with mock.patch.object(checks.ctl, "resolve",
                               lambda tok, m=None: _resolve(tok, c)):
            # latest row already recovered past the sag -> FAIL
            r = checks.check_assertion(
                {"telemetry_value": {"unit": "u1", "field": "bat_v",
                                     "lt": 11.0}}, c)
            self.assertEqual(r["status"], "FAIL")
            # the recorded series still proves the dip happened
            r = checks.check_assertion(
                {"telemetry_value": {"unit": "u1", "field": "bat_v",
                                     "lt": 11.0, "pick": "min"}}, c)
            self.assertEqual(r["status"], "PASS")
            r = checks.check_assertion(
                {"telemetry_value": {"unit": "u1", "field": "bat_v",
                                     "lt": 11.0, "pick": "bogus"}}, c)
            self.assertEqual(r["status"], "FAIL")

    def test_speedup_is_median(self):
        import telemetry
        t = telemetry.Telemetry([], os.devnull)
        t._speeds["a"] = [1.0, 1.0, 9.0, 1.0, 1.0]
        self.assertEqual(t.speedup("a"), 1.0)
        self.assertIsNone(t.speedup("none"))

    def test_zombie_is_not_alive(self):
        self.assertTrue(checks._pid_running(os.getpid()))
        self.assertFalse(checks._pid_running(2 ** 22 + 12345))

    def test_fc_line_re_is_word_bounded(self):
        for bad in ("alarm raised", "swarm size 3", "warm start"):
            self.assertIsNone(checks.FC_LINE_RE.search(bad), bad)
        for good in ("arm requested", "disarm", "MISSION_ITEM sent"):
            self.assertTrue(checks.FC_LINE_RE.search(good), good)
        self.assertIsNone(__import__("re").search(
            checks.CAUSE_RE, "mission download done"))

    def test_run_verify_uses_json_and_exit_codes(self):
        op = {"x": 1}
        with tempfile.TemporaryDirectory() as d:
            cli = os.path.join(d, "v.js")
            with open(cli, "w"):
                pass
            scn = {"operation": os.path.join(d, "op.json"), "file": d,
                   "verify": {"cli": cli}}

            def fake(rc, out):
                return mock.Mock(returncode=rc, stdout=out, stderr="")
            errs = json.dumps([{"sev": "error", "rule": "LIV-001"},
                               {"sev": "warning", "rule": "W"}])
            with mock.patch.object(scenario_runner.subprocess, "run",
                                   return_value=fake(1, errs)) as m:
                # exit 1 + errors, but no expectation -> blocked
                self.assertFalse(scenario_runner.run_verify(scn, d))
                self.assertIn("--json", m.call_args[0][0])
                # negative scenario expecting exactly those errors:
                # the findings match (verify.json ok) but the gate still
                # blocks - a dirty op never reaches the fleet
                scn["verify"] = {"cli": cli, "expect_errors": 1,
                                 "expect_rules": ["LIV-001"]}
                self.assertFalse(scenario_runner.run_verify(scn, d))
            obj = json.dumps({"error_count": 1,
                              "errors": [{"rule": "bad"}]})
            with mock.patch.object(scenario_runner.subprocess, "run",
                                   return_value=fake(1, obj)):
                scn["verify"] = {"cli": cli, "expect_errors": 1,
                                 "expect_rules": ["bad"]}
                self.assertFalse(scenario_runner.run_verify(scn, d))
            with mock.patch.object(scenario_runner.subprocess, "run",
                                   return_value=fake(0, "not json")):
                scn["verify"] = {"cli": cli}
                self.assertFalse(scenario_runner.run_verify(scn, d))
            with mock.patch.object(scenario_runner.subprocess, "run",
                                   return_value=fake(2, "[]")):
                self.assertFalse(scenario_runner.run_verify(scn, d))
            with mock.patch.object(scenario_runner.subprocess, "run",
                                   return_value=fake(0, "[]")):
                self.assertTrue(scenario_runner.run_verify(scn, d))

    def test_cmp_applies_every_bound(self):
        self.assertEqual(checks._cmp({"at_least": 1, "at_most": 3}, 2)[0],
                         True)
        self.assertEqual(checks._cmp({"at_least": 1, "at_most": 3}, 5)[0],
                         False)
        self.assertEqual(checks._cmp({"gt": 0, "lt": 10}, 10)[0], False)
        self.assertEqual(checks._cmp({}, 0)[0], False)

    def test_link_offline_open_interval_needs_min_s(self):
        import time as _t
        now = int(_t.time() * 1000)
        c = {"manifest": {"name": "t", "units": [U1]},
             "link_events": {"p1": [(now - 2000, False)]}}
        with mock.patch.object(checks.ctl, "resolve",
                               lambda tok, m=None: [U1]):
            r = checks.check_assertion(
                {"link_offline": {"unit": "u1", "min_s": 30}}, c)
            self.assertEqual(r["value"], 0)    # open 2 s, not 30
            c["link_events"]["p1"] = [(now - 40000, False)]
            r = checks.check_assertion(
                {"link_offline": {"unit": "u1", "min_s": 30}}, c)
            self.assertEqual(r["value"], 1)

    def test_journal_lines_time_order_across_files(self):
        with tempfile.TemporaryDirectory() as d:
            u = {"name": "u1", "sim": "t"}
            jd = os.path.join(d, "journal")
            os.makedirs(jd)
            with mock.patch.object(simlib, "journal_dir", lambda _u: jd):
                a, b = os.path.join(jd, "a.log"), os.path.join(jd, "b.log")
                open(b, "w").write("1000 E m 0 NOTE first\n")
                open(a, "w").write("2000 E m 0 NOTE second\n")
                # a later append to the older file must not reorder events
                open(b, "a").write("3000 E m 0 NOTE third\n")
                msgs = [ln[5] for ln in simlib.journal_lines(u)]
                self.assertEqual(msgs, ["first", "second", "third"])
                # incremental reads: every line once, partial lines held
                offs = {}
                got = [ln[5] for ln in simlib.journal_read_new(u, offs)]
                self.assertEqual(got, ["first", "second", "third"])
                open(a, "a").write("4000 E m 0 NOTE fou")
                self.assertEqual(simlib.journal_read_new(u, offs), [])
                open(a, "a").write("rth\n")
                got = [ln[5] for ln in simlib.journal_read_new(u, offs)]
                self.assertEqual(got, ["fourth"])

    def test_wait_reply_survives_ring_trim(self):
        import operator_bot
        bot = operator_bot.OperatorBot(
            {"name": "t", "units": [], "server": {"type": "fake"}})
        for i in range(450):
            bot._on_frame({"mt": 1087, "sd": "x", "ms": {"i": i}})
        mark = bot.reply_mark()
        bot._on_frame({"mt": 1094, "sd": "u1", "ms": {"a": "aborted"}})
        hit = bot.wait_reply(lambda m: m.get("mt") == 1094, 1, since=mark)
        self.assertIsNotNone(hit)
        # an answer from before the mark is not an answer to this request
        mark = bot.reply_mark()
        self.assertIsNone(bot.wait_reply(lambda m: m.get("mt") == 1094,
                                         0.3, since=mark))

    def test_policy_decides_each_request_once(self):
        import operator_bot
        bot = operator_bot.OperatorBot(
            {"name": "t", "units": [], "server": {"type": "fake"}},
            policy={"approve": {"launch": {"after_s": 0}}})
        sent = []
        bot._submit = lambda coro: (sent.append(1), coro.close())
        bot._on_frame({"mt": 1087, "sd": "u1",
                       "ms": {"approvals": [{"req": "r1", "kind": "launch"}]}})
        for _ in range(5):
            bot._policy_once()
        self.assertEqual(len(sent), 1)

    def test_seed_slot_base_fits_camera_fleets(self):
        import scenario
        units = [{"block": "copter_camera"}, {"block": "copter_camera"}]
        for s in range(9):
            sb = scenario._seed_slot_base(s, units)
            self.assertLessEqual(64000 + 10 * (sb + 1), 65535)
        # a fleet that fits keeps the historical block
        self.assertEqual(scenario._seed_slot_base(7, [{"block": "copter"}]),
                         10 + 7 * 25)

    # -------------------------------------- P5-10 recall / time to find

    def _truth_file(self, rows):
        f = tempfile.NamedTemporaryFile("w", suffix=".jsonl",
                                         delete=False)
        for r in rows:
            f.write(json.dumps(r) + "\n")
        f.close()
        self.addCleanup(os.unlink, f.name)
        return f.name

    def _bot(self, targets):
        class Bot:
            def targets(self):
                return targets
        return Bot()

    def test_detection_recall_ai_fraction_and_dominant_why(self):
        rows = [
            {"tid": "pa", "cls": "person", "t_ms": 1000, "lat": -35.0,
             "lng": 149.0, "alt_m": 0.0},
            {"tid": "pb", "cls": "person", "t_ms": 1000, "lat": -35.1,
             "lng": 149.1, "alt_m": 0.0},
            {"type": "detection", "src": "ai", "unit": "u1",
             "tid": "pa", "cls": "person", "t_ms": 5000, "sent": True},
            {"type": "detection", "src": "ai", "unit": "u1",
             "tid": "pb", "cls": "person", "t_ms": 6000, "sent": False,
             "why": "occluded", "blockers": ["hut_1"]},
            {"type": "detection", "src": "ai", "unit": "u1",
             "tid": "pb", "cls": "person", "t_ms": 7000, "sent": False,
             "why": "occluded", "blockers": ["hut_1"]},
            {"type": "detection", "src": "ai", "unit": "u1",
             "tid": "pb", "cls": "person", "t_ms": 8000, "sent": False,
             "why": "miss"},
        ]
        f = self._truth_file(rows)
        a = {"detection_recall": {"truth": ["pa", "pb"], "src": "ai",
                                  "min": 0.5, "truth_file": f}}
        r = run(a, {})
        self.assertEqual(r["status"], "PASS")
        self.assertEqual(r["value"], 0.5)
        self.assertIn("missed pb (occluded)", r["detail"])
        # recall defaults to "found them all" when no bound is given
        self.assertEqual(run({"detection_recall": {
            "truth": ["pa", "pb"], "src": "ai", "truth_file": f}}, {}
        )["status"], "FAIL")
        # a row from another stub unit is not this one's evidence
        rows_u2 = rows + [
            {"type": "detection", "src": "ai", "unit": "u2",
             "tid": "pb", "cls": "person", "t_ms": 8000, "sent": True}]
        f2 = self._truth_file(rows_u2)
        r = run({"detection_recall": {"truth": "pb", "src": "ai",
                                       "unit": "u1", "truth_file": f2}}, {})
        self.assertEqual((r["status"], r["value"]), ("FAIL", 0.0))

    def test_detection_recall_negative_control_and_never_in_frame(self):
        rows = [
            {"tid": "ph", "cls": "person", "t_ms": 1000, "lat": -35.0,
             "lng": 149.0, "alt_m": 0.0},
            {"tid": "pz", "cls": "person", "t_ms": 1000, "lat": -35.2,
             "lng": 149.2, "alt_m": 0.0},
            {"type": "detection", "src": "ai", "tid": "ph",
             "cls": "person", "t_ms": 5000, "sent": False,
             "why": "occluded", "blockers": ["hut_1"]},
        ]
        f = self._truth_file(rows)
        r = run({"detection_recall": {"truth": "ph", "src": "ai",
                                       "max": 0, "truth_file": f}}, {})
        self.assertEqual(r["status"], "PASS")     # correctly stayed hidden
        self.assertIn("occluded", r["detail"])
        # pz was never in frame: no detection row at all
        r = run({"detection_recall": {"truth": "pz", "src": "ai",
                                       "max": 0, "truth_file": f}}, {})
        self.assertEqual(r["status"], "PASS")
        self.assertIn("no in-frame row", r["detail"])

    def test_detection_recall_by_class_and_sdr(self):
        rows = [
            {"tid": "pa", "cls": "person", "t_ms": 1000, "lat": -35.0,
             "lng": 149.0, "alt_m": 0.0},
            {"tid": "pc", "cls": "person", "t_ms": 1000, "lat": -35.1,
             "lng": 149.1, "alt_m": 0.0},
            {"tid": "f1", "cls": "rf", "t_ms": 1000, "lat": -35.0,
             "lng": 149.0, "alt_m": 0.0},
            {"type": "detection", "src": "ai", "tid": "pa",
             "cls": "person", "t_ms": 5000, "sent": True},
            {"type": "detection", "src": "sdr", "tid": "f1",
             "cls": "rf", "t_ms": 9000, "sent": True},
        ]
        f = self._truth_file(rows)
        r = run({"detection_recall": {"truth": "person", "src": "ai",
                                       "truth_file": f}}, {})
        self.assertEqual((r["status"], r["value"]), ("FAIL", 0.5))
        r = run({"detection_recall": {"truth": "rf", "src": "sdr",
                                       "truth_file": f}}, {})
        self.assertEqual((r["status"], r["value"]), ("PASS", 1.0))

    def test_detection_recall_bot_chain_match_m(self):
        rows = [
            {"tid": "pa", "cls": "person", "t_ms": 1000, "lat": -35.0,
             "lng": 149.0, "alt_m": 0.0},
            {"tid": "pb", "cls": "person", "t_ms": 1000, "lat": -35.1,
             "lng": 149.1, "alt_m": 0.0},
        ]
        f = self._truth_file(rows)
        # a person target 10 m of pa's truth: the whole chain found pa
        bot = self._bot({"t1": {"cls": "person",
                                "pos": [-35.0, 149.00009], "t_ms": 9000}})
        r = run({"detection_recall": {"truth": ["pa", "pb"],
                                      "truth_file": f}}, {},
                bot=bot, world_epoch_ms=1000)
        self.assertEqual((r["status"], r["value"]), ("FAIL", 0.5))
        # the same target is 10+ km from pb - match_m is per truth entity
        # ... and a phone target never matches a person truth
        bot2 = self._bot({"t1": {"cls": "person",
                                 "pos": [-35.0, 149.00009], "t_ms": 9000},
                          "t2": {"cls": "phone",
                                 "pos": [-35.1, 149.1], "t_ms": 9000}})
        r = run({"detection_recall": {"truth": ["pa", "pb"],
                                      "truth_file": f}}, {}, bot=bot2,
                world_epoch_ms=1000)
        self.assertEqual(r["value"], 0.5)
        # no operator session -> SKIP, not a silent pass
        self.assertEqual(run({"detection_recall": {
            "truth": "pa", "truth_file": f}}, {})["status"], "SKIP")
        # bad src fails loud
        self.assertEqual(run({"detection_recall": {
            "truth": "pa", "src": "nope", "truth_file": f}}, {},
            bot=bot)["status"], "FAIL")

    def test_time_to_detect_from_the_epoch(self):
        rows = [
            {"tid": "pa", "cls": "person", "t_ms": 1000, "lat": -35.0,
             "lng": 149.0, "alt_m": 0.0},
            {"type": "detection", "src": "ai", "tid": "pa",
             "cls": "person", "t_ms": 6000, "sent": True},
            {"type": "detection", "src": "sdr", "tid": "pa",
             "cls": "person", "t_ms": 4000, "sent": True},
        ]
        f = self._truth_file(rows)
        r = run({"time_to_detect": {"truth": "pa", "src": "ai",
                                     "max_s": 10, "truth_file": f}}, {},
                world_epoch_ms=1000)
        self.assertEqual((r["status"], r["value"], r["detail"][:3]),
                         ("PASS", 5.0, "pa "))
        r = run({"time_to_detect": {"truth": "pa", "src": "ai",
                                     "max_s": 3, "truth_file": f}}, {},
                world_epoch_ms=1000)
        self.assertEqual(r["status"], "FAIL")
        # never found fails even with a generous budget
        r = run({"time_to_detect": {"truth": "pa", "src": "ai",
                                     "max_s": 99,
                                     "truth_file": self._truth_file(
                                         [rows[0]])}}, {},
                world_epoch_ms=1000)
        self.assertEqual(r["status"], "FAIL")
        # needs max_s / one entity / the epoch
        self.assertIn("needs max_s", run({"time_to_detect": {
            "truth": "pa", "truth_file": f}}, {},
            world_epoch_ms=1000)["detail"])
        self.assertIn("exactly one entity", run({"time_to_detect": {
            "truth": ["pa", "pb"], "src": "ai", "max_s": 10,
            "truth_file": f}}, {}, world_epoch_ms=1000)["detail"])
        self.assertEqual(run({"time_to_detect": {
            "truth": "pa", "max_s": 10, "truth_file": f}}, {})["status"],
            "SKIP")                     # no world epoch
        # realtime: true scenarios SKIP it when speed-up is off
        self.assertEqual(run({"time_to_detect": {
            "truth": "pa", "max_s": 10, "truth_file": f}}, {},
            world_epoch_ms=1000, realtime_required=True,
            realtime_ok=False)["status"], "SKIP")


if __name__ == "__main__":
    unittest.main()
