#!/usr/bin/env python3
"""Offline unit tests for checks.py / scenario_runner.run_verify - the
assertion engine itself must be trustworthy. No SITL needed:

    python3 engine/test_checks.py
"""
import json
import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

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

    def test_decision_resent_while_request_pends(self):
        """js_approvalQueue.js parity: a lost 1090 is resent every
        APPROVAL_RESEND_S (fresh envelope) until the request leaves the
        unit's 1087."""
        import asyncio
        import operator_bot
        bot = operator_bot.OperatorBot(
            {"name": "t", "units": [], "server": {"type": "fake"}})
        wire = []

        class Client:
            async def send(self, msg):
                wire.append(msg)

            def fleet_ms(self):
                return 0
        bot._client = Client()
        bot._submit = lambda coro: asyncio.run(coro)
        bot._on_frame({"mt": 1087, "sd": "u1", "ms": {"run": "R",
                       "approvals": [{"req": "r1", "kind": "action"}]}})
        asyncio.run(bot._decide_async("u1", "r1", "action", "approve", 0))
        self.assertEqual(len(wire), 1)
        t0 = bot._answers[("u1", "r1")]["last"]
        bot._resend_answers(t0 + 1.0)
        self.assertEqual(len(wire), 1)
        bot._resend_answers(t0 + bot.APPROVAL_RESEND_S)
        self.assertEqual(len(wire), 2)
        a, b = wire[0]["ms"], wire[1]["ms"]
        self.assertEqual(b["p"], a["p"])
        self.assertNotEqual(b["id"], a["id"])
        self.assertGreater(b["seq"], a["seq"])
        # the vehicle consumed it - no more resends
        bot._on_frame({"mt": 1087, "sd": "u1",
                       "ms": {"run": "R", "approvals": []}})
        bot._resend_answers(t0 + 100)
        self.assertEqual(len(wire), 2)
        self.assertNotIn(("u1", "r1"), bot._answers)

    def test_policy_launch_none_never_answers(self):
        """P4F-07: approve.launch: none is the 'no operator' posture -
        the request pends forever, other kinds still get their rules."""
        import operator_bot
        bot = operator_bot.OperatorBot(
            {"name": "t", "units": [], "server": {"type": "fake"}},
            policy={"approve": {"launch": "none",
                                "action": {"after_s": 0}}})
        sent = []
        bot._submit = lambda coro: (sent.append(1), coro.close())
        bot._on_frame({"mt": 1087, "sd": "u1", "ms": {"approvals": [
            {"req": "r1", "kind": "launch"},
            {"req": "r2", "kind": "action"}]}})
        for _ in range(5):
            bot._policy_once()
        # only the action request was decided - the launch one pends
        self.assertEqual(len(sent), 1)
        self.assertEqual(sorted(bot.approvals()["u1"]),
                         ["r1", "r2"])

    def test_no_journal(self):
        j = {"u1": [line(1, "A", "LAUNCH_GO")], "u2": []}
        r = run({"no_journal": {"unit": "all",
                                "re": "APPROVAL_REQ launch"}}, j)
        self.assertEqual((r["status"], r["value"]), ("PASS", 0))
        j["u2"].append(line(2, "A", "APPROVAL_REQ launch x"))
        r = run({"no_journal": {"unit": "all",
                                "re": "APPROVAL_REQ launch"}}, j)
        self.assertEqual(r["status"], "FAIL")
        self.assertIn("u2: 1", r["detail"])
        # no units resolved can never pass (UNIT_ASSERTS gate)
        c, _ = ctx({})
        c["manifest"]["units"] = []
        with mock.patch.object(checks.ctl, "resolve",
                               lambda tok, m=None: []):
            self.assertEqual(checks.check_assertion(
                {"no_journal": {"unit": "all", "re": "x"}}, c)["status"],
                "FAIL")

    def test_status_field(self):
        class Bot:
            def status_of(self, p):
                return {"st": "RUNNING",
                        "launch": {"grant": "valid", "id": "abcd1234",
                                   "uses_left": 1, "left_s": 13000.0,
                                   "ready": True,
                                   "last_refusal": "battery 40"}}
        j = {}
        a = {"status_field": {"unit": "u1", "path": "launch.grant",
                              "equals": "valid"}}
        self.assertEqual(run(a, j, bot=Bot())["status"], "PASS")
        a["status_field"]["equals"] = "used"
        self.assertEqual(run(a, j, bot=Bot())["status"], "FAIL")
        # numeric bounds + re on the same dotted path
        a = {"status_field": {"unit": "u1", "path": "launch.left_s",
                              "lt": 14400}}
        self.assertEqual(run(a, j, bot=Bot())["status"], "PASS")
        a = {"status_field": {"unit": "u1", "path": "launch.id",
                              "re": "^[0-9a-f]{8}$"}}
        self.assertEqual(run(a, j, bot=Bot())["status"], "PASS")
        a = {"status_field": {"unit": "u1", "path": "launch.last_refusal",
                              "re": "battery"}}
        self.assertEqual(run(a, j, bot=Bot())["status"], "PASS")
        # a path that does not exist is a FAIL, never a crash or pass
        a = {"status_field": {"unit": "u1", "path": "launch.missing",
                              "equals": 1}}
        self.assertEqual(run(a, j, bot=Bot())["status"], "FAIL")
        # no operator session -> SKIP
        self.assertEqual(run({"status_field": {"unit": "u1",
                                               "path": "launch.grant",
                                               "equals": "valid"}},
                             j)["status"], "SKIP")

    def test_op_authorize_builds_and_sends_grant(self):
        """P4F-07: op_authorize mints the fn_buildGrant shape from the
        deploy's stored launch meta and tracks the unit's answer."""
        import operator_bot
        bot = operator_bot.OperatorBot(
            {"name": "t", "units": [], "server": {"type": "fake"}})
        bot._launch_meta["p1"] = {
            "role": "ambulance", "run": "r1",
            "envelope": {"valid_for_s": 14400, "max_launches": 1},
            "envelope_hash": "sha256:aa", "grant": ""}
        sent = []

        def fake_send(msg, kind, text):
            sent.append(msg)
            g = msg["ms"]["grant"]
            bot._on_frame({"mt": 1094, "sd": "p1",
                           "ms": {"a": "grant", "grant": g["id"],
                                  "st": "valid"}})
            return True
        bot._send = fake_send
        self.assertTrue(bot.op_authorize("p1", {"valid_for_s": 120}))
        g = sent[0]["ms"]["grant"]
        self.assertEqual(sent[0]["ms"]["a"], "authorize")
        self.assertEqual(
            (g["role"], g["party"], g["envelope_hash"],
             g["valid_for_s"], g["max_launches"]),
            ("ambulance", "p1", "sha256:aa", 120, 1))
        self.assertTrue(g["id"])
        self.assertEqual(bot._launch_meta["p1"]["grant"], g["id"])
        # bounds > the envelope are refused before anything is sent
        self.assertFalse(bot.op_authorize(
            "p1", {"valid_for_s": 99999}))
        self.assertEqual(len(sent), 1)
        # a rejected grant is a False outcome, not a crash
        def reject_send(msg, kind, text):
            sent.append(msg)
            bot._on_frame({"mt": 1094, "sd": "p1",
                           "ms": {"a": "grant", "grant": "",
                                  "st": "rejected", "why": "site_config"}})
            return True
        bot._send = reject_send
        self.assertFalse(bot.op_authorize("p1", {}))
        # no envelope at all -> nothing on the wire
        bot._launch_meta["p2"] = {"role": "searcher", "run": "r1",
                                  "envelope": {}, "envelope_hash": "",
                                  "grant": ""}
        self.assertFalse(bot.op_authorize("p2", {}))
        self.assertEqual(len(sent), 2)

    def test_op_revoke_launch_ids(self):
        """P4F-07: revoke names the grant the bot sent, falling back to
        the 1087 launch.id prefix."""
        import operator_bot
        bot = operator_bot.OperatorBot(
            {"name": "t", "units": [], "server": {"type": "fake"}})
        bot._launch_meta["p1"] = {"role": "ambulance", "run": "r1",
                                  "envelope": {}, "envelope_hash": "h",
                                  "grant": "uuid-1234"}
        sent = []

        def fake_send(msg, kind, text):
            sent.append(msg)
            bot._on_frame({"mt": 1094, "sd": "p1",
                           "ms": {"a": "grant", "grant": "uuid-1234",
                                  "st": "void", "why": "revoked"}})
            return True
        bot._send = fake_send
        self.assertTrue(bot.op_revoke_launch("p1"))
        self.assertEqual(sent[0]["ms"],
                         {"a": "revoke_launch", "run": "r1",
                          "grant": "uuid-1234"})
        # no recorded grant but the status block still carries the id8
        bot._launch_meta["p1"]["grant"] = ""
        bot._on_frame({"mt": 1087, "sd": "p1",
                       "ms": {"launch": {"grant": "valid",
                                         "id": "abcd1234"}}})
        self.assertTrue(bot.op_revoke_launch("p1"))
        self.assertEqual(sent[1]["ms"]["grant"], "abcd1234")
        # nothing known -> refused locally
        bot._launch_meta["p2"] = {"role": "x", "run": "r1",
                                  "envelope": {}, "envelope_hash": "",
                                  "grant": ""}
        self.assertFalse(bot.op_revoke_launch("p2"))
        self.assertEqual(len(sent), 2)

    def test_seed_slot_base_fits_camera_fleets(self):
        import scenario
        units = [{"block": "copter_camera"}, {"block": "copter_camera"}]
        for s in range(9):
            sb = scenario._seed_slot_base(s, units)
            self.assertLessEqual(64000 + 10 * (sb + 1), 65535)
        # a fleet that fits keeps the historical block
        self.assertEqual(scenario._seed_slot_base(7, [{"block": "copter"}]),
                         10 + 7 * 25)

    def test_slot_override_moves_derived_server_port(self):
        import scenario
        import simlib
        spec = scenario.load_scenario(
            os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), "tests/p4f/p4f05_abort.yaml"),
            8)["sim"]
        with mock.patch.dict(os.environ, {"SIM_SLOT_BASE_OVERRIDE": "60"}):
            sim = simlib.load_sim(spec)
            self.assertEqual(sim["slot_base"], 60)
            self.assertEqual(sim["server"]["port"], simlib.SERVER_PORT + 60)
            pinned = dict(spec, server=dict(spec["server"], port=9999))
            self.assertEqual(simlib.load_sim(pinned)["server"]["port"], 9999)

    def test_slot_block_skips_host_held_port(self):
        import socket
        import scenario
        import simlib
        held = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        held.bind(("0.0.0.0", 0))
        port = held.getsockname()[1]
        orig = (simlib.load_sim, simlib.used_ports, simlib.live_sim_names)
        try:
            simlib.load_sim = lambda spec: spec
            simlib.used_ports = lambda sim: sim["ports"]
            simlib.live_sim_names = lambda: []

            def sim_at(sb):
                return {"slot_base": sb,
                        "ports": {port} if sb == 10 else set()}
            self.assertEqual(
                scenario._host_free_sim(sim_at, [10, 35])["slot_base"], 35)
            self.assertEqual(
                scenario._host_free_sim(sim_at, [10])["slot_base"], 10)
        finally:
            simlib.load_sim, simlib.used_ports, simlib.live_sim_names = orig
            held.close()
        self.assertEqual(simlib.ports_held({port}), [])

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
        self.assertIn("after the epoch", r["detail"])
        # a `start` step stamps ctx["start_ms"] and becomes the t0: the
        # arm/EKF warm-up between epoch and start is not detection time
        r = run({"time_to_detect": {"truth": "pa", "src": "ai",
                                     "max_s": 10, "truth_file": f}}, {},
                world_epoch_ms=1000, start_ms=5000)
        self.assertEqual((r["status"], r["value"]), ("PASS", 1.0))
        self.assertIn("after the start step", r["detail"])
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


def _module_log_unit(tmp_root, name="u1"):
    """A unit whose logs/ dir lives under a patched temp RUN_DIR - real
    journal_mark offsets over real log files, no sim needed."""
    u = {"name": name, "party_id": "p1", "fc": None, "sim": "tsim"}
    os.makedirs(os.path.join(simlib.unit_dir(u), "logs"), exist_ok=True)
    return u


def _append_log(u, proc, lines):
    path = os.path.join(simlib.unit_dir(u), "logs", proc + ".log")
    with open(path, "a") as f:
        f.write("".join(ln + "\n" for ln in lines))


def _module_log_ctx(u, marks):
    return {"manifest": {"name": u["sim"], "units": [u],
                         "server_procs": []},
            "marks": {u["name"]: marks}, "file": "/x/s.yaml",
            "results_dir": None}


def run_module_log(a, u, marks):
    c = _module_log_ctx(u, marks)
    with mock.patch.object(checks.ctl, "resolve",
                           lambda tok, m=None: [u]):
        return checks.check_assertion(a, c)


class TNoSep(unittest.TestCase):
    """no_sep_violation: an unmeasured separation never passes."""

    def _run(self, rows, **a):
        class Tele:
            def rows(self, n):
                return rows.get(n, [])
        c, _ = ctx({}, telemetry=Tele())
        with mock.patch.object(checks, "_units", lambda arg, c: [U1, U2]):
            return checks._no_sep_violation("no_sep_violation", a, c)

    def fix(self, t, lat, alt=30.0):
        return {"t_ms": t, "lat": lat, "lng": 31.0, "alt_m": alt, "fix_type": 3}

    def test_no_overlap_fails(self):
        r = self._run({"u1": [self.fix(0, 30.0)], "u2": [self.fix(60000, 30.001)]})
        self.assertEqual(r["status"], "FAIL")

    def test_no_recorder_fails(self):
        c, _ = ctx({}, telemetry=None)
        with mock.patch.object(checks, "_units", lambda arg, c: [U1, U2]):
            r = checks._no_sep_violation("no_sep_violation", {}, c)
        self.assertEqual(r["status"], "FAIL")

    def test_one_fc_unit_skips(self):
        c, _ = ctx({}, telemetry=None)
        with mock.patch.object(checks, "_units", lambda arg, c: [U1]):
            r = checks._no_sep_violation("no_sep_violation", {}, c)
        self.assertEqual(r["status"], "SKIP")

    def test_vertically_separated_only_passes(self):
        r = self._run({"u1": [self.fix(0, 30.0, 30)], "u2": [self.fix(0, 30.0, 60)]},
                      h_m=10, v_m=10)
        self.assertEqual(r["status"], "PASS")

    def test_layer_exactly_v_m_apart_passes(self):
        """the P4B-10 e_loss30 pair: 82 m / 76 m layers over homes 1 cm
        apart read 5.99 m AMSL - still the 6 m layer; 5.9 m is not."""
        self.assertEqual(self._run({"u1": [self.fix(0, 30.0, 666.07)],
                                    "u2": [self.fix(0, 30.0, 660.08)]},
                                   h_m=25, v_m=6)["status"], "PASS")
        self.assertEqual(self._run({"u1": [self.fix(0, 30.0, 666.0)],
                                    "u2": [self.fix(0, 30.0, 660.1)]},
                                   h_m=25, v_m=6)["status"], "FAIL")

    def test_close_pair_fails_far_pair_passes(self):
        self.assertEqual(self._run({"u1": [self.fix(0, 30.0)],
                                    "u2": [self.fix(0, 30.00001)]}, h_m=10)["status"], "FAIL")
        self.assertEqual(self._run({"u1": [self.fix(0, 30.0)],
                                    "u2": [self.fix(0, 30.001)]}, h_m=10)["status"], "PASS")


class TOsFaults(unittest.TestCase):
    """SL-0014: the cpu_starve worker, the journal perm toggle, the
    libfaketime lookup and the clock_spread min_ms extension."""

    def test_clock_spread_min_ms(self):
        j = {"u1": [line(1, "CLOCK", "fleet_minus_unix_ms=-3600000")],
             "u2": [line(1, "CLOCK", "fleet_minus_unix_ms=0")]}
        a = {"clock_spread": {"unit": "all", "min_ms": 3000000}}
        self.assertEqual(run(a, j)["status"], "PASS")
        j["u2"] = [line(1, "CLOCK", "fleet_minus_unix_ms=-3599000")]
        self.assertEqual(run(a, j)["status"], "FAIL")   # 1 s apart only
        # min_ms still honours a max_ms ceiling
        a["clock_spread"]["max_ms"] = 4000000
        j["u2"] = [line(1, "CLOCK", "fleet_minus_unix_ms=-5000000")]
        self.assertEqual(run(a, j)["status"], "FAIL")   # spread 5 s > max

    def test_faketime_lib_honours_setting(self):
        with mock.patch.dict(os.environ,
                             {"SIM_FAKETIME_LIB": "/no/such/libfaketime.so.1"}):
            self.assertIsNone(simlib.faketime_lib())
        with tempfile.NamedTemporaryFile(suffix=".so.1") as f:
            with mock.patch.dict(os.environ, {"SIM_FAKETIME_LIB": f.name}):
                self.assertEqual(simlib.faketime_lib(), f.name)

    def test_clock_step_validation(self):
        import scenario
        raw = {"version": 1, "name": "x",
               "fleet": [{"id": "u1", "cls": "copter"}],
               "timeline": [{"t": 5, "fault": "clock_step", "unit": "u1",
                             "offset_s": 60}]}
        # libfaketime present, but the target was never booted under it
        with mock.patch.object(simlib, "faketime_lib",
                               return_value="/tmp/libfaketime.so.1"):
            with self.assertRaises(simlib.SimError) as cm:
                scenario._validate(raw, "x.yaml")
            self.assertIn("faketime", str(cm.exception))
            raw["fleet"][0]["faketime"] = True
            scenario._validate(raw, "x.yaml")       # passes now
        # no libfaketime at all: `faketime: true` alone must fail
        with mock.patch.object(simlib, "faketime_lib", return_value=None):
            with self.assertRaises(simlib.SimError) as cm:
                scenario._validate(raw, "x.yaml")
            self.assertIn("libfaketime", str(cm.exception))

    def test_journal_set_writable_round_trip(self):
        import ctl
        u = {"name": "u1", "sim": "_test_jrnl"}
        jdir = simlib.journal_dir(u)
        os.makedirs(jdir, exist_ok=True)
        self.addCleanup(shutil.rmtree, os.path.dirname(jdir), True)
        f = os.path.join(jdir, "none.pre.log")
        simlib.write_text(f, "x\n")
        self.assertTrue(ctl.journal_set_writable(u, False))
        self.assertFalse(os.stat(jdir).st_mode & 0o200, "dir still writable")
        self.assertFalse(os.stat(f).st_mode & 0o200, "file still writable")
        with self.assertRaises(OSError):
            open(os.path.join(jdir, "new.log"), "w")
        self.assertTrue(ctl.journal_set_writable(u, True))
        self.assertTrue(os.stat(jdir).st_mode & 0o200)
        self.assertTrue(os.stat(f).st_mode & 0o200)

    def test_starve_worker_stops_and_releases(self):
        import ctl
        import subprocess
        import threading
        import time
        run_dir = os.path.join(simlib.RUN_DIR, "_test_starve")
        pids = os.path.join(run_dir, "pids")
        os.makedirs(pids, exist_ok=True)
        self.addCleanup(shutil.rmtree, run_dir, True)
        # the sleeper's argv carries a var/run path so pid_is_ours gates
        # it in - the same ownership rule live units pass
        script = os.path.join(run_dir, "sleeper.py")
        open(script, "w").write("import time\ntime.sleep(120)\n")
        proc = subprocess.Popen([sys.executable, script])
        self.addCleanup(proc.kill)
        pf = os.path.join(pids, "u1.de_comm.pid")
        spf = os.path.join(pids, "u1.de_comm.starve.pid")
        simlib.write_text(pf, str(proc.pid))
        t = threading.Thread(target=ctl._starve_worker,
                             args=(pf, spf, 80.0, 1.2, 200.0))
        t.start()
        saw_stop = False
        deadline = time.time() + 3
        while time.time() < deadline and t.is_alive():
            try:
                st = open(f"/proc/{proc.pid}/stat").read() \
                    .rsplit(")", 1)[1].split()[0]
                if st == "T":
                    saw_stop = True
            except OSError:
                break
            time.sleep(0.05)
        t.join(5)
        self.assertFalse(t.is_alive(), "worker did not finish dur_s")
        self.assertTrue(saw_stop, "worker never stopped the proc")
        st = open(f"/proc/{proc.pid}/stat").read().rsplit(")", 1)[1] \
            .split()[0]
        self.assertNotEqual(st, "T", "proc left SIGSTOPped")
        self.assertFalse(os.path.exists(spf), "worker pidfile left behind")


class TRunSuiteResults(unittest.TestCase):
    """ci/run_suite: a run is judged only on results it wrote itself."""

    def test_stale_results_dir_ignored(self):
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), "ci"))
        import run_suite
        with tempfile.TemporaryDirectory() as root:
            old = os.path.join(root, "20260101T000000Z", "scn")
            os.makedirs(old)
            os.utime(old, (1000, 1000))
            with mock.patch.object(run_suite, "RESULTS", root):
                self.assertIsNone(run_suite.latest_results_dir("scn", 2000))
                new = os.path.join(root, "20260102T000000Z", "scn")
                os.makedirs(new)
                self.assertEqual(run_suite.latest_results_dir("scn", 2000), new)


class TModuleLog(unittest.TestCase):
    """PL-07: module_log_count + wait_log - assertions/steps over a
    module's own stdout log (logs/<module>.log), not the journal."""

    RE = r"EVENT_OUT precision_landing\.\w+"
    LINES = ["PRECLAND state -> 0",
             'EVENT_OUT precision_landing.enabled {"src":"event"}',
             "PRECLAND state -> 1",
             "PRECLAND state -> 2",
             'EVENT_OUT precision_landing.locked {"range_m":1.2}']

    def test_module_log_count_comparisons(self):
        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(simlib, "RUN_DIR", td):
            u = _module_log_unit(td)
            _append_log(u, "de_precland", self.LINES)
            a = {"unit": "u1", "module": "de_precland", "re": self.RE}
            # {} mark = from the start: 2 of 5 lines match
            for bound, want, status in (
                    ({"equals": 2}, 2, "PASS"),
                    ({"equals": 3}, 3, "FAIL"),
                    ({"at_least": 2}, None, "PASS"),
                    ({"at_least": 3}, None, "FAIL"),
                    ({"at_most": 2}, None, "PASS"),
                    ({"at_most": 1}, None, "FAIL")):
                r = run_module_log({"module_log_count": dict(a, **bound)},
                                   u, {})
                self.assertEqual(r["status"], status, f"{bound}: {r}")
            self.assertEqual(r["name"], "module_log_count")

    def test_module_log_count_since_the_mark(self):
        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(simlib, "RUN_DIR", td):
            u = _module_log_unit(td)
            _append_log(u, "de_precland", self.LINES)
            marks = simlib.journal_mark(u)   # snapshot log offsets too
            _append_log(u, "de_precland",
                        ['EVENT_OUT precision_landing.disabled '
                         '{"src":"event"}', "PRECLAND state -> 0"])
            a = {"unit": "u1", "module": "de_precland", "re": self.RE}
            r = run_module_log({"module_log_count": dict(a, equals=1)},
                               u, marks)
            self.assertEqual(r["status"], "PASS")
            # pre-mark lines stay excluded: all EVENT_OUT lines = FAIL
            r = run_module_log({"module_log_count": dict(a, equals=3)},
                               u, marks)
            self.assertEqual(r["status"], "FAIL")

    def test_module_log_count_missing_file_is_zero(self):
        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(simlib, "RUN_DIR", td):
            u = _module_log_unit(td)
            a = {"unit": "u1", "module": "de_precland", "re": self.RE}
            r = run_module_log({"module_log_count": dict(a, equals=0)},
                               u, {})
            self.assertEqual(r["status"], "PASS")

    def test_wait_log_matches_and_times_out(self):
        from runner import _wait_log
        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(simlib, "RUN_DIR", td):
            u = _module_log_unit(td)
            _append_log(u, "de_precland", self.LINES)
            marks = {u["name"]: {}}
            # a line already there returns at once
            self.assertTrue(_wait_log(u, "de_precland", "EVENT_OUT", 5,
                                      marks))
            # an absent pattern burns the timeout and fails the step
            t0 = __import__("time").time()
            self.assertFalse(_wait_log(u, "de_precland", "NEVER_EMITTED",
                                       1.5, marks))
            self.assertGreaterEqual(__import__("time").time() - t0, 1.5)

    def test_wait_log_only_sees_post_mark_lines(self):
        from runner import _wait_log
        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(simlib, "RUN_DIR", td):
            u = _module_log_unit(td)
            _append_log(u, "de_precland", self.LINES)
            mark = simlib.journal_mark(u)
            marks = {u["name"]: mark}
            # the matching lines are all pre-mark -> timeout
            self.assertFalse(_wait_log(u, "de_precland", "EVENT_OUT", 1.5,
                                       marks))
            # a post-mark append unblocks it
            _append_log(u, "de_precland",
                        ["EVENT_OUT precision_landing.lost {}"])
            self.assertTrue(_wait_log(u, "de_precland", "EVENT_OUT", 3,
                                      marks))


class TFreeze(unittest.TestCase):
    """TASK-04: ctl freeze/unfreeze signal only pidfile pids that
    simlib.pid_is_ours accepts - a recycled/foreign pid is refused."""

    def _proc_state(self, pid):
        return open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1] \
            .split()[0]

    def _pidfile(self, td, owner="u1", proc="de_mavlink"):
        pids = os.path.join(td, "tsim", "pids")
        os.makedirs(pids, exist_ok=True)
        return os.path.join(pids, f"{owner}.{proc}.pid")

    def test_freeze_refuses_foreign_pid(self):
        """`sleep 30` is a live pid whose cmdline does not point into this
        repo - pid_is_ours rejects it, so freeze must not signal it."""
        import signal
        import subprocess
        import ctl as ctl_mod
        child = subprocess.Popen(["sleep", "30"])
        try:
            with tempfile.TemporaryDirectory() as td, \
                    mock.patch.object(simlib, "RUN_DIR", td), \
                    mock.patch.object(ctl_mod, "owner_sim",
                                      lambda o: "tsim"):
                with open(self._pidfile(td), "w") as f:
                    f.write(str(child.pid))
                self.assertFalse(ctl_mod.freeze_proc("u1", "de_mavlink"))
                self.assertFalse(
                    ctl_mod._signal_proc("u1", "de_mavlink",
                                         signal.SIGCONT))
            self.assertNotEqual(self._proc_state(child.pid), "T")
        finally:
            child.kill()
            child.wait()

    def test_freeze_unfreeze_own_pid(self):
        """A child tagged with RUN_DIR on its cmdline is 'ours': freeze
        stops it (state T) and unfreeze resumes it."""
        import signal
        import subprocess
        import time
        import ctl as ctl_mod
        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(simlib, "RUN_DIR", td), \
                mock.patch.object(ctl_mod, "owner_sim",
                                  lambda o: "tsim"):
            child = subprocess.Popen(
                [sys.executable, "-c",
                 "import time; time.sleep(30)", os.path.join(td, "tsim")])
            try:
                with open(self._pidfile(td), "w") as f:
                    f.write(str(child.pid))
                self.assertTrue(ctl_mod.freeze_proc("u1", "de_mavlink"))
                self.assertEqual(self._proc_state(child.pid), "T")
                self.assertTrue(
                    ctl_mod._signal_proc("u1", "de_mavlink",
                                         signal.SIGCONT))
                for _ in range(20):
                    if self._proc_state(child.pid) != "T":
                        break
                    time.sleep(0.1)
                self.assertNotEqual(self._proc_state(child.pid), "T")
            finally:
                child.kill()
                child.wait()


class TInvariants(unittest.TestCase):
    """SL-0011 run-wide invariant assertions over synthetic
    samples.jsonl rows / journals."""

    def _samples(self):
        return [{"t_ms": t, "key": "u1.de_comm", "pid": 111,
                 "rss_kb": 10000, "fd": 10}
                for t in range(0, 61000, 1000)]

    def test_no_unexpected_restart(self):
        spec = {"no_unexpected_restart": {"proc": "de_comm"}}
        rows = self._samples()
        # pid changes once at t=30s with no fault window -> FAIL
        for r in rows:
            if r["t_ms"] >= 30000:
                r["pid"] = 222
        self.assertEqual(run(spec, {}, samples=rows)["status"], "FAIL")
        # a fault window at the change -> PASS
        ev = [{"t_ms": 30000, "unit": "u1", "proc": "de_comm"}]
        self.assertEqual(run(spec, {}, samples=rows,
                             proc_events=ev)["status"], "PASS")
        # no change at all -> PASS
        self.assertEqual(run(spec, {}, samples=self._samples(),
                             proc_events=[])["status"], "PASS")

    def test_no_unexpected_restart_vanished(self):
        spec = {"no_unexpected_restart": {"proc": "de_comm"}}
        rows = [{"t_ms": t, "key": "u1.de_comm", "pid": 111,
                 "rss_kb": 1, "fd": 1} for t in range(0, 6000, 1000)]
        rows += [{"t_ms": t, "key": "u2.de_comm", "pid": 222,
                  "rss_kb": 1, "fd": 1} for t in range(0, 61000, 1000)]
        # u1.de_comm disappears at t=6s of a 61s run, no event -> FAIL
        self.assertEqual(run(spec, {}, samples=rows)["status"], "FAIL")
        ev = [{"t_ms": 6000, "unit": "u1", "proc": "de_comm"}]
        self.assertEqual(run(spec, {}, samples=rows,
                             proc_events=ev)["status"], "PASS")

    def test_rss_growth(self):
        # +1 kb/s for 60 s -> ~3.5 MB/h
        rows = self._samples()
        for r in rows:
            r["rss_kb"] = 10000 + r["t_ms"] // 1000
        spec = {"rss_growth": {"proc": "de_comm", "max_mb_per_h": 5,
                               "warmup_s": 0}}
        self.assertEqual(run(spec, {}, samples=rows)["status"], "PASS")
        spec["rss_growth"]["max_mb_per_h"] = 1
        self.assertEqual(run(spec, {}, samples=rows)["status"], "FAIL")
        # flat rss never trips
        spec["rss_growth"]["max_mb_per_h"] = 0
        self.assertEqual(run(spec, {}, samples=self._samples())["status"],
                         "PASS")

    def test_fd_growth(self):
        rows = self._samples()
        for r in rows:
            r["fd"] = 10 + r["t_ms"] // 15000      # 10 -> 14 over 60 s
        spec = {"fd_growth": {"proc": "de_comm", "max": 5}}
        self.assertEqual(run(spec, {}, samples=rows)["status"], "PASS")
        spec["fd_growth"]["max"] = 2
        self.assertEqual(run(spec, {}, samples=rows)["status"], "FAIL")

    def test_journal_size(self):
        import tempfile
        spec = {"journal_size": {"unit": "u1", "max_mb": 1}}
        with tempfile.TemporaryDirectory() as td:
            with open(os.path.join(td, "journal.jsonl"), "w") as f:
                f.write("x" * 1000)
            with mock.patch.object(simlib, "journal_dir",
                                   lambda u: td):
                self.assertEqual(run(spec, {})["status"], "PASS")
            big = os.path.join(td, "big.jsonl")
            with open(big, "w") as f:
                f.write("x" * (1200 * 1024))
            with mock.patch.object(simlib, "journal_dir",
                                   lambda u: td):
                self.assertEqual(run(spec, {})["status"], "FAIL")

    def test_recovers_after_fault(self):
        class _Tele:
            def __init__(self, rows):
                self._rows = rows

            def rows(self, name=None):
                return self._rows

        spec = {"recovers_after_fault": {"within_s": 60}}
        ev = [{"t_ms": 10000, "unit": "u1", "kind": "link_cut"}]
        # link back at t=20s, telemetry fresh at t=30s -> PASS
        kw = dict(restore_events=ev,
                  link_events={"p1": [(12000, False), (20000, True)]},
                  telemetry=_Tele([{"t_ms": 30000}]))
        self.assertEqual(run(spec, {}, **kw)["status"], "PASS")
        # link never comes back -> FAIL
        kw = dict(restore_events=ev,
                  link_events={"p1": [(12000, False)]},
                  telemetry=_Tele([{"t_ms": 30000}]))
        self.assertEqual(run(spec, {}, **kw)["status"], "FAIL")
        # link back but telemetry stale -> FAIL
        kw = dict(restore_events=ev,
                  link_events={"p1": [(12000, False), (20000, True)]},
                  telemetry=_Tele([{"t_ms": 5000}]))
        self.assertEqual(run(spec, {}, **kw)["status"], "FAIL")

    def test_no_stuck_state(self):
        spec = {"no_stuck_state": {"unit": "u1",
                                   "re": "STATE", "max_s": 60}}
        j = {"u1": [line(t, "ST", "STATE a")
                    for t in (0, 30000, 70000)]}
        # same state for 70 s > 60 -> FAIL
        self.assertEqual(run(spec, j)["status"], "FAIL")
        j = {"u1": [line(0, "ST", "STATE a"), line(30000, "ST", "STATE b"),
                    line(70000, "ST", "STATE a")]}
        self.assertEqual(run(spec, j)["status"], "PASS")


if __name__ == "__main__":
    unittest.main()
