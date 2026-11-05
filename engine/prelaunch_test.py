"""prelaunch_test.py - pure tests of the runner: no network and no sim."""

import json
import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import prelaunch  # noqa: E402


def _reg(p0_impl="prelaunch_stubs:ok", p1_impl="prelaunch_stubs:ok"):
    return {"version": 1,
            "features": [
                {"id": "a", "title": "A", "severity": "P0",
                 "components": ["x"], "checks": ["c.a"]},
                {"id": "b", "title": "B", "severity": "P1",
                 "components": ["x"], "checks": ["c.b"]}],
            "checks": {
                "c.a": {"stage": 0, "kind": "probe", "impl": p0_impl,
                        "title": "a"},
                "c.b": {"stage": 0, "kind": "probe", "impl": p1_impl,
                        "title": "b"}}}


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp)
        self.target = os.path.join(self.tmp, "t.yaml")
        with open(self.target, "w") as fh:
            yaml.safe_dump({"name": "t", "services": {"auth": {"host": "h"}},
                            "stages": ["0"]}, fh)
        p = mock.patch.object(prelaunch.simlib, "RESULTS_DIR", self.tmp)
        p.start()
        self.addCleanup(p.stop)

    def run_gate(self, reg, *extra, version="v1"):
        path = os.path.join(self.tmp, "f.yaml")
        with open(path, "w") as fh:
            yaml.safe_dump(reg, fh)
        with mock.patch("sys.stdout"):
            return prelaunch.main(["--target", self.target, "--features", path,
                                   "--version", version, *extra])

    def report(self, version="v1"):
        with open(os.path.join(self.tmp, "prelaunch", version, "t",
                               "report.json")) as fh:
            return json.load(fh)


class GateRule(Base):
    def test_stage_0_with_stubs_writes_both_reports(self):
        self.assertEqual(self.run_gate(_reg(), "--stage", "0"), 0)
        d = os.path.join(self.tmp, "prelaunch", "v1", "t")
        self.assertTrue(os.path.isfile(os.path.join(d, "report.md")))
        self.assertEqual(self.report()["verdict"], "GO")

    def test_p0_fail_is_nonzero_and_no_go(self):
        self.assertEqual(
            self.run_gate(_reg(p0_impl="prelaunch_stubs:bad"), "--stage", "0"),
            1)
        self.assertEqual(self.report()["verdict"], "NO-GO")

    def test_p0_skip_is_nonzero_and_no_go(self):
        self.assertEqual(
            self.run_gate(_reg(p0_impl="prelaunch_stubs:skip"),
                          "--stage", "0"), 1)
        self.assertEqual(self.report()["verdict"], "NO-GO")

    def test_probe_not_built_is_skip_not_pass(self):
        self.assertEqual(
            self.run_gate(_reg(p0_impl="prelaunch_checks.nope:x"),
                          "--stage", "0"), 1)
        self.assertEqual(self.report()["checks"]["c.a"]["status"], "SKIP")

    def test_p1_fail_does_not_block(self):
        self.assertEqual(
            self.run_gate(_reg(p1_impl="prelaunch_stubs:bad"), "--stage", "0"),
            0)
        self.assertEqual(self.report()["verdict"], "GO")

    def test_crash_is_fail(self):
        self.assertEqual(
            self.run_gate(_reg(p0_impl="prelaunch_stubs:crash"),
                          "--stage", "0"), 1)
        self.assertEqual(self.report()["checks"]["c.a"]["status"], "FAIL")

    def test_required_stage_not_run_is_incomplete(self):
        reg = _reg()
        reg["features"][0]["checks"].append("c.m")
        reg["checks"]["c.m"] = {"stage": 0, "kind": "probe", "title": "m",
                                "impl": "prelaunch_stubs:ok"}
        reg["checks"]["c.n"] = {"stage": 2, "kind": "probe", "title": "n",
                                "impl": "prelaunch_stubs:ok"}
        reg["features"][0]["checks"].append("c.n")
        with open(self.target, "w") as fh:
            yaml.safe_dump({"name": "t", "services": {"a": {}},
                            "stages": ["0", "2"]}, fh)
        self.assertEqual(self.run_gate(reg, "--stage", "0"), 0)
        self.assertEqual(self.report()["verdict"], "INCOMPLETE")


class Signoff(Base):
    def _manual(self):
        reg = _reg()
        reg["checks"]["c.a"] = {"stage": 0, "kind": "manual", "title": "a",
                                "text": "do it"}
        return reg

    def test_manual_is_skip_until_signed(self):
        self.assertEqual(self.run_gate(self._manual(), "--stage", "0"), 1)
        self.assertEqual(self.report()["verdict"], "NO-GO")
        self.assertEqual(self.run_gate(self._manual(),
                                       "--signoff", "c.a=pass"), 0)
        self.assertEqual(self.report()["checks"]["c.a"]["status"], "PASS")
        self.assertEqual(self.report()["verdict"], "GO")

    def test_signoff_cannot_pass_a_fail(self):
        reg = _reg(p0_impl="prelaunch_stubs:bad")
        self.run_gate(reg, "--stage", "0")
        self.run_gate(reg, "--signoff", "c.a=pass")
        self.assertEqual(self.report()["checks"]["c.a"]["status"], "FAIL")

    def test_override_is_in_the_report(self):
        reg = _reg(p0_impl="prelaunch_stubs:bad")
        self.run_gate(reg, "--stage", "0")
        self.assertEqual(self.run_gate(reg, "--override", "known fault"), 0)
        rep = self.report()
        self.assertEqual(rep["verdict"], "NO-GO")
        self.assertEqual(rep["override"]["reason"], "known fault")


class Safety(Base):
    def test_secret_is_scrubbed(self):
        self.run_gate(_reg(p0_impl="prelaunch_stubs:leak"), "--stage", "0")
        d = os.path.join(self.tmp, "prelaunch", "v1", "t")
        for f in ("report.json", "report.md"):
            self.assertNotIn("hunter2", open(os.path.join(d, f)).read())

    def test_target_with_a_password_is_refused(self):
        with open(self.target, "w") as fh:
            yaml.safe_dump({"name": "t", "services": {"a": {}},
                            "account": {"password": "x"}}, fh)
        with self.assertRaises(SystemExit):
            prelaunch.load_target(self.target)

    def test_versions_are_recorded(self):
        with mock.patch.object(prelaunch, "collect_versions",
                               return_value={"de_comm": "abc123"}):
            self.run_gate(_reg(), "--stage", "0")
        self.assertEqual(self.report()["versions"]["de_comm"], "abc123")

    def test_real_targets_load(self):
        for f in os.listdir(prelaunch.TARGETS_DIR):
            if f.endswith(".yaml") and f not in ("mirror.yaml",):
                prelaunch.load_target(os.path.join(prelaunch.TARGETS_DIR, f))

    def test_unset_port_is_refused_by_name(self):
        with open(self.target, "w") as fh:
            yaml.safe_dump({"name": "t", "services": {
                "comm": {"host": "h", "port": None}}}, fh)
        with self.assertRaises(SystemExit) as e:
            prelaunch.load_target(self.target)
        self.assertIn("services.comm.port", str(e.exception))

    def test_check_for_another_role_does_not_count(self):
        reg = _reg()
        reg["checks"]["c.m"] = {"stage": 0, "kind": "probe", "title": "m",
                                "impl": "prelaunch_stubs:bad",
                                "roles": ["mirror"]}
        reg["features"][0]["checks"].append("c.m")
        self.assertEqual(self.run_gate(reg, "--stage", "0"), 0)
        self.assertNotIn("c.m", self.report()["checks"])

    def test_live_target_refuses_normal_stages(self):
        with open(self.target, "w") as fh:
            yaml.safe_dump({"name": "t", "role": "live",
                            "services": {"a": {"host": "h"}}}, fh)
        with self.assertRaises(SystemExit):
            self.run_gate(_reg(), "--stage", "0")

    def test_list_has_no_side_effects(self):
        before = set(os.listdir(self.tmp))
        with mock.patch("sys.stdout"):
            self.assertEqual(prelaunch.main(["--list"]), 0)
        self.assertEqual(set(os.listdir(self.tmp)), before)


if __name__ == "__main__":
    unittest.main()
