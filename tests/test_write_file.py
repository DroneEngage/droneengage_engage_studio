#!/usr/bin/env python3
"""Offline unit test for the `write_file` step (GE-07): ctl's
write_module_file drops a JSON document into run/<unit>/<module>/
atomically and refuses a path that escapes the module directory.
GF-07 adds `delay_ms` on `also` entries: ctl sleeps inside the call so
a delayed write lands a real delay later while order is preserved.
No sim needed:

    python3 tests/test_write_file.py
"""
import json
import os
import sys
import tempfile
import time
import unittest

# point the generated-tree root at a temp dir BEFORE simlib reads it
_tmp = tempfile.mkdtemp(prefix="sim_write_file_")
os.environ["SIM_RUN_DIR"] = _tmp

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "engine"))
import simlib  # noqa: E402
import ctl     # noqa: E402
import runner  # noqa: E402


class T(unittest.TestCase):
    def setUp(self):
        self.unit = {"sim": "t", "name": "u1"}
        os.makedirs(simlib.unit_dir(self.unit), exist_ok=True)

    @staticmethod
    def _read(p):
        with open(p) as f:
            return json.load(f)

    def test_writes_json_into_the_module_dir(self):
        p = ctl.write_module_file(self.unit, "de_gpio",
                                  "gpio_inputs.json", {"17": 1})
        self.assertEqual(p, os.path.join(
            os.path.realpath(simlib.unit_dir(self.unit)),
            "de_gpio", "gpio_inputs.json"))
        self.assertEqual(self._read(p), {"17": 1})
        # atomic write: no .tmp file left behind
        self.assertFalse(os.path.exists(p + ".tmp"))

    def test_creates_the_module_dir_and_subdirs(self):
        p = ctl.write_module_file(self.unit, "de_gpio",
                                  "sub/dir/in.json", {"a": [1, 2]})
        self.assertTrue(p.startswith(
            os.path.realpath(os.path.join(simlib.unit_dir(self.unit),
                                          "de_gpio")) + os.sep))
        self.assertEqual(self._read(p), {"a": [1, 2]})

    def test_overwrite(self):
        ctl.write_module_file(self.unit, "de_gpio", "gpio_inputs.json",
                              {"17": 0})
        p = ctl.write_module_file(self.unit, "de_gpio", "gpio_inputs.json",
                                  {"17": 1})
        self.assertEqual(self._read(p), {"17": 1})

    def test_path_escape_refused(self):
        for bad in ("../x.json", "/etc/x.json", "sub/../../x.json"):
            with self.assertRaises(simlib.SimError):
                ctl.write_module_file(self.unit, "de_gpio", bad, {})


class DelayedWritesT(unittest.TestCase):
    """GF-07: `write_file` `also` entries may carry delay_ms. ctl's
    write_module_files sleeps inside the call so delayed writes land a
    real delay apart and still in order; the runner maps delay_ms to a
    `+N` marker in the ctl argv."""

    def setUp(self):
        self.unit = {"sim": "t", "name": "u1"}
        os.makedirs(simlib.unit_dir(self.unit), exist_ok=True)

    def test_delay_ms_honoured_and_order_preserved(self):
        writes = [("a.json", {"n": 0}, 0),
                  ("b.json", {"n": 1}, 60),
                  ("c.json", {"n": 2}, 0)]
        t0 = time.monotonic()
        paths = ctl.write_module_files(self.unit, "de_gpio", writes)
        self.assertGreaterEqual(time.monotonic() - t0, 0.06)
        # all three landed, in argument order
        names = [os.path.basename(p) for p in paths]
        self.assertEqual(names, ["a.json", "b.json", "c.json"])
        mdir = os.path.join(os.path.realpath(simlib.unit_dir(self.unit)),
                            "de_gpio")
        for name, want in (("a.json", 0), ("b.json", 1), ("c.json", 2)):
            with open(os.path.join(mdir, name)) as f:
                self.assertEqual(json.load(f), {"n": want})

    def test_delayed_path_still_escape_checked(self):
        with self.assertRaises(simlib.SimError):
            ctl.write_module_files(self.unit, "de_gpio",
                                   [("../x.json", {}, 60)])

    def test_runner_maps_delay_ms_to_marker(self):
        calls = []
        orig = runner.run_ctl
        runner.run_ctl = lambda *a: calls.append(a) or True
        try:
            ok = runner.run_step(
                {"write_file": {
                    "unit": "u1", "module": "de_gpio",
                    "path": "a.json", "json": {"n": 0},
                    "also": [{"path": "b.json", "json": {"n": 1},
                              "delay_ms": 150},
                             {"path": "c.json", "json": {"n": 2}}]}},
                {"__file": __file__}, {})
        finally:
            runner.run_ctl = orig
        self.assertTrue(ok)
        self.assertEqual(len(calls), 1)
        self.assertEqual(
            calls[0],
            ("write_file", "u1", "de_gpio",
             "a.json", '{"n": 0}',
             "+150", "b.json", '{"n": 1}',
             "c.json", '{"n": 2}'))

    def test_runner_no_delay_stays_one_batched_call(self):
        calls = []
        orig = runner.run_ctl
        runner.run_ctl = lambda *a: calls.append(a) or True
        try:
            runner.run_step(
                {"write_file": {
                    "unit": "u1", "module": "de_gpio",
                    "path": "a.json", "json": {"n": 0},
                    "also": [{"path": "b.json", "json": {"n": 1}}]}},
                {"__file": __file__}, {})
        finally:
            runner.run_ctl = orig
        self.assertEqual(len(calls), 1)
        self.assertNotIn("+", [str(x)[:1] for x in calls[0]])


if __name__ == "__main__":
    unittest.main(verbosity=2)
