#!/usr/bin/env python3
"""Offline unit test for the `write_file` step (GE-07): ctl's
write_module_file drops a JSON document into run/<unit>/<module>/
atomically and refuses a path that escapes the module directory. No
sim needed:

    python3 tests/test_write_file.py
"""
import json
import os
import sys
import tempfile
import unittest

# point the generated-tree root at a temp dir BEFORE simlib reads it
_tmp = tempfile.mkdtemp(prefix="sim_write_file_")
os.environ["SIM_RUN_DIR"] = _tmp

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "engine"))
import simlib  # noqa: E402
import ctl     # noqa: E402


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


if __name__ == "__main__":
    unittest.main(verbosity=2)
