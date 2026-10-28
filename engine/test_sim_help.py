#!/usr/bin/env python3
"""test_sim_help.py - SL-0121: per-sim `--help` / `./sim help <sim>`.

Covers (no SITL, no fleet - render_help on temp sim files + one
subprocess check on the ./sim entry):
  - description first line + wrapped rest, runbook path, usage line for
    the typed verb, params with default/choices/type/help, the common
    flags section and the spec's own example;
  - a sim without params prints 'Parameters:\\n  none';
  - an absent `example:` generates one from a choices param;
  - `./sim up <sim> --help` exits 0 and creates no var/run/<name> dir.
"""

import io
import os
import subprocess
import sys
import tempfile
import unittest

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

import sim_help  # noqa: E402
import simlib  # noqa: E402

SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class _Tty(io.StringIO):
    """StringIO that claims to be a terminal - print_usage colorizes
    only when sys.stdout.isatty() is true."""
    def isatty(self):
        return True


def _print_usage_captured(doc, tty):
    buf = _Tty() if tty else io.StringIO()
    old = sys.stdout
    sys.stdout = buf
    try:
        simlib.print_usage(doc)
    finally:
        sys.stdout = old
    return buf.getvalue()

SIM_YAML = """version: 1
description: |
  The Andruav app on an Android device as a native unit (app mode), on its
  own real server instance (auth + comm, pinned ports) with one SITL copter.
  Preset the app once - see the runbook.
example:
  - ./sim up helptest --set serial=66d0200f --set net=adb_reverse
params:
  serial: {default: emulator-5554, help: adb serial of the device}
  net: {default: auto, choices: [auto, adb_reverse, lan],
        help: how the phone reaches this host}
  server_port: {default: 62900, type: int,
                help: comm server port; auth is +10}
units:
  - {block: copter, name: copter1, count: 1}
"""

NO_PARAMS_YAML = """version: 1
description: |
  Plain copter sim.
units:
  - {block: copter, name: copter1, count: 1}
"""


class TestRenderHelp(unittest.TestCase):

    def _write(self, tmp, name, text):
        p = os.path.join(tmp, name + ".yaml")
        with open(p, "w") as fh:
            fh.write(text)
        return p

    def test_full_sections(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write(tmp, "helptest", SIM_YAML)
            with open(path[:-5] + ".md", "w") as fh:
                fh.write("# runbook\n")
            out = sim_help.render_help(path, "up")

        self.assertTrue(out.startswith(
            "helptest - The Andruav app on an Android device"), out[:200])
        self.assertIn("Preset the app once", out)
        self.assertIn("Runbook: " + os.path.join(
            os.path.relpath(tmp, simlib.SIM_ROOT), "helptest.md"), out)
        self.assertIn("./sim up helptest [--set NAME=VALUE ...]", out)
        self.assertIn("serial", out)
        self.assertIn("emulator-5554", out)
        self.assertIn("choices: auto, adb_reverse, lan", out)
        self.assertIn("(int)", out)
        self.assertIn("how the phone reaches this host", out)
        self.assertIn("Common flags:", out)
        self.assertIn("--fresh", out)
        self.assertIn("--slot-base", out)
        self.assertIn("--force", out)
        self.assertIn("./sim up helptest --set serial=66d0200f "
                      "--set net=adb_reverse", out)

    def test_no_params(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write(tmp, "plain", NO_PARAMS_YAML)
            out = sim_help.render_help(path, "up")
        self.assertIn("Parameters:\n  none", out)
        self.assertIn("./sim up plain", out)

    def test_generated_example(self):
        with tempfile.TemporaryDirectory() as tmp:
            spec = SIM_YAML.replace(
                "example:\n  - ./sim up helptest --set serial=66d0200f "
                "--set net=adb_reverse\n", "")
            path = self._write(tmp, "helptest", spec)
            out = sim_help.render_help(path, "up")
        self.assertIn("./sim up helptest --set net=auto", out)

    def test_verb_specific(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write(tmp, "helptest", SIM_YAML)
            out = sim_help.render_help(path, "run")
        self.assertIn("./sim run helptest", out)

    def test_render_help_colored_on_tty(self):
        # SL-0125: per-sim help is printed through simlib.print_usage -
        # the Usage/Example command lines and param/flag rows get the
        # ./sim colours; not a tty -> the plain text, unchanged.
        with tempfile.TemporaryDirectory() as tmp:
            path = self._write(tmp, "helptest", SIM_YAML)
            doc = sim_help.render_help(path, "up")
        out = _print_usage_captured(doc, tty=True)
        self.assertIn("\x1b[32m  ./sim up helptest \x1b[33m"
                      "[--set NAME=VALUE ...]", out)
        # flag and params: rows are parameters: yellow, not command green
        self.assertIn("\x1b[33m  serial", out)
        self.assertIn("\x1b[33m  --fresh", out)
        self.assertEqual(_print_usage_captured(doc, tty=False),
                         doc + "\n")

    def test_wants_sim_help(self):
        self.assertTrue(sim_help.wants_sim_help(
            "up", ["copters3", "--help"]))
        self.assertTrue(sim_help.wants_sim_help(
            "up", ["copters3", "help"]))
        self.assertTrue(sim_help.wants_sim_help(
            "up", ["copters3", "-h"]))
        self.assertFalse(sim_help.wants_sim_help("up", ["--help"]))
        self.assertFalse(sim_help.wants_sim_help("up", ["copters3"]))
        self.assertFalse(sim_help.wants_sim_help("down",
                                                 ["copters3", "--help"]))


class TestCliHelp(unittest.TestCase):

    def test_up_help_starts_nothing(self):
        sim_bin = os.path.join(SIM_ROOT, "sim")
        r = subprocess.run([sim_bin, "up", "copters3", "--help"],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("copters3", r.stdout)
        self.assertIn("Parameters:", r.stdout)
        self.assertFalse(os.path.isdir(
            os.path.join(SIM_ROOT, "var", "run", "copters3")),
            "a help call must not create the sim run dir")

    def test_help_sim_same_text(self):
        sim_bin = os.path.join(SIM_ROOT, "sim")
        r1 = subprocess.run([sim_bin, "help", "copters3"],
                            capture_output=True, text=True)
        r2 = subprocess.run([sim_bin, "up", "copters3", "--help"],
                            capture_output=True, text=True)
        self.assertEqual(r1.returncode, 0, r1.stderr)
        self.assertEqual(r1.stdout, r2.stdout)


if __name__ == "__main__":
    unittest.main()
