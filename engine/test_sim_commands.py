#!/usr/bin/env python3
"""test_sim_commands.py - SL-0115: the central command registry.

Covers:
  - every command name main() dispatches in the `sim` script has a
    COMMANDS entry (the dispatch names are parsed out of the script with
    a regex, so adding a command without a registry entry fails here);
  - overview() mentions every registered command;
  - command_help('up') shows its own flags (--fresh, --open-web);
  - allowed_flags('up') includes the shared flag set;
  - subprocess: `./sim up help`, `./sim clean help`, `./sim help clean`
    exit 0 and print that command's usage, running nothing;
  - subprocess: `./sim up --bogus` exits non-zero naming the bad flag;
  - simlib.resolve_sim_arg: path passthrough, bare sim name, unknown
    name, ambiguous name (all against a tmp SIM_ROOT).
"""

import os
import re
import subprocess
import sys
import tempfile
import unittest

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

import sim_commands  # noqa: E402
import simlib  # noqa: E402

SIM = os.path.join(simlib.SIM_ROOT, "sim")


def _dispatched_commands():
    """The command names main() dispatches in the `sim` script: every
    `c == "name"` and every name in a `c in ("a", "b", ...)` tuple,
    parsed out of the source so a new command without a registry entry
    fails the test."""
    src = open(SIM).read()
    names = set(re.findall(r'\bc\s*==\s*"([^"]+)"', src))
    for tup in re.findall(r"\bc\s+in\s*\(([^)]*)\)", src):
        names.update(re.findall(r'"([^"]+)"', tup))
    return names


class RegistryTest(unittest.TestCase):
    def test_every_dispatched_command_is_registered(self):
        missing = _dispatched_commands() - set(sim_commands.COMMANDS)
        self.assertEqual(missing, set(),
                         f"commands with no registry entry: {missing}")

    def test_overview_mentions_every_command(self):
        text = sim_commands.overview()
        for name in sim_commands.COMMANDS:
            self.assertIn(f"./sim {name}", text,
                          f"overview() is missing {name!r}")

    def test_command_help_up(self):
        text = sim_commands.command_help("up")
        self.assertIn("--fresh", text)
        self.assertIn("--open-web", text)

    def test_allowed_flags_up_includes_shared(self):
        flags = sim_commands.allowed_flags("up")
        for f in ("--fresh", "--open-web", "--user", "--password",
                  "--auth-server", "--slot-base", "--with", "--shared",
                  "--force"):
            self.assertIn(f, flags)


class CliHelpTest(unittest.TestCase):
    def _run(self, *args):
        return subprocess.run([SIM, *args], cwd=simlib.SIM_ROOT,
                              capture_output=True, text=True, timeout=30)

    def test_command_help_forms_exit_0(self):
        for args in (("up", "help"), ("clean", "help"),
                     ("help", "clean"), ("down", "--help")):
            p = self._run(*args)
            self.assertEqual(p.returncode, 0,
                             f"./sim {' '.join(args)}: {p.stderr}")
            self.assertIn("usage:", p.stdout,
                          f"./sim {' '.join(args)} printed no usage")

    def test_overview_forms(self):
        for args in ((), ("help",)):
            p = self._run(*args)
            self.assertEqual(p.returncode, 0)
            self.assertIn("./sim up", p.stdout)

    def test_bad_flag_names_the_flag(self):
        p = self._run("up", "--bogus")
        self.assertNotEqual(p.returncode, 0)
        self.assertIn("unknown --bogus - ./sim up help",
                      p.stderr + p.stdout)

    def test_missing_positional_points_at_help(self):
        # SL-0124: a missing <file> argument exits non-zero with the
        # registry usage line plus the ./sim <cmd> help pointer.
        p = self._run("show")
        self.assertNotEqual(p.returncode, 0)
        self.assertIn("./sim show help", p.stderr + p.stdout)


class ResolveSimArgTest(unittest.TestCase):
    def _tmp_root(self):
        root = tempfile.mkdtemp()
        os.makedirs(os.path.join(root, "sims"))
        os.makedirs(os.path.join(root, "tests", "area"))
        return root

    def test_existing_path_passthrough(self):
        self.assertEqual(
            simlib.resolve_sim_arg("sims/copters3.yaml"),
            "sims/copters3.yaml")

    def test_bare_sim_name(self):
        self.assertTrue(
            simlib.resolve_sim_arg("copters3")
            .endswith(os.path.join("sims", "copters3.yaml")))

    def test_unknown_name_is_error(self):
        with self.assertRaises(simlib.SimError) as cm:
            simlib.resolve_sim_arg("nosuchsim")
        self.assertIn("no sim or test named", str(cm.exception))

    def test_tmp_root_kinds_and_ambiguity(self):
        """sims/ match with default kinds; tests/** only when "tests"
        is in kinds; a name in both is ambiguous."""
        root = self._tmp_root()
        open(os.path.join(root, "sims", "foo.yaml"), "w").close()
        open(os.path.join(root, "tests", "area", "bar.yaml"),
             "w").close()
        open(os.path.join(root, "tests", "area", "foo.yaml"),
             "w").close()
        old = simlib.SIM_ROOT
        simlib.SIM_ROOT = root
        try:
            self.assertEqual(simlib.resolve_sim_arg("foo"),
                             os.path.join(root, "sims", "foo.yaml"))
            with self.assertRaises(simlib.SimError):
                # tests are not searched under the default kinds
                simlib.resolve_sim_arg("bar")
            self.assertEqual(
                simlib.resolve_sim_arg("bar", ("sims", "tests")),
                os.path.join(root, "tests", "area", "bar.yaml"))
            with self.assertRaises(simlib.SimError) as cm:
                simlib.resolve_sim_arg("foo", ("sims", "tests"))
            self.assertIn("ambiguous", str(cm.exception))
        finally:
            simlib.SIM_ROOT = old


if __name__ == "__main__":
    unittest.main()
