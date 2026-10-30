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

import importlib.machinery
import importlib.util
import io
import json
import os
import re
import subprocess
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
sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

import sim_commands  # noqa: E402
import simlib  # noqa: E402

SIM = os.path.join(simlib.SIM_ROOT, "sim")


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

    def test_command_help_colored_on_tty(self):
        # SL-0125: command help is printed through simlib.print_usage -
        # green commands, yellow <params>/[flags], plain when not a tty.
        doc = sim_commands.command_help("up")
        out = _print_usage_captured(doc, tty=True)
        self.assertIn("\x1b[32m./sim up \x1b[33m"
                      "<sim.yaml|de_servers|de_webclient>", out)
        self.assertIn("\x1b[33m[--fresh]", out)
        # flag rows are parameters: yellow, not command green
        self.assertIn("\x1b[33m  --fresh", out)
        self.assertEqual(_print_usage_captured(doc, tty=False),
                         doc + "\n")

    def test_usage_error_exit_colored_on_tty(self):
        # usage errors (missing arg, bad flag) get the print_usage
        # colors on stderr: command green, <params>/[flags] yellow, the
        # ./sim <cmd> help pointer green; plain text off a tty.
        for tty in (True, False):
            buf = _Tty() if tty else io.StringIO()
            old = sys.stderr
            sys.stderr = buf
            try:
                with self.assertRaises(SystemExit):
                    simlib.usage_error_exit(
                        "./sim show <sim.yaml> [--json]"
                        " - ./sim show help")
            finally:
                sys.stderr = old
            if tty:
                self.assertIn("\x1b[32m./sim show \x1b[33m<sim.yaml>",
                              buf.getvalue())
                self.assertIn("\x1b[33m[--json]", buf.getvalue())
                self.assertIn("\x1b[32m./sim show help", buf.getvalue())
            else:
                self.assertNotIn("\x1b[", buf.getvalue())
                self.assertIn("./sim show help", buf.getvalue())

    def test_allowed_flags_up_includes_shared(self):
        flags = sim_commands.allowed_flags("up")
        for f in ("--fresh", "--open-web", "--user", "--password",
                  "--auth-server", "--slot-base", "--with", "--shared",
                  "--force"):
            self.assertIn(f, flags)

    def test_every_command_has_a_group(self):
        # SL-0135: the overview prints the commands under intent-group
        # headers - an entry without a "group", or one naming a group
        # not in GROUPS, fails here
        groups = {g for g, _ in sim_commands.GROUPS}
        for name, cmd in sim_commands.COMMANDS.items():
            self.assertIn(cmd.get("group"), groups,
                          f"{name} has no group in GROUPS")

    def test_overview_groups_each_command_once(self):
        # SL-0135: every GROUPS header is printed and every command's
        # usage line appears exactly once (under its group)
        text = sim_commands.overview()
        for group, _ in sim_commands.GROUPS:
            self.assertIn(f"{group} - ", text,
                          f"overview() is missing the {group} header")
        for name, cmd in sim_commands.COMMANDS.items():
            self.assertEqual(text.count(cmd["usage"][0]), 1,
                             f"{name}'s usage line is not printed "
                             "exactly once")


class KindOfTest(unittest.TestCase):
    """SL-0137: kind is inferred from the file's place in the tree -
    never stored, so it cannot drift."""

    def test_kind_of_paths(self):
        self.assertEqual(simlib.kind_of("tests/faults/x.yaml"), "test")
        self.assertEqual(simlib.kind_of("tests/a/b/x.yaml"), "test")
        self.assertEqual(simlib.kind_of("scenarios/demos/x.yaml"),
                         "demo")
        self.assertEqual(simlib.kind_of("scenarios/examples/x.yaml"),
                         "example")
        self.assertEqual(simlib.kind_of("scenarios/challenges/x.yaml"),
                         "challenge")

    def test_kind_of_absolute_path(self):
        p = os.path.join(simlib.SIM_ROOT, "tests", "smoke_copters.yaml")
        self.assertEqual(simlib.kind_of(p), "test")

    def test_kind_of_non_run_files(self):
        self.assertIsNone(simlib.kind_of("sims/copters3.yaml"))
        self.assertIsNone(simlib.kind_of("scenarios/loose.yaml"))
        self.assertIsNone(simlib.kind_of("/etc/fstab"))


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

    def test_list_task_filter(self):
        # SL-0137: --task selects only files whose `tasks:` carry the
        # id; the --json row carries kind + tasks
        tagged = os.path.join(simlib.SIM_ROOT, "tests",
                              "zz_sl0137_tagged.yaml")
        with open(tagged, "w") as fh:
            fh.write("# zz_sl0137_tagged - temporary --task filter "
                     "fixture\ntasks: [SL-9999]\n")
        try:
            p = self._run("list", "tests", "--task", "SL-9999")
            self.assertEqual(p.returncode, 0, p.stderr)
            self.assertIn("zz_sl0137_tagged.yaml", p.stdout)
            self.assertNotIn("smoke_copters", p.stdout)
            p = self._run("list", "tests", "--task", "SL-9999", "--json")
            rows = json.loads(p.stdout)["roots"][0]["files"]
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["tasks"], ["SL-9999"])
            self.assertEqual(rows[0]["kind"], "test")
            # a different id selects nothing
            p = self._run("list", "tests", "--task", "SL-9998")
            self.assertNotIn("zz_sl0137_tagged.yaml", p.stdout)
        finally:
            os.unlink(tagged)

    def test_list_kind_filter(self):
        # SL-0137: --kind keeps only the path-inferred kind
        p = self._run("list", "--kind", "demo")
        self.assertEqual(p.returncode, 0, p.stderr)
        for line in p.stdout.splitlines():
            line = line.strip()
            if line.endswith(".yaml") or "yaml " in line:
                self.assertTrue(line.startswith("scenarios/demos/"),
                                line)
        self.assertFalse(re.search(r"^\s+tests/\S+\.yaml", p.stdout,
                                   re.M))


class StatusColorTest(unittest.TestCase):
    """SL-0155: the status/catalog commands (ports, sims, show, list,
    verbs, clean) colorize on a tty with the help palette and emit zero
    escapes when piped."""

    def _sim(self):
        loader = importlib.machinery.SourceFileLoader("sim_cli", SIM)
        spec = importlib.util.spec_from_loader("sim_cli", loader)
        mod = importlib.util.module_from_spec(spec)
        loader.exec_module(mod)
        return mod

    def _main_out(self, mod, argv, tty):
        buf = _Tty() if tty else io.StringIO()
        old = sys.stdout
        sys.stdout = buf
        try:
            mod.main(list(argv))
        finally:
            sys.stdout = old
        return buf.getvalue()

    def test_catalog_commands_color_on_tty(self):
        mod = self._sim()
        for argv in (["verbs"], ["list"], ["show", "copters3"],
                     ["clean", "--dry-run"]):
            out = self._main_out(mod, argv, tty=True)
            self.assertIn("\x1b[", out, argv)

    def test_ports_color_on_tty(self):
        mod = self._sim()
        rows = [("u1.de_comm", 62000, "udp"),
                ("u1.sitl.fcb", 8160, "tcp")]
        with mock.patch.object(simlib, "sim_names",
                               return_value=["fake1"]), \
             mock.patch.object(simlib, "read_pids", return_value=[1]), \
             mock.patch.object(simlib, "load_manifest",
                               return_value={"units": [{}]}), \
             mock.patch.object(simlib, "port_rows", return_value=rows), \
             mock.patch.object(simlib, "port_bound",
                               side_effect=lambda p, proto: p == 62000):
            out = self._main_out(mod, ["ports", "--all"], tty=True)
        self.assertIn("\x1b[32mbound", out)   # bound green
        self.assertIn("\x1b[33mfree", out)    # free yellow
        self.assertIn("\x1b[36m 62000", out)  # ports cyan
        self.assertIn("\x1b[1mfake1", out)    # sim name bold

    def test_sims_color_on_tty(self):
        mod = self._sim()
        manifest = {"units": [{}], "server": {"type": "real"}}
        with mock.patch.object(simlib, "sim_names",
                               return_value=["fake1"]), \
             mock.patch.object(simlib, "read_pids", return_value=[1]), \
             mock.patch.object(simlib, "load_manifest",
                               return_value=manifest):
            out = self._main_out(mod, ["sims"], tty=True)
        self.assertIn("\x1b[1mfake1", out)    # sim name bold
        self.assertIn("\x1b[36mreal", out)    # server type cyan
        self.assertIn("\x1b[32m1", out)       # alive count green

    def test_piped_commands_have_no_escapes(self):
        for argv in (["verbs"], ["list"], ["show", "copters3"],
                     ["clean", "--dry-run"], ["sims", "--json"],
                     ["verbs", "--json"], ["list", "--json"]):
            p = subprocess.run([SIM, *argv], cwd=simlib.SIM_ROOT,
                               capture_output=True, text=True, timeout=30)
            self.assertNotIn("\x1b", p.stdout,
                             f"./sim {' '.join(argv)} leaked ANSI")
            self.assertEqual(p.returncode, 0,
                             f"./sim {' '.join(argv)}: {p.stderr}")

    def test_json_forms_stay_valid(self):
        for argv in (["sims", "--json"], ["verbs", "--json"],
                     ["list", "--json"]):
            p = subprocess.run([SIM, *argv], cwd=simlib.SIM_ROOT,
                               capture_output=True, text=True, timeout=30)
            json.loads(p.stdout)


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
