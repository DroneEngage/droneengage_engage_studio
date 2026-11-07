#!/usr/bin/env python3
"""test_sim_instance.py - SL-0210: `./sim up --name <id>` named instances.

Covers (no SITL, no fleet - simlib.load_sim resolution + a fake live sim):
  - --name <id> (SIM_NAME_INSTANCE) renames the sim and every unit with
    a _<id> suffix (inst_demo -> inst_demo_copter2, u1 -> u1_copter2)
    and the GCS party follows (GCS_INST_DEMO_COPTER2);
  - an uppercase --name is lowercased and accepted (case is never a
    fault); a space or a non-alnum first character is an error;
  - SIM_NAME_AUTO (the --shared re-up channel): no --name + a live
    same-named sim -> automatic s<slot> suffix on sim and unit names;
    without AUTO the base name is kept (the restart path);
  - --slot-base any + AUTO: the suffix uses the slot that actually won;
  - launcher.down(<instance>) stops only that instance's pids - the
    other instance stays live;
  - subprocess: --name is refused on verbs that are not `up` and the
    `up` help documents it.
"""

import json
import os
import subprocess
import sys
import tempfile
import time
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

import simlib  # noqa: E402
import launcher  # noqa: E402

SIM = os.path.join(simlib.SIM_ROOT, "sim")

_ENV_KEYS = ("SIM_NAME_INSTANCE", "SIM_NAME_AUTO", "SIM_SLOT_BASE_OVERRIDE")


def _spec(name="inst_demo", slot=10):
    """A minimal sim spec - comm_only is a no-fc block, so this loads
    without any SITL or flight controller. The server port is written as
    SERVER_PORT + slot_base, the form scenario specs use, so a
    --slot-base override moves it like every unit port."""
    return {
        "version": 1,
        "name": name,
        "slot_base": slot,
        "server": {"type": "fake", "port": simlib.SERVER_PORT + slot},
        "units": [{"block": "comm_only", "name": "u1"}],
    }


def _fake_live(name, run_dir, procs, ports=(63100, 63101)):
    """A live sim on paper: manifest + pidfile of a sleeper whose cmdline
    carries RUN_DIR (what pid_is_ours checks)."""
    os.makedirs(simlib.pids_dir(name), exist_ok=True)
    json.dump({"name": name, "units": [],
               "server": {"type": "fake", "port": ports[0],
                          "control_port": ports[1], "auth": False},
               "owner": simlib.owner_record()},
              open(simlib.manifest_path(name), "w"))
    p = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(60)", run_dir])
    procs.append(p)
    open(os.path.join(simlib.pids_dir(name), "u1.proc.pid"),
         "w").write(str(p.pid))


class SimInstanceTest(unittest.TestCase):
    def setUp(self):
        self._saved = {k: os.environ.get(k) for k in _ENV_KEYS}
        for k in _ENV_KEYS:
            os.environ.pop(k, None)

    def tearDown(self):
        for k, v in self._saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

    def test_explicit_name(self):
        os.environ["SIM_NAME_INSTANCE"] = "copter2"
        sim = simlib.load_sim(_spec())
        self.assertEqual(sim["name"], "inst_demo_copter2")
        self.assertEqual([u["name"] for u in sim["units"]],
                         ["u1_copter2"])
        # on a shared external stack each instance needs its own GCS
        # party - it is derived from the instance name
        sim = simlib.load_sim(_spec() | {
            "server": {"type": "external", "ip": "127.0.0.1",
                       "port": 9966, "auth": False}})
        self.assertEqual(sim["server"]["gcs_party"],
                         "GCS_INST_DEMO_COPTER2")

    def test_explicit_name_lowercased(self):
        os.environ["SIM_NAME_INSTANCE"] = "CoPtEr2"
        sim = simlib.load_sim(_spec())
        self.assertEqual(sim["name"], "inst_demo_copter2")
        self.assertEqual([u["name"] for u in sim["units"]],
                         ["u1_copter2"])

    def test_explicit_name_rejected(self):
        for bad in ("has space", "-lead", "_lead", ""):
            os.environ["SIM_NAME_INSTANCE"] = bad
            with self.assertRaises(simlib.SimError, msg=bad):
                simlib.load_sim(_spec())

    def test_counted_fleet_names(self):
        os.environ["SIM_NAME_INSTANCE"] = "b"
        spec = _spec()
        spec["units"] = [{"block": "comm_only", "name": "u", "count": 3}]
        sim = simlib.load_sim(spec)
        self.assertEqual([u["name"] for u in sim["units"]],
                         ["u1_b", "u2_b", "u3_b"])

    def test_auto_name_on_shared_reup(self):
        os.environ["SIM_NAME_AUTO"] = "1"
        with mock.patch.object(simlib, "live_sim_names",
                               return_value=["inst_demo"]):
            sim = simlib.load_sim(_spec())
        self.assertEqual(sim["name"], "inst_demo_s10")
        self.assertEqual([u["name"] for u in sim["units"]],
                         ["u1_s10"])

    def test_no_auto_without_live_base(self):
        os.environ["SIM_NAME_AUTO"] = "1"
        with mock.patch.object(simlib, "live_sim_names",
                               return_value=["other_sim"]):
            sim = simlib.load_sim(_spec())
        self.assertEqual(sim["name"], "inst_demo")
        self.assertEqual([u["name"] for u in sim["units"]], ["u1"])

    def test_no_auto_without_flag(self):
        # plain load (the restart path): same name, no suffix
        with mock.patch.object(simlib, "live_sim_names",
                               return_value=["inst_demo"]):
            sim = simlib.load_sim(_spec())
        self.assertEqual(sim["name"], "inst_demo")

    def test_auto_name_slot_base_any(self):
        # a real live sim under an isolated RUN_DIR - the free-slot scan
        # reads its manifest, so a live_sim_names mock is not enough here
        procs = []
        os.environ["SIM_NAME_AUTO"] = "1"
        os.environ["SIM_SLOT_BASE_OVERRIDE"] = "any"
        try:
            with tempfile.TemporaryDirectory() as td:
                with mock.patch.object(simlib, "RUN_DIR", td):
                    # the live base instance holds slot 10's ports, so
                    # the scan must park the copy on a different slot
                    _fake_live("inst_demo", td, procs,
                               ports=(simlib.SERVER_PORT + 10,
                                      simlib.SERVER_PORT + 11))
                    sim = simlib.load_sim(_spec())
        finally:
            for p in procs:
                p.terminate()
        self.assertEqual(sim["name"], f"inst_demo_s{sim['slot_base']}")
        self.assertNotEqual(sim["slot_base"], 10)
        self.assertEqual([u["name"] for u in sim["units"]],
                         [f"u1_s{sim['slot_base']}"])

    def test_down_one_instance(self):
        procs = []
        with tempfile.TemporaryDirectory() as td:
            with mock.patch.object(simlib, "RUN_DIR", td):
                _fake_live("inst_demo", td, procs)
                _fake_live("inst_demo_s10", td, procs)
                self.assertEqual(sorted(simlib.live_sim_names()),
                                 ["inst_demo", "inst_demo_s10"])
                launcher.down("inst_demo_s10", quiet=True)
                self.assertEqual(simlib.live_sim_names(), ["inst_demo"])
                self.assertEqual(simlib.read_pids("inst_demo_s10"), {})
        for p in procs:
            p.terminate()

    def test_name_flag_only_for_up(self):
        r = subprocess.run([SIM, "show", "copters3", "--name", "x"],
                           capture_output=True, text=True)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("--name only works with ./sim up",
                      r.stdout + r.stderr)

    def test_up_help_documents_name(self):
        r = subprocess.run([SIM, "up", "help"],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0)
        self.assertIn("--name", r.stdout)


if __name__ == "__main__":
    unittest.main()
