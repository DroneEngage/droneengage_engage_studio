#!/usr/bin/env python3
"""test_sim_params.py - SL-0116: sim yaml `params:` block with ${name}
substitution and `--set name=value` overrides.

Covers (no SITL, no fleet - pure simlib.load_sim resolution):
  - a sim yaml with `params:` and `${name}` references substitutes the
    resolved value into the server port before block/sim resolution;
  - `--set name=value` overrides the default (via set_param_overrides,
    the same hook the ./sim --set CLI flag uses);
  - type coercion: an int param resolves to a real int, not a string;
  - choices validation: a value outside the param's choices is an error;
  - an unknown --set name is a clear error listing the available params;
  - --set on a sim with no `params:` block at all is an error.
"""

import os
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

import simlib  # noqa: E402


def _sim_spec(params=None, server_port_ref="${server_port}"):
    """A minimal sim spec with a params block and a ${server_port}
    reference in the server port. comm_only is a no-fc block, so this
    loads without any SITL or flight controller."""
    spec = {
        "version": 1,
        "name": "params_test",
        "slot_base": 10,
        "server": {"type": "fake", "port": server_port_ref},
        "units": [{"block": "comm_only", "name": "u1"}],
    }
    if params is not None:
        spec["params"] = params
    return spec


class SimParamsTest(unittest.TestCase):
    def setUp(self):
        # --set overrides are a process-global (like --module); isolate
        # every test and clear the SIM_PARAMS env channel too
        simlib.set_param_overrides({})
        os.environ.pop("SIM_PARAMS", None)

    def tearDown(self):
        simlib.set_param_overrides({})
        os.environ.pop("SIM_PARAMS", None)

    def test_substitution_from_yaml_file(self):
        """a tmp sim yaml with params + ${server_port} substitutes the
        resolved int into the server port before resolution."""
        yaml_text = (
            "# what it proves: ${name} substitution from a params block\n"
            "version: 1\n"
            "name: params_file\n"
            "slot_base: 10\n"
            "params:\n"
            "  server_port:\n"
            "    default: 62900\n"
            "    type: int\n"
            "    help: comm server port\n"
            "server:\n"
            "  type: fake\n"
            "  port: ${server_port}\n"
            "units:\n"
            "  - {block: comm_only, name: u1}\n"
        )
        with tempfile.NamedTemporaryFile("w", suffix=".yaml",
                                         delete=False) as fh:
            fh.write(yaml_text)
            path = fh.name
        try:
            sim = simlib.load_sim(path)
        finally:
            os.unlink(path)
        self.assertEqual(sim["server"]["port"], 62900)
        self.assertEqual(sim["param_values"]["server_port"], 62900)
        self.assertIsInstance(sim["param_values"]["server_port"], int)
        self.assertIn("server_port", sim["params"])

    def test_set_override(self):
        """--set overrides the param default and flows into the server."""
        simlib.set_param_overrides({"server_port": "63000"})
        sim = simlib.load_sim(_sim_spec(params={
            "server_port": {"default": 62900, "type": "int",
                            "help": "port"}}))
        self.assertEqual(sim["server"]["port"], 63000)
        self.assertEqual(sim["param_values"]["server_port"], 63000)

    def test_type_coercion_int(self):
        """an int param resolves to a real int, not a string, both from
        the default and from a --set override."""
        sim = simlib.load_sim(_sim_spec(params={
            "server_port": {"default": 62900, "type": "int",
                            "help": "port"}}))
        self.assertIsInstance(sim["param_values"]["server_port"], int)
        self.assertEqual(sim["param_values"]["server_port"], 62900)
        # override coerced to int too
        simlib.set_param_overrides({"server_port": "63100"})
        sim = simlib.load_sim(_sim_spec(params={
            "server_port": {"default": 62900, "type": "int",
                            "help": "port"}}))
        self.assertIsInstance(sim["param_values"]["server_port"], int)
        self.assertEqual(sim["param_values"]["server_port"], 63100)

    def test_type_coercion_float(self):
        """a float param coerces a --set string to float."""
        simlib.set_param_overrides({"rate": "1.5"})
        sim = simlib.load_sim(_sim_spec(
            params={"rate": {"default": 1.0, "type": "float", "help": "x"},
                    "server_port": {"default": 62900, "type": "int",
                            "help": "port"}},
            server_port_ref=62900))
        self.assertIsInstance(sim["param_values"]["rate"], float)
        self.assertEqual(sim["param_values"]["rate"], 1.5)

    def test_bad_int_is_error(self):
        """a non-numeric --set on an int param is a clear error."""
        simlib.set_param_overrides({"server_port": "notanint"})
        with self.assertRaises(simlib.SimError):
            simlib.load_sim(_sim_spec(params={
                "server_port": {"default": 62900, "type": "int",
                            "help": "port"}}))

    def test_choices_validation(self):
        """a --set value outside the param's choices is an error."""
        simlib.set_param_overrides({"net": "wifi"})
        with self.assertRaises(simlib.SimError):
            simlib.load_sim(_sim_spec(
                params={"net": {"default": "auto", "help": "net",
                                "choices": ["auto", "adb_reverse", "lan"]},
                        "server_port": {"default": 62900, "type": "int",
                            "help": "port"}},
                server_port_ref=62900))

    def test_choices_default_passes(self):
        """the default is accepted even when choices are set."""
        sim = simlib.load_sim(_sim_spec(
            params={"net": {"default": "auto", "help": "net",
                            "choices": ["auto", "adb_reverse", "lan"]},
                    "server_port": {"default": 62900, "type": "int",
                            "help": "port"}},
            server_port_ref=62900))
        self.assertEqual(sim["param_values"]["net"], "auto")

    def test_unknown_param_is_error(self):
        """--set with a name not in the params block is a clear error
        listing the available params."""
        simlib.set_param_overrides({"nope": "1"})
        with self.assertRaises(simlib.SimError) as cm:
            simlib.load_sim(_sim_spec(params={
                "server_port": {"default": 62900, "type": "int",
                            "help": "port"}}))
        self.assertIn("server_port", str(cm.exception))

    def test_no_params_block_with_set_is_error(self):
        """--set on a sim with no params: block at all is an error."""
        simlib.set_param_overrides({"server_port": "63000"})
        with self.assertRaises(simlib.SimError):
            simlib.load_sim(_sim_spec(server_port_ref=62900))

    def test_embedded_substitution(self):
        """a ${name} embedded in a larger string is str()-substituted."""
        sim = simlib.load_sim({
            "version": 1, "name": "embed_test", "slot_base": 10,
            "params": {"serial": {"default": "emulator-5554", "help": "adb serial"}},
            "server": {"type": "fake", "port": 62900},
            "units": [{"block": "comm_only", "name": "u1",
                       "config": {"de_comm": {"sv_unit_description":
                                               "dev-${serial}"}}}]})
        desc = sim["units"][0]["config"]["de_comm"]["sv_unit_description"]
        self.assertEqual(desc, "dev-emulator-5554")

    def test_unknown_ref_is_error(self):
        """a ${name} that matches no declared param is a clear error."""
        with self.assertRaises(simlib.SimError) as cm:
            simlib.load_sim({
                "version": 1, "name": "unset_test", "slot_base": 10,
                "server": {"type": "fake", "port": 62900},
                "units": [{"block": "comm_only", "name": "u1",
                           "config": {"de_comm": {"sv_unit_description":
                                                   "keep-${unknown}"}}}]})
        self.assertIn("unknown param ${unknown}", str(cm.exception))

    def test_env_channel_override(self):
        """the SIM_PARAMS env var is the subprocess inheritance channel:
        overrides set there apply like --set."""
        os.environ["SIM_PARAMS"] = '{"server_port": "63200"}'
        sim = simlib.load_sim(_sim_spec(params={
            "server_port": {"default": 62900, "type": "int",
                            "help": "comm server port"}}))
        self.assertEqual(sim["server"]["port"], 63200)

    def test_bool_coercion(self):
        """a bool param coerces --set strings (true/false/1/0/yes/no,
        case-insensitive); anything else is a clear error."""
        decl = {"realtime": {"default": False, "type": "bool",
                             "help": "run wall-clock"}}
        for raw, want in (("true", True), ("YES", True), ("1", True),
                          ("false", False), ("0", False), ("No", False)):
            simlib.set_param_overrides({"realtime": raw})
            sim = simlib.load_sim(_sim_spec(
                params={**decl,
                        "server_port": {"default": 62900, "type": "int",
                                        "help": "port"}},
                server_port_ref=62900))
            self.assertIs(sim["param_values"]["realtime"], want, raw)
        simlib.set_param_overrides({"realtime": "maybe"})
        with self.assertRaises(simlib.SimError):
            simlib.load_sim(_sim_spec(
                params={**decl,
                        "server_port": {"default": 62900, "type": "int",
                                        "help": "port"}},
                server_port_ref=62900))

    def test_missing_help_is_error(self):
        """a param without a one-line help: is a clear error naming it."""
        with self.assertRaises(simlib.SimError) as cm:
            simlib.load_sim(_sim_spec(params={
                "server_port": {"default": 62900, "type": "int"}}))
        self.assertIn("server_port", str(cm.exception))

    def test_bad_type_is_error(self):
        """a type outside str|int|float|bool names the param."""
        with self.assertRaises(simlib.SimError) as cm:
            simlib.load_sim(_sim_spec(params={
                "server_port": {"default": 62900, "type": "port",
                                "help": "x"}}))
        self.assertIn("server_port", str(cm.exception))

    def test_param_changes_hash(self):
        """different resolved values -> different sim hash (an `up`
        regenerates instead of reusing the warm sim)."""
        params = {"server_port": {"default": 62900, "type": "int",
                                  "help": "port"}}
        sim1 = simlib.load_sim(_sim_spec(params=params))
        simlib.set_param_overrides({"server_port": "63100"})
        sim2 = simlib.load_sim(_sim_spec(params=params))
        self.assertNotEqual(sim1["hash"], sim2["hash"])


if __name__ == "__main__":
    unittest.main()
