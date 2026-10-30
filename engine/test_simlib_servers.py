#!/usr/bin/env python3
"""test_simlib_servers.py - SL-0145: the real server stack execs the
Rust server binaries instead of `node ./src/server.js`.

Covers (no fleet - pure simlib resolution + gen_server output):
  - server_bin() answers <repo>/target/release/<name> for all four
    servers;
  - binaries_needed() for a real sim with auth + udp_relay lists
    andruavmainserver, andruavauth and droneengage-udp-proxy and no
    src/server.js;
  - gen_server writes launch.<proc>.sh scripts that exec the binary
    with --config=<rel> and the --sim_run pid marker, from the repo
    cwd.
"""

import os
import shutil
import sys
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


SIM_NAME = "_test_srv_bins"


def _real_sim(auth=True, udp_relay=True):
    """The resolved shape gen_server() consumes: a real server spec
    with auth and udp_relay, one no-fc unit."""
    return {
        "name": SIM_NAME,
        "slot_base": 40,
        "server": {
            "type": "real",
            "ip": "127.0.0.1",
            "port": 62008,
            "control_port": 62009,
            "auth": auth,
            "auth_ip": "127.0.0.1",
            "auth_port": 62018,
            "auth_s2s_port": 62019,
            "auth_verify_ssl": False,
            "account": {"user": "sim@sim.droneengage.com",
                        "code": "sim1234"},
            "gcs_party": "GCS_TEST",
            "seed": 1337,
            "clock_offset_ms": 0,
            "noclock": [],
            "link_proxy": False,
            "udp_relay": {"port": 62020, "gcs_peer_policy": "open",
                          "ssl": True} if udp_relay else None,
        },
        "units": [{"name": "u1", "modules": [], "fc": None,
                   "ports": {}, "android": None}],
    }


class ServerBinTest(unittest.TestCase):
    def test_server_bin_all_four(self):
        for proc, bin_name in simlib.SERVER_BINS.items():
            self.assertEqual(
                simlib.server_bin("/some/repo", bin_name),
                os.path.join("/some/repo", "target", "release",
                             bin_name),
                proc)

    def test_binaries_needed_lists_server_bins(self):
        sim = _real_sim()
        out = simlib.binaries_needed(sim, simlib.load_catalog())
        for want in ("andruavmainserver", "andruavauth",
                     "droneengage-udp-proxy"):
            hit = [b for b in out if b.endswith(
                os.path.join("target", "release", want))]
            self.assertTrue(hit, f"{want} not in {out}")
        self.assertFalse([b for b in out if "server.js" in b], out)

    def test_binaries_needed_no_auth_no_relay(self):
        sim = _real_sim(auth=False, udp_relay=False)
        out = simlib.binaries_needed(sim, simlib.load_catalog())
        self.assertFalse([b for b in out
                          if "andruavauth" in b
                          or "droneengage-udp-proxy" in b], out)
        self.assertTrue([b for b in out
                         if b.endswith("andruavmainserver")], out)


class GenServerScriptsTest(unittest.TestCase):
    def tearDown(self):
        shutil.rmtree(simlib.run_dir(SIM_NAME), True)

    def test_launch_scripts_exec_the_binaries(self):
        sim = _real_sim()
        procs = simlib.gen_server(sim)
        self.assertEqual(procs, ["de_auth", "udp_relay", "de_server",
                                 "gcs_bridge"])
        sdir = simlib.server_dir(SIM_NAME)
        expect = {"de_auth": ("andruavauth", simlib.AUTH_REPO),
                  "udp_relay": ("droneengage-udp-proxy",
                                simlib.UDP_RELAY_REPO),
                  "de_server": ("andruavmainserver",
                                simlib.SERVER_REPO)}
        for proc, (bin_name, repo) in expect.items():
            path = os.path.join(sdir, f"launch.{proc}.sh")
            with open(path) as f:
                text = f.read()
            self.assertIn(f"cd '{repo}'", text, proc)
            self.assertIn(
                "exec '%s' --config="
                % simlib.server_bin(repo, bin_name), text, proc)
            self.assertIn("--sim_run=", text, proc)
            self.assertNotIn("server.js", text, proc)


if __name__ == "__main__":
    unittest.main()
