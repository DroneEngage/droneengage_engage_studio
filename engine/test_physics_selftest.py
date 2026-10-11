#!/usr/bin/env python3
"""P5-16 pure gate: Mavlink3DMap's physics core selftest
(frontend/tools/physics_selftest.mjs - free fall, terminal velocity,
crosswind drift, determinism, roof/canopy contacts, frame-rate
independence) in plain Node, no browser, no GPU. Skips when the
Mavlink3DMap checkout (MAVLINK3DMAP_DIR) or node is absent.

    python3 engine/test_physics_selftest.py
"""
import os
import shutil
import subprocess
import sys
import unittest

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import simlib  # noqa: E402

SELFTEST = os.path.join(simlib.MAVLINK3DMAP_DIR or "", "frontend", "tools",
                        "physics_selftest.mjs")
NODE = shutil.which("node")


@unittest.skipUnless(simlib.MAVLINK3DMAP_DIR and os.path.isfile(SELFTEST)
                     and NODE, "Mavlink3DMap physics core / node absent")
class PhysicsSelftest(unittest.TestCase):
    def test_selftest_passes(self):
        p = subprocess.run([NODE, SELFTEST], capture_output=True,
                           text=True, timeout=300)
        sys.stdout.write(p.stdout)
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        self.assertNotIn("FAIL", p.stdout)


if __name__ == "__main__":
    unittest.main()
