#!/usr/bin/env python3
"""test_golden.py - byte-compare the stub model replays against
stubs/golden/*.log (TASK-P5-10, spec step 0).

Each golden is the --selftest log a stub produced on worlds/forest_a
seed 7 with the world_determinism poses (the same unit, seed and
pose files that scenario's stub_deterministic replays use):

    stub_ai_box    worlds/pose_cam1.jsonl   30 s
    stub_sdr       worlds/pose_mast.jsonl   30 s
    stub_payload   worlds/pose_drop.jsonl   20 s

A fresh replay must be byte-identical: a world with no obstacles and
the default sensor keys keeps today's RNG stream and output (the D1
acceptance). The models run in-process through stubs/model_import.py,
so this needs no databus checkout:

    python3 stubs/test_golden.py             # compare (CI, pure)
    python3 stubs/test_golden.py --record    # rewrite the goldens
"""
import json
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, HERE)

import model_import                     # noqa: E402

FOREST = os.path.join(ROOT, "worlds", "forest_a.yaml")
SEED = 7
UNIT = "st1"

# module, pose file, --dur seconds
GOLDENS = [
    ("stub_ai_box", "pose_cam1.jsonl", 30),
    ("stub_sdr", "pose_mast.jsonl", 30),
    ("stub_payload", "pose_drop.jsonl", 20),
]


def _sdr_config(w):
    """The signal_detect block world.prepare_sim injects on a fleet
    unit (ref = the sdr model's ref_dbm_1m + the first rf entity's tx)."""
    sdr = w.sensors["sdr"]
    tx = [e.tx_dbm for e in w.rf_entities() if e.tx_dbm is not None]
    return {"ref_power_dbm": sdr["ref_dbm_1m"] + (tx[0] if tx else 0.0),
            "path_loss_n": sdr["path_loss_n"], "debounce_sweeps": 3}


def replay(module, pose_file, dur_s, scratch):
    """Run <module> --selftest <pose file> on a scratch run dir; -> the
    stub log bytes. The poses path is passed relative to the repo root
    (the stub logs it verbatim), so the golden is machine-independent."""
    mods = model_import.import_stubs()
    world_mod = mods["stub_base"].world_mod
    mdir = os.path.join(scratch, "run", "sim", UNIT, module)
    os.makedirs(mdir)
    cfg = {"stub": {"world": FOREST, "seed": SEED, "unit": UNIT,
                   "sysid": 1, "pose_port": 0},
           "s2s_udp_target_port": "1"}
    if module == "stub_sdr":
        w = world_mod.load(FOREST, SEED)
        cfg["signal_detect"] = _sdr_config(w)
    cfg_path = os.path.join(mdir, f"{module}.config.json")
    with open(cfg_path, "w") as f:
        json.dump(cfg, f)
    cwd = os.getcwd()
    os.chdir(ROOT)
    try:
        rc = mods[module].main(["-c", cfg_path, "--selftest",
                                os.path.join("worlds", pose_file),
                                "--dur", str(dur_s)])
    finally:
        os.chdir(cwd)
    if rc != 0:
        raise RuntimeError(f"{module} selftest exited {rc}")
    log = os.path.join(scratch, "run", "sim", "stubs", f"{UNIT}.log")
    with open(log, "rb") as f:
        return f.read()


class T(unittest.TestCase):
    def test_replays_are_byte_identical_to_the_goldens(self):
        for module, pose_file, dur in GOLDENS:
            with self.subTest(module=module):
                with tempfile.TemporaryDirectory() as d:
                    fresh = replay(module, pose_file, dur, d)
                golden = os.path.join(HERE, "golden", f"{module}.log")
                self.assertTrue(os.path.isfile(golden),
                                f"{golden} missing - record it first")
                with open(golden, "rb") as f:
                    want = f.read()
                self.assertTrue(want, f"{golden} is empty")
                self.assertEqual(fresh, want,
                                 f"{module}: the selftest replay differs "
                                 "from its golden (RNG stream or output "
                                 "changed for an obstacle-free world)")


def record():
    """Rewrite stubs/golden/*.log from the current model code."""
    gdir = os.path.join(HERE, "golden")
    os.makedirs(gdir, exist_ok=True)
    for module, pose_file, dur in GOLDENS:
        with tempfile.TemporaryDirectory() as d:
            data = replay(module, pose_file, dur, d)
        path = os.path.join(gdir, f"{module}.log")
        with open(path, "wb") as f:
            f.write(data)
        print(f"recorded {os.path.relpath(path, ROOT)} ({len(data)} B)")


if __name__ == "__main__":
    if "--record" in sys.argv:
        record()
        sys.exit(0)
    unittest.main()
