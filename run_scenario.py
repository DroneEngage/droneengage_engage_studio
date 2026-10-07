#!/usr/bin/env python3
"""run_scenario.py - de.scenario/1 runner (TASK-P5-07).

  ./run_scenario.py <scenario.yaml> [--seed N] [--keep] [--hold]
  ./run_scenario.py --suite <dir> --seeds 1,2,3

Compatibility shim - `./sim run <file> [--seed N] [--keep] [--hold]`
and `./sim suite <dir> --seeds ...` are the same engine; new work uses
./sim.

--hold parks the run after the timeline with the world stream and the
render procs still live (the interactive Mavlink3DMap session - ./sim
view is the same thing) until Ctrl-C, then assertions/teardown run as
usual.

Each run gets its own port block (seed-derived slot_base) and its own
var/results/<utc>/<name>/ directory: journals, process logs, telemetry.jsonl
(tlog), operator.log, truth.jsonl, verify.json, summary.json, junit.xml.
Exit 0 only when every non-SKIP assertion passed.
"""

import os
import sys

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "var"), "pycache"))

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "engine"))
import scenario_runner

if __name__ == "__main__":
    sys.exit(scenario_runner.main(sys.argv[1:]))
