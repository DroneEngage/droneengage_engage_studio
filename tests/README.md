# tests/ — the automated regression library

Everything here is run by CI (`ci/run_suite.py`, `./sim suite`). Files
for people to run, watch or copy live in `../scenarios/` instead.

## Two file formats

| Format | Where | Shape |
|---|---|---|
| legacy test | top-level `smoke_copters*.yaml`, `fault_link_loss.yaml`, `mixed_bringup.yaml` | `sim:` + `steps:` (engine/runner.py) |
| de.scenario/1 | every area folder, plus top-level `distance_to.yaml`, `verify_gate.yaml` | `fleet:` + `timeline:` + `assert:` (engine/scenario_runner.py) |

`./sim run` dispatches on the shape, so both run the same way. New tests
should use de.scenario/1 (see `../docs/SCENARIOS.md`).

## Area folders

Folders are named after the feature or the project milestone that
introduced them:

| Folder | What it covers |
|---|---|
| `p0/` | Phase-0 reference plan, end to end |
| `p2/` | Phase-2 mission engine: gates, contingencies, handlers |
| `p4a/` | Phase-4A fleet bus: static chains, link loss, catch-up, restarts, barriers, phases/end |
| `p4b/` | Phase-4B SAR fleet: injected detections, approvals, searcher/ambulance flows (P4B-10 a–k) |
| `p4f/` | Phase-4F launch grants and author-gone cases |
| `p5/` | Phase-5 worlds: canopy/occlusion recall, coverage/POD, render farm |
| `su/` | static units (masts, fixed cameras) taking part in operations |
| `faults/` | battery, link, server and comm faults during missions |
| `closed_env/` | sealed-environment clock semantics |
| `verify/` | deploy-time verifier gate (deadlocks, bad ops) |
| `precland/` | precision landing (stub and real de_precland) |
| `capacity/` | oversized plans: wire/package/FC/GCS capacity and the no-space boundary |
| `camera/`, `gpio/`, `sound/`, `payload/` | per-module capability events and actions |
| `soak/` | long wall-clock chaos soaks (30 min, speed-up 1) - **not** in the default suite; run with `./sim suite tests/soak` or `--roots tests/soak` |

Every file opens with a one-paragraph `#` header saying what it proves;
`ci/check_library.py` enforces it, and `./sim list tests` prints the
first line of each.
