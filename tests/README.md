# tests/ — the automated regression library

Everything here is run by CI (`ci/run_suite.py`, `./sim suite`). Files
for people to run, watch or copy live in `../scenarios/` instead.

## File format

| Format | Where | Shape |
|---|---|---|
| de.scenario/1 | everything in this tree | `fleet:` + `timeline:` + `assert:` (engine/scenario_runner.py) |

All tests are de.scenario/1 (see `../docs/SCENARIOS.md`); the old
`sim:` + `steps:` format has been removed.

## Running

One-time setup per machine:

    cp ../sim.local.env.example ../sim.local.env   # edit paths for your machine

Run from the repo root (`../`):

    ./sim list tests                  # list tests (first line of each header)
    ./sim validate tests/foo.yaml     # schema-check without running
    ./sim run tests/smoke_copters.yaml         # single test
    ./sim suite                       # default CI suite (ci/run_suite.py)
    ./sim suite tests/soak            # or a specific area folder
    ./sim run tests/foo.yaml --keep   # skip teardown; inspect via ./sim ctl status
    ./sim try tests/foo.yaml --module de_mavlink=<path-to-binary>  # test a fresh module build
    ./sim down                        # tear everything down

Exit code 0 = every step and assertion passed; captures land in
`../var/results/<ts>-<test>/` (journals + logs). `./sim run` is
exclusive - it downs every other live sim first (`SIM_SHARED=1` keeps it
additive on a shared machine). Back-to-back runs on the same sim reuse
the warm fleet and skip the 1-2 min SITL/EKF warm-up. Machine consumers:
`--json` on read-only commands, `RESULT <dir>/summary.json` as the last
stdout line of a run - shapes in `../docs/MACHINE_API.md`.

**Agents:** if another sim is live or ports clash, don't stop and don't
`sim down` what you didn't start - rerun additively:
`./sim run <test> --shared --slot-base any`. `any` picks the lowest slot
whose whole port set (units + server) is free; unit names may repeat on
per-sim servers. Only `server: {type: external}` tests and rendered
cameras can't share the machine - ask in that case. See `../AGENTS.md`.

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
