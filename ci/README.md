# ci/ - scenario-library CI (TASK-P5-09)

Two layers, split by what the machine can do:

| layer | file | runner | what it needs |
|---|---|---|---|
| pure gates | `check_library.py`, `check_verify_ops.py`, `check_verbs.py` + `.github/workflows/pure-gates.yml` | GitHub-hosted | python3 + pip, node + webclient repo (verify ops) |
| scenario suite | `run_suite.py` + `.github/workflows/nightly-sitl.yml` | self-hosted `linux sitl` | the full workspace + SITL |

Nothing in the pure layer starts a sim, a server, or SITL. Anything
that does lives in the nightly layer only.

## check_verify_ops.py

    python3 ci/check_verify_ops.py

Runs `tools/verify_wrap.js` on every `*.op.json` /
`*.deop` under `tests/`. Ops named by a `verify:` block with
`expect_rules` must be blocked with exactly those rules; everything
else must verify clean. This is the push-time form of the deploy gate -
the same scenarios prove the full runner path nightly (GATE-PASS
verdicts).

## check_verbs.py

    python3 ci/check_verbs.py

The verb-registry gate (SL-0024): every `do:` step, `fault:` kind and
`assert:` name used under `tests/`/`scenarios/` must exist in
`engine/registry.py`, and every registry verb must appear in
`docs/SCENARIOS.md`. Exit 1 on any gap.

## run_suite.py

    python3 ci/run_suite.py --roots tests --seeds 1,2,3
    python3 ci/run_suite.py --roots tests/verify --seeds 1   # fast gate check
    python3 ci/run_suite.py --roots tests/faults --seeds 1,2,3

Serial per-file, per-seed runs of `run_scenario.py`. Verdicts:

- `PASS` - assertions all green.
- `GATE-PASS` - a `verify:` file that declares `expect_rules` /
  `expect_errors` was blocked by the verifier AND `verify.json`
  recorded exactly those rules (the deploy gate working by design).
- `FAIL` - anything else.
- `FLAKY` - first attempt FAIL, the single permitted rerun passed. The
  summary keeps BOTH attempts and junit marks the case `type="flaky"`
  so CI cannot silently green it.

Outputs `var/results/nightly/summary.md`, `junit.xml`, `runs.json`; exit 1
when anything is FAIL or FLAKY.

## Self-hosted nightly runner prerequisites

The job assumes a machine that may own the machine:

- the `de_code` sibling layout: this repo plus `droneengage_webclient_react`,
  `droneengage_communication`, `droneengage_mavlink`, `droneengage_server`,
  `droneengage_authenticator` checked out side by side (or the `DE_*_DIR`
  env vars set),
- sibling binaries built (`de_comm`, `de_mavlink`, `de_server`,
  `de_camera` for camera blocks),
- ArduPilot SITL reachable via `sim.local.env` / env settings,
- `sim.local.env` copied and filled in,
- nothing else bound to the sim port ranges; the workflow `./sim down`s
  before and after.

Runner labels: `self-hosted, linux, sitl`.

## Local equivalent of the nightly job

    ./sim down
    python3 ci/run_suite.py --roots tests --seeds 1,2,3
    cat var/results/nightly/summary.md

Or a cron on a dedicated machine:

    17 2 * * * cd ~/de_code/droneengage_studio && ./sim down && \
        python3 ci/run_suite.py --roots tests --seeds 1,2,3 \
        >> var/results/nightly/cron.log 2>&1
