# AGENTS.md — droneengage_studio

Headless, multi-vehicle DroneEngage simulation built from **reusable
blocks**. A block describes one unit type (SITL or not, which DE modules,
config overlays). A sim composes blocks into a fleet. A test runs steps and
assertions on a sim against the **real** `droneengage_server`, optionally
with `droneengage_authenticator`. A fake server exists only for link
faults. `README.md` is the user-facing reference.

Python 3 only (`websockets`, `pymavlink`, `pyyaml`). No build step — it
launches binaries from the sibling repos (the servers are the Rust ports:
`<repo>/target/release/<name>` after `cargo build --release`).

## Quickstart

    cp sim.local.env.example sim.local.env    # once per machine; edit paths
    ./sim show sims/copters3.yaml             # resolved units/ports (no side effects)
    ./sim run tests/smoke_copters.yaml        # up -> steps -> asserts -> teardown
    ./sim down

## Index — `.agents/wiki/`

Read only the file(s) the task needs — each file covers one topic.

| File | What it covers |
|---|---|
| [`agents_overview.md`](.agents/wiki/agents_overview.md) | when to use the sim, repo layout (engine/, blocks/, sims/, tests/, var/) |
| [`agents_run.md`](.agents/wiki/agents_run.md) | the `./sim` CLI: up/down/run/try/ctl semantics, additive vs exclusive, --json, exit codes |
| [`agents_config.md`](.agents/wiki/agents_config.md) | config precedence chain, `simlib.setting()`, the port map |
| [`agents_extend.md`](.agents/wiki/agents_extend.md) | adding blocks, catalog modules, sims, tests, steps/assertions, commands |
| [`agents_stubs.md`](.agents/wiki/agents_stubs.md) | stub sensors: what they are, wiring, using them in tests, worked example |
| [`agents_stub_dev.md`](.agents/wiki/agents_stub_dev.md) | developing a stub: scaffold, sensor.yaml contract, StubBase API, determinism rules |
| [`agents_notes.md`](.agents/wiki/agents_notes.md) | invariants & gotchas (pid safety, link faults, manifests, speedup) + verifying engine changes |

See `../AGENTS.md` for the workspace architecture.
