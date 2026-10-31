# droneengage_studio — overview & layout

See the parent `../AGENTS.md` for the workspace architecture; this file
covers what is specific to this repo. `README.md` is the user-facing
reference.

## When to use it

- An end-to-end check of a change in `de_comm`, `de_mavlink`,
  `droneengage_server` or the authenticator, with several units, through
  the real routing. Examples: mission supervisor / fleet bus / package
  deploy, comm-line loss, server restarts, login path.
- Personal simulations on a real server stack the user runs themselves
  (`server: {type: external}`) - e.g. `sims/quadcopter.yaml`,
  `sims/rover.yaml`, `sims/plane.yaml` run in parallel.
- Regression tests that must run the same on any developer machine.
- Reproducing a field problem with a specific fleet mix (copters + plane +
  fixed camera ...).

Not for: unit tests of one module (use that repo's `test/`), WebClient UI
work, or hardware/camera-device testing. The camera blocks need a real or
virtual V4L2 device and the `de_camera` binary.

The older copter-only, fake-server harness this was generalized from
(`Tasks/mission_planner/Phase-4A-Tasks/tools/`) has been removed — this repo
is now the only simulation/testing location.

## Layout

    sim                       single CLI: blocks|block|sims|show|up|down|status|ctl|
                              run|suite|view|list|new|validate|logs|clean
    run_scenario.py           compat shim -> the same engine as ./sim run
    sim.local.env.example     per-machine paths -> copy to sim.local.env (gitignored)
    blocks/<name>/            block.yaml + optional <module>.json / sitl.parm overlays
    sims/*.yaml               fleets of blocks (+ server type); each has a
                              sims/<name>.md runbook (setup, run, teardown,
                              ports) - keep the pair when adding a sim
    tests/**/*.yaml           AUTOMATED cases - what CI/nightly runs: the
                              de.scenario/1 library in area subdirs
                              (p4a/, faults/, verify/ ...)
    scenarios/**/*.yaml       HUMAN-run scenarios: demos/, examples/, user files
                              - each yaml pairs with a <name>.md runbook
                              (same convention as sims/)
    docs/MAP.md               the one-page model map: the compose/execute/
                              substrate axes, the noun diagram, the
                              folder table and the sim vs sims/ vs run
                              file glossary
    docs/SCENARIOS.md         the de.scenario/1 authoring reference (LLM-friendly)
    docs/MACHINE_API.md       --json output shapes + the run's RESULT/summary.json
                              contract (agents: read this before scraping text)
    docs/SENSORS.md           the SL-0030 sensor contract: a world-driven
                              sensor is one blocks/<name>/ folder (block +
                              catalog fragment + <stub>.sensor.yaml +
                              stub_<name>.py); ./sim new --sensor scaffolds
    templates/scenario.yaml   what ./sim new scaffolds
    worlds/*.yaml             seeded ground-truth worlds for scenario world:
    packages/*.json           hand-written DE packages shared by
                            demos/examples; test-only fixtures live in
                            tests/packages/ (compiled
                            outputs of in-repo .deop sources are
                            NOT committed - the runner compiles them
                            on demand into var/build/packages/)
    tools/                    support CLIs (verify_wrap.js, compile_op.js, ...)
    modules/catalog.yaml      how each DE module is launched + wired
    modules/templates/        base de_comm config, common/copter SITL parms
    engine/simlib.py          settings, block/sim resolution, ports, config gen, journals, pids
    engine/launcher.py        up / down / ensure (reuse-or-restart)
    engine/ctl.py             operator console (./sim ctl)
    engine/runner.py          the `do:` step set + ctl plumbing scenarios reuse
    engine/scenario.py        de.scenario/1 loader (schema: de_scenario.schema.json)
    engine/scenario_runner.py de.scenario/1 runner (./sim run/view on timeline files)
    engine/gcs_bridge.py      persistent GCS session on the real server, control-port protocol
    engine/fcb_proxy.py       opt-in MAVLink proxy between de_mavlink and SITL
                              serial1 (`fcb_proxy: true`, SL-0008): cut/stall/
                              corrupt/drop/inject on a per-unit control port
    engine/link_proxy.py      opt-in per-unit byte-level link-fault proxy in
                              front of de_comm (`server: {type: real,
                              link_proxy: true}`, SL-0009): reuses link_model
                              for stall/GE/bw/jitter/delay/cut on a raw TCP
                              relay of the wss stream
    engine/fake_fleet_server.py  fault-injection server (copied from Phase-4A, uses simlib)
    var/run/<sim>/            generated per sim (gitignored): sim.json, configs,
                              launch scripts, logs, journals, pids
    var/results/              per-test captures (gitignored)
    var/pycache/              python bytecode cache (PYTHONPYCACHEPREFIX,
                              set by simlib - no __pycache__ in the tree)
