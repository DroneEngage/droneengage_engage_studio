# AGENTS.md — droneengage_simulation_land

Headless, multi-vehicle DroneEngage simulation built from **reusable
blocks**. A block describes one unit type (SITL or not, which DE modules,
config overlays). A sim composes blocks into a fleet. A test runs steps and
assertions on a sim against the **real** `droneengage_server`, optionally
with `droneengage_authenticator`. A fake server exists only for link
faults. See the parent `../AGENTS.md` for the workspace architecture; this
file covers what is specific to this repo. `README.md` is the user-facing
reference.

Python 3 only (`websockets`, `pymavlink`, `pyyaml`). The repo has no build
step. It launches binaries and node servers from the sibling repos.

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
    sims/*.yaml               fleets of blocks (+ server type)
    tests/**/*.yaml           AUTOMATED cases - what CI/nightly runs: legacy
                              sim+steps+assert files plus the de.scenario/1
                              library in area subdirs (p4a/, faults/, verify/ ...)
    scenarios/**/*.yaml       HUMAN-run scenarios: demos/, examples/, user files
    docs/SCENARIOS.md         the de.scenario/1 authoring reference (LLM-friendly)
    templates/scenario.yaml   what ./sim new scaffolds
    worlds/*.yaml             seeded ground-truth worlds for scenario world:
    packages/*.json           compiled DE packages used by tests
    tools/                    support CLIs (verify_wrap.js, compile_op.js, ...)
    modules/catalog.yaml      how each DE module is launched + wired
    modules/templates/        base de_comm config, common/copter SITL parms
    engine/simlib.py          settings, block/sim resolution, ports, config gen, journals, pids
    engine/launcher.py        up / down / ensure (reuse-or-restart)
    engine/ctl.py             operator console (./sim ctl)
    engine/runner.py          legacy test runner + the step set scenarios reuse
    engine/scenario.py        de.scenario/1 loader (schema: de_scenario.schema.json)
    engine/scenario_runner.py de.scenario/1 runner (./sim run/view on timeline files)
    engine/gcs_bridge.py      persistent GCS session on the real server, control-port protocol
    engine/fcb_proxy.py       opt-in MAVLink proxy between de_mavlink and SITL
                              serial1 (`fcb_proxy: true`, SL-0008): cut/stall/
                              corrupt/drop/inject on a per-unit control port
    engine/fake_fleet_server.py  fault-injection server (copied from Phase-4A, uses simlib)
    var/run/<sim>/            generated per sim (gitignored): sim.json, configs,
                              launch scripts, logs, journals, pids
    var/results/              per-test captures (gitignored)
    var/pycache/              python bytecode cache (PYTHONPYCACHEPREFIX,
                              set by simlib - no __pycache__ in the tree)

## How to use

    cp sim.local.env.example sim.local.env    # once per machine; edit paths
    ./sim blocks                              # what unit types exist
    ./sim show sims/copters3.yaml             # resolved units, ports, positions (no side effects)
    ./sim run tests/smoke_copters.yaml        # up if needed -> steps -> asserts -> teardown
    ./sim view scenarios/demos/world_canopy_viewer.yaml  # sim + Mavlink3DMap viewer
                                              # (= run --hold) until Ctrl-C
    ./sim ctl status                          # live procs + units seen online by the GCS
    ./sim logs copter1 de_comm                # tail a process log
    ./sim clean --days 7                      # prune var/run/ + var/results/ (never a live sim)
    ./sim down

- `./sim up` is additive: it restarts only the same-named sim and refuses
  ports/unit names that clash with a live sim. `./sim down <name>` stops
  one sim, `./sim down` all of them. `./sim run` is exclusive - a test
  downs every other sim first. `SIM_SHARED=1 ./sim run ...` keeps it
  additive (same rules as `up`) when other sessions share the machine.
- `./sim ctl -s <sim>` pins a command to one sim; without it commands
  that need a single sim use the only live one, and unit lookups search
  live sims first.
- `./sim run` reuses the running sim only when its hash matches and every
  process is alive; otherwise `up --fresh`. Run tests on the same sim back to
  back to skip the 1-2 min SITL/EKF warm-up.
- `./sim run <test> --keep` skips the teardown so you can inspect the fleet
  with `./sim ctl ...`.
- Exit code 0 = every step and assertion passed. Read the printed
  PASS/FAIL lines and `var/results/<ts>-<test>/` (journals + logs), not just the
  exit code.
- Before claiming a change works, run the relevant test(s) and quote the
  `N/M assertions passed` line.

## How config is built (precedence, lowest -> highest)

1. module base template: its repo's `*.config.module.json`; de_comm uses
   `modules/templates/de_comm.json`
2. catalog `set:` defaults
3. block overlays along the `extends:` chain (`<module>.json` files, then
   inline `config:`)
4. per-unit `config:` / `fc:` in the sim
5. engine wiring and catalog `wire:` - ports, unitID/party, server and auth
   settings, state files, journal dir. Never put these in a block;
   they are overwritten.

Machine paths come only from `simlib.setting()`: environment variable >
`sim.local.env` > default. **Never hard-code `~/de_code/...` or
`~/code/ardupilot` in engine code, blocks or sims.**

## Ports

Slot `k = slot_base (10) + unit index`. SITL `-I k`, sysid `k`, pilot
`5760+10k`, fcb `7660+10k`. The opt-in fcb proxy (`fcb_proxy: true`,
SL-0008) takes `7760+10k` (de_mavlink dials it instead of `fcb_port`) and
`7860+10k` (fault control port). Module s2s ports are `port_base + 10k`
(de_comm 62000, de_mavlink 63000, de_camera 64000, de_tracker 65000). Server
wss 62008, control 62009, authenticator +10/+11. Keep `slot_base` >= 10 so
sim ports never collide with the developer's own servers.

## Extending

**New unit type → a block (no code).** `blocks/<name>/block.yaml`:

    description: ...
    extends: copter            # optional
    add_modules: [de_camera]   # | modules: [...] | remove_modules: [...]
    fc: {vehicle: copter|plane|rover, model: quad, params: {PARAM: value}}   # fc: null = no SITL
    config: {de_comm: {sv_comm_lost_s: 20}}
    location: {north_m: 0, east_m: 0}

Put larger overlays in `blocks/<name>/<module>.json` (comments allowed) and
extra SITL params in `blocks/<name>/sitl.parm`. Check the result with
`./sim block <name>`.

**New DE module → a catalog entry (no code)** in `modules/catalog.yaml`:
`repo`, `dir_env`, `bin` (`bin_env` if built elsewhere), `template`,
`port_base` (pick an unused range; slot ports must stay < 65535),
`start_delay`, `needs_fc`, `local` (`arg: -b` only if the binary's getopt
accepts it, otherwise `file:` in the cwd), `set`, `wire` (placeholders
`{fcb_port} {pilot_port} {sysid} {comm_port} {port} {unit}`), and `ready_re`
(a journal line proving the module is up; `sim up` waits for it). Add the
`dir_env`/`bin_env` keys to `sim.local.env.example`.

**New fleet → a sim (no code).** `units:` entries take `block`, `name`,
`count` (→ name1..nameN), `at`, `config`, `fc`. Use `extends: other.yaml`
to reuse a fleet with a different `server:`.

**New test/scenario → a file (no code)** when existing steps, faults and
assertions suffice. Prefer the de.scenario/1 form (`fleet:` + `timeline:`,
docs/SCENARIOS.md) — `./sim new <name>` scaffolds one, `./sim validate`
checks it. Automated cases go under `tests/<area>/`, human ones under
`scenarios/`; every file opens with a `#`-comment "what it proves"
paragraph (CI enforces it). The legacy `sim:`+`steps:` form still works.
Start mission cases with `stop`, `clear`, `mode: {unit: all, mode: GUIDED}`.

**New step / assertion / ctl command (code):**
1. the operator action → `ctl.py` `main()`, also listed in its docstring;
2. the step → `runner.run_step` calls it through `run_ctl`, plus an entry in
   `_UNIT_STEPS` if it takes `unit:`;
3. the assertion → `runner.check_assertion`. Read journals only through
   `simlib.journal_match` / `journal_lines` with the test's `marks`;
4. document it in the runner docstring and README.

**New vehicle type**: add to `simlib.FC_KINDS` (SITL binary, model, defaults,
pymavlink mode table, arm mode, and how `land` brings it down).

**New server behaviour**: the bridge and the fake server share one control
protocol. Implement a command in both, or have the bridge answer `ERR ...`
and make `ctl.ctrl` refuse it for that server type.

## Invariants and gotchas

- **Only kill our own pids.** `simlib.pid_is_ours` matches only command
  lines containing `RUN_DIR`/`ENGINE_DIR`. Never `pkill` by binary name:
  the Phase-4A fleet and the developer's own servers use the same binaries.
- **SITL must stay a child of its launch shell.** SITL exits when its
  parent dies, so `launch.sitl.sh` runs it with `&` and `wait`s, never
  `exec`. Every other launch script `exec`s and writes its own pidfile.
- **Real servers run from their repos with a generated
  `--config=<relative path>`.** The common config loader joins the path
  onto the repo dir. Never edit or depend on the repos' own
  `server.config(.local)`.
- **The real server does not stamp `sd`.** It routes the client's
  `sd`. The bridge sets `sd: GCS_FLEET`. The fake server stamps `sd`, so it
  is not proof for sender-identity behaviour.
- de_comm journals `comm line offline/online` only while a v2 package is
  loaded, and `nav adapter hello` once per de_comm process. That is why
  `reset` restarts SITL + de_comm + modules together.
- Authenticator: the account must be alphanumeric or an e-mail with a TLD;
  a bad one comes back as "API endpoint not found". Logins are limited to
  10/min per IP and every unit is 127.0.0.1.
- Link-fault steps (`loss`, `delay`, `link_cut`, `kick`, `clock`) need
  `server: {type: fake}`. `ctl` refuses them on a real server.
- **A downed sim keeps its manifest.** `var/run/<sim>/sim.json` stays for
  post-mortem logs/journals, so a dead sim's units can share names with a
  live one. `simlib.find_unit` and ctl resolution always prefer *live*
  sims (`live_sim_names`); never iterate `all_manifests()` when the
  intent is the running fleet.
- `server: {type: external}` spawns no server - the sim's units and GCS
  bridge join a server stack the user runs. Its `account` (or
  `SIM_AUTH_USER`/`SIM_AUTH_CODE` in `sim.local.env`, or `./sim --user`/
  `--password`/`--auth-server` on the command line - they win over both)
  must exist on that authenticator; with `account_storage_type: db`
  create it in the webadmin first.
- `land`: copter LAND, rover HOLD + disarm, plane → `reset` (it cannot land
  in place). Always tear down: an airborne reused fleet drains its battery
  and the next test trips `battery_low`.
- `ensure` decides reuse by the sim hash. Any change to a block, sim or
  server settings changes the hash and forces a fresh `up`. That is
  intended.
- The world stream (`de.worldstream/1`) is a thread inside the scenario
  runner - it dies when the run exits. `./sim view` / `run --hold`
  parks the run after the timeline so a viewer session survives; the
  render procs (vite, mav_bridge) are pid-tracked daemons like units,
  so a leftover viewer is stopped by `sim down`, not by the run ending.

## Verifying engine changes

Minimum before handing back:

    python3 -c "import ast,glob;[ast.parse(open(f).read()) for f in glob.glob('engine/*.py')]"
    python3 engine/test_checks.py                      # assertions, journal reader, bot, ports
    python3 stubs/test_stubs.py                        # stub sensor models (needs the databus checkout)
    for f in sims/*.yaml; do ./sim show $f; done      # resolution, no side effects
    ./sim run tests/smoke_copters.yaml                 # real server path
    ./sim run tests/fault_link_loss.yaml               # if fake server / ctl faults touched
    ./sim run tests/smoke_copters_auth.yaml            # if login / bridge / server gen touched
    ./sim run tests/mixed_bringup.yaml                 # if FC_KINDS / pilot commands touched
    ./sim down
