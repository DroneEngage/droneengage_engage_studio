# DroneEngageStudio

(repo directory `droneengage_studio`)

DroneEngageStudio is a vehicle-stack-agnostic simulation and ground-truth
testing studio. DroneEngage is its flagship integration - the first and
best-supported stack it drives, not its owner (see `docs/ADAPTER.md` for
the stack-agnostic `de.stack/1` interface).

Reusable building blocks for simulations (flagship: DroneEngage). You describe a vehicle
type once, compose fleets out of these types, and run tests against a fleet.
Everything runs headless on one machine against the **real**
`droneengage_server` (optionally with `droneengage_authenticator`) on its own
ports. A fake server is available for link-fault tests.

New here? The [User Guide](wiki/USER_GUIDE.md) is the plain-English start;
this README is the technical reference.

```
blocks/     unit types   - what one unit runs (SITL? which modules? config)
sims/       fleets       - blocks composed: names, counts, placement, server
tests/      test cases   - the automated library CI runs (steps + asserts)
scenarios/  human runs   - demos + examples a person drives by hand
worlds/     ground truth - seeded worlds scenarios can load
packages/   operations   - hand-written ops demos/examples deploy
                         (test-only fixtures live in tests/packages/;
                         a .deop source compiles on demand into
                         var/build/packages/<key>/)
templates/  scaffold     - the file ./sim new copies
docs/       authoring    - SCENARIOS.md: the de.scenario/1 reference;
                         SENSORS.md: the world-driven sensor contract
                         (./sim new --sensor <name>);
                         MAP.md: the one-page model map
wiki/       user docs    - USER_GUIDE.md: the plain-English guide
```

## The model

Everything in this repo answers one of three questions:

- **COMPOSE** - *what exists*: `modules/catalog.yaml` -> `blocks/<unit>`
  -> `sims/<fleet>`, fed by `worlds/` (ground truth), `packages/`
  (operations) and the dev stacks.
- **EXECUTE** - *why you run it*: `tests/` (machine-checked) vs
  `scenarios/demos|examples|challenges` (human-run) - every file a
  de.scenario/1 run file, selectable by `tasks:` and `tier:`.
- **SUBSTRATE** - *where it runs*: host SITL (default), `./sim up
  <stack>` (de_servers / de_webclient), `tools/rpi` and `tools/hil`
  hardware, android phones.

`./sim help` groups its commands the same way. The one-page map with
the noun diagram and glossary is [docs/MAP.md](docs/MAP.md).

### I want to...

| I want to... | command |
|---|---|
| run one copter | `./sim up quadcopter` (or `./sim studio quadcopter`) |
| run the real servers | `./sim up de_servers` |
| run the webclient dev server | `./sim up de_webclient` |
| run copters on their own sim server | `./sim studio copters3` |
| join my own running stack | `./sim up de_servers`, then a sim with `server: {type: external}` (e.g. `sims/plane.yaml`) |
| run the smoke tests | `./sim suite --tier smoke` |
| run the tests for a task | `./sim suite --task SL-0099` |
| run one test | `./sim run tests/smoke_copters.yaml` |
| rebuild + retry one module | `./sim try <test> --module de_mavlink=<path>` |
| use ESP32 hardware | `./sim esp32 list` (see `docs/HIL.md`) |
| use Pi hardware | see `tools/rpi/README.md` |
| use an android phone | `blocks/android_copter` (see `docs/HIL.md`) |
| record a session | `./sim record <sim> -o <file.yaml>` |

## Quickstart (Docker)

One image with SITL, the DE binaries, the sim engine and the
Mavlink3DMap viewer baked in - the fastest way to see a scenario run:

```bash
docker build -f docker/Dockerfile -t de-sim .
docker run -it --network host de-sim          # runs the canopy-search demo
```

The run brings the fleet up, uploads a lawnmower mission over the
forest-canopy world (the classic `Upload_DE_Mission` path), flies the
sweep and then parks with the viewer live - open the printed
`render viewer:` URL in a browser, and `Ctrl-C` when done to see the
graded assertions. `--network host` is required (the sim binds its
127.0.0.1 slots; Linux only). See [docker/README.md](docker/README.md)
for details, other commands, and what stays host-side or gated on
upstream mirrors (supervised op deploys need the mission-supervisor
code the public de_comm/de_mavlink mirrors do not publish yet).

## Setup (per machine)

```bash
cp sim.local.env.example sim.local.env    # gitignored: this machine's paths
```

Edit `sim.local.env` for your machine: `DE_CODE` (root of the
DroneEngage checkouts), `ARDUPILOT_DIR`, and per-repo or per-binary
overrides (`DE_MAVLINK_DIR`, `DE_CAMERA_BIN`, `NODE`, ...). An environment
variable with the same name wins over the file.

Then check the machine:

```bash
./sim doctor                          # one OK/MISSING/OPTIONAL line per item,
                                      # each MISS with a fix line; exit 1 on gaps
./sim doctor sims/quadcopter.yaml     # same, scoped to what that sim needs
./sim doctor --json                   # machine-readable [{item,status,detail,fix}]
./sim doctor --esp32                  # also require + probe the optional ESP32 HIL gear
```

Bluetooth FC link testing: `tools/hil/bt_spp_relay/` makes this host a
Bluetooth telemetry radio wired to SITL, for the Andruav phone or an RPi
(`/usr/bin/python3 -I tools/hil/bt_spp_relay/bt_spp_relay.py 6000`; see its README).

Optional ESP32 HIL firmwares (`tools/hil/<name>/`, e.g. `esp32_fake_fc` - a
fake ArduCopter that proves a phone's USB-OTG FCB link):

```bash
./sim esp32 list                      # firmwares under tools/hil/
./sim esp32 upload                    # build + flash esp32_fake_fc + run its host check
./sim esp32 check --port /dev/ttyUSB0 # only the host check against the flashed board
```

The machine also needs:

- ArduPilot SITL built (`./waf configure --board sitl && ./waf copter plane rover`).
- `de_comm` and `de_ardupilot` built (`bin/` in their repos). Other modules
  are only needed when a block uses them.
- `cargo build --release` done in `droneengage_server` and
  `droneengage_authenticator` (plus `droneengage_udp_proxy` for
  `udp_relay:` sims) - a Rust toolchain, e.g. via rustup.
- `pip install websockets pymavlink pyyaml`, and `openssl` (fake server only).

No root needed. Servers and modules use 127.0.0.1. de_comm's s2s UDP port
and SITL's TCP ports listen on all interfaces.

## Quick start

```bash
./sim studio sims/copters3.yaml       # the human session: fleet + planner + 3D viewer
./sim list                            # every runnable file + one-line summary
./sim run tests/smoke_copters.yaml    # brings the sim up if needed, runs, lands
./sim view scenarios/demos/world_canopy_viewer.yaml   # a demo + 3D viewer
./sim ctl status                      # live processes / online units /
                                      # the webclient deep link
./sim status --open-web               # open the webclient logged into this sim
./sim down
./sim clean --dry-run                 # what pruning var/run/ + var/results/ frees
```

More of the catalog:

```bash
./sim blocks                          # the reusable unit types
./sim list sims                       # fleet definitions
./sim show sims/copters3.yaml         # what a sim expands to (units, ports)
./sim new my_run                      # scaffold scenarios/my_run.yaml
./sim validate scenarios/my_run.yaml  # schema + refs, starts nothing
./sim expand tests/soak/soak_copters3.yaml  # the chaos: schedule, run nothing
./sim suite tests/faults              # a whole directory x seeds
```

`./sim run` reuses a running sim only if it is the **same** sim (hash of the
resolved units and server) and every process is alive. Otherwise it brings
the sim up fresh. Several tests can share one sim and run back to back without
restarting SITL. Handed a bare `sims/*.yaml` file instead of a test, `run`
(and `view`) just ensure the sim is up - there are no steps to execute.

File arguments take a bare name too: `./sim up copters3` (or
`copters3.yaml`) finds `sims/copters3.yaml`; `run`/`view`/`validate` also
search `tests/**` and `scenarios/**`. A name that matches files in more
than one directory prints a numbered list to pick from (a hard error
when stdin is not a tty). A path with `/` is always used as given.

## Blocks - reusable unit types

`blocks/<name>/block.yaml`, plus optional per-module config overlays in the
same folder:

```yaml
description: Copter with an onboard camera
extends: copter              # inherit modules, fc, overlays
add_modules: [de_camera]     # or modules: [...] to replace, remove_modules: [...]
fc:                          # flight controller (omit / null = no SITL)
  vehicle: copter            # copter | plane | rover
  model: quad                # SITL -M
  params: {SIM_BATT_CAP_AH: 0.5}   # extra SITL params
config:                      # inline overlays, per module
  de_comm: {sv_battery_low_pct: 60}
location: {north_m: 0, east_m: 0}  # default position (fc-less units)
```

| file in the block folder | overlays |
|---|---|
| `de_comm.json`, `de_mavlink.json`, `de_camera.json`, ... | that module's config (comments allowed) |
| `sitl.parm` | appended to the SITL defaults |

Config is built in layers, later layers winning:

1. the module's base template (its repo's `*.config.module.json`; for
   de_comm, `modules/templates/de_comm.json`);
2. the catalog `set:` defaults;
3. the overlays down the block's `extends` chain;
4. the per-unit `config:` in the sim;
5. the engine's wiring: ports, ids, server, state files and journal dir.
   No block can break these.

Included blocks:

| block | runs |
|---|---|
| `copter` | ArduCopter SITL + de_comm + de_mavlink |
| `plane` | ArduPlane SITL + de_comm + de_mavlink |
| `rover` | ArduRover SITL + de_comm + de_mavlink |
| `copter_camera` | copter + de_camera (extends copter) |
| `copter_lowbat` | copter with a small battery and a high battery_low threshold |
| `camera_fixed` | de_comm + de_camera, no flight controller |
| `comm_only` | de_comm only (static unit / ground node) |

**Modules** are defined once in `modules/catalog.yaml`: repo, binary,
template, port base, and how it is wired to de_comm and the FC. Adding a new
DroneEngage module (sdr, gpio, ...) is a catalog entry, with no engine change.

## Sims - fleets

```yaml
version: 1
extends: copters3.yaml               # optional: reuse a fleet, change the server
server: {type: real}                 # real (default) | fake | external ; auth: true
units:
  - {block: copter, name: copter, count: 3}       # copter1..copter3
  - {block: plane, name: plane1, at: {east_m: 60}}
  - block: camera_fixed
    name: tower1
    cameras: [DE-CAM2]               # binds that pool label; cameras: 2 = two auto labels
    at: {north_m: -50, east_m: 30}
  - {block: copter, name: quad1,
     account: {user: "{name}@sim.droneengage.com", code: sim1234}}  # own login
```

`--with a,b` layers `sims/overlays/<name>.yaml` twists on top of any sim
(server swap, extra units, config/fc pushes) — the overlay wins, later
overrides earlier, and every applied name appends to the sim name so a
variant gets its own `var/run/` dir and never reuses another variant's
warm sim. A scenario picks a named fleet with `sim: {base: copters3,
with: [faults]}` instead of a `fleet:` block. See
[sims/overlays/README.md](sims/overlays/README.md).

A unit's `account:` is its own login on the authenticator (defaults to
`server.account`); `{name}` expands to the unit name so a `count:` fleet
gets `quad1@..`, `quad2@..` Every account must exist on the
authenticator in the **same team** as the GCS account - a unit in
another team is invisible to your WebClient.

Server types:

| `server:` | runs | use it for |
|---|---|---|
| `{type: real}` | `droneengage_server` in `local_server_enabled` mode | default: the real routing |
| `{type: real, auth: true}` | + `droneengage_authenticator` (single account); every unit and the GCS log in | the login path |
| `{type: real, link_proxy: true}` | + `engine/link_proxy.py` per unit between de_comm and the server wss | the SL-0007 link model on the real routing: byte-level stall / Gilbert-Elliot / bandwidth / jitter / delay / cut, per unit (auth: false only) |
| `{type: fake}` | `engine/fake_fleet_server.py` | seeded per-link loss / delay / cut plus the SL-0007 link model: per-direction loss, Gilbert-Elliot bursts (drop or in-order stall), bandwidth cap, jitter, named profiles |
| `{type: external, host, port}` | nothing - your own running server stack | personal sims on your real servers |

`external` is for sims that join a server stack you run yourself (start
it with `./sim up de_servers` - the headless
`servers/sh_start_servers.sh`: auth, comm, storage and udp_proxy run
from their repos' own `server.config(.local)` on the real ports; it is
infrastructure, not a sim, so `down`/`run`/`clean` never touch it and
only `./sim down de_servers` stops it). `./sim up de_webclient`
starts the webclient dev server the same way (`npm start` in
`DE_WEBCLIENT_DIR`, https on `SIM_WEBCLIENT_URL` - :3000 default;
`--open-web` opens it): no server processes are
spawned, every
unit's de_comm connects to `host:port` - directly when `auth: false`, or
through your authenticator when `auth: true, auth_host, auth_port` are
set. Two logins, both on the **same team** or the GCS won't see the
units: `server.account` (or `SIM_AUTH_USER`/`SIM_AUTH_CODE`) is the GCS
bridge login and needs the GCS permission bit; `server.unit_account`
(or `SIM_UNIT_AUTH_USER`/`SIM_UNIT_AUTH_CODE`, defaulting to `account`)
is the units' login - a unit-only account is right for it (a unit
account cannot login as GCS). `up` fails early if the server isn't
reachable.

The same login and authenticator can come from the command line instead
of the yaml / `sim.local.env`:

    ./sim up sims/quadcopter.yaml --user me@x.com --password secret --auth-server 203.0.113.9:19408

`--user`/`--password` make the whole sim (units + GCS bridge) log in
with that account - they override `server.account`, `server.unit_account`,
per-unit `account:` entries and the `SIM_*_AUTH_*` vars.
`--auth-server <host[:port]>` overrides `auth_ip`/`auth_port` and
is only valid on an external server with `auth: true`.

The real servers run from their repos with a config **generated** under
`var/run/<sim>/server/`. Their own `server.config(.local)` files are never
read or changed.

With a real or external server, `engine/gcs_bridge.py` holds one GCS
session (`GCS_FLEET`, or `GCS_<sim>` for external sims) and serves the
same control protocol as the fake server, so `./sim ctl` behaves the
same on all of them. Link faults on a real server are refused unless
`link_proxy: true` is set - then `./sim ctl link <unit> ...` goes to that
unit's own proxy (the GCS bridge session is never proxied). Message-level
loss (`loss`, `loss_tx`, `loss_rx`, loss-knob profiles like
`uplink_dead`) and the clock faults stay fake-only: a raw TLS relay
cannot see inside the stream.

The same bridge also watches a stack that has no sim manifest - e.g. the
external `de_servers` a developer runs for the Android app or a board:
`./sim watch --auth-ip <h> --auth-port <p> --account <u> --code <c>` logs
in through the authenticator as a GCS party and prints party join/leave/id
events live. `--no-auth --server-ip <h> --server-port <p>` connects
straight to an auth-off comm server, `--party` names the party and
`--ctl-port` keeps the list/listen/req control protocol available. The
account needs GCS permission (unit accounts are refused by `/w/wl`).

### Running several sims in parallel

**Default posture: interactive work is additive, tests are exclusive.**
`up`, `studio`, `web`, `ctl` and a `run`/`view` of a bare `sims/*.yaml`
(no steps to execute) all stay next to whatever is already running. A
`run`/`view`/`suite` of a file with `steps:` owns the machine - it downs
every other sim first, because a test needs a clean machine to be
meaningful. On a shared box pass `--shared` (or export `SIM_SHARED=1`)
and even scenario runs stay additive; exclusive mode is the CI posture.

`./sim up` is additive: it only restarts the same-named sim and refuses
to start one whose ports collide with a live sim (pick a different
`slot_base`, or let `--slot-base any` choose). Unit names may repeat
across parallel sims - a spawned fake/real server gives each fleet its
own party registry - except where the fleets could actually meet or
share devices: on a `server: {type: external}` stack (a unit name is the
login identity there) or when either fleet runs rendered cameras (the
DE-SRC-k/DE-AI-k v4l2loopback cards are machine-global). Both keep the
old uniqueness refusal. `var/run/<sim>/` holds each sim's manifest, configs,
logs and pids, so sims are fully independent:

```bash
./sim up sims/quadcopter.yaml
./sim up sims/rover.yaml          # runs next to quadcopter
./sim sims                        # every generated sim, live proc count
./sim ctl -s quadcopter arm all   # -s pins a command to one sim
./sim down quadcopter             # stop one sim; ./sim down stops all
```

`ctl` commands that need one sim (server procs, `reset`, deploy) use the
`-s` sim, else the only live sim; unit lookups (`arm quad1`, `logs`)
search live sims first - a bare unit name that exists in several live
sims is refused as ambiguous, pin it with `-s`. `sim run` stays exclusive for tests - a test
owns the machine and downs the other sims (a bare `sims/*.yaml`
`run`/`view` is additive, like `up`). `--shared` applies the additive
rules to a `run`/`view`/`suite` instead, and `--slot-base <N>` overrides
the slot every port derives from for that invocation (wins over the
sim's `slot_base:` and `SIM_SLOT_BASE`); `--slot-base any` picks the
lowest base whose ports are all free - both against live sims and ports
already bound on the host. Note the reuse caveat: the slot feeds the sim
hash, so when `any` picks a different slot than last time the run does a
fresh `up` (~1-2 min SITL warm-up) rather than reusing - pin a number
when iterating, use `any` when you don't care.

```bash
./sim run tests/smoke_copters.yaml --shared --slot-base 90   # next to a live sim
./sim run tests/smoke_copters.yaml --shared --slot-base any  # picks one itself
```

`sims/quadcopter.yaml`, `sims/rover.yaml` and `sims/plane.yaml` are
ready-made single-box personal sims for an external server, on disjoint
`slot_base`s (50/60/70).

### `./sim studio` - the human session

```bash
./sim studio sims/copters3.yaml [--world sar_forest] [--record out.yaml] [--down]
```

One command for sitting at a sim: `ensure` (reuse if warm, additive like
`up`), then the webclient opened in planning mode, then the render/world
side, then the recorder, then one status block - deep links, units
online, record file - and it parks until Ctrl-C.

- **Viewer**: the sim file's `render:` wins; a spec without one gets
  `render: {mode: viewer}` so Mavlink3DMap (mav_bridge + vite) and the
  world stream's `stream_port` exist. When `MAVLINK3DMAP_DIR` is missing
  or incomplete the render spec is downgraded to `headless: true` -
  the session still runs and prints `viewer: skipped` (studio works
  without the viewer, per the project rule). Injected/downgraded specs
  rehash, so they never collide with a plain `./sim up` of the same
  file.
- **`--world <path|name>`**: loads `worlds/<name>.yaml` (or the given
  path), injects it into the spec exactly like a scenario run
  (`world_mod.prepare_sim`: stub configs, geo_projection, SITL wind),
  then runs the parked-scenario world wiring minus assertions - epoch,
  the 1 Hz TruthRecorder (react authority), the ADS-B feeder, and the
  `de.worldstream/1` feed on `render.stream_port`.
- **`--record <file.yaml>`**: arms the same `command_plugin` capture as
  `./sim record` before the browser opens; Ctrl-C finalizes the
  de.scenario/1 file (+ sidecars/runbook stub).
- **Exit**: the sim stays up (the world stream, telemetry and recorder
  die with the process; render procs are pid-tracked so `./sim down`
  still reaps them). `--down` tears the sim down on exit.

### Ports (slot k = slot_base + unit index; slot_base defaults to 10)

| use | port |
|---|---|
| SITL pilot (serial0) / aux (serial2) | 5760+10k / 5762+10k |
| de_mavlink -> SITL (serial1) | 7660+10k, sysid k |
| fcb proxy listen / control (`fcb_proxy: true`) | 7760+10k / 7860+10k |
| de_comm / de_mavlink / de_camera s2s | 62000 / 63000 / 64000 + 10k |
| link proxy relay / control (`link_proxy`) | 61500 / 61501 + 10k |
| comm server wss / control | 62008 / 62009 (`server.port`) |
| authenticator https / s2s | port+10 / port+11 |

The default slot base keeps clear of low-numbered ports so sims never
collide with a developer's own servers. Sims meant to run **in parallel**
need disjoint `slot_base`s - `up` verifies there is no overlap with a
live sim before starting anything.

## Tests - the automated library

A `run`/`view`/`suite` of a file with `steps:` is **exclusive**: it downs
every other sim first - that is the CI posture (a test needs a clean
machine), and the interactive commands stay additive (see
*Running several sims in parallel*). `--shared` / `SIM_SHARED=1` keeps a
run additive on a shared box.

`tests/` holds everything automation runs: the `de.scenario/1` library in
its feature subdirectories (`faults/`, `fleet_bus/`, `missions/`, `sar/`, `worlds/`, `verify/`, ...).
The nightly suite is `./sim suite tests` (a.k.a. `ci/run_suite.py --roots
tests`); `ci/check_library.py` validates every file on push. For a fast
first pass, `./sim suite --tier smoke` runs only the files marked
`tier: smoke` - one probe per subsystem (bring-up, auth, faults, mission
supervisor, events, modules, world detect, verify gate). Smoke is a
selection heuristic, not evidence - the unmarked files stay the
regression tier.

See the docstring in `engine/runner.py` for every `do:` step and
`docs/SCENARIOS.md` for the format. In short:

- **`do:` steps:** `deploy`, `arm`, `start`, `stop`, `clear`, `fire`, `mode`,
  `land`, `reset`, `wait`, `wait_journal`, `wait_log`, `wait_online`,
  `wait_ready`,
  `kill` / `restart` / `freeze` / `unfreeze` (also `unit: server`;
  `freeze` is SIGSTOP - a hang, not a crash), `cpu_starve` /
  `cpu_unstarve` (a SIGSTOP/SIGCONT duty cycle), `journal_unwritable` /
  `journal_writable` (unit journal dir read-only, auto-restored on
  `dur_s` and always by `sim down`), `clock_step` (per-process clock skew
  on a `faketime: true` unit's de_comm via libfaketime's
  `FAKETIME_TIMESTAMP_FILE`), `reboot` (de_comm under a new
  boot id), `server_up`, `loss` (fake server only) / `delay` /
  `link_cut` / `link_clear`
  (fake server, or real with `link_proxy: true`), `link` (`{unit, set}` -
  raw link-model sub-lines:
  `stall`, `ge`, `bw`, `jitter`, `loss_tx`, `loss_rx`, `profile`; loss*
  is fake-only),
  `commline`, `send`, `udp_chunks` (`{unit, module,
  drop_pct, reorder, dup_pct, seed}` - knobs on the unit's
  module↔de_comm chunk proxy), `write_file` (`{unit, module,
  path, json}` - a JSON file into `var/run/<unit>/<module>/`, for module file
  seams like de_rpi_gpio's `gpio_inputs.json` or de_precland's
  `precland_scene.json`; `also: [{path, json, delay_ms?}]`
  writes extra files inside the same call — an entry with `delay_ms`
  sleeps N ms before its write, keeping the order);
- **`.deop` operations (TASK-23):** `operation:` may name a
  de.operation/2 source (`ops/x.deop`) - the runner compiles it
  on demand with tools/compile_op.js into
  `var/build/packages/<key>/` (source+compiler+ctx hash; edits
  recompile, unchanged runs reuse). Its `<x>.ctx.json` sidecar
  supplies the binding + capability registry; without one the
  binding derives from the fleet (`role:` tags, then fleet order)
  and the dir's single `*.unit.caps.json` feeds --caps;
- **fleet ops (P4A-09):** when `operation:` names a deploy spec (a top-level
  `roles` map - see `packages/p4a05.deploy.json`), `deploy` runs the
  two-phase DE_OP_CONTROL (1094) PREPARE per role, and the run goes up
  with `opstart` / down with `opabort` (`opstatus`, `opmsg` for probes).
  A fleet package refuses the plain `start`;
- **step tokens** (`send`/`opmsg`): `{RUN}`, `{OP_RUN}`, `{PKG:<file>}`,
  `{HASH:<file>}` (paths resolve as given, under `packages/` and the
  on-demand build's `var/build/packages/by-name/`; test fixtures use
  the explicit `tests/packages/<file>` path), `{WM_TID:<unit>}`;
- **assertions:** `journal_count`, `module_log_count` (same bounds over
  `logs/<module>.log` - module stdout, for facts that never reach the
  journal like de_precland's `EVENT_OUT` lines), `journal_order`,
  `journal_time_delta`, `mission_current`, `clock_spread`;
- **teardown:** `land` (default) | `reset` | `none`.

`unit:` takes a unit name, its 1-based position, `block:<name>`, a comma
list, or `all`.

`land` brings each vehicle type down its own way: copter LAND, rover
HOLD + disarm. A plane cannot land in place, so it gets a `reset`: a power-cycle
of SITL (fresh, at home), de_comm and its modules.

Results land in `var/results/<timestamp>-<test>/`: journals, logs, the test file
and the sim manifest.

## Scenarios - human-run

`scenarios/` is the person's side of the library: `demos/` (things to
watch, like the Mavlink3DMap canopy viewer), `examples/` (small commented
starters), and wherever you drop your own files. `./sim new <name>`
scaffolds one; `./sim validate <file>` checks it without starting
anything. **docs/SCENARIOS.md is the authoring reference** - paste it to
an LLM and it can write a scenario for you.

## The scenario format (de.scenario/1)

One versioned file rehearses a whole operation on the desk - fleet,
operation, operator policy, scripted timeline, faults, assertions:

```
./sim run tests/faults/link_cut.yaml                    # one scenario - the
                                                        # file's `seed:` applies
./sim run tests/faults/link_cut.yaml --seed 7           # a seed run (overrides
                                                        # the file's `seed:`)
./sim suite tests/faults --seeds 1,2                    # suite x seeds
./sim view scenarios/demos/world_canopy_viewer.yaml     # sim + Mavlink3DMap
                                                        # viewer, held till Ctrl-C
```

(`run_scenario.py` still works - a shim over the same engine.)

Format (strict - unknown keys fail against
`engine/de_scenario.schema.json`):

- `version`, `name`, `seed`, `realtime`, `transport` (`fake` | `real`),
  `home`, `slot_base`, `world` (P5-08 `worlds/*.yaml`: seeded ground
  truth - entities, motion, attachments, transmitters, clutter,
  camera and SDR models; P5-10 adds `obstacles` - canopy crowns and
  buildings as vertical prisms the stubs attenuate/occlude through
  (`transmittance`, `rf_loss_db`) -, a camera `lens:` option
  (`rectilinear` the default since P5-18 - what a real pinhole lens and
  the product projector do; `angle_linear` kept for comparison) and
  optional Brown `k1`/`k2` distortion,
  ground-anchored clutter (`anchor: ground` - a false positive is a
  fixed ground point, not a sliding pixel); P5-11 adds per-entity
  `behaviour:` (seeded `stationary`/`wander`/`trail`/`seek` tracks that
  avoid buildings), `trails:`/`features:`/`environment:` (sun,
  visibility, precip, wind - the wind reaches every FC unit's
  sim.parm as `SIM_WIND_SPD`/`SIM_WIND_DIR`/`SIM_WIND_TURB`, and the
  camera model dims at dusk unless `kind: thermal`), plus a `state`
  field on the truth rows; P5-12 adds `areas:` - named search
  polygons the coverage asserts grade; PL-08 adds `cls: pad`
  precision-landing boards (`yaw_deg`, `size_m`, `tags_min_m`/
  `tags_max_m` tag ladder, `occlude` world-second windows) and the
  `sensors.precland` downward tag-camera model (FOV, `min_px_tag`,
  `max_rmse_px`, `noise_m`, `p_drop`) - `stub_ai_box` never reports a
  pad as a detection class). Consumed by the `stub_*`
  modules and the truth recorder, which writes
  `var/run/<sim>/truth.jsonl` at 1 Hz plus per-detection rows),
  `operation`, `verify` (P5-06 CLI, absent = recorded skip),
  `operator` (the bot's standing approve/confirm policy), `fleet`,
  `timeline`, `assert`, `teardown`, `settle_s`, `render` (P5-13 opt-in:
  `{mode: viewer|cameras|physics|full, headless}` opens a
  `de.worldstream/1` WebSocket server inside the runner on
  `61000 + slot_base` - hello snapshot + 10 Hz entity/clutter ticks +
  `cmd payload_release` + a 1 Hz `overlay` block (bot world-model
  targets, per-tid seen/miss/occluded states, recent detection
  markers, P5-14), with client `event` frames appended to
  truth.jsonl as `src: render` - and gives every FC unit a
  `--serial4 udpclient` MAVLink fan-out to `16450 + slot_base` for
  Mavlink3DMap. Absent = today's headless behaviour, unchanged ports
  and sim hash; `engine/worldstream_probe.py` is the test client.
  P5-14: `mode: viewer|full` with `headless: false` also launches
  Mavlink3DMap's `udp2websocket.js` (UDP `16450 + slot_base` ->
  ws `8801 + slot_base`) and a vite dev server on `8070 + slot_base`
  from `$MAVLINK3DMAP_DIR` (sim.local.env), pid-tracked under
  `var/run/<sim>/render/` - `./sim up` prints the viewer URL
  `http://127.0.0.1:<web>/?world=ws://127.0.0.1:<stream>`
  and opens no browser. `headless: true` streams without them.
  The world stream lives inside the runner process: `./sim view
  <scenario>` (or `./sim run <scenario> --hold`) parks the run after
  the timeline so the stream and the viewer stay live until Ctrl-C,
  then assertions and teardown run as usual.
  P5-15: `mode: cameras|full` adds the rendered-camera farm on
  `blocks/copter_cam3d` units (real `de_yolo_ai` + `de_camera`):
  headless Chromium (`$CHROMIUM_BIN`) renders each unit's
  hello-advertised camera into `[32-byte header][JPEG]` frames on a
  `8090 + slot_base` binary WebSocket; `render_sink.js` demuxes them
  into one ffmpeg per unit writing that unit's `DE-SRC-<cam_index>`
  v4l2loopback device. de_yolo_ai detects on it and writes annotated
  frames to `DE-AI-<cam_index>`, which de_camera streams. A 16-cell
  frame-number strip in the bottom rows lets `engine/render_probe.py`
  (one per unit) decode frames back off the device and post
  `src: render_probe` `frame_seen` rows; every 5th frame a
  quarter-res ID pass recolours truth entities to unique flats and
  posts `src: render_vis` `visibility` rows (px + normalized box) -
  the honest "was it actually visible" truth for
  `detection_recall {src: render_vis}`, `visible_px`,
  `detection_precision_render` (IoU vs the 1078 boxes) and
  `render_latency` (pose -> device p95). Needs one-time
  `sudo tools/setup_v4l2loopback.sh <n>` (two labelled loopbacks per
  camera unit) - `sim up` fails with that hint when the labels are
  missing. Statistical tier, not byte-deterministic: nightly/
  on-demand, never the pure gates).
- `fleet:` entries take `id`, `cls` (`copter|plane|rover|static|
  comm_only` or a `blocks/` name), `role`, absolute `home`/`base`,
  relative `at`, `modules`, `params`, `config`, `account`. `modules`
  can add catalog entries such as the P5-08 world stubs
  (`stub_ai_box`, `stub_sdr`, `stub_payload`, `stub_precland`):
  python databus modules
  in `stubs/` that read the vehicle pose (SITL serial5 on FC units,
  the intermodule 9102 feed on statics), run the world's seeded
  camera/SDR/payload models, and emit the real advert/event contracts
  (1077/1078 AI messages, `sdr.signal`, `payload.released`).
  `stub_precland` (PL-08) stands in for the real `de_precland` where
  there is no AprilTag renderer: it decides pad visibility from the
  pose + world geometry and publishes the real PRECLAND_TARGET (6537)
  / PRECLAND_STATUS (6538) wire messages plus the `precision_landing`
  capability advert (`stubs/precland_advert.json`, byte-identical to
  `de_precland --print-advert`) and facts (`stubs/precland_events.py`
  ports CPreclandEventModel, pinned by the module's golden vectors).
  `blocks/copter_precland` wires it to a copter with the FC `PLND_*`
  overlay and the de_mavlink `precland` gate open; a scenario whose
  `world:` is `worlds/pad_field.yaml` gives it a pad to land on.
- `timeline:` steps are `{t: <s>, do: <step>, ...}` or
  `{t: <s>, fault: <kind>, ...}`; `t` is seconds from scenario start and
  may be negative (runs at once). `{on: {unit, re}}` arms a journal
  watcher instead; `once` defaults true; regex groups substitute as
  `{M1}`..`{M9}`. The run lasts `last_t + settle_s`.
- `do:` names either a runner.py step (`arm`, `mode`, `takeoff`,
  `param`, `param_get`, `wait`, `link_clear`, ...) or an operator action
  (`deploy`, `start`, `abort`,
  `operator act: approve|deny|confirm|dismiss|fire|<1086>`).
  `wait_journal` takes `any: true` to finish on the first of its units
  that journals the line (a line only a runtime-chosen unit writes, e.g.
  the takeover adopter); `start` takes `expect_fail: [unit, ...]` for
  units whose START refusal is the expected outcome. `on:` watchers are
  polled between steps only - a long blocking wait starves them.
- The operator bot mirrors the WebClient's 1093 resend watch: it keeps
  every decision of the run and re-sends it unchanged (2..30 s backoff)
  while the units' roster `wm` digests disagree.
- `fault:` kinds: `link_cut`, `link_stall`, `link_profile`, `loss`,
  `delay`, `gcs_disconnect`,
  `server_down`, `kill`, `restart`, `module_kill`, `param`,
  `mode`, `rc_override`, `server_clock`, `no_time_sync` (fake transport
  for the link/clock ones) plus the FC param presets `battery`,
  `gps_off`, `gps_jam`, `gps_glitch`, `gps_degrade`, `motor_out`,
  `rc_fail`, `compass_fail`, `vibration`. `param` writes a `set:
  {NAME: value}` map through the verified serial0 param path; every
  param fault (presets included) reads each value before writing, so
  `dur_s` restores what the FC actually reported - never a hardcoded
  default. `dur_s` auto-restores where it applies.
  default. `udp_chunks` runs a seeded drop/reorder/dup
  chunk proxy on the unit's module↔de_comm s2s link - works on any
  transport; the loader wires the `udp_proxy_<module>` proc itself.
  `dur_s` auto-restores where it applies.
- `assert:` adds `proc_alive`/`proc_dead`, `link_offline`,
  `roster_value`, `operator_log_count`, `telemetry_value`, `fc_mode`,
  `distance_to` (incl. `detected_cls:` against the bot's seen targets),
  `coverage_complete` (P5-12 gains an `area:`/`poly:` form: the tlog's
  camera-unit poses - now including ATTITUDE - replayed through the
  world camera/LOS model onto a `cell_m:` grid; `min:` the covered
  fraction; `coverage.json` + `coverage.geojson` land in the run dir),
  `pod_mean` (mean cumulative POD over the area, `min:`) and `pod_at`
  (the POD of the cell holding a truth entity at `t_s:`/run end -
  `min:`/`max:`; paired with `detection_recall` it splits planning
  failures - low POD - from sensor/chain failures - high POD, no
  find), `no_sep_violation`, `detection_precision`,
  `detection_recall` (P5-10: the fraction of the named truth entities
  that were found - `src: ai|sdr|bot` (bot = the whole product chain),
  `match_m:` for the bot src, `max: 0` is the "must stay hidden"
  negative control, the detail line lists each miss's dominant why),
  `time_to_detect` (P5-10: seconds from the `start` step - the world
  epoch when none ran - to the first find, `max_s:`;
  realtime-sensitive), `signals_in_range`,
  `stub_deterministic` to the journal assertion set.
  Run-wide invariants (SL-0011): a 1 Hz sampler records every tracked
  process's pid/RSS/fds into `results/samples.jsonl`;
  `no_unexpected_restart` (pid changes only inside `fault:` kill/
  restart/module_kill/server_down windows - `do: kill` fails it),
  `rss_growth` (least-squares MB/h after `warmup_s`), `fd_growth`,
  `journal_size`, `recovers_after_fault` (link+telemetry back within
  `within_s` of each timed fault restore), `no_stuck_state` (a `re`
  state line must change within `max_s`).
  `journal_count` takes `total: true` to compare the sum over its units
  instead of every unit's own count; `module_log_count` accepts the same
  `total:` flag (B4-01 - for a line exactly one runtime-chosen unit
  prints).
  `realtime: true` gates timing asserts - they SKIP with a visible
  reason when SITL speed-up is not ~1. `telemetry_value` reads the last
  sample by default; `pick: min|max` reads the extreme of the recorded
  series (a sag that recovers by disarm still counts).

Scenario runs fly SITL at `--speedup 3` by default (only SITL's physics
clock scales; modules, the world truth tick and rendered camera feeds
stay wall-clock). A scenario pins its own rate with `speedup:` -
`realtime: true` (timing asserts) or `speedup: 1` (render-farm/real
cameras, world-driven stubs on a moving SITL, closed module<->FC
loops). `sim run --speedup N`/`SIM_SPEEDUP` overrides, and a `sim:`
file's own `speedup:` still wins over the default.

Each run gets a seed-derived port block (no cross-seed collisions) and
`var/results/<utc>/<name>/`: journals, per-unit and server logs,
`links.jsonl` (link up/down transitions the monitor saw),
`telemetry.jsonl` (per-unit FC samples = the tlog), `operator.log` (the
bot's GCS session journal), `truth.jsonl` when a world supplies it,
`verify.json`, `summary.json` (PASS/FAIL/SKIP + commit ids + speed-up)
and `junit.xml`.

**SITL capacity at speed-up 1 (this machine, 14 cores):** 24 copter
SITLs ran together for 75 s with measured speed-up 0.999-1.002 per
unit (see `speedup` in `summary.json` - FC `time_boot_ms` vs wall).
Beyond the tested 24 the FC loops may fall behind; `realtime: true`
scenarios then report a visible SKIP instead of a timing FAIL.

## Operator console

`./sim ctl` with no arguments prints the full list. `./sim ctl -s <sim>`
pins a command to one sim when several are live. The most used:

```
./sim ctl status | fc all | arm all | mode copter2 RTL | land all | reset plane1
./sim ctl deploy packages/copter_square.json | start | fire area.clear | stop | clear
./sim ctl journal copter1 | roster 10 3 | kill server de_server | server_up
./sim logs copter1 de_comm | ./sim logs server gcs_bridge
```

## Verified (2026-10-01, this machine)

| test | sim / server | result |
|---|---|---|
| `smoke_copters` | 3 copters, real server | 5/5 |
| `smoke_copters_auth` | 3 copters, real server + authenticator | 5/5 |
| `server_restart` | 3 copters, real server killed + restarted | 2/2 |
| `fault_link_loss` | 3 copters, fake server (40% loss, 45 s cut) | 5/5 |
| `mixed_bringup` | copter, plane and rover each arm and come down their own way; ground node stays online | steps pass |

Not yet run: `copter_and_cameras`. The `de_camera` binary isn't built on this
machine, so `sim up` stops with a clear "missing binaries" error.

## Notes

- **The real server does not stamp `sd`.** It routes whatever sender id the
  client wrote (`js_chat_routing.js` only injects `p`). The fake server
  stamps `sd`, so a forged-sender test that passes on the fake server
  means nothing on the real one. The bridge sets `sd: GCS_FLEET` itself.
- de_comm journals `comm line offline/online` only while a v2 package is
  loaded, and `nav adapter hello` only once per de_comm process. Tests and
  `reset` are written around both.
- The authenticator allows 10 logins/min per IP. All units share 127.0.0.1,
  so an auth sim with more than ~9 units will see some de_comm logins retried.
- SITL needs ~1-2 min for GPS/EKF before `arm` succeeds. `arm` waits for it.
- **P5-18 lens flip (release note):** the projection model changed from
  angle-linear to rectilinear in `de_common::geo_projection` and the
  harness together. Detection ground positions move by up to ~2.5 % of
  altitude off-axis, so positions/uncertainties recorded before the fix
  are **not directly comparable** to post-fix replays (the stub goldens
  were re-recorded for this reason).
