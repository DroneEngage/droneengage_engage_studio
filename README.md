# droneengage_simulation_land

Reusable building blocks for DroneEngage simulations. You describe a vehicle
type once, compose fleets out of these types, and run tests against a fleet.
Everything runs headless on one machine against the **real**
`droneengage_server` (optionally with `droneengage_authenticator`) on its own
ports. A fake server is available for link-fault tests.

```
blocks/   unit types   - what one unit runs (SITL? which modules? config)
sims/     fleets       - blocks composed: names, counts, placement, server
tests/    scenarios    - a sim + steps + assertions
```

## Setup (per machine)

```bash
cp sim.local.env.example sim.local.env    # gitignored: this machine's paths
```

Edit `sim.local.env` for your machine: `DE_CODE` (root of the
DroneEngage checkouts), `ARDUPILOT_DIR`, and per-repo or per-binary
overrides (`DE_MAVLINK_DIR`, `DE_CAMERA_BIN`, `NODE`, ...). An environment
variable with the same name wins over the file.

The machine also needs:

- ArduPilot SITL built (`./waf configure --board sitl && ./waf copter plane rover`).
- `de_comm` and `de_ardupilot` built (`bin/` in their repos). Other modules
  are only needed when a block uses them.
- `npm install` done in `droneengage_server` and `droneengage_authenticator`.
  `node` must be on PATH, or set `NODE`.
- `pip install websockets pymavlink pyyaml`, and `openssl` (fake server only).

No root needed. Servers and modules use 127.0.0.1. de_comm's s2s UDP port
and SITL's TCP ports listen on all interfaces.

## Quick start

```bash
./sim blocks                          # list the reusable unit types
./sim show sims/copters3.yaml         # what a sim expands to (units, ports)
./sim run tests/smoke_copters.yaml    # brings the sim up if needed, runs, lands
./sim ctl status                      # live processes / online units
./sim down
```

`./sim run` reuses a running sim only if it is the **same** sim (hash of the
resolved units and server) and every process is alive. Otherwise it brings
the sim up fresh. Several tests can share one sim and run back to back without
restarting SITL.

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
server: {type: real}                 # real (default) | fake ; auth: true
units:
  - {block: copter, name: copter, count: 3}       # copter1..copter3
  - {block: plane, name: plane1, at: {east_m: 60}}
  - block: camera_fixed
    name: tower1
    at: {north_m: -50, east_m: 30}
    config: {de_camera: {camera: {camera_start_index: 3, camera_end_index: 3}}}
```

Server types:

| `server:` | runs | use it for |
|---|---|---|
| `{type: real}` | `droneengage_server` in `local_server_enabled` mode | default: the real routing |
| `{type: real, auth: true}` | + `droneengage_authenticator` (single account); every unit and the GCS log in | the login path |
| `{type: fake}` | `engine/fake_fleet_server.py` | seeded per-link loss / delay / cut |
| `{type: external, host, port}` | nothing - your own running server stack | personal sims on your real servers |

`external` is for sims that join a server stack you run yourself (e.g.
`servers/sh_start_servers.sh`): no server processes are spawned, every
unit's de_comm connects to `host:port` - directly when `auth: false`, or
through your authenticator when `auth: true, auth_host, auth_port` are
set. The account (`server.account: {user, code}` or `SIM_AUTH_USER` /
`SIM_AUTH_CODE` in `sim.local.env`) must exist on that authenticator;
`up` fails early if the server isn't reachable.

The real servers run from their repos with a config **generated** under
`run/<sim>/server/`. Their own `server.config(.local)` files are never
read or changed.

With a real or external server, `engine/gcs_bridge.py` holds one GCS
session (`GCS_FLEET`, or `GCS_<sim>` for external sims) and serves the
same control protocol as the fake server, so `./sim ctl` behaves the
same on all of them. Link faults are refused on a real server.

### Running several sims in parallel

`./sim up` is additive: it only restarts the same-named sim and refuses
to start one whose ports or unit names collide with a live sim (pick a
different `slot_base`). `run/<sim>/` holds each sim's manifest, configs,
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
search live sims first. `sim run` stays exclusive - a test owns the
machine and downs the other sims.

`sims/quadcopter.yaml`, `sims/rover.yaml` and `sims/plane.yaml` are
ready-made single-box personal sims for an external server, on disjoint
`slot_base`s (50/60/70).

### Ports (slot k = slot_base + unit index; slot_base defaults to 10)

| use | port |
|---|---|
| SITL pilot (serial0) / aux (serial2) | 5760+10k / 5762+10k |
| de_mavlink -> SITL (serial1) | 7660+10k, sysid k |
| de_comm / de_mavlink / de_camera s2s | 62000 / 63000 / 64000 + 10k |
| comm server wss / control | 62008 / 62009 (`server.port`) |
| authenticator https / s2s | port+10 / port+11 |

The default slot base keeps clear of low-numbered ports so sims never
collide with a developer's own servers. Sims meant to run **in parallel**
need disjoint `slot_base`s - `up` verifies there is no overlap with a
live sim before starting anything.

## Tests

See the docstring in `engine/runner.py` for every step and assertion. In
short:

- **steps:** `deploy`, `arm`, `start`, `stop`, `clear`, `fire`, `mode`,
  `land`, `reset`, `wait`, `wait_journal`, `wait_online`, `wait_ready`,
  `kill` / `restart` (also `unit: server`), `reboot` (de_comm under a new
  boot id), `server_up`, `loss` / `delay` / `link_cut` / `link_clear`
  (fake server only), `commline`, `send`;
- **fleet ops (P4A-09):** when `operation:` names a deploy spec (a top-level
  `roles` map - see `packages/p4a05.deploy.json`), `deploy` runs the
  two-phase DE_OP_CONTROL (1094) PREPARE per role, and the run goes up
  with `opstart` / down with `opabort` (`opstatus`, `opmsg` for probes).
  A fleet package refuses the legacy `start`;
- **step tokens** (`send`/`opmsg`): `{RUN}`, `{OP_RUN}`, `{PKG:<file>}`,
  `{HASH:<file>}` (paths resolve under `packages/` too), `{WM_TID:<unit>}`;
- **assertions:** `journal_count`, `journal_order`, `journal_time_delta`,
  `mission_current`, `clock_spread`;
- **teardown:** `land` (default) | `reset` | `none`.

`unit:` takes a unit name, its 1-based position, `block:<name>`, a comma
list, or `all`.

`land` brings each vehicle type down its own way: copter LAND, rover
HOLD + disarm. A plane cannot land in place, so it gets a `reset`: a power-cycle
of SITL (fresh, at home), de_comm and its modules.

Results land in `results/<timestamp>-<test>/`: journals, logs, the test file
and the sim manifest.

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
