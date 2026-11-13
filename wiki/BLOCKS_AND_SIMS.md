# Blocks, sims and servers

[← Wiki index](README.md) · [← Studio README](../README.md)

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
the noun diagram and glossary is [docs/MAP.md](../docs/MAP.md).

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
                                     # storage: true adds a real storage server
                                     # (DE_STORAGE_DIR; persist: true keeps its
                                     # DB across down/up; storage_port: pins it)
bus: udp                             # udp (default) | unix - unix-bus units
                                     # run the module bus on a per-unit AF_UNIX
                                     # sockdir (unit.<u>.sockdir lease), no s2s
                                     # ports; only unix-capable modules move
                                     # (catalog `unix: true`), stubs/udp_proxy
                                     # stay UDP. Per-unit `bus:` overrides.
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
[sims/overlays/README.md](../sims/overlays/README.md).

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
only `./sim down de_servers` stops it; add `--out <host>` to make it
reachable from other machines such as a phone, see `stacks/README.md`).
`./sim up de_webclient`
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

Running several sims at once, `./sim studio` and the port allocator: see [Parallel sims, studio and ports](PARALLEL_SIMS_AND_PORTS.md).
