# Parallel sims, studio and ports

[← Wiki index](README.md) · [← Studio README](../README.md)

## Running several sims in parallel

**Default posture: interactive work is additive, tests are exclusive.**
`up`, `studio`, `web`, `ctl` and a `run`/`view` of a bare `sims/*.yaml`
(no steps to execute) all stay next to whatever is already running. A
`run`/`view`/`suite` of a file with `steps:` owns the machine - it downs
every other sim first, because a test needs a clean machine to be
meaningful. On a shared box pass `--shared` (or export `SIM_SHARED=1`)
and even scenario runs stay additive; exclusive mode is the CI posture.

`./sim up` is additive: it only restarts the same-named sim. Ports
cannot collide - every port a sim binds is leased from the machine-wide
allocator inside the `up` transaction, so two fleets simply get
different leases. Unit names may repeat
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
rules to a `run`/`view`/`suite` instead. `--slot-base <N>` overrides
the slot the fleet identity derives from for that invocation (SITL
`-I`/sysid and unit positioning; wins over the sim's `slot_base:` and
`SIM_SLOT_BASE`). No port derives from it - every port is an allocator
lease (`--slot-base any` is accepted as "I don't pin the identity"
and means the same as a bare `slot_base:`). Allocated port values are
not part of the sim hash, so a moved or shifted fleet still
warm-reuses - `run` does not pay the ~1-2 min SITL warm-up for a port
move alone.

```bash
./sim run tests/smoke_copters.yaml --shared                    # next to a live sim
./sim run tests/smoke_copters.yaml --shared --slot-base 90     # distinct SITL identity
```

`sims/quadcopter.yaml`, `sims/rover.yaml` and `sims/plane.yaml` are
ready-made single-box personal sims for an external server, on distinct
`slot_base`s (50/60/70) so their SITL identities/sysids stay apart.

## `./sim studio` - the human session

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

## Ports (slot k = slot_base + unit index; slot_base defaults to 10)

**Every** local port a sim binds is a named request to the machine-wide
allocator - there are no formulas left. The table lists the lease
names; the values come from the pool, usually the port the same sim
held last run (the allocator remembers released ports and tries them
first). Read `var/run/<sim>/sim.json`'s `ports` map, `./sim ports` or
`./sim ctl status` for the values a running sim actually bound; `sim
up` prints `moved <name> <old> -> <new>` when a hinted value could not
land.

| use | lease name (proto) |
|---|---|
| comm server wss / control | `server.wss` / `server.ctrl` (tcp) |
| authenticator https / s2s | `auth.https` / `auth.s2s` (tcp) |
| udp relay wss / UDP pair window | `udp_relay.relay.wss` (tcp) / `.range` (udp block) |
| render stream / mav / mav-ws / web / sink | `render.stream` / `.mav` / `.mav_ws` / `.web` / `.sink` (tcp) |
| unit s2s (`de_comm`, `de_mavlink`, `de_camera`, stubs, ...) | `unit.<u>.<module>` (udp) |
| unit link proxy listen / control (`link_proxy`) | `unit.<u>.link_proxy` / `.link_proxy.ctrl` (tcp) |
| unit SITL serials (block: serial0 pilot +0, serial2 aux +2, serial5 pose +5) | `unit.<u>.sitl` (tcp block) |
| unit de_mavlink -> SITL (serial1) | `unit.<u>.sitl.fcb` (tcp), sysid k |
| unit SITL RC-in / sim out / sim in / irlock (UDP) | `unit.<u>.sitl.rcin` / `.sim_out` / `.sim_in` / `.irlock` |
| unit fcb proxy listen / control (`fcb_proxy: true`) | `unit.<u>.fcb_proxy` / `.fcb_proxy.ctrl` (tcp) |
| external-server GCS bridge control | `bridge.ctrl` (tcp) |

SITL launches with `--base-port <block> --rc-in-port <leased>` (and
`--sim-port-in/--sim-port-out/--irlock-port`) *before* `-I`, so `-I`
stays the eeprom/identity slot only and every socket is the leased
value. The two viewer defaults (render.mav_ws 8811, render.web 8080)
are the only static preferences - a stock Mavlink3DMap still finds the
sim where it looks first.

One rule survives the allocator: yaml-explicit ports (`server.port`,
`server.control_port`, render ports, a relay `udp_port_range`) pin -
exact value or `up` refuses. Sim names and unit names still
disambiguate fleets; sims meant to run **in parallel** no longer need
disjoint `slot_base`s at all, only `--shared` discipline.

## Port leases

Every port a sim binds is leased, atomically and machine-wide, before a
single process starts (`docs/port_allocation.md` is the design). The
registry lives at `$XDG_RUNTIME_DIR/de_sim/ports.json` (fallback
`~/.cache/de_sim/ports.json`, override `SIM_PORT_REGISTRY` or
`SIM_PORT_LAST` for the previous-port memory in
`~/.cache/de_sim/last_ports.json`) - outside `var/` on purpose, so every
worktree of this repo shares it: a sim brought up in a `_wt/` worktree
blocks the same ports for a sim `up`ed here. `sim down`/`sim clean`
release the leases; a `sim up` killed with `kill -9` leaves `reserved`
leases the next transaction sweeps away on its own. The transaction
result - the flat request-name -> port map - is written to
`var/run/<sim>/sim.json` as `ports`, so post-mortems and `ctl` read the
real leased values.

```bash
./sim ports            # this worktree's leases
./sim ports <sim>      # who holds that sim's ports (any worktree)
./sim ports --all      # every lease in the registry
./sim ports gc         # sweep leases whose holder/sim is dead
./sim doctor           # reports registry health among the checks
```
