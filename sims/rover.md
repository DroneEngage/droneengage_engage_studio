# rover — how to run

Personal sim: ONE rover box — ArduRover SITL + de_comm + de_mavlink — on
**your own** server stack. See `quadcopter.md` for the external-server
fields (accounts, `--user`/`--auth-server` overrides).

## Setup

- Start your servers first (`servers/sh_start_servers.sh`):
  authenticator on 19408, comm server on 9966.
- Accounts via `SIM_AUTH_*` / `SIM_UNIT_AUTH_*` in `sim.local.env` or
  `./sim up` flags.

## Run

```bash
./sim up sims/rover.yaml
./sim ctl -s rover status
./sim status -s rover --open-web
```

`land` on the rover = HOLD + disarm. Runs in parallel with
`quadcopter.yaml` / `plane.yaml` — each personal sim has its own
slot_base.

## Teardown

```bash
./sim down rover
```

## Ports

Every local port (SITL, de_comm, de_mavlink, the bridge control port) is
leased from the machine-wide allocator at `up` — `./sim ports -s rover`
lists what was actually bound. `slot_base: 60` is identity only (rover1
sysid 60). External server wss 9966, auth 19408 — those are the remote
server's own ports, not leases.
