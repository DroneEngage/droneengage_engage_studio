# quadcopters-4 — how to run

Personal sim: FOUR quad boxes — ArduCopter SITL + de_comm + de_mavlink
each — on **your own** server stack. Same external-server wiring as
`quadcopter.yaml` (see its runbook for the full account/server notes).

## Setup

- Start your servers first (`servers/sh_start_servers.sh`):
  authenticator on 19408, comm server on 9966.
- Accounts via `SIM_AUTH_*` / `SIM_UNIT_AUTH_*` in `sim.local.env`, or
  `--user`/`--password`/`--auth-server` on `./sim up`. Per-unit
  `account:` with `{name}` expansion works if each quad needs its own
  login.

## Run

```bash
./sim up sims/quadcopters-4.yaml
./sim ctl -s quadcopters-4 status
./sim status -s quadcopters-4 --open-web
```

## Teardown

```bash
./sim down quadcopters-4
```

## Ports

Every local port (SITL, de_comm, de_mavlink, the bridge control port) is
leased from the machine-wide allocator at `up` — `./sim ports -s
quadcopters-4` lists what was actually bound. `slot_base: 50` is
identity only (quad1..quad4 sysids 50..53). External server wss 9966,
auth 19408 — those are the remote server's own ports, not leases.
