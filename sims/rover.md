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
./sim ctl -s rover web --print
```

`land` on the rover = HOLD + disarm. Runs in parallel with
`quadcopter.yaml` / `plane.yaml` — each personal sim has its own
slot_base.

## Teardown

```bash
./sim down rover
```

## Ports

slot_base 60: rover1 sysid 60 (pilot 6360, fcb 8260, de_comm 62600,
de_mavlink 63600), ctl 62069. External server wss 9966, auth 19408.
