# boat — how to run

Personal sim: ONE motor boat — ArduRover SITL (motorboat frame) +
de_comm + de_mavlink — on **your own** server stack. See `quadcopter.md`
for the external-server fields (accounts, `--user`/`--auth-server`
overrides).

## Setup

- Start your servers first (`servers/sh_start_servers.sh`):
  authenticator on 19408, comm server on 9966.
- Accounts via `SIM_AUTH_*` / `SIM_UNIT_AUTH_*` in `sim.local.env` or
  `./sim up` flags.

## Run

```bash
./sim up sims/boat.yaml
./sim ctl -s boat status
./sim status -s boat --open-web
```

`land` on the boat = HOLD + disarm. Runs in parallel with
`quadcopter.yaml` / `rover.yaml` / `plane.yaml` — each personal sim has
its own slot_base. To add it next to a live sim without a slot clash:

```bash
./sim up boat --shared --slot-base any
```

## Teardown

```bash
./sim down boat
```

## Ports

slot_base 90: boat1 sysid 90 (pilot 6660, fcb 8560, de_comm 62900,
de_mavlink 63900), ctl 62099. External server wss 9966, auth 19408.
