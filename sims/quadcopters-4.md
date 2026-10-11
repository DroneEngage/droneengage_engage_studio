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
./sim ctl -s quadcopters-4 web --print
```

## Teardown

```bash
./sim down quadcopters-4
```

## Ports

slot_base 50: quad1..quad4 sysid 50..53 (pilot 6260..6290, fcb
8160..8190, de_comm 62500..62530, de_mavlink 63500..63530), ctl 62059.
External server wss 9966, auth 19408.
