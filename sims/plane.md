# plane — how to run

Personal sim: ONE fixed-wing box — ArduPlane SITL + de_comm +
de_mavlink — on **your own** server stack. See `quadcopter.md` for the
external-server fields (accounts, `--user`/`--auth-server` overrides).

## Setup

- Start your servers first (`servers/sh_start_servers.sh`):
  authenticator on 19408, comm server on 9966.
- Accounts via `SIM_AUTH_*` / `SIM_UNIT_AUTH_*` in `sim.local.env` or
  `./sim up` flags.

## Run

```bash
./sim up sims/plane.yaml
./sim ctl -s plane status
./sim status -s plane --open-web
```

The plane arms in MANUAL; `land` resets the SITL (a plane cannot land
in place). Runs in parallel with `quadcopter.yaml` / `rover.yaml` —
each personal sim has its own slot_base.

## Teardown

```bash
./sim down plane
```

## Ports

slot_base 70: plane1 sysid 70 (pilot 6460, fcb 8360, de_comm 62700,
de_mavlink 63700), ctl 62079. External server wss 9966, auth 19408.
