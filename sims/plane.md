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

Every local port (SITL, de_comm, de_mavlink, the bridge control port) is
leased from the machine-wide allocator at `up` — `./sim ports -s plane`
lists what was actually bound. `slot_base: 70` is identity only (plane1
sysid 70). External server wss 9966, auth 19408 — those are the remote
server's own ports, not leases.
