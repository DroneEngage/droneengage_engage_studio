# quadcopter — how to run

Personal sim: ONE quad box — ArduCopter SITL (raw binary over TCP, like
sitl_tcp.sh) + de_comm + de_mavlink — on **your own** server stack.
Includes `render: {mode: viewer}`: a Mavlink3DMap mav_bridge + vite dev
page comes up with the sim.

## Setup

- Start your servers first (`servers/sh_start_servers.sh`):
  authenticator on 19408, comm server on 9966. Nothing server-side is
  spawned — only this sim's GCS bridge, which logs in like a GCS client.
- Accounts: the GCS login needs the GCS permission bit, the unit login
  the unit bit, both on the same team — set `SIM_AUTH_*` /
  `SIM_UNIT_AUTH_*` in `sim.local.env`, or pass
  `./sim up sims/quadcopter.yaml --user me@x.com --password secret
  --auth-server 203.0.113.9:19408`. Per-unit `account:` with `{name}`
  expansion is supported in the yaml.
- `MAVLINK3DMAP_DIR` in `sim.local.env` for the viewer.

## Run

```bash
./sim up sims/quadcopter.yaml
./sim ctl -s quadcopter status       # prints the viewer + webclient links
./sim status -s quadcopter --open-web
./sim ctl -s quadcopter arm quad1    # after ~1-2 min GPS/EKF warm-up
```

Runs in parallel with `plane.yaml` / `rover.yaml` — each personal sim
has its own slot_base.

## Teardown

```bash
./sim down quadcopter
```

## Ports

slot_base 50: quad1 sysid 50 (pilot 6260, fcb 8160, de_comm 62500,
de_mavlink 63500), ctl 62059. External server wss 9966, auth 19408.
