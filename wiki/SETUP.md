# Setup and requirements

[← Wiki index](README.md) · [← Studio README](../README.md)

## Quickstart (Docker)

One image with SITL, the DE binaries, the sim engine and the
Mavlink3DMap viewer baked in - the fastest way to see a scenario run:

```bash
docker build -f docker/Dockerfile -t de-sim .
docker run -it --network host de-sim          # runs the canopy-search demo
```

The run brings the fleet up, uploads a lawnmower mission over the
forest-canopy world (the classic `Upload_DE_Mission` path), flies the
sweep and then parks with the viewer live - open the printed
`render viewer:` URL in a browser, and `Ctrl-C` when done to see the
graded assertions. `--network host` is required (the sim binds its
127.0.0.1 slots; Linux only). See [docker/README.md](../docker/README.md)
for details, other commands, and what stays host-side or gated on
upstream mirrors (supervised op deploys need the mission-supervisor
code the public de_comm/de_mavlink mirrors do not publish yet).

## Setup (per machine)

```bash
cp sim.local.env.example sim.local.env    # gitignored: this machine's paths
```

Edit `sim.local.env` for your machine: `DE_CODE` (root of the
DroneEngage checkouts), `ARDUPILOT_DIR`, and per-repo or per-binary
overrides (`DE_MAVLINK_DIR`, `DE_CAMERA_BIN`, `NODE`, ...). An environment
variable with the same name wins over the file.

Then check the machine:

```bash
./sim doctor                          # one OK/MISSING/OPTIONAL line per item,
                                      # each MISS with a fix line; exit 1 on gaps
./sim doctor sims/quadcopter.yaml     # same, scoped to what that sim needs
./sim doctor --json                   # machine-readable [{item,status,detail,fix}]
./sim doctor --esp32                  # also require + probe the optional ESP32 HIL gear
```

Bluetooth FC link testing: `tools/hil/bt_spp_relay/` makes this host a
Bluetooth telemetry radio wired to SITL, for the Andruav phone or an RPi
(`/usr/bin/python3 -I tools/hil/bt_spp_relay/bt_spp_relay.py 6000`; see its README).

Optional ESP32 HIL firmwares (`tools/hil/<name>/`, e.g. `esp32_fake_fc` - a
fake ArduCopter that proves a phone's USB-OTG FCB link):

```bash
./sim esp32 list                      # firmwares under tools/hil/
./sim esp32 upload                    # build + flash esp32_fake_fc + run its host check
./sim esp32 check --port /dev/ttyUSB0 # only the host check against the flashed board
```

The machine also needs:

- ArduPilot SITL built (`./waf configure --board sitl && ./waf copter plane rover`).
- `de_comm` and `de_ardupilot` built (`bin/` in their repos). Other modules
  are only needed when a block uses them.
- `cargo build --release` done in `droneengage_server` and
  `droneengage_authenticator` (plus `droneengage_udp_proxy` for
  `udp_relay:` sims) - a Rust toolchain, e.g. via rustup.
- `pip install websockets pymavlink pyyaml`, and `openssl` (fake server only).

No root needed. Servers and modules use 127.0.0.1. de_comm's s2s UDP port
and SITL's TCP ports listen on all interfaces.
