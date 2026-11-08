# relay_single — how to run

One quad copter on a spawned real `droneengage_server` **with the
authenticator** (`auth: true`, account `sim@sim.droneengage.com`) **plus
the standalone `droneengage_udp_proxy` relay**.
The comm server config carries `udp_relay: [R1]`, so the unit's 9008
pair opens land on the relay's UDP sockets.

Requirements:

- `DE_SERVER_DIR` (sim.local.env) points at a comm server build.
- `DE_UDP_RELAY_DIR` (sim.local.env) points at the `droneengage_udp_proxy`
  checkout with `cargo build --release` done
  (`target/release/droneengage-udp-proxy` must exist).

## Run

```bash
./sim up sims/relay_single.yaml
./sim ctl -s relay_single status
./sim run tests/udp_relay/relay_telemetry.yaml   # the proof
./sim down relay_single
```

## Ports

Every port is leased from the machine-wide allocator at `up` — `./sim
ports -s relay_single` lists what was actually bound (server wss +
control, relay wss, and the relay's UDP pair block, sized 2×the
de_mavlink units). `slot_base: 10` is identity only (copter1 sysid 10).
The committed relay window is `server.udp_relay.udp_port_range` in
`var/run/relay_single/sim.json`.
