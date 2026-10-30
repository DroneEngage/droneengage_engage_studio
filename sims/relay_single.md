# relay_single — how to run

One quad copter on a spawned real `droneengage_server` (local mode, no
authenticator) **plus the standalone `droneengage_udp_proxy` relay**.
The comm server config carries `udp_relay: [R1]`, so the unit's 9008
pair opens land on the relay's UDP sockets instead of the built-in
proxy.

Requirements:

- `DE_SERVER_DIR` (sim.local.env) points at a comm server build with the
  GL-0063+ relay client — until the chain merges that is the
  `_wt/droneengage_server-GL-0063` worktree.
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

slot_base 10: copter1 sysid 10 (pilot 5860, fcb 7760, de_comm 62100,
de_mavlink 63100). Server wss 62008, control 62009, relay wss 62020.
The relay's UDP pair window is `40000+40*slot_base` (40400+, sized
2×the de_mavlink units) — see `server.udp_relay.udp_port_range` in
`var/run/relay_single/sim.json`.
