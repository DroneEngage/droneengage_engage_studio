# sitl_udp — how to run

One quad copter SITL whose FCB MAVLink stream is pushed over UDP to a
remote listener at **10.1.72.253:14550** — the SL-0106
`fcb_link: {kind: udp}` path.

The unit runs SITL + `fcb_proxy` + the `fcb_udp` socat bridge only
(`fc_client: external` removes de_comm/de_mavlink). The FC client —
an Andruav phone, a remote `de_mavlink`, any UDP MAVLink listener —
lives off-host, listens on `peer`, and learns its peer from the first
inbound datagram. The bridge always plays the active side.

## Setup

- The listener at `10.1.72.253` must listen on UDP port `14550`
  (Andruav app's UDP server port by default).
- Repoint the listener without editing the file:
  `./sim up sims/sitl_udp.yaml --out 192.168.1.20` (a bare ip keeps
  the yaml's port; `ip:port` overrides both).
- Nothing else is required — UDP has no connect to refuse; an
  unreachable peer just sees zero bytes.

## Run

```bash
./sim up sims/sitl_udp.yaml
./sim up sims/sitl_udp.yaml --out 192.168.1.20:14550
./sim ctl -s sitl_udp status
```

The `fcb_udp` bridge pushes telemetry the moment the proxy is up;
heartbeats/param requests from the remote client flow back through
the same socket into SITL.

## Teardown

```bash
./sim down sitl_udp
```
