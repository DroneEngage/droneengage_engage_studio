# sitl_bt — how to run

One quad copter SITL whose FCB MAVLink stream is served over the
host's Bluetooth adapter — the SL-0105 `fcb_link: {kind: bt_spp}`
path.

The unit runs SITL + `fcb_proxy` + the `bt_spp` bridge
(`engine/bt_spp_bridge.py`) only (`fc_client: external` removes
de_comm/de_mavlink). The FC client — an Andruav phone, a Pi with an
rfcomm-bound `/dev/rfcommN`, any SPP MAVLink client — lives off-host,
pairs with this host, and dials RFCOMM channel **1** like a drone FC
port. One client at a time.

## Setup

- The host's Bluetooth controller must be UP, and discoverable while
  pairing: `bluetoothctl power on`, `bluetoothctl discoverable on`.
- SPP discovery (a phone's SDP lookup) needs the BlueZ `--compat`
  sdpd socket — see docs/HIL.md §bt_spp for the one-time systemd fix.
  A client that dials the channel directly works without it.
- `./sim doctor sims/sitl_bt.yaml` reports the adapter state, a
  bluetooth-capable python, and the compat socket.
- Move the channel by editing `channel:` (1–30); each bt_spp unit
  needs its own. `./sim ctl bt_squatters copter1` names foreign
  RFCOMM holders that would steal the dial (kdeconnectd's SPP
  listener is the known thief).

## Run

```bash
./sim up sims/sitl_bt.yaml
./sim ctl -s sitl_bt status
```

Pair the client with the host, then dial the channel — Andruav: FCB
tile → Bluetooth → pick this host; Pi: `rfcomm bind /dev/rfcomm0
<host-mac> 1`. Heartbeats flow once the client connects; `fcb_stats`
counts the bytes.

## Teardown

```bash
./sim down sitl_bt
```
