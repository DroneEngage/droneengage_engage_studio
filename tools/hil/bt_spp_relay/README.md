# bt_spp_relay — Bluetooth FC link against SITL

Makes this host look like a **Bluetooth telemetry radio** whose other end is
SITL. It runs a Bluetooth SPP (RFCOMM serial) server and passes the bytes to
a MAVLink TCP port (SITL `serial0`, default `127.0.0.1:6000`). Use it to test
a unit's Bluetooth FC link without real hardware.

Works with any client that can open an SPP link:
- **Andruav (Android)** — the native unit's FC link bridge.
- **RPi** — `rfcomm` + de_mavlink on a serial device.

## Requirements (host)
- BlueZ running, adapter powered: `bluetoothctl show`
- System Python with `dbus` and `gi` (python3-dbus, python3-gi). Use
  `/usr/bin/python3` (conda's Python has no `AF_BLUETOOTH`).
- A SITL listening on TCP, e.g. instance 24 → port 6000:
  `arducopter -S -I 24 --model quad --defaults .../copter.parm`
  (SITL serial0 takes **one** client — nothing else may hold that port).

## Run
```bash
/usr/bin/python3 -I tools/hil/bt_spp_relay/bt_spp_relay.py 6000           # -> 127.0.0.1:6000
/usr/bin/python3 -I tools/hil/bt_spp_relay/bt_spp_relay.py 5760 10.0.0.5  # other host
/usr/bin/python3 -I tools/hil/bt_spp_relay/bt_spp_relay.py 6000 --channel 3
```
It registers an SPP profile (UUID `00001101-…`, RFCOMM channel 22 by
default) with BlueZ and logs `BT connected: <device>` / `relaying to SITL`
per client. Ctrl-C stops it. Pair the client with this host first; if the
host is not paired yet, make it visible for a scan:
`bluetoothctl discoverable on` (3 min).

## Andruav phone
1. Home → **FCB** tile → **Bluetooth** → pick this host (paired devices are
   listed without a scan).
2. de_mavlink restarts, talks to the in-app bridge on `127.0.0.1:5760`, and
   the bridge connects over Bluetooth.
3. Check: relay logs `BT connected`; `adb logcat | grep -E "FcLinkBridge|ArduCopter"`
   shows `link up: bt` and `ArduCopter V4.x`.

## RPi
1. Pair once: `bluetoothctl` → `scan on`, `pair <HOST_MAC>`, `trust <HOST_MAC>`.
2. Bind a serial device to the relay's channel:
   `sudo rfcomm connect /dev/rfcomm0 <HOST_MAC> 22 &`
3. Point de_mavlink at it in `de_mavlink.local` (or the module json):
   ```json
   "fcb_connection_uri": { "type": "serial", "port": "/dev/rfcomm0", "baudrate": 57600 }
   ```
4. Check: relay logs `BT connected`; de_mavlink prints `Serial Connection Initializing`
   and the ArduCopter status text.
