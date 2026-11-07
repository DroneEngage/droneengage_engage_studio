# esp32_fake_fc

A fake ArduCopter (sysid 1, quad) on a classic ESP32 dev board, served on
**three links at once**:

| Link | Where | Use |
|---|---|---|
| USB-serial | UART0 behind the CP2102/CH340, 115200 8N1 | Andruav **USB (OTG)** FCB link, host check |
| UART2 pins | `RX=GPIO16`, `TX=GPIO17`, 115200 8N1 | wire to an RPi UART or a USB-serial adapter |
| Bluetooth SPP | Classic BT, name `DE-FakeFC` | Andruav **Bluetooth** FCB link, `rfcomm` from a host |

Telemetry goes out on every link; commands arriving on any link are acked
on all of them.

What it does:

- HEARTBEAT, SYS_STATUS, GPS_RAW_INT at 1 Hz; ATTITUDE (slow yaw spin) +
  GLOBAL_POSITION_INT at 5 Hz, parked at a fixed spot.
- `SET_MODE` and `COMMAND_LONG DO_SET_MODE` → `COMMAND_ACK ACCEPTED` for known
  copter modes (DENIED otherwise); the next HEARTBEAT carries the new mode and
  a STATUSTEXT `Fake FC: mode <NAME>` is sent.
- `COMPONENT_ARM_DISARM` → accepted, armed flag in HEARTBEAT.
- Minimal param list (4 params) and an empty mission so GCS loaders finish.
- On-board LED (GPIO2) toggles on every received MAVLink packet.

## Build + flash

From the repo root (`./sim doctor --esp32` checks the toolchain + board
first):

```bash
./sim esp32 upload          # build + flash + check_fake_fc.py, one command
```

Bluetooth links Bluedroid in, so the firmware does not fit the default
1.2MB app partition — the `fqbn` file next to the sketch pins
`PartitionScheme=min_spiffs` (1.9MB APP) and `./sim esp32` picks it up
automatically. Flashing a different partition scheme changes the board's
partition table.

By hand (headless, Arduino 1.8.x builder + esp32 core 3.0.x):

```bash
IDE=~/arduino-1.8.19        # extracted arduino-1.8.19-linux64.tar.xz
MAVLINK=../../../../droneengage_mavlink/c_library_v2
$IDE/arduino-builder -compile -hardware $IDE/hardware -hardware ~/.arduino15/packages \
  -tools $IDE/tools-builder -tools ~/.arduino15/packages -built-in-libraries $IDE/libraries \
  -fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs -build-path /tmp/fakefc_build \
  -prefs "build.defines=-I$(realpath $MAVLINK) -Wno-address-of-packed-member" esp32_fake_fc.ino
python3 ~/.arduino15/packages/esp32/tools/esptool_py/4.6/esptool.py --chip esp32 \
  --port /dev/ttyUSB0 --baud 921600 write_flash 0x0 /tmp/fakefc_build/esp32_fake_fc.ino.merged.bin
```

## Verify from the host

```bash
python3 check_fake_fc.py --device /dev/ttyUSB0
```

Expect 5 `PASS` lines: heartbeat, then GUIDED and STABILIZE each acked and
reflected in the heartbeat. The same check works on the other links: point
`--device` at a UART adapter on GPIO16/17, or at `/dev/rfcomm0` after
`sudo rfcomm bind 0 <ESP32_MAC> 1`.

## Use with Andruav — USB

1. Move the ESP32 from the PC to the phone's USB-C port via an OTG adapter.
2. Andruav → FC connection → **USB**, baud **115200** → CONNECT (grant the
   USB permission prompt).
3. The unit shows as an ArduCopter in STABILIZE; change mode to GUIDED from
   the WebClient — it is accepted and the mode display flips.

## Use with Andruav — Bluetooth

1. Pair the phone with `DE-FakeFC` (Android BT settings, or the app's BT
   picker).
2. Andruav → **FCB** tile → **Bluetooth** → pick `DE-FakeFC`.
3. Same behaviour as USB: ArduCopter in STABILIZE, modes/arm acked.

If the board resets/hangs in its bootloader when the app opens the port, it
is the DTR/RTS auto-reset circuit — reconnect, or hold nothing on GPIO0.
