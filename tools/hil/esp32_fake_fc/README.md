# esp32_fake_fc

A fake ArduCopter (sysid 1, quad) on a classic ESP32 dev board's USB-serial
(CP2102/CH340 → UART0, 115200 8N1). Used to prove Andruav's **USB (OTG) FCB
link** without a real flight controller or two crossed USB-serial adapters:
plug the ESP32 into the phone's OTG port and it looks like a wired FC.

What it does:

- HEARTBEAT, SYS_STATUS, GPS_RAW_INT at 1 Hz; ATTITUDE (slow yaw spin) +
  GLOBAL_POSITION_INT at 5 Hz, parked at a fixed spot.
- `SET_MODE` and `COMMAND_LONG DO_SET_MODE` → `COMMAND_ACK ACCEPTED` for known
  copter modes (DENIED otherwise); the next HEARTBEAT carries the new mode and
  a STATUSTEXT `Fake FC: mode <NAME>` is sent.
- `COMPONENT_ARM_DISARM` → accepted, armed flag in HEARTBEAT.
- Minimal param list (4 params) and an empty mission so GCS loaders finish.
- On-board LED (GPIO2) toggles on every received MAVLink packet.

## Build + flash (headless, Arduino 1.8.x builder + esp32 core 3.0.x)

```bash
IDE=~/arduino-1.8.19        # extracted arduino-1.8.19-linux64.tar.xz
MAVLINK=../../../../droneengage_mavlink/c_library_v2
$IDE/arduino-builder -compile -hardware $IDE/hardware -hardware ~/.arduino15/packages \
  -tools $IDE/tools-builder -tools ~/.arduino15/packages -built-in-libraries $IDE/libraries \
  -fqbn esp32:esp32:esp32 -build-path /tmp/fakefc_build \
  -prefs "build.defines=-I$(realpath $MAVLINK) -Wno-address-of-packed-member" esp32_fake_fc.ino
python3 ~/.arduino15/packages/esp32/tools/esptool_py/4.6/esptool.py --chip esp32 \
  --port /dev/ttyUSB0 --baud 921600 write_flash 0x0 /tmp/fakefc_build/esp32_fake_fc.ino.merged.bin
```

## Verify from the host

```bash
python3 check_fake_fc.py --device /dev/ttyUSB0
```

Expect 5 `PASS` lines: heartbeat, then GUIDED and STABILIZE each acked and
reflected in the heartbeat.

## Use with Andruav

1. Move the ESP32 from the PC to the phone's USB-C port via an OTG adapter.
2. Andruav → FC connection → **USB**, baud **115200** → CONNECT (grant the
   USB permission prompt).
3. The unit shows as an ArduCopter in STABILIZE; change mode to GUIDED from
   the WebClient — it is accepted and the mode display flips.

If the board resets/hangs in its bootloader when the app opens the port, it
is the DTR/RTS auto-reset circuit — reconnect, or hold nothing on GPIO0.
