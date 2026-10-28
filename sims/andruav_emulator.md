# andruav_emulator — how to run

The Andruav app on the Android **emulator** (adb serial `emulator-5554`)
as a native DroneEngage unit, `mode: app`: the app owns its module
configuration (its own settings — the sim pushes **no** de_override
files and writes no `de_comm.local` on the device), and its
NativeUnitController starts `de_comm`/`de_mavlink` itself. The sim
brings up **its own pinned server instance** (authenticator + comm
server on `62910`/`62900` — never the `./sim up de_servers` stack on
`19408`/`9966`), one ArduPilot SITL copter on the host, and wires the
emulator to both over `adb reverse` (the emulator cannot reach the
host's LAN IP; it reaches host TCP ports as `127.0.0.1`).

Ports are **pinned**, so the app preset never goes stale:
`server_port` 62900 → auth 62910, comm wss 62900, control 62901;
emu1 fcbpx 8160 (the app's FCB dial goes to the fcb proxy, which
fronts SITL's fcb port 8060). `--set serial=<other>` /
`--set server_port=<n>` override the defaults; the auth port is
always `server_port + 10` (62910 by default).

`./sim up andruav_emulator --help` prints this sim's description,
parameters, flags and examples without starting anything.

## One-time app preset (on the emulator)

Type this once into the app's Settings — the sim never touches it:

| Setting | Value |
|---|---|
| Native unit | ON |
| Unit name (party id) | `emu1` |
| Mission Server → host | `127.0.0.1` |
| Mission Server → port | `62910` (= `server_port` + 10) |
| Andruav cloud server | OFF |
| Require SSL verification | OFF |
| Login → email | `sim@sim.droneengage.com` |
| Login → access code | `sim1234` |
| FCB connection | TCP `127.0.0.1` port `8160` |

(`8160` is emu1's `fcbpx` port on the default slot — confirm with
`./sim show andruav_emulator`, which prints `fcbpx=<port>`; the app
dials the fcb **proxy**, not SITL's fcb port directly. If you `--set
server_port=` another value, the auth port is `server_port + 10`.)

## Run

```bash
./sim up andruav_emulator
```

The launcher probes the device, cleans any `de_override` files a
previous `mode: sim` sim left, `adb reverse`-forwards 62900/62910/8160,
then launches the app — wait for the app home card:

- **Native Unit Online**
- Com **LIVE**
- **FCB LINKED**

and `sim up` ends with `sim up.` — all units online + modules ready.

```bash
./sim down andruav_emulator    # stops the sim + removes the adb
                               # reverse rules (`adb reverse --remove-all`)
```

## Verify

```bash
adb -s emulator-5554 reverse --list   # tcp:62900 tcp:62910 tcp:8160
./sim ctl -s andruav_emulator status
```

`adb shell ls /sdcard/Android/data/ardupilot.andruav/files/` should
show **no** `de_override` dir.

## Real phone over USB, no network

A USB-attached phone with Wi-Fi and mobile data off works the same way
as the emulator - the phone reaches the host over `adb reverse` only:

```bash
./sim up andruav_emulator --set serial=<adb serial> --set net=adb_reverse
```

Use the same one-time app preset (host `127.0.0.1`, ports pinned). The
`net` parameter picks the transport: `auto` chooses `adb_reverse` for
emulator serials and `lan` for anything else - pin `adb_reverse`
explicitly for a networkless phone, `lan` for a phone on the LAN.

adb reverse is **TCP-only**: the comm server's UDP range (40000+,
telemetry relay and WebRTC video) cannot reach the device on this
path - the GCS-side UDP story stays emulator/LAN only.

## Troubleshooting

- **Unit online but no server traffic** — check `public_host` in
  `var/run/andruav_emulator/server/de_server.config` is `127.0.0.1`
  (the emulator reaches the comm server through adb reverse; a stale
  LAN IP there means `ANDROID_HOST_IP` leaked in from the environment).
- **FCB not linked** — `adb -s emulator-5554 reverse --list` must show
  `tcp:8160`; the app must dial the **fcbpx** port (8160), not SITL's
  fcb port (8060).
- **App starts but modules die** — a stale `mode: sim`
  `de_override`/`de_comm.local` on the device can fight the app's own
  settings; `sim up` cleans them, but check
  `/sdcard/Android/data/ardupilot.andruav/files/` is empty of
  `de_override` anyway.
- **`app's de_comm/de_mavlink did not come up in 40s`** — the app is
  not preset (table above) or 'Native unit' is off.
- **adb reverse is TCP-only** — the comm server's UDP proxy range
  (40000+) will not reach the emulator; GCS-side UDP (e.g. WebRTC
  video) is not available over this path. See docs/HIL.md.
