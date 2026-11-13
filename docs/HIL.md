# HIL — hardware-in-the-loop units

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

SPIKE output (SL-0017). A fleet unit can run its DroneEngage
processes (de_comm + modules) on a real Pi host. These are the
`de-connect` skill's `pi@droneengage.local` / `pi@dronecode.local`
hosts. SITL and the server side stay on the dev machine. Chaos tests
on x86 cannot cover hours-on-the-Pi behaviour: CPU, memory, thermal,
SD-card wear.

Scope rule: a HIL unit moves only the DE process set (`de_comm` +
catalog modules). These parts stay on the dev host: SITL, the
comm/authenticator servers, the GCS bridge and every engine-side
MAVLink client (ctl `pilot_port`, telemetry `aux_port`). Remote
operation is per unit. A mixed fleet is the point: one Pi copter plus
twenty local copters. The Pi can only carry a unit or two.

## Q1 — network path for fcb / pilot / s2s ports across hosts

Four links, four answers:

| link | endpoints | path |
|---|---|---|
| fcb | de_mavlink (Pi) -> SITL serial1 `tcp:7660+10k` (host) | direct TCP to the dev host's LAN IP |
| pilot/aux | engine ctl/telemetry (host) -> SITL serial0/2 | loopback, unchanged |
| s2s | module <-> its de_comm, UDP `port_base+10k` | loopback on the Pi, unchanged |
| server | de_comm (Pi) -> comm server `wss` (host) | server binds `0.0.0.0`; de_comm's `local_comm_server_ip` = host LAN IP |

**fcb.** The PoC below verifies this. The ArduPilot SITL TCP serial
listener binds `INADDR_ANY`: `_listen_sockaddr` is memset-zeroed and
`sin_addr` is never set (`libraries/AP_HAL_SITL/UARTDriver.cpp`).
`--serial1 tcp:<port>` with port > 1000 is a literal port. This is how
`fcb_port` 7660+10k already works. So remote de_mavlink connects to
`<host-LAN-IP>:<fcb_port>` with zero SITL-side changes. The only
engine change is the de_mavlink `wire:` in modules/catalog.yaml.
`{ip: "127.0.0.1"}` becomes `{ip: "{fcb_host}"}`, filled per unit:
`127.0.0.1` for local units, the dev host's routable address for
remote ones.

**pilot/aux.** `ctl.py:297` (`tcp:127.0.0.1:{pilot_port}`) and
`telemetry.py:30` (`aux_port`) are engine-side clients. SITL is local,
so nothing changes. The Pi sees no pilot traffic. Arm/mode/mission
commands ride the fcb link, like on a real vehicle.

**s2s.** A unit's modules and its de_comm run together on the Pi. So
the whole `s2s_udp_*` config block keeps `127.0.0.1` targets and the
same `port_base + 10*slot` math. The ports are only bound on the
remote host. de_comm's `s2s_udp_listening_ip` can even stay `0.0.0.0`
(the template default). It can also be tightened to `127.0.0.1`, since
no off-Pi module ever reaches it.

**server.** Today the generated `de_server.config` pins
`server_ip: 127.0.0.1` (simlib `gen_server`). The Pi cannot reach it.
Recommended: when a sim contains any remote unit, generate the server
with `server_ip: "0.0.0.0"`. The server's own default is `"::"`, which
also works. Set each remote unit's `local_comm_server_ip` / `auth_ip`
to the host LAN IP. Local units and the GCS bridge keep using
`127.0.0.1`. A `0.0.0.0` bind covers loopback too. The de_comm wss
client never verifies the cert. The boost `ssl_ctx_` keeps the default
`verify_none` (`andruav_comm_ws.hpp`). The curl auth path has its own
`auth_verify_ssl` flag. So the generated self-signed cert with
`CN=127.0.0.1` still works over the LAN address.

Rejected alternatives:

- **ssh `-L` forwards for the data plane** (Pi `localhost:62008` ->
  host `:62008`, same for fcb): keeps the server loopback-only. But
  it adds a persistent tunnel per remote host, *on top of* the
  control ssh channel we already need. It also hides exactly the LAN
  latency/loss the HIL unit exists to expose. Rejected for the data
  plane. ssh stays for the control plane only (Q2/Q3).
- **SITL on the Pi**: defeats the purpose. We want Pi-CPU coverage of
  DE code, not of ArduPilot. It also doubles the remote binaries to
  ship. `de_mavlink`'s UART path is already TCP. Nothing is gained.
- **VPN/overlay network**: solves a problem we do not have. The Pi
  hosts and the dev box share a LAN.

## Q2 — remote launch and pid tracking

Keep the existing launch-script mechanism, moved to the remote host:

1. `simlib.gen_unit` writes a remote twin of `var/run/<sim>/<unit>/`
   under a per-host remote root — e.g. `<run_dir>/<sim>/<unit>/` where
   `remote.run_dir` defaults to `~/de_hil_run`. Same `launch.<proc>.sh`
   shape (`cd; echo $$ > pids/<unit>.<proc>.pid; exec <bin> -c cfg -b
   local`), same journal/logs subdirs, same state files. Configs and
   scripts are generated locally exactly as today. They are pushed
   (`rsync`/`scp`) before launch.
2. `ctl.start_proc` gains a remote branch: `ssh <host>
   'setsid nohup bash <remote-script>'` with stdout tee'd to the remote
   `logs/<proc>.log`. The pidfile lands on the remote host. The
   manifest records the proc as `<unit>.<proc>` plus a `host` field.
3. **"Only kill our own pids" travels with the marker.** Today
   `pid_is_ours` greps local `/proc/<pid>/cmdline` for RUN_DIR /
   ENGINE_DIR. On the remote host the same check runs over ssh: read
   each remote pidfile, `grep -F <remote_run_dir> /proc/<pid>/cmdline`,
   signal only matches. The remote run dir embeds the sim name
   (`de_hil_run/<sim>`), so it is exactly as specific as RUN_DIR is
   locally. **Never `pkill`/`killall` by binary name.** On
   droneengage.local that would hit the production systemd units
   running the same de_comm/de_mavlink binaries. The marker-path check
   is the only thing that makes remote kills safe.
4. `down` gets a remote stray scan mirroring `_stray_pids`: list remote
   `/proc` cmdlines for the remote run dir, kill what no pidfile
   tracks. If ssh to a remote host fails during `down`, local teardown
   still runs. The orphan list is printed with a one-line manual
   cleanup command. Without it, remote de_comm processes live forever.
5. `kill`, `restart`, `freeze`/`unfreeze`, `reboot`, `reset` all route
   per-proc through the same ssh channel. Fault latency over ssh
   (~tens of ms on LAN) is acceptable. `dur_s` auto-restore keeps
   working, since the runner schedules it, not the remote side.

This is recommended over a remote agent daemon. Reasons: no new
protocol, no code on the Pi beyond the DE binaries we already deploy,
and the pid-invariant is a verbatim port of the local one. A
persistent agent would be the follow-up if ssh per-call latency shows
up in fault timing. A reused ssh-mux `ControlMaster` is enough for
that.

## Q3 — journal and log collection into var/results

The remote processes write the journals and proc logs, under the
remote run dir. Every engine reader expects local paths under
`var/run/<sim>/<unit>/`: `journal_mark/lines/match`, `log_match`,
`wait_ready` ready-regexes, `collect_results`, `./sim logs`.

Recommended: **continuous mirror into the local tree.** A
per-remote-host pull loop mirrors `<remote>/<sim>/<unit>/
{journal,logs}` into the real `var/run/<sim>/<unit>/{journal,logs}`.
The loop can be `rsync -a` every ~2 s, or `ssh cat` + append per file
with byte offsets. `journal_mark` already tracks offsets, so a
`tail`-style follower is cheap. Then *nothing else changes*: marks,
`wait_ready`, every journal/log assertion, `./sim logs <unit> <proc>`,
and `collect_results`' final `copytree` all work verbatim.
`collect_results` only gains a last blocking pull before the copy, so
the capture is complete. Mirror lag (~2 s worst case) is far inside
every `wait_*` timeout.

Rejected: per-poll `ssh cat` from the readers. Correct, but it turns
`wait_ready`'s 1 Hz loop and every `journal_match` into ssh round
trips. `collect_results` would also need a second remote-aware path.
Also rejected: sshfs/NFS mounts. They add host setup and have opaque
failure modes. The mirrored files also serve as the result archive we
want anyway.

## Q4 — clock sync for journal_time_delta

Journal timestamps are `de_journal` unix-ms. de_comm stamps them with
its **local clock on the host it runs on**. So once a unit goes
remote, `journal_time_delta`, `journal_order`, `max_latency` and
`clock_spread` compare clocks across machines.

Two layers, both cheap:

1. **NTP on the Pi is the required baseline.** Use `systemd-timesyncd`
   or chrony. Ideally both hosts use the same LAN time source. The dev
   box can serve `chrony` with `local stratum 10`. On a LAN this holds
   skew to ~1-10 ms. That is fine for every seconds-level assert.
2. **Measured offset at `sim up`, applied on ingest.** For each remote
   host, `sim up` samples `date +%s%3N` remotely and locally over the
   ssh session. It picks the min-RTT sample. It stores
   `clock_offset_ms` per host in `sim.json`. The journal readers
   subtract it when they merge remote journal lines. So
   `journal_time_delta`/`max_latency` stay honest without trusting NTP
   quality. The existing `clock_spread` assert (de_comm's
   `fleet_minus_unix_ms` CLOCK lines) becomes a live skew monitor for
   free. It measures each unit's clock against the server-stamped
   fleet clock. That clock is the same reference for local and remote
   units.

Rejected: trusting raw NTP with no correction. LAN NTP is good but
not bounded. A suspended Pi or dead timesyncd silently corrupts
timing asserts, and `max_latency` is millisecond-level. Also rejected:
re-stamping journal lines at collection time. It loses writer order;
the journal's value is that the writer stamps it. `server: {type:
fake}` clock faults (`clock_offset_ms`, `noclock`) are unaffected.
They skew the fleet clock the server stamps, not the writer's unix
clock.

## Q5 — how `cls: hil_copter` declares the remote host

`cls:` already resolves to `blocks/<name>` (scenario.py `CLS_BLOCK`
plus any blocks/ directory), so `cls: hil_copter` is just a block:

```yaml
# blocks/hil_copter/block.yaml
description: copter whose DE processes run on a remote Pi host
extends: copter          # modules + fc stay identical
remote:
  host: pi@droneengage.local        # ssh target (de-connect alias ok)
  run_dir: ~/de_hil_run             # remote root - the kill marker
  # remote binary locations; required per module unless the host's
  # default layout ($run_dir/bin/<module>) applies
  bin: {de_comm: ~/drone_engage/de_comm/bin/de_comm,
        de_mavlink: ~/drone_engage/de_mavlink/bin/de_ardupilot}
```

Semantics of `remote:`: the unit's *DE process set* (de_comm + every
catalog module) launches on `host`. Its `fc:` still means SITL — on
the dev host, unchanged. `procs_of` keeps listing sitl first; each
proc's start/kill just picks a host. Block-level `remote:` inherits
through `extends:` like `fc:`/`config:` do. A sim `units:` entry and a
scenario `fleet:` entry each take an optional `remote:` that overrides
the block's. (`{name}`/`{host}` expansion is not needed — it is a
literal spec.) Engine keys to widen: `load_block`'s allowed-key set,
`_expand_unit`'s entry keys, the sim `units:` key set, and
`de_scenario.schema.json`'s fleet properties — plus a `{fcb_host}`
wire context (Q1).

The host declaration lives in the block, not in `sim.local.env`. The
Pi is part of the fleet's identity (which unit is real hardware).
*Machine paths on the dev host* stay per-machine. Remote-side paths
go in `remote.bin`/`remote.run_dir` inside the block. A block named
`hil_copter` is already machine-flavoured, same as `sim.local.env`
naming hosts.

Rejected: a sim-wide `server:`/`remote:` section. Remote operation is
per unit — that is the whole design. Also rejected: putting the host
in the scenario `fleet:` entry only. Then `sims/*.yaml` fleets and
`--count` clones could not express it, and the block stops being
self-contained. Also rejected: naming the host via a `de-connect`
indirection file. The aliases are already `user@host` syntax — keep
one source of truth.

## Q6 — what breaks in sim hash / ensure reuse

- **Sim hash**: computed over `[server, home, units]`. A `remote:`
  dict inside the unit spec changes it, so `ensure` correctly does a
  fresh `up` when the host or remote paths change. Works as intended.
  Known gap (same as local today): the remote *binary contents* are
  invisible to the hash. Redeploying a new de_comm build to the Pi
  does not restart the sim. Acceptable; document it.
- **`ensure`'s aliveness check**: `read_pids` only inspects local
  `/proc` + local pidfiles. Remote procs must be probed over ssh
  (`kill -0` + the remote cmdline marker from Q2). Without it, every
  HIL sim reports "incomplete - restarting" and can never reuse — the
  exact opposite of what multi-hour Pi runs need. Manifests gain
  `host` per proc so `need <= alive` spans both.
- **`used_ports` / `ports_held` / `check_conflicts` / `--slot-base
  any`**: remote units bind their s2s ports on the *remote* host. The
  local bind probe must skip remote-owned ports. A dev's process on
  62000+10k must not block a Pi unit — and a Pi process must not block
  a local unit's fcb port math. Remote-side freeness is probed over
  ssh at `up` time. The *local* port set (SITL serials, server, bridge
  control port) is checked as today.
- **Journal/log reads**: Q3 — every journal reader keeps working off
  the mirror.
- **`write_file` steps** (`gpio_inputs.json`, `precland_scene.json`)
  write into `unit_dir`. For remote units they must `scp` into the
  remote module dir, same marker discipline.
- **`sim up` reachability precheck**: before launching, ssh-check that
  the Pi can reach the host's fcb port and that the host can ssh the
  Pi. Then a cabling/firewall mistake fails at `up`, not as a 180 s
  `wait_online` timeout.
- **`wait_online`**: server-side party listing — unaffected. The unit
  looks identical to the server wherever its processes run.
- **Teardown/`reset`/`land`/`reboot`**: route per-proc by host (Q2).
  `reset`'s "restart SITL + de_comm + modules together" spans hosts,
  so its wait-ready marks must cover the remote journal mirror.

## Proof-of-concept record

Target from the brief: de_comm (+ the module chain) on
droneengage.local against a host-side SITL fcb port, telemetry seen.

**Pi hosts unreachable from this machine.** `droneengage.local`
and `dronecode.local` resolve via mDNS on the dev host's LAN. But ARP/ping/ssh all fail
("No route to host"). Both Pis appear powered off. The Pi-side half
could not be executed and is **not claimed verified**.

Verified locally (2026-10-09, this machine) — the half that decides
whether the design works:

```
# SITL exactly like a sim unit would launch it (literal fcb port 8560):
$ arducopter -w -M quad -I 90 --sysid 90 --serial0 tcp:0 --serial1 tcp:8560 --serial2 tcp:2 &
bind port 6660 for SERIAL0 ... SERIAL1 on TCP port 8560 ...
$ ss -tln | grep 8560
LISTEN 0 5 0.0.0.0:8560 0.0.0.0:*        # INADDR_ANY - remote-reachable

# the remote de_mavlink's view: MAVLink over the LAN IP, not loopback
$ python3 -c 'from pymavlink import mavutil
c=mavutil.mavlink_connection("tcp:10.52.224.200:8560")
print(c.wait_heartbeat(20))'
HEARTBEAT sysid 90 ...                    # + SYS_STATUS telemetry

# the real binary with the config the Pi would get:
$ de_ardupilot -c cfg(fcb_connection_uri={ip:"10.52.224.200",port:8560,type:"tcp"}) -b local
TCP Client connected to 10.52.224.200:8560
 ** Mavlink Connected
```

So the fcb path (the only link that crosses hosts under the design)
works verbatim with `fcb_connection_uri.ip` = the dev host's LAN IP.
Remaining, unverified steps for when the Pi is up — documented as the
follow-up task's PoC checklist:

```
# on droneengage.local (no systemd changes, marker-scoped run dir):
mkdir -p ~/de_hil_run/poc/mav && cd ~/de_hil_run/poc/mav
# push a sim-generated de_comm/de_mavlink config pair, then:
setsid nohup ~/drone_engage/de_mavlink/bin/de_ardupilot -c de_mavlink.config.json -b de_mavlink.local &
setsid nohup ~/drone_engage/de_comm/bin/de_comm -c de_comm.config.json -b de_comm.local &
# expect: "Mavlink Connected" + "nav adapter hello" in the journal,
# unit online on the host's server (./sim ctl status / GCS roster)
```

## Notes

Design decision: **do HIL.** The gap is real: long-run
CPU/memory/thermal/SD behaviour on the real target. The cross-host
surface is small: one TCP fcb link + ssh control plane. Every piece
above has a local analogue already. Follow-up tasks for the human to
approve:

1. **"HIL: `remote:` unit spec + remote launch/pid layer"** — Touches:
   engine/simlib.py, engine/ctl.py, engine/launcher.py,
   modules/catalog.yaml, blocks/hil_copter/. Remote run-dir layout,
   ssh launch/kill with the marker-path `pid_is_ours` port, remote
   proc aliveness for `ensure`, `{fcb_host}` wire, server `0.0.0.0`
   generation for HIL sims, remote port probe.
2. **"HIL: remote journal/log mirror + clock offset"** — Touches:
   engine/simlib.py (journal readers), engine/launcher.py (offset
   probe at up), engine/runner.py + scenario_runner.py (write_file
   scp, results pull). Mirror into var/run, `clock_offset_ms` per
   remote host in sim.json, ingest-time correction.
3. **"HIL: scenario surface"** — Touches:
   engine/de_scenario.schema.json, engine/scenario.py (`remote:` on
   fleet entries), docs/SCENARIOS.md, docs/HIL.md examples.
4. **"HIL: Pi bring-up + recorded PoC"** — Touches: docs/HIL.md (PoC
   section), no code. Power on droneengage.local, run the checklist
   above against a `sim up`'d SITL, record de_comm-online +
   telemetry output in this file's PoC section, then soak a
   `hil_copter` unit for hours and record CPU/mem/thermal.

## host_serial — an off-host FC client on a real USB-serial wire (SL-0104)

The HIL units above move the unit's own DE processes. `fcb_link:
{kind: host_serial}` covers the opposite edge. The FC **client** lives
off-host — an Android phone running the Andruav app, or a Pi running
de_comm + de_mavlink. It talks MAVLink to the sim's SITL over a real
serial cable, exactly like a drone's FC port.

```yaml
fleet:
  - {id: c1, cls: copter, fc_client: external,
     fcb_link: {kind: host_serial, device: /dev/ttyUSB0, baud: 57600}}
```

- `fc_client: external` starts only SITL + the fcb proxy + the bridge
  for the unit — no local de_comm/de_mavlink. The external device is
  the client. The unit never joins the server roster, so `sim up`
  does not wait for it online.
- The bridge proc `fcb_tty` runs
  `socat <device>,raw,echo=0,ispeed=<baud>,ospeed=<baud>
  tcp:127.0.0.1:<fcb_proxy port>`.
  Before dialing, it arms the link baud as a proxy `rate` rule, so the
  wire cannot move bytes faster than a real UART. `fcb_cut`,
  `fcb_rate`, `fcb_corrupt` apply on the proxy as usual. `fcb_unplug`
  stops the bridge — the client's adapter goes silent, a cable pull
  at the far end. `dur_s` replugs it.
- `./sim doctor` requires the device to exist as a char device and be
  readable+writable. Join `dialout` (or add a udev rule) for rw
  access. `tests/fcb_link/host_serial_external.yaml` SKIPs cleanly
  when the adapter is absent.

### Wiring

Use two USB-serial adapters back to back. The **host** one is the
`device:` the bridge opens. The **client** one belongs to the device
under test (USB-OTG on a phone, a second adapter on a Pi):

| host adapter | client adapter | note |
|---|---|---|
| TX | RX | crossed |
| RX | TX | crossed |
| GND | GND | common ground is required |
| VCC | — | do **not** connect (hosts power their own ends) |

Use 3.3 V adapters (ArduPilot-level signalling). A 5 V adapter into a
3.3 V client pin can damage it. Prefer `/dev/serial/by-id/...` paths
over `/dev/ttyUSBn` on benches where several adapters move around.
The kernel never renumbers them.

## bt_spp — an off-host FC client over Bluetooth SPP (SL-0105)

`fcb_link: {kind: bt_spp, channel: N}` is the wireless twin of
`host_serial`. The pidfile'd `bt_spp` proc runs
`engine/bt_spp_bridge.py`: an RFCOMM SPP server on the host's
Bluetooth controller, piped to the fcb proxy, one client at a time.

```yaml
fleet:
  - {id: c1, cls: copter, fc_client: external,
     fcb_link: {kind: bt_spp, channel: 1}}
```

### Host setup

1. The controller must be **UP** — `bluetoothctl power on` (check
   `hciconfig hci0` for `UP RUNNING`). `./sim doctor` reports this.
2. Keep the host **discoverable** while pairing:
   `bluetoothctl discoverable on`.
3. SPP discovery needs the BlueZ **--compat** sdpd socket
   (`/var/run/sdp`). Modern BlueZ (>= 5) ships it off by default, so
   `sdptool add --channel=N SP` fails and a phone's SPP-UUID lookup
   finds nothing. Fix once per bench — `systemctl edit bluetooth`:
   ```
   [Service]
   ExecStart=
   ExecStart=/usr/libexec/bluetooth/bluetoothd --compat
   ```
   (path may be `/usr/lib/bluetooth/bluetoothd` on Debian-family
   systems — check the unit's existing ExecStart), then
   `systemctl daemon-reload && systemctl restart bluetooth`.
   `./sim doctor` reports the compat socket state. The bridge still
   serves the channel without it, so a client that dials the channel
   directly (no SDP lookup) works regardless.

### Pairing the client

- **Android / Andruav**: pair the phone with the host in the system
  Bluetooth settings (host appears by its adapter name). Then point
  the app's Bluetooth FC connection at the host. The SPP record
  resolves the channel. MAVLink heartbeats and mode should appear.
- **Pi / de_comm+de_mavlink**: `bluetoothctl pair <host-mac>` +
  `rfcomm bind /dev/rfcomm0 <host-mac> <channel>` gives the Pi a
  `/dev/rfcomm0` serial node. The app stack opens it like a UART.

`fcb_unplug` stops the bridge. The RFCOMM listener disappears — the
client's socket dies like a dropped link. `dur_s` replugs it on the
same channel. `fcb_rate` still paces the proxy when `baud` is
set on the link. Two units must not share a channel.

**Channel squatters.** RFCOMM delivers a dial to whichever listener
bound the channel first — our bridge binds *last*. A desktop app with
its own SPP listener on the same channel can steal the dial.
kdeconnectd is the known thief: it accepts the phone's dial, reads
MAVLink as garbage and resets. The app shows a connect/close loop
while `fcb_stats` stays at zero and the bridge log never prints
`client connected`. Diagnose with `./sim ctl bt_squatters <unit>` —
it names foreign RFCOMM holders — then kill the culprit. The
`bt_no_squatters` assertion (see
`tests/fcb_link/bt_spp_andruav.yaml`) makes a run FAIL with the
thief's name instead of a bare zero-bytes miss. The scenario runner
prints the same warning at bring-up.

## Android `mode:` — sim vs app, and the emulator (SL-0112)

An `android:` unit spec takes a `mode` key:

```yaml
units:
  - {block: android_copter, name: emu, count: 1,
     android: {serial: emulator-5554,
               remote_modules: [de_comm, de_mavlink], mode: app}}
```

- **`mode: sim`** (default) — today's behaviour, unchanged. The sim
  pushes generated configs into the app's `de_override/` dir and a
  `de_comm.local` into its private files. Then it STARTs each module
  service by intent.
- **`mode: app`** — the app owns its module configuration (its own
  settings: auth server, login, FCB link, unit name). Its
  NativeUnitController starts `de_comm`/`de_mavlink` itself. The sim
  pushes nothing. It removes any `de_override`/sim-written
  `de_comm.local` a previous `mode: sim` run left on the device
  (`clean_sim_overrides`). On `up` it launches the app's launcher
  activity instead of sending START intents (waits up to 40 s for the
  module processes). `ctl` kill/start still route to the module
  services as usual. `mode: app` requires `remote_modules` to be
  exactly `[de_comm, de_mavlink]`.

### `net:` — how the device reaches the host (SL-0118)

`android: {net: auto|adb_reverse|lan}` picks the transport per unit:

- **`auto`** (default) — emulator serials (`emulator-*`) use
  `adb reverse`. Anything else uses `host_ip()` LAN probing. A real
  phone on Wi-Fi/LAN behaves exactly as before.
- **`adb_reverse`** — always 127.0.0.1 + `adb reverse`, for **any**
  serial. The case: a **real phone on USB with Wi-Fi and mobile data
  OFF**. It has no LAN address at all. But `adb reverse` runs over
  the USB cable, so the phone still reaches host TCP ports as
  `127.0.0.1`.
- **`lan`** — always `host_ip()` LAN probing, even for an emulator.

`uses_adb_reverse(u)` in engine/android.py decides. `sim up` forces
`ANDROID_HOST_IP=127.0.0.1` and arms the reverse forwards only for
such units, so a LAN phone in the same fleet is untouched. Note:
`ANDROID_HOST_IP` is process-wide — do not mix `adb_reverse` and LAN
android units in one sim.

### Emulator networking — `adb reverse`

An emulator has no route to the dev host's LAN IP. `adb reverse
tcp:P tcp:P` makes the device's `127.0.0.1:P` reach the host's `P`
over the USB/debug channel. When a unit resolves to `adb reverse`
(`net: auto` and an `emulator-*` serial, or `net: adb_reverse`),
`sim up`:

1. forces `ANDROID_HOST_IP=127.0.0.1` **before** config generation —
   so `host_ip()` resolves to 127.0.0.1 everywhere (de_comm's
   `auth_ip`, de_mavlink's `fcb_connection_uri`, the comm server's
   `public_host`, the generated `server_ip` binds);
2. after generation, forwards the sim's comm-server port, the auth
   port (when `server: {auth: true}`) and the unit's fcb connect port
   (the fcb proxy port — the phone-side de_mavlink always goes through
   the proxy) with `adb reverse`;
3. on `sim down`, removes all reverse rules
   (`adb reverse --remove-all`).

**TCP only.** `adb reverse` carries TCP, never UDP. So the comm
server's UDP proxy range (40000–40999) and any WebRTC video path do
not reach the emulator (or a `net: adb_reverse` USB phone). The
phone-side modules' s2s UDP is unaffected — it is device-loopback
between the app's own processes. This is a known limitation, not a
bug.

**USB phone without network** — plug the phone in by USB, turn Wi-Fi
and mobile data off, and use `net: adb_reverse` on its unit:

```yaml
units:
  - {block: android_copter, name: phone, count: 1,
     android: {serial: <phone serial>, net: adb_reverse,
               remote_modules: [de_comm, de_mavlink]}}
```

The wiring is the same as the emulator: 127.0.0.1 + `adb reverse`,
TCP-only — no comm-server UDP proxy, no WebRTC video over the cable.
The alternative when UDP matters is **USB tethering + `net: lan`**.
The rndis interface gives the phone a real LAN path (carries UDP
too). But the tether subnet can change per plug-in, so it needs the
`host_ip()` probing (or a hand-set `ANDROID_DEVICE_IP` /
`ANDROID_HOST_IP`) rather than a pinned address.

`sims/andruav_emulator.yaml` (+ its runbook `sims/andruav_emulator.md`)
is the ready-made sim: one emulator `mode: app` unit on a pinned real
server instance (wss 62900 / auth 62910), with the one-time app preset
table.
