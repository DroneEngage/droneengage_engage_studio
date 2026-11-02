# Port allocation management — design

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

Status: **design, not implemented** — except `./sim ports [name|--all]`,
which already exists in the lightweight form (no allocator): it lists
every port a generated sim declares (from `var/run/*/sim.json`) and probes
whether it is bound right now, including `udp_relay.relay.wss` and its whole
`udp_relay.relay.range` as an `n/N bound` row (SL-0088). The allocator described
below would replace that probe with real lease records. Task 2 of 3 in the
"real servers + mesh in the sim" work. Task 1 is the comm-server plugin
events (`droneengage_server/wiki/PluginEvents.md`). Task 3, the `servers:`
topology / mesh design, uses the request API defined here.

## Goal

The sim should reserve every port it needs (units, comm servers, storage,
authenticator, several comm servers per sim including comm servers attached
to units, proxies, control ports) through a single machine-wide allocator.
That allocator must work with any number of sims running at once, across
worktrees. It replaces three things:

- the fixed formulas (`SERVER_PORT + slot_base`, `port+1/+10/+11`,
  `base + 10*slot`)
- the free-slot search in `simlib._load_sim_free_slot` and
  `scenario._host_free_sim` / `_seed_slot_bases`
- the after-the-fact clash check in `simlib.check_conflicts`

Module: `engine/port_allocation_management.py` (the user's name for it),
called "the allocator" below.

---

## 1. Survey: every port the sim uses today

`k` is the unit's slot (`slot_base + unit index`), and `sb` is the sim's
`slot_base` (default `SIM_SLOT_BASE`, 10). Scenarios pick
`sb = 10 + 25*(seed % 9)` plus in-between fallbacks
(`scenario._seed_slot_base(s)`).

### 1.1 Unit ports (`simlib._expand_unit`)

| Port | Formula | Proto | Bound by | Read by |
|---|---|---|---|---|
| de_comm s2s | `62000 + 10k` (`DE_COMM_PORT_BASE`) | UDP | de_comm | every module config (`{comm_port}`), gen_unit |
| module s2s | `catalog[m].port_base + 10k` | UDP | each module | gen_unit, stubs (`stub_base.py`) |
| udp_proxy (SL-0010) | module port `+1` | UDP | `engine/udp_proxy.py` | gen_unit rewrites the module's `s2s_udp_target_port` |
| link_proxy data (SL-0009) | `61500 + 10k` | TCP | `engine/link_proxy.py` | de_comm (dials it instead of the server) |
| link_proxy ctrl | `61501 + 10k` | TCP | link_proxy | `./sim ctl`, scenario `link` lines |
| SITL serial0 (pilot) | `5760 + 10k` (SITL `-I k`) | TCP | SITL | ctl, telemetry, scenario_runner |
| SITL serial2 (aux) | `5762 + 10k` | TCP | SITL | telemetry |
| SITL serial1 (fcb) | `7660 + 10k` (explicit `--serial1 tcp:P`) | TCP | SITL | de_mavlink `{fcb_port}` |
| SITL serial5 (pose) | `5765 + 10k` (only with a `pose` module) | TCP | SITL | P5-08 stubs (`stub.pose_port`) |
| SITL RC in | `5501 + 10k` (SITL internal, from `-I`) | UDP | SITL | **not tracked** by `used_ports` |
| SITL sim in/out, irlock | `9003/9002/9005 + 10k` (SITL internal) | UDP | SITL, only for external physics / irlock | **not tracked**; verify which ones the internal model binds |
| fcb_proxy data (SL-0008) | `7760 + 10k` | TCP | `engine/fcb_proxy.py` | de_mavlink `{fcb_connect_port}` |
| fcb_proxy ctrl | `7860 + 10k` | TCP | fcb_proxy | ctl |
| MAVLink sysid | `= k` (an ID, not a port; must be 1..250) | — | SITL `--sysid`, `-I` | de_mavlink, stubs, GCS |

Catalog `port_base`s (`modules/catalog.yaml`): de_mavlink 63000,
de_camera 64000, de_tracker 65000, de_gpio 61000, de_yolo_ai 57000,
de_precland 56000, stub_ai_box 58000, stub_sdr 59000, stub_payload 60000,
stub_precland 55000, de_sound 54000.

### 1.2 Server-side ports (`simlib.load_server`, `gen_server`)

| Port | Formula | Bound by | Read by |
|---|---|---|---|
| comm server wss | `server.port`; scenarios write `62008 + sb` (`scenario._server_spec`, `sim_at`) | de_server (real) or fake_fleet_server | de_comm configs, GCS bridge, webclient deep link |
| control (real) | `control_port` or `port + 1` | `gcs_bridge.py` | `ctl.py` (`manifest.server.control_port`), launcher readiness wait |
| control (fake) | `port + 1` (hard-wired in `fake_fleet_server.py:624`) | fake_fleet_server | same |
| authenticator https | `port + 10` | de_auth | de_comm login, bridge |
| authenticator s2s | `port + 11` | de_auth | de_server `s2s_ws_target_port` |
| UDP proxy pairs | inside the relay's `udp_port_range` (`server: {udp_relay}` only; without it a unit gets `en:false`) | udp_relay | units, via the server |
| UDP proxy, fixed | the unit's de_mavlink `udp_proxy_fixed_port` becomes `socket2.port`, which the relay binds when `allow_fixed_port: true` and the port is inside its range; `socket1` stays random | de_server | GCS telemetry clients. **Not used by the sim today**; see §2.2 |
| storage server | **not run** (`enable_storage_server: False`) | — | — |
| external server | `port` (default 9966), `auth_port` (19408): remote, not ours | — | de_comm, bridge |
| external bridge control | `control_port`, default `62009 + sb` | gcs_bridge | ctl |

The comm server's repo config also carries `storage_server_port: 9000`
and the authenticator's `webadmin_port: 8089`. The sim disables both
today. Once storage is enabled they become allocations, not repo defaults.

### 1.3 Render / world ports (`simlib._render_spec`, only with `render:`)

Mavlink3DMap is optional, so none of these are leased unless the sim has
a `render:` block.

| Port | Formula | Note |
|---|---|---|
| world stream ws | `61000 + sb` | `world_stream.py`; probes default to 61010 |
| MAVLink UDP fan-out | `16450 + sb` | SITL `--serial4 udpclient:` |
| Mavlink3DMap bridge ws | `8801 + sb` | 8811 = Mavlink3DMap's own default when sb = 10 |
| viewer http | `8070 + sb` | 8080 when sb = 10 |
| frame-farm sink | `8090 + sb` | |

These use a stride of **1 per sim**, not 10 per unit.

### 1.4 Who checks for clashes today

- `simlib.used_ports(sim)`: the set of ports a sim binds. It misses SITL's
  RC-in / sim / irlock UDP ports and the server's ephemeral UDP proxies.
- `simlib.ports_held(ports)`: a test bind on 0.0.0.0, TCP and UDP. It
  exists because the module blocks 54000–60999 sit **inside Linux's
  ephemeral range** (`ip_local_port_range` = 32768–60999 on this host).
- `simlib.check_conflicts`: compares against live sims' manifests at `up`.
- `simlib._load_sim_free_slot` (`--slot-base any`): resolves the whole sim
  for up to 200 candidate bases and keeps the first with no clash.
- `scenario._host_free_sim`: the same idea, over seed-ordered bases.
- `launcher.port_busy`: connect probes for the server and control ports.

### 1.5 Consumers that hard-code ports

- `engine/test_world_stream.py:120-168`: asserts 61010 / 16460 / 8811 / 8080.
- `engine/test_record.py:29`: `"port": 62008` in a fixture.
- `engine/render_probe.py`, `engine/worldstream_probe.py`: docstring
  examples use 61010.
- `sims/quadcopter.yaml`, `quadcopters-4.yaml`, `rover.yaml`, `plane.yaml`:
  comments such as "de_mavlink s2s 63500, ctl 62059, sysid 50". The
  9966/19408 in these files are the external stack's ports, which is
  correct and stays.
- `blocks/_template_sensor/catalog.yaml`: a comment about 62000+10k.
- `tests/worlds/render_feed_smoke.yaml`: a comment already says "check
  sim.json". Good.
- `tools/rpi/*`: real hardware, out of scope.

Everything else (`ctl.py`, `gcs_bridge.py`, `telemetry.py`,
`scenario_runner.py`, `world.py`, stubs, launcher, studio via
`launcher.studio → load_sim`) already reads ports from the manifest
(`var/run/<sim>/sim.json`) or from the generated configs. **The formulas
live in one place, load time, and that is where the allocator plugs in.**

### 1.6 Weaknesses of the current scheme

1. **Families alias each other across sims.** Safety depends on the probe
   at bring-up. Examples:
   - de_gpio `61000+10k` at k=60 equals link_proxy `61500+10k` at k=10.
   - viewer `8070+sb` at sb=30 equals sink `8090+sb` at sb=10.
   - the server's `62008+sb`, `+1`, `+10`, `+11` fall inside the de_comm
     decade layout `62000+10k` for small bases.
2. **Upper bounds cut fleet size.** de_tracker `65000+10k` overflows at
   k ≥ 54, hence `_slot_fits`. The 9×25 slot blocks cap parallel scenario
   runs at nine.
3. **N servers per sim don't fit.** Fixed offsets from one `server.port`
   can't express a parent, a child, a server per unit, storage, and a
   proxy on each link.
4. **The ephemeral range** (54000–60999 module blocks) lets any OS socket
   steal a module port at random.
5. **Conflicts are found after the fact.** The free-slot search resolves
   the whole sim up to 200 times, and two sims coming up at once can pick
   the same base, because nothing is reserved between the check and the
   bind.
6. **Worktrees don't see each other's manifests** (`../_wt/*` have their
   own `var/run`). The only thing protecting them is the `ports_held` probe.

---

## 2. Pinned vs free ports

There are four kinds of port request. Each request says which kind it is.

| Kind | Meaning | Examples |
|---|---|---|
| **free** | Any port from the pool | comm server wss/ctrl/s2s, storage, auth, all proxies, control ports, module s2s ports |
| **block** | A run of `size` consecutive free ports at a fixed internal layout | SITL: serial*N* is `base + N`, so one 10-port block is base+0 (pilot), +2 (aux), +5 (pose). The rest of the block is unused but reserved |
| **prefer** | Try this exact port first, else free | Mavlink3DMap 8811/8080, so a viewer with default settings still works; the legacy formula ports during migration; "last time's port" stickiness |
| **pin** | Exactly this port, or fail | A port written explicitly in yaml (`server.port: 62100`, `render.web_port`); `--slot-base N` in compat mode |

### SITL is not really pinned

ArduPilot's `SITL_cmdline.cpp` has `--base-port` and `--rc-in-port`, and
both must come before `-I`. With them:

- the TCP serial ports come from an **allocated block**
  (`--base-port B` gives serial0 = B, serial2 = B+2, serial5 = B+5).
  serial1 is already passed explicitly (`--serial1 tcp:P`) and can be a
  separate free port, or `B+1`.
- RC-in comes from a free UDP port (`--rc-in-port R`), so it gets tracked
  for the first time.
- `-I` keeps choosing only the eeprom and log identity. It no longer
  decides the ports, and it doesn't need to be unique across sims:
  each unit has its own working dir (`run/<unit>/sitl`).
- Phase 2 must verify which of `sim-port-in/out` (9003/9002) and irlock
  (9005) the internal physics model binds. If any of them is bound, pass
  explicit allocated ports for it too (`--sim-port-in/out`).

### sysid is an ID, not a port

The MAVLink sysid stays separate from ports, with `alloc_id("sysid")` in
1..250, unique **per sim only** (decided). Several instances of the same
sim or scenario can then run side by side with identical sysids and
different ports. The allocator does no cross-sim sysid check, and sysid
leases live in the sim's manifest, not the machine registry. The default
keeps today's value (`sysid = slot`), so telemetry, scoring and
recordings don't change.

### Pools

- **Default free pool: 20000–32767**, outside Linux's ephemeral range,
  which the allocator reads from `/proc/sys/net/ipv4/ip_local_port_range`
  at startup and refuses to overlap. Configurable as `SIM_PORT_POOL` in
  `sim.local.env`.
- **Deny list:** ports developers' own stacks use, so the sim never takes
  them: 3000 (webclient), 9966, 19408, 19001, 9000, 8089, and 5760–5859
  (a developer's SITL at I=0..9). Configurable as `SIM_PORT_DENY`.
- **Legacy ranges** (54000–65535, 5760+, 7660+, 8070–8900, 16450+) are
  used only through `prefer` and `pin` during migration, then dropped.

### 2.1 Unix domain sockets for the module bus (fewer ports)

The DE module bus can already run over Unix datagram sockets
(`de_common/de_databus`: `CModule::init(..., use_unix_socket)`,
`unixDgramClient`, and de_comm's `unixDgramCommunicator`), and production
configs turn it on (`de_comm/de_mavlink/de_camera` `*.config.module.json`
all have `"use_unix_socket": true`). The sim turns it **off**
(`modules/templates/de_comm.json`, `simlib.py:1666/1803`) for one reason:

- **The socket path is machine-wide.** `getUnixSocketBaseDir()`
  (`de_common/helpers/helpers.cpp:379`) is hard-coded to `/run/de_comm`
  or `/tmp/de_comm`, and the broker is always `de_comm_broker.sock`. Two
  de_comm instances on one host fight over the same broker socket (the
  de_mavlink config even comments "SHOULD BE FALSE when Multiple DE_COMM
  modules are running on the same board").

**Recommendation: worth doing, and it pays off twice.**

1. **Fewer ports.** Module s2s ports are the largest group of ports
   (one per module per unit), and they're the ones sitting in the
   ephemeral range. Over Unix sockets they need **no port** at all.
   de_comm still binds its UDP listener (the broker always starts
   `cUDPClient`; `de_modules_manager.cpp:118`), so each unit keeps one
   `unit.<u>.de_comm` UDP lease, and modules never need one.
2. **The sim tests the production path.** Today the sim exercises UDP
   while production boards use Unix sockets.

**Required change (in `droneengage_common`, small, needed by production too):**
`getUnixSocketBaseDir()` honours a configured directory: a
`unix_socket_dir` config key passed into `CModule::init` and the broker,
with the `DE_UNIX_SOCKET_DIR` environment variable as fallback, and
today's `/run` → `/tmp` order when neither is set. Production is
unchanged by default. It also gets a supported way to run two de_comm
instances on one board, which the de_mavlink comment says is broken
today.

**Sim side:**
- Each unit gets its own socket directory, allocated like a lease
  (`unit.<u>.sockdir`, kind `path`) under
  `$XDG_RUNTIME_DIR/de_sim/s/<8-char hash of worktree+sim+unit>/`.
  It must be short because `sun_path` is limited to 108 bytes, and a
  worktree path plus `var/run/<sim>/<unit>/de_comm_<module_id>_<module_key>.sock`
  can go over that. The directory is removed on release.
- New sim key `bus: unix | udp`, per sim or per unit. Default `udp`
  until the de_common change lands and is pulled, then `unix`.
- **These stay on UDP:**
  - modules with a `udp_proxy:` (SL-0010 injects chunk faults on the
    UDP path, so a unit that asks for it keeps UDP for that module;
    the proxy could later learn Unix datagrams)
  - Python stubs and `droneengage_sound_module`, until they learn
    `AF_UNIX` datagrams (`stubs/stub_base.py` speaks UDP)
  - anything that goes across boards (not a sim case)
- **Mixed is fine.** The broker listens on both, and `CModule::init`
  already falls back to UDP for non-localhost targets.

### 2.2 Fixed UDP-proxy ports (the comm server binds them for the unit)

The answer to "will we run with fixed ports?" is yes, either by asking
for specific ports or by using fixed defaults. What happens today:
de_mavlink's `udp_proxy_fixed_port` (in its config, and **persisted in
the unit's local config file** by `cLocalConfigFile.addNumericField`,
`fcb_main.cpp:330`) is sent as `socket2.port`. The relay binds it only
when `allow_fixed_port: true` and the port is inside its range.

Design:
- New unit key `udp_proxy_port: <N> | auto`. `N` is a **pin** (that exact
  port, or fail at `up`), and `auto` is **free**. The lease is
  `unit.<u>.udp_telemetry` (UDP). It's in the sim's name, but the
  **comm server** binds it, so the lease records `bound_by: server.<s>`
  for diagnostics.
- `gen_unit` writes `udp_proxy_fixed_port` into de_mavlink's config
  **and** resets the persisted local value. A port left over in the
  unit's local file from a previous run would otherwise win and bypass
  the allocator. This has to be verified against how `cLocalConfigFile`
  orders config vs local values.
- `gen_server` keeps the relay's `allow_fixed_port: true` as soon as any
  unit routed to it has `udp_proxy_port`.
- Fixed defaults (e.g. "use 14550 like a real board"): written as a pin.
  Only one sim on the machine can hold such a port, and the second gets
  the normal clash error naming the sim that holds it. That's the
  correct behaviour for a literally fixed port.
- `socket1` stays random (the server picks it). If the server is ever
  given a fixed port range, that becomes a `server.<s>.udp_proxy_range`
  block.

---

## 3. The registry

### Location: machine-wide, not per worktree

`$XDG_RUNTIME_DIR/de_sim/ports.json` (on this host `/run/user/1000/…`,
a tmpfs cleared at logout, which suits leases), falling back to
`~/.cache/de_sim/ports.json`. It can be overridden with
`SIM_PORT_REGISTRY`.

It must **not** live under `var/`: every worktree has its own `var/`, and
the whole point is that main and `_wt/*` sims see each other.

Docker (decided: keep it simple, warn only, never require):
- The image sets `SIM_PORT_REGISTRY=/run/de_sim/ports.json`.
- `docker/README.md` documents one optional flag,
  `-v $XDG_RUNTIME_DIR/de_sim:/run/de_sim`, for anyone running the
  container next to host sims.
- Without the mount, the container has its own private registry, and the
  `ports_held` bind probe (which sees host ports, because of
  `--network host`) still stops real clashes. That's exactly today's
  protection. `entrypoint.sh` prints one line,
  `port registry not shared with the host (see docker/README.md)`,
  and carries on.

### Locking and writes

- `fcntl.flock(LOCK_EX)` on `ports.lock` next to the registry for each
  whole transaction (read → change → write).
- Writes go to `ports.json.tmp` followed by `os.replace`, so a crash
  never leaves a half-written file.
- A transaction covers a **whole sim's request**, all or nothing. This
  closes the race between two `./sim up`s choosing at the same moment.

### Lease record

```json
{
  "port": 20417, "proto": "tcp",            // tcp | udp | both
  "kind": "free",                           // free | block | prefer | pin
  "block": null,                            // {"base": 20410, "size": 10} for block members
  "sim": "copters3_faults",
  "worktree": "/home/you/de_code/droneengage_studio",
  "name": "server.hq.wss",                  // hierarchical; see section 4
  "state": "reserved",                      // reserved -> bound -> (released)
  "holder": {"pid": 81234, "start": 991223344},  // the ./sim up process during bring-up
  "created": "2026-10-21T10:02:11Z",
  "touched": "2026-10-21T10:02:40Z"
}
```

A lease belongs to the pair `(worktree, sim)`, not to a single process. A
sim is several processes, and the one that reserved the ports (`./sim up`)
exits as soon as bring-up finishes.

### Releasing and reclaiming

- `./sim down <sim>` and `./sim clean` release every lease of
  `(worktree, sim)`.
- A **stale-lease sweep** runs at the start of every transaction. A lease
  is reclaimed when:
  - its sim has no live pids (`read_pids` on `<worktree>/var/run/<sim>/pids`)
    **and** its holder pid is dead, or
  - it is still `reserved` (never marked `bound`) and its holder is dead,
    which covers a `./sim up` killed with -9 halfway through.
  A pid counts as alive only if `/proc/<pid>/stat` field 22 (start time)
  still matches what the lease recorded, so a reused pid isn't mistaken
  for the original.
- A lease whose worktree directory no longer exists is reclaimed too.
- `./sim ports gc` runs the sweep by hand. `./sim doctor` reports stale
  leases and any registry/host mismatch.

### The reserve → bind race

The registry stops **sims** colliding with each other. It can't stop a
**foreign** process (a browser, wsdd, a developer's server) taking a port
between the reservation and the bind. Mitigations, in order:

1. **Prefer ports nobody else picks by chance.** The free pool is outside
   the ephemeral range, so the kernel never hands those ports to
   short-lived outbound sockets.
2. **Test-bind at reservation time.** Inside the transaction, every port
   gets a `ports_held` bind (TCP and UDP for `both`). A port that fails
   is skipped and recorded as `foreign` for 60 s so the next pick skips
   it too.
3. **Bind soon.** The launcher starts processes right after `commit()`.
   The gap is the config-generation time (milliseconds to seconds).
4. **Recover when a bind fails.** The launcher already waits for each
   process to be ready. When a process dies with EADDRINUSE (or its
   readiness wait fails and `ports_held` shows a foreign owner), the
   launcher calls `reassign(name)`: a new port for that one lease, the
   affected configs regenerated (gen_unit / gen_server are pure functions
   of the manifest), and only the processes involved restarted. One retry,
   then fail with a message naming the foreign process (from `ss -lptn`).

Handing already-bound sockets to child processes (fd passing) was
considered and rejected: SITL, the C++ modules, and the Rust servers
can't adopt an inherited socket.

---

## 4. The API

```python
from port_allocation_management import Allocator, Request

alloc = Allocator()                       # registry path from SIM_PORT_REGISTRY / XDG

with alloc.transaction(sim="copters3", worktree=SIM_ROOT) as tx:
    tx.alloc("server.hq.wss", proto="tcp")
    tx.alloc("server.hq.ctrl", proto="tcp")
    tx.alloc("unit.c1.de_comm", proto="udp", prefer=62100)      # legacy formula as a hint
    tx.alloc_block("unit.c1.sitl", size=10, proto="tcp")        # -> base; members .p0.. .p9
    tx.alloc("unit.c1.sitl.rcin", proto="udp")
    tx.alloc("render.mav_ws", proto="tcp", prefer=8811)         # only when render: present
    tx.alloc_id("unit.c1.sysid", space="sysid", prefer=11)
    ports = tx.commit()                   # {name: port}; all-or-nothing

alloc.mark_bound("copters3", ["server.hq.wss", ...])   # launcher, after readiness
alloc.lookup("copters3", "server.hq.wss")              # -> 20417
alloc.leases(sim=None)                                 # -> [lease, ...] for ./sim ports
alloc.release("copters3")                              # ./sim down
alloc.reassign("copters3", "unit.c1.de_comm")          # bind-failure recovery
```

Properties:

- **Sticky by name.** If `(worktree, sim, name)` already holds a live
  lease, `alloc` returns it unchanged. This is what `ensure` and same-name
  restarts need (see Hash below).
- **Remembers the previous run.** On release, the allocator records
  `last[(worktree, sim, name)] = port` in `~/.cache/de_sim/last_ports.json`
  (capped and pruned by age, so it survives logout). The next
  `alloc` with no explicit `prefer` tries that port first, so a developer's
  bookmarked URLs and webclient deep links keep working between runs.
- **Generic requests.** The allocator knows nothing about units or
  servers. Names are dotted strings, with `<owner>.<id>.<role>` by
  convention:

| Prefix | Roles |
|---|---|
| `unit.<u>.` | `de_comm`, `<module>` (UDP bus only), `sockdir` (path, Unix bus), `udp_telemetry` (fixed UDP-proxy port, bound by the comm server), `udp_proxy.<module>`, `link_proxy.data/ctrl`, `fcb_proxy.data/ctrl`, `sitl` (block), `sitl.fcb`, `sitl.rcin`, `sysid` (id) |
| `server.<s>.` | `wss`, `ctrl`, `s2s_listen` (super-server listener), `udp_proxy_range` (block, if fixed UDP-proxy ports are ever enabled) |
| `auth.<s>.` | `https`, `s2s` |
| `storage.<s>.` | `wss` |
| `link.<a>-><b>.` | `proxy.data`, `proxy.ctrl` (a link_proxy on any connection, for task 3) |
| `bridge.<s>.` | `ctrl` |
| `render.` | `stream`, `mav`, `mav_ws`, `web`, `sink` |

  A server attached to a unit is simply `server.<u>.*`, with its owner
  recorded in the lease, and it's released with the sim like everything
  else.

### The manifest

`var/run/<sim>/sim.json` gets a flat `ports` map, which is the source of
truth:

```json
"ports": {"server.hq.wss": {"port": 20417, "proto": "tcp"}, ...}
```

The existing nested fields (`server.port`, `server.control_port`,
`units[i].ports.*`, `units[i].fc.pilot_port`, `render.*`) are **filled
from the map** at load time. All current readers (`ctl.py`,
`gcs_bridge.py`, `telemetry.py`, `scenario_runner.py`, `world.py`, stubs,
studio) therefore work without changes. New code reads the flat map
through `simlib.port(sim, name)`. `simlib.used_ports(sim)` becomes "the
values of `ports`", which fixes the untracked SITL UDP ports for free.

New CLI:

- `./sim ports [<sim>|--all]`: table of name, port, proto, state, sim,
  worktree.
- `./sim ports gc`: run the stale-lease sweep.
- `./sim show` prints the allocated ports, or "would allocate" with the
  preferred values when nothing is up.

Scenarios and tests that need a port literal use
`${port:server.hq.wss}` substitution, the same way `{fcb_port}` is
already filled in catalog `wire:`. Today no scenario under `scenarios/`
needs a literal.

### Hash

Today `sim_hash` covers `units`, which include their ports, so a
different allocation changes the fleet hash and `ensure` performs a
fresh `up`. Rules:

- Sticky-by-name means a live sim always gets the same ports back, so
  `ensure` against a live sim is stable.
- After `down` the leases are gone, so the next `up` is fresh anyway.
- Phase 2 and later: compute `sim_hash` over the **request** (the spec,
  with ports removed) and store the allocation beside it. A run that only
  moved ports then counts as "same fleet". This matters once ports stop
  being a function of `slot_base`.

### Seeds and reproducibility

Port numbers must never affect a run's outcome. Phase 1 needs one audit
for this: link_model / link_proxy / fake server seeding uses party IDs
and the run seed, not ports (to confirm), and recordings and
scoreboards must not compare ports. Once that holds, the seed→slot
mapping (`_seed_slot_base`) only picks `sysid` / `prefer` hints, and
`--seeds` runs in parallel need no slot blocks at all.

---

## 5. Migration

### Compatibility contract

At every phase, existing `sims/*.yaml`, scenarios, `--slot-base N` and
`--slot-base any` produce **the same ports and the same `sim.json` as
today** unless a run opts into the new behaviour. The final phase
removes that guarantee, and only after its acceptance criteria pass.

| Flag | Phase 0–1 | Phase 2+ |
|---|---|---|
| (none) | legacy formulas as **pin**: clash → error, as today | formulas as **prefer**: clash → a free port instead (behaviour change, announced) |
| `--slot-base N` | legacy formulas at N as **pin** | the same (explicit means explicit) |
| `--slot-base any` | allocator picks a base whose whole pinned set is free (replaces the 200-candidate search with one registry transaction plus probes) | allocator, all **free** with prefer hints; `any` becomes an alias for the default |
| `server.port: P` in yaml | pin | pin |

`slot_base` survives as a **hint**: the sysid base, the formula-prefer
base, and unit positions (`SPACING_M * (slot - slot_base)`). It stops
being the port source.

### How a `servers:` topology requests ports (generic; task 3 defines the topology)

The topology loader turns each node and connection into a list of
`Request(name, proto, kind, prefer=None, pin=None, size=None)` and passes
them all to **one** `tx`. Example of what a two-tier mesh plus storage
produces:

```
server.hq.wss  server.hq.ctrl  server.hq.s2s_listen
auth.hq.https  auth.hq.s2s
storage.db1.wss
server.edge1.wss
server.c3.wss                      # the comm server on unit c3
link.edge1->hq.proxy.data / .ctrl  # only when that connection carries a link_proxy
bridge.hq.ctrl
```

The topology code then wires configs by name: the child's
`s2s_relay_to_super_server_port = port("link.edge1->hq.proxy.data")` if
the connection is proxied, else `port("server.hq.s2s_listen")`. The
allocator never needs to know what a parent is.

---

## 6. Phased plan

### Phase 0: allocator and registry, legacy-pin mode

- `engine/port_allocation_management.py`: the registry, locking, leases,
  stale-lease sweep, `alloc` / `alloc_block` / `alloc_id` / `commit` /
  `lookup` / `release` / `reassign` / `mark_bound`. Unit tests with a
  temp registry, a fake `/proc`, and two processes racing.
- `load_sim` computes ports exactly as today, then **leases them as pins**
  in one transaction. `down` / `clean` release them. `check_conflicts`
  becomes "the transaction failed", with the same error text.
- `./sim ports`, `./sim ports gc`, a `doctor` check.

**Acceptance**
- For every `sims/*.yaml` and every scenario under `scenarios/` and
  `tests/`, the generated `sim.json` and `sim_hash` are byte-identical to
  before (a golden diff run in CI).
- `./sim up` of the same sim from main and from a `_wt/` worktree:
  the second is refused with the usual message. Today the second only
  fails later, if a probe happens to catch it.
- `kill -9` of `./sim up` during bring-up: the next `./sim up` reclaims
  the leases with no manual cleanup.
- `./sim ports --all` lists both worktrees' leases.

### Phase 1: free allocation for new and side ports, `--slot-base any` on the allocator

- Server control, auth https/s2s, link_proxy, fcb_proxy, udp_proxy, the
  bridge control port, and SITL RC-in / sim ports switch to **free**
  (pool 20000–32767), with legacy `prefer` hints.
- `--slot-base any` and `scenario._host_free_sim` are reimplemented as one
  transaction (no more resolving the sim 200 times).
- Port-independence audit (seeds, recordings, scoreboards).

**Acceptance**
- `./sim up copters3` plus `./sim up --shared --slot-base any --with faults
  copters3` (two names): both come up. Every proxy and control port
  comes from the pool, and the second sim's unit block is found in a
  single registry transaction. Without a flag the second sim still gets
  today's clash error (see Decisions).
- `run_scenario.py --seeds 1..12` in parallel (more than the 9 canonical
  slot blocks): every run comes up, with no "no free slot_base".
- A foreign listener on a preferred port is skipped, not reported as an
  error.

### Phase 2: units on the allocator

- de_comm and module ports become free, with legacy prefer hints.
- SITL launches with `--base-port <block> --rc-in-port <free>` before
  `-I`. `-I` stays the slot (eeprom identity only).
- `sim_hash` moves to request-only.
- The default (no flag) switches to formulas-as-prefer. Announced in
  `./sim --help` and the README.
- `_slot_fits` and its 65535 ceiling go away (ports no longer grow with
  the slot). The sysid ≤ 250 check stays.

**Acceptance**
- A sim with 60 de_tracker units comes up (fails today at k ≥ 54).
- Two `copters3` copies with different names and **the same** `slot_base:`
  in yaml, without `--shared`, both come up on distinct ports.
- `test_world_stream.py`, `test_record.py` and the render tests assert
  against `sim.json` / `simlib.port()` rather than literals, and pass with
  render on and off. With render off, no `render.*` lease exists (Mavlink3DMap
  stays optional).
- A full `tests/` suite run with the result set unchanged against the
  phase 0 baseline.

### Phase 2b: Unix-socket module bus (after the `droneengage_common` change)

- `droneengage_common`: a configurable socket directory
  (`unix_socket_dir` key, falling back to `DE_UNIX_SOCKET_DIR`, then
  today's `/run` → `/tmp`). Production defaults are unchanged.
- Sim: `bus: unix|udp` per sim or unit, per-unit `unit.<u>.sockdir`
  leases, and modules on the Unix bus take no port.
- `udp_proxy:` modules and the Python stubs stay on UDP.

**Acceptance**
- Two instances of the same scenario with `bus: unix` run in parallel,
  and each de_comm only sees its own modules (no shared
  `de_comm_broker.sock`).
- A unix-bus `copters3` leases **zero** module ports (`./sim ports`
  shows only de_comm, SITL, the server side and proxies).
- A mixed unit (de_mavlink on Unix, a stub on UDP, a `udp_proxy:` module
  on UDP) gets all its modules ready.
- On a production board with no new keys, de_comm and the modules still
  use `/run/de_comm` (no behaviour change).

### Phase 2c: fixed UDP-proxy ports

- `udp_proxy_port: N|auto` on units, the `unit.<u>.udp_telemetry`
  lease, de_mavlink's `udp_proxy_fixed_port` written and the persisted
  local value reset, and `allow_fixed_port: true` on the serving relay.

**Acceptance**
- A unit with `udp_proxy_port: auto`: a GCS telemetry client receives
  MAVLink on the leased port shown by `./sim ports`.
- Two parallel sims each with a `udp_proxy_port: 24550` pin: the second
  is refused at `up` with the name of the sim holding it.
- Re-running with a different port: the new port is used, not the one
  persisted from the previous run.

### Phase 3: remove the formulas

- Delete `SERVER_PORT + slot_base`, `port+1/+10/+11`, `*_PORT_BASE + 10*slot`,
  `catalog.port_base` (it becomes an optional `prefer` hint, or is
  removed), `_load_sim_free_slot`, `_seed_slot_bases`' port role,
  `fake_fleet_server`'s hard-wired `port + 1` (it takes `--ctrl-port`).
- Fix the stale port comments in `sims/*.yaml` and
  `blocks/_template_sensor`. Probe docstrings point at `./sim ports`.

**Acceptance**
- `grep` for the old constants in `engine/` finds nothing.
- The CI golden diff is retired, and a new golden covers the **request
  list** each sim produces.

### Phase 4: topology requests (with task 3)

- The `servers:` loader emits `Request`s as in section 5. gen_server
  becomes N config writers keyed by name.

**Acceptance**
- A parent + child + a server per unit + storage + one proxied mesh
  connection, run **twice in parallel** (two sim names) plus one more
  sim from a worktree, all at once. `./sim ports --all` shows no
  overlap, and `down` of any one releases only its own leases.

---

## Decisions from review (2026-10-21)

- **sysid:** unique per sim only, so several instances of the same
  scenario can run with different ports (§2).
- **Docker:** warn, don't require the registry mount. Keep it simple (§3).
- **Fixed UDP-proxy ports:** supported, either by asking for a specific
  port (pin) or `auto` (free), through `udp_proxy_port` (§2.2).
- **Unix sockets for the module bus:** adopt them to remove module ports.
  This needs a configurable socket directory in `droneengage_common`
  (§2.1, phase 2b).
- **Registry location:** leases live in `$XDG_RUNTIME_DIR/de_sim/ports.json`
  (cleared at logout, which is fine for leases). The "previous port"
  memory lives in `~/.cache/de_sim/last_ports.json`, so it survives
  logout and reboot. Both can be overridden: `SIM_PORT_REGISTRY` and
  `SIM_PORT_LAST`.
- **The default changes in phase 2,** from formulas-as-pin to
  formulas-as-prefer. Each moved port prints one line at `./sim up`:
  `moved unit.c1.de_comm 62110 -> 20431 (held by sim X)`.
- **Phase 1, two sims with no flag:** the second sim still gets today's
  clash error. Moving it automatically starts with the phase 2 change.
- **Unix-socket directory:** both the `unix_socket_dir` config key and the
  `DE_UNIX_SOCKET_DIR` environment variable. The config key wins, then
  the environment variable, then today's `/run/de_comm` → `/tmp/de_comm`.
  The sim writes the config key, so the directory shows up in the
  generated configs.

No open questions remain. The next step is turning the phases into tasks.
