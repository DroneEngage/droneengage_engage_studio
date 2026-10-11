# Writing scenarios — `de.scenario/1`

This file is the complete authoring reference for a scenario: one YAML
file that describes a rehearsal — fleet, operation, scripted timeline,
faults, assertions. It is written so a human **or an LLM** can produce a
working file. Paste it whole into an LLM prompt together with what you
want rehearsed.

Where files live:

| directory | for |
|---|---|
| `scenarios/` | human-run scenarios — demos, experiments, your own work |
| `tests/` | automated cases — what CI/nightly runs; keep deterministic |
| `sims/` | reusable fleet definitions (legacy `sim:` tests reference these) |
| `blocks/` | unit types (`copter`, `plane`, `rover`, `comm_only`, `copter_camera`, …) — `./sim blocks` lists them |
| `worlds/` | seeded ground-truth worlds for `world:` |
| `packages/` | compiled operations referenced by `operation:` |
| `tools/` | `verify_wrap.js` (the verify gate CLI) |

Commands:

```bash
./sim new my_scenario                       # scaffold scenarios/my_scenario.yaml
./sim validate scenarios/my_scenario.yaml   # schema + refs, starts nothing
./sim run scenarios/my_scenario.yaml        # run it (owns the machine)
./sim run <file> --seed 7                   # seeded rerun
./sim expand <file> [--seed N]              # print the chaos: schedule, run nothing
./sim view <file>                           # run + hold with the 3D viewer
./sim suite tests/faults --seeds 1,2        # a directory x seeds
./sim suite tests/soak                      # soak suites are opt-in (excluded by default)
./sim list                                  # catalog everything runnable
```

## File shape

```yaml
# <name> - one paragraph: what the scenario rehearses and what PASS
# proves. REQUIRED - the first line must be a '#' comment.
version: 1
name: my_scenario              # required; [A-Za-z0-9_-] - becomes the run dir
seed: 42                       # optional; derives ports + fault/world RNG
realtime: false                # true gates timing asserts (SKIP unless SITL ~1x)
transport: fake                # fake | real | {type: real, link_proxy: true}
                               # (link faults need fake, or real+link_proxy;
                               # clock faults always need fake)
home: [lat, lng, alt, yaw]     # optional fleet home (defaults to SITL home)
slot_base: 40                  # optional; default is derived from the seed
world: ../../worlds/forest_a.yaml        # optional seeded world (stubs)
operation: ../../packages/x.deploy.json  # optional; what `do: deploy` sends
verify: {cli: ../../tools/verify_wrap.js}        # optional pre-deploy gate
settle_s: 20                   # quiet seconds after the last timeline step
teardown: land                 # land (default) | reset | none
render: {mode: viewer, headless: false}        # optional Mavlink3DMap
fleet:    [...]                # required
operator: {...}                # optional standing approve/confirm policy
timeline: [...]                # required (may be [])
assert:   [...]                # optional (may be [])
```

Unknown top-level keys are rejected — the file is validated against
`engine/de_scenario.schema.json` before anything starts.

**All paths are relative to the scenario file.** `scenarios/x.yaml` uses
`../packages/…`, `scenarios/examples/x.yaml` uses `../../packages/…`.

## `fleet:` — the units

```yaml
fleet:
  - {id: s1, cls: static}                          # comm-only ground node
  - {id: c1, cls: copter, at: {north_m: 0, east_m: 40}}
  - id: cam1
    cls: copter_camera         # or any blocks/<name>
    role: cam_scout            # optional role tag (deploy bindings)
    modules: [stub_ai_box]     # extra catalog modules (world stubs)
    params: {SIM_BATT_CAP_AH: 0.5}   # FC param overrides
    config: {de_comm: {sv_comm_lost_s: 20}}      # module config overlay
    home: [-35.36, 149.16, 584, 90]              # absolute; or `at` offsets
    base: [-35.36, 149.16]                       # statics: absolute position
    account: {user: u@x.com, code: secret}       # own login (auth sims)
    fcb_proxy: true            # route de_mavlink via the SL-0008 fcb
                             # proxy (enables the fcb_* faults/steps)
    faketime: true           # launch de_comm under libfaketime - needed
                             # by the clock_step fault (SL-0014)
```

`cls` is `copter | plane | rover | static | comm_only` or a `blocks/`
name. `static`/`comm_only` run no SITL. Unit `id` is the name used by
`unit:` in steps and assertions.

## `timeline:` — what happens when

Each step is exactly one of:

```yaml
- {t: 30, do: <step>, ...args}                    # at t seconds
- {t: -5, do: deploy}                             # negative t: before the clock
- {t: 60, fault: link_cut, unit: s1, dur_s: 30}   # inject a fault
- {on: {unit: s1, re: "comm line offline"},       # journal trigger
   do: fire, event: go.now, once: true}
- {on: {world: person_1, re: "react"},            # world-event trigger
   do: fire, event: world.reacted, unit: s1}
```

- `t` is seconds from scenario start (after fleet bring-up + verify).
- `on:` arms a watcher on `unit`'s de_comm journal; regex match
  groups substitute into the step's string args as `{M1}`..`{M9}`.
  `once` defaults to `true`. Watchers are polled between steps — a long
  blocking `wait` starves them; prefer a `wait_journal` with timeout.
- `on: {world: <entity_id>, re: ...}` (SL-0013) watches `truth.jsonl`'s
  `world_event` rows for that entity instead of a journal — e.g. the
  `react` row the truth tick logs when a `react:` trigger latches. The
  step's `unit:` is NOT defaulted from the trigger; name it yourself.
- The run ends `settle_s` seconds after the last `t`.

### `do:` steps

Operator actions (go through the GCS operator bot — the fleet sees one
consistent operator):

| `do:` | args | meaning |
|---|---|---|
| `deploy` / `prepare` | `operation`, `timeout`, `authorize: {unit: {valid_for_s, max_launches}}` | deploy the operation (1094 two-phase for specs); `authorize` sends each listed unit its launch grant after READY (P4F-07) |
| `start` | `run`, `in_ms`, `start: {role: mission}`, `targets: []`, `expect_fail: []` | START the deployed run |
| `abort` | `run`, `why` | ABORT the run |
| `authorize` | `unit`, `valid_for_s`, `max_launches`, `timeout` | send/replace a launch grant mid-run (1094) |
| `revoke_launch` | `unit`, `timeout` | void the unit's launch grant (1094) |
| `operator` | `act` | act as the operator (below) |

`operator` acts: `approve`/`deny` (`unit`, optional `req`, `kind`),
`confirm`/`dismiss` (`tid`), `fire` (`event`, `payload`, `unit`),
`activate`/`pause`/`resume`/`stop`/`clear` (mission control 1086),
`deploy`, `start`, `abort`.

All runner steps (every other name delegates to the test runner; `unit:`
takes a name, `all`, a comma list, or `block:<name>`):

| step | args |
|---|---|
| `arm` / `disarm` / `land` / `reset` / `journal` / `kick` / `link_clear` / `fc` | `unit` |
| `mode` | `unit`, `mode` (GUIDED, AUTO, RTL, …) |
| `takeoff` | `unit`, `alt` (10) — NAV_TAKEOFF, waits until ~80% of alt |
| `param` / `param_get` | `unit`, `name`, `value` — verified SITL param set / live read-back |
| `wait` | `seconds` |
| `wait_journal` | `unit`, `re`, `timeout` (60), `any: true` |
| `wait_log` | `unit`, `module`, `re`, `timeout` — module stdout, not journal |
| `wait_ready` / `wait_online` | `unit`/`timeout` |
| `start` / `stop` / `pause` / `resume` / `clear` | mission control (`mission` optional) — note `start`/`stop` through ctl when no op is deployed |
| `deploy` | `package`, `unit` (with `binding`) |
| `opstart` / `opabort` / `opstatus` / `opmsg` | 1094 fleet ops (`opmsg: {tg, ms, expect, secs}`) |
| `fire` | `unit`, `event`, `payload`, `until: {re, count, tries, every_s}` |
| `kill` / `restart` | `unit`, `proc` (`all`, `de_comm`, `de_mavlink`, `server`…) |
| `freeze` / `unfreeze` | `unit`, `proc` (`dur_s` self-resume on freeze) — SIGSTOP/SIGCONT: a hang, not a crash |
| `reboot` | `unit` — de_comm under a fresh boot id |
| `delay` / `link_cut` / `commline` | `unit`, `ms`/`seconds` — fake transport, or real+`link_proxy` (commline is in-band: any transport) |
| `loss` | `unit`, `percent` — **fake transport only** (message-level loss cannot ride a TLS byte relay) |
| `link` | `unit`, `set` — raw link-model sub-line(s), e.g. `set: "ge 0.05 0.2 0 90 stall"` or `set: ["loss_tx 0", "loss_rx 100"]`; fake transport, or real+`link_proxy` (loss* subs stay fake-only) |
| `write_file` | `unit`, `module`, `path`, `json`, `also: []` (`delay_ms`) |
| `udp_chunks` | `unit`, `module`, `drop_pct`, `reorder`, `dup_pct`, `seed` — knobs on the unit's chunk proxy (needs `udp_proxy:` on the unit) |
| `server_up` / `links` / `roster_peers` / `event` / `send` / `clock_offset` | server/console misc |

### `fault:` kinds

| fault | args |
|---|---|
| `link_cut` | `unit`, `dur_s` (30) — sever the server link; fake transport, or real+`link_proxy` (the proxy kills the session and accept-drops re-dials) |
| `link_stall` | `unit`, `dur_s` (30) — hold every byte in order for the window (a buffered blackout: the socket stays up, the unit's own watchdog notices); fake transport, or real+`link_proxy` |
| `link_profile` | `unit`, `profile` (a `link_model.PROFILES` name: `lte_edge`, `lte_stall`, `uplink_dead`, `dial_up`) and/or `cmds` (raw `link` sub-lines like `loss_rx 100`); `dur_s` clears the link. fake transport, or real+`link_proxy` — loss-knob subs/profiles (`loss*`, `uplink_dead`) stay fake-only and GE `drop` kills the session (a byte stream cannot skip bytes) |
| `loss` | `unit`, `percent` (50) — seeded packet loss, **fake only** |
| `delay` | `unit`, `ms` (500) — link delay; fake transport, or real+`link_proxy` |
| `gcs_disconnect` | `dur_s` (30) — drop the operator's GCS session |
| `server_down` | `proc` (all), `dur_s` — kill server procs |
| `kill` | `unit`, `proc` (all) — SIGKILL unit processes |
| `restart` | `unit`, `proc` (all) |
| `module_kill` | `unit`, `module` — kill one module proc |
| `freeze` | `unit`, `proc`\|`module`, `dur_s` — SIGSTOP the process (alive but silent); `dur_s` schedules the SIGCONT, without it the freeze holds until teardown |
| `param` | `unit`, `set: {NAME: value, ...}`, `dur_s` — generic FC param write: each param is read BEFORE it is set (verified PARAM_VALUE echo), `dur_s` restores the read-back values |
| `battery` | `unit`, `volts` (10.5), `cap_mah`, `res_ohm`, `dur_s`, `ok_volts`/`ok_cap_mah` — preset on `param`: SIM_BATT_VOLTAGE + optional BATT_CAPACITY / SIM_BATT_RES_OHM |
| `gps_off` | `unit`, `dur_s`, `fixtype` (0) — SIM_GPS1_FIXTYPE |
| `gps_jam` | `unit`, `dur_s`, `jam` (1) — SIM_GPS1_JAM: sats/lock/velocity go random |
| `gps_glitch` | `unit`, `dur_s`, `x`/`y`/`z` (40/0/0 m) — SIM_GPS1_GLTCH_* position offset |
| `gps_degrade` | `unit`, `dur_s`, `sats` (4), `noise` (10 m) — SIM_GPS1_NUMSATS + SIM_GPS1_NOISE |
| `motor_out` | `unit`, `dur_s`, `motors` (bitmask, 1), `mul` (0) — SIM_ENGINE_FAIL + SIM_ENGINE_MUL |
| `rc_fail` | `unit`, `dur_s`, `how` (1) — SIM_RC_FAIL: 1 no pulses, 2 throttle-950, 3 failsafe bit; airborne copter hits the FS_THR failsafe |
| `compass_fail` | `unit`, `dur_s`, `how` (1 no data / 2 frozen), `ofs_x`/`ofs_y`/`ofs_z`, `noise` — SIM_MAG1_FAIL + optional SIM_MAG1_OFS_* / SIM_MAG_RND |
| `vibration` | `unit`, `dur_s`, `freq` (60 Hz), `amp` (40), `noise` (20 m/s²) — SIM_VIB_FREQ_* + SIM_VIB_MOT_MAX + SIM_ACC1_RND; only bites with the motors on |
| `server_clock` | `offset_ms` (-3600000) — skew the fleet clock, fake only |
| `no_time_sync` | `unit`, `dur_s` — units stop carrying `ts`, fake only |
| `rc_override` | `unit`, `chan`/`channels`+`pwm` (3:1600), `dur_s` (15) |
| `mode` | `unit`, `mode` — force an FC mode |
| `fcb_cut` | `unit`, `dur_s` (30) — sever the de_mavlink↔SITL link via the fcb proxy; reconnects are accepted-then-dropped until expiry |
| `fcb_stall` | `unit`, `ms` or `dur_s` — hold the fcb link silent (sockets stay open) |
| `fcb_corrupt` | `unit`, `byte_drop_pct`/`bit_flip_pct`, `seed`, `dur_s` — seeded frame-aware noise on the fcb link |
| `cpu_starve` | `unit`, `proc`\|`module`, `duty_pct` (80), `cycle_ms` (1000), `dur_s` — SL-0014: a detached SIGSTOP/SIGCONT worker holds the proc stopped `duty_pct`% of each cycle (pid + sockets kept); `dur_s` ends the cycle, without it `cpu_unstarve` or teardown does |
| `journal_unwritable` | `unit`, `dur_s` — SL-0014: the unit's journal dir + files go read-only; open fds still write but every new journal sink open fails. `dur_s` restores, `sim down` always restores |
| `clock_step` | `unit`, `offset_s`, `dur_s` — SL-0014: step a `faketime: true` unit's de_comm clock by signed seconds via its `FAKETIME_TIMESTAMP_FILE` (per-process libfaketime, no adjtimex); `dur_s` steps back to `+0` |
| `udp_chunks` | `unit`, `module`, `drop_pct`, `reorder`, `dup_pct`, `seed`, `dur_s` — seeded chunk faults on the module↔de_comm s2s proxy (any transport) |

`dur_s` auto-restores where it applies: `link_cut` expires server-side;
`loss`/`delay` run `link_clear`; `kill`/`module_kill` restart the proc;
`server_down` relaunches the server; `freeze`/`battery`/`gps_off`/
`no_time_sync`/`server_clock`/`rc_override`/`gcs_disconnect` undo
themselves. `unit` accepts `all`.

## `chaos:` — a seeded fault schedule (optional)

A `chaos:` block draws a fault schedule at **load time** and inlines it
into the timeline as ordinary `fault:` steps — no RNG exists at run
time, so the same seed always produces the same schedule:

```yaml
chaos:
  faults:                          # the draw pool (required)
    - loss                         #   a bare kind
    - {kind: freeze, proc: de_comm}#   or {kind, ...step args}
    - {kind: mode, mode: RTL}      #   `mode` needs an explicit mode:
  units: [c1, c2, c3]              # fleet ids to hit (default: the fleet)
  every_s: [120, 300]              # seconds between draws (default [60,180])
  dur_s: [20, 60]                  # fault window for dur_s kinds (default [10,60])
  from_t: 120                      # start of the draw window (default 0)
  until_t: 1700                    # end (default: the timeline's last t)
```

Rules the loader enforces at `validate` time:

- Every kind must be one the **transport** serves — link/clock faults
  (`link_cut`, `loss`, `delay`, `server_clock`, `no_time_sync`) only on
  `transport: fake`.
- FC faults (`battery`, `gps_off`, `rc_override`, `mode`) need at least
  one FC unit in `units:`, and draw only from FC units.
- Overlapping windows of the **same unit+kind** are never drawn (a
  scripted `fault:` step's window counts too).
- A chaos scenario must carry **at least one run-wide invariant
  assertion** (`no_unexpected_restart`, `rss_growth`, `fd_growth`,
  `journal_size`, `recovers_after_fault`, `no_stuck_state`) — an
  unscripted run proves nothing without one.
- `module_kill`/`freeze` without a `module:`/`proc:` pick a random module
  of the drawn unit; `server_down`/`gcs_disconnect`/`server_clock` are
  fleet-scoped and get no `unit:`.

The expansion is printed when the run starts and written to
`var/results/<run>/expanded.yaml` — a valid de.scenario/1 file (chaos:
removed, steps inlined, paths absolutized). `./sim expand <file>
[--seed N]` prints the same file without running anything; a failing
seed's `expanded.yaml` reruns as a fixed regression
(`./sim run <out>/expanded.yaml`).

Long wall-clock soaks live in `tests/soak/` — the default `./sim suite`
/ `ci/run_suite.py` roots skip them; run one explicitly with
`./sim suite tests/soak` or `--roots tests/soak` (DE timers are
wall-clock: soaks always run at speed-up 1, e.g.
`tests/soak/soak_copters3.yaml`).

The `fcb_*` faults need `fcb_proxy: true` on the unit (or the sim) —
they ride a per-unit MAVLink-aware TCP proxy (`engine/fcb_proxy.py`)
inserted between `de_mavlink` and SITL's serial1 port:

    de_mavlink ──▶ proxy listen 7760+10k ──▶ SITL serial1 7660+10k
                        control 7860+10k (`./sim ctl fcb_*`, runner steps)

The proxy is a pidfile'd unit proc (`proc_alive: {proc: fcb_proxy}`
works; `sim down` kills it) and works on **any transport** — it is
ours, not the server's, so `transport: real` is fine. Cut exercises
de_mavlink's reconnect path (its log shows `TCP connection closed by
remote` then `TCP Client connected`); stall keeps the sockets open so
no close is logged — a frozen UART, not a cut; corrupt drops whole
frames or flips one bit inside a frame (frame-aware, so the TCP stream
never desyncs — a "dropped byte" kills its whole frame at
`pct × frame_len` rate, matching the same byte-error channel). The
same control port takes `do:`/`ctl` steps `fcb_drop` (one msgid only),
`fcb_inject` (raw MAVLink hex towards de_mavlink — the TASK-14 ADS-B
hook), `fcb_clear` and `fcb_stats`. The serial2 aux telemetry port and
the GCS pilot port bypass the proxy, so `telemetry_*`/`mission_items`
asserts keep working through a cut. Without `fcb_proxy` a unit is
unchanged — same hash, `de_mavlink` dials SITL directly.

`clock_step` needs the unit's fleet entry to carry `faketime: true` —
de_comm must launch under libfaketime (there is no retroactive
LD_PRELOAD), and validation fails when libfaketime is not installed
(`SIM_FAKETIME_LIB` in `sim.local.env` pins a non-standard `.so`).

`udp_chunks` is the local-transport fault, not a server-link one, so it
works on `transport: real` too. A `fault: udp_chunks` step makes the
loader wire a `udp_proxy_<module>` process between that module and
de_comm from sim-up (the unit `udp_proxy: [module]` opt-in is injected
automatically; you may also write it on the fleet/sim entry yourself to
get the proxy without a fault step — it then relays pass-through until
a `do: udp_chunks` or `./sim ctl udp_chunks` flips knobs). The proxy is
a pidfile-tracked unit proc (starts before its module, dies with the
sim, shows in `proc_alive`), and the same `drop_pct`/`reorder`/`dup_pct`
knobs are reachable live as `./sim ctl udp_chunks <unit> <module> ...`
(`clear`/`stats` included). See `engine/udp_proxy.py` for the chunk
semantics: drop is per-datagram, dup/reorder act on whole messages so a
multi-chunk transfer is never corrupted by reordering.

## `assert:` — what proves it

Each entry is one `{name: {args}}` mapping. Common ones:

| assertion | args | checks |
|---|---|---|
| `journal_count` | `unit`, `re`, `equals`/`at_least`/`at_most`, `total` | journal line count |
| `no_journal` | `unit`, `re` | the pattern never appears |
| `status_field` | `unit`, `path`, `equals`/`re`/bounds | a field of the latest 1087 (e.g. `launch.grant`) |
| `module_log_count` | same + `module` | a module's stdout log |
| `journal_order` | `unit`, `first`, `then`, `proc` | ordering; `proc` reads `logs/<proc>.log` instead of the journal (module stdout lines) |
| `journal_time_delta` | `u1`, `re1`, `u2`, `re2`, `max_s` | timing between lines |
| `proc_alive` / `proc_dead` | `unit`, `proc` | process state at end |
| `adsb_vehicle` | `icao` (hex/dec), `tid` (world entity), `at_least` (1), `max_m` (50), `log`/`unit` | the GCS bridge (or `log:`/unit proc stdout) saw ≥ `at_least` `ADSB_VEHICLE` frames with this ICAO, each within `max_m` of the entity's truth position at frame time |
| `link_offline` | `unit`, `at_least`, `min_s` | monitor saw it drop |
| `no_unexpected_restart` | `proc` (+`unit`, `tol_s`) | every pid change in `samples.jsonl` sits inside a `fault:` kill/restart/module_kill/server_down window (or a `do:` reset/server_up/land) |
| `rss_growth` | `proc` (+`unit`), `max_mb_per_h`, `warmup_s` | least-squares RSS slope after warm-up |
| `fd_growth` | `proc` (+`unit`), `max` | open-fd count growth over the run |
| `journal_size` | `unit`, `max_mb` | journal dir total size |
| `recovers_after_fault` | `within_s` | every timed fault restore is followed by link online + fresh telemetry |
| `no_stuck_state` | `unit`, `re`, `max_s` | a matching journal state line changes at least every `max_s` |
| `telemetry_value` | `unit`, `field`, bounds, `pick: min\|max` | tlog sample(s) |
| `fc_mode` | `unit`, `mode` | last FC mode |
| `mission_items` | `unit`, `equals`/`at_least`/`at_most` | FC mission count, serial0 read-back |
| `mission_pull` | `unit`, `secs`, bounds | the GCS 501→1027 plan read-back a WebClient draws; last chunk `i=999` required |
| `roster_value` | `unit`, `key`, `equals`/`…` | roster field |
| `roster_pos_near` | `unit`, `lat`, `lng`, `max_m` | last roster `pos` within `max_m` of a reference point (a held position never jumped) |
| `roster_field_absent` | `unit`, `field` | last roster frame omits the field entirely (compact-form rule); `roster_value` SKIPs a missing field, this one requires it |
| `operator_log_count` | `re`, bounds | the bot's decisions |
| `clock_spread` | `unit`, `max_ms` (100) or `min_ms` | spread of the units' latest CLOCK `fleet_minus_unix_ms`; `min_ms` proves a `clock_step` landed |
| `distance_to` | `unit`, `re`, `truth`, `min_m`/`max_m` | world truth geometry |
| `coverage_complete` / `pod_mean` / `pod_at` | `area`, `cell_m`, `min` | coverage grading |
| `detection_recall` / `detection_precision` | `src`, `match_m`, bounds | who was found |
| `time_to_detect` | `max_s` | first find latency |
| `clock_spread` / `mission_current` / `journal_consistent` / `final_state` / `fc_items_unchanged_without_cause` / `no_fc_command_to_static` / `max_latency` / `stub_deterministic` / `signals_in_range` / `visible_px` / `render_latency` / `no_sep_violation` / `detection_precision_render` | see `engine/checks.py` `DISPATCH` |

An unknown assertion or fault name fails **validation**, not the run —
`./sim validate` catches it before anything is launched.

**Run-wide invariants.** A 1 Hz sampler thread records every tracked
process's pid / RSS / open-fd count into `results/samples.jsonl` for the
whole run — one row per second per pidfile process, simulator-owned pids
only. The invariant assertions above read that file (plus
`links.jsonl`, the tlog and journals), so they work on chaos/soak runs
with no per-step scripting. A pid change is *expected* only inside a
fault window the runner recorded (`kill`, `restart`, `module_kill`,
`server_down`, or the `do:` lifecycle steps `reset`/`server_up`/`land`);
anything else fails `no_unexpected_restart` — a `do: kill` is *not* a
fault window (see `scenarios/examples/unexpected_restart.yaml` for the
negative case). `recovers_after_fault` needs `transport: fake` or
`link_proxy` for the link evidence to be meaningful; on non-FC units it
checks the link only.

## `operator:` — standing policy (optional)

```yaml
operator:
  approve: {launch: {after_s: 5}, action: {after_s: 3}}
  confirm: {person: {after_s: 10}}
```

The bot auto-approves/-confirms matching requests after the delay, and
re-sends its decisions (2–30 s backoff) while roster `wm` digests
disagree — like the WebClient's 1093 resend. A rule value of `none`
(`approve: {launch: none}`) is the explicit "the operator never
answers" posture: the request stays pending and times out live.

## `verify:` — the deploy gate (optional)

```yaml
verify: {cli: ../../tools/verify_wrap.js}                    # normal
verify: {cli: ..., expect_errors: 1}                          # gate must block
verify: {cli: ..., expect_rules: [GEO-003, LANE-001]}        # with these rules
```

Without `expect_*` a clean verify is required. With them, the run is a
GATE-PASS only when the verifier blocks with exactly those findings.

## `world:` — seeded truth (optional)

`worlds/*.yaml` define entities, motion, obstacles, transmitters,
clutter, environment (sun/wind/visibility/ambient temperature) and
camera/SDR/precland sensor models — entities may carry `temp_c` for the
thermal camera's contrast estimate (see `engine/world.py`'s header). A `world:` drives the `stub_*` modules (`modules:` entries on
fleet units), feeds `truth.jsonl`, and powers the truth/coverage/
detection assertions. `render:` (`viewer|cameras|physics|full`,
`headless`) adds the world stream + Mavlink3DMap.

An entity may react to the fleet (SL-0013):

```yaml
entities:
  - {id: person_1, cls: person, pos: [..],
     react: {near: {unit: cam1, dist_m: 30,          # | unit_cls: copter
                    then: {kind: flee, speed_mps: 1.2}}}}
```

`near.unit` names a fleet unit, `near.unit_cls` matches the fleet `cls`
(exactly one required); `dist_m` is horizontal ground distance.
`then.kind` is `flee` (walk straight away from the recorded threat
pose at `speed_mps`), `freeze` (hold the trigger position) or `seek`
(walk to the recorded pose). The reaction is evaluated by a SINGLE
authority — the runner's 1 Hz truth tick — which latches one knot per
entity into `var/run/<sim>/world_overrides.jsonl` and logs a
`world_event` row to `truth.jsonl`; the stubs and the world stream
read the same file, so truth, detections and the render feed agree.
A reactive world is excluded from golden byte-identity (the authority
is wall-clock — statistically reproducible only). See
`tests/p5/world_react_flee.yaml`.

### `cls: aircraft` — scripted ADS-B intruders

An entity with `cls: aircraft` is an airborne intruder the runner feeds
down the fcb proxy: once a second the scenario runner encodes the
aircraft's truth position into a MAVLink `ADSB_VEHICLE` frame and
`inject`s it through the control port of every `fcb_proxy: true` FC
unit, so de_mavlink parses and forwards it exactly like SITL's own
SIM_ADSB traffic — GCS-side, it is indistinguishable. See
`tests/p5/world_adsb_intruder.yaml` + `worlds/airspace_intruder.yaml`.

```yaml
- {id: intruder_1, cls: aircraft, icao: "0xAC0DE1", callsign: INTRUDER,
   alt_m: 150, speed_mps: 25, loop: true,
   path: [[-35.3615, 149.1652], [-35.3615, 149.1696], ...]}
```

- `icao` (required): ICAO-24 address — hex (`"0xAC0DE1"`, `"ac0de1"`,
  `"AC:0D:E1"`) or decimal; asserted as `ADSB_VEHICLE.ICAO_address`.
- `callsign`: ≤ 8 chars (default `ADRONE`), padded to the 9-byte field.
- `alt_m`: metres **above home**; the frame carries
  `altitude_type: 0` (baro) and `home_alt + alt_m` in the millimetre
  AMSL field. A bare `alt_m` with no `pos`/`path` parks the aircraft
  directly over the fleet's first home.
- `heading`, `hor_velocity`, `ver_velocity`: cm-deg / cm/s / cm/s
  values stamped into every frame (default heading follows the path).
- `emitter_type` (1 = light), `squawk` (1200), `tslc` (0): raw
  `ADSB_VEHICLE` fields; `flags` always mark position/altitude/
  heading/callsign/ICAO valid.
- `speed_mps` (10), `turn_rate`, `loop`, `attach`, `behaviour`: the
  usual world motion machinery applies, so an intruder can patrol,
  follow a person, or run a swarm behaviour like any other entity.

The feeder only runs when the scenario declares a `world:` **and** at
least one fleet unit sets `fcb_proxy: true` — without a proxy there is
no injection path. It needs no extra scenario keys; the `adsb_vehicle`
assertion reads the evidence the GCS bridge logs.

SITL can also spawn **random** ADS-B traffic with no world and no
code: `SIM_ADSB_COUNT` / `SIM_ADSB_RADIUS` / `SIM_ADSB_ALT` /
`SIM_ADSB_TX_FAIL` are ordinary SITL params, so they go through fleet
`params:` like any other override:

```yaml
fleet:
  - {id: c1, cls: copter, params: {SIM_ADSB_COUNT: 3,
       SIM_ADSB_RADIUS: 3000, SIM_ADSB_ALT: 200}}
```

Those intruders are random per run — use a world aircraft when the
assertion needs a known ICAO and position, SIM_ADSB when you want to
stress the GCS display path with uncontrolled traffic.

## Conventions & gotchas

- First line must be a `#` comment — the "what it proves" paragraph.
- Link faults refuse on bare `transport: real` — add `link_proxy: true`
  to run the byte-level ones against the real server (`link_cut`,
  `link_stall`, `link_profile`, `delay`, `kick`, `link` subs
  stall/ge/bw/jitter/delay/profile/cut/clear). `loss`/`loss_tx`/`loss_rx`,
  loss-knob profiles and all clock faults stay fake-only. Real transport
  exercises the actual comm server (and optional authenticator).
- `teardown: land` for anything that flies — a reused airborne fleet
  drains batteries and fails the next run's `battery_low`.
- SITL needs ~1–2 min for GPS/EKF; `arm` waits for it, but plan the
  timeline (`deploy` before `arm`, `mode: GUIDED` before a nav deploy).
- `on:` is a YAML 1.1 boolean key — write it bare, the loader fixes it.
- Each run writes `var/results/<utc>/<name>/`: journals, logs, tlog,
  operator.log, truth.jsonl, samples.jsonl, verify.json, summary.json,
  junit.xml.
  Exit 0 = every non-SKIP assertion passed.

## A complete example

```yaml
# demo_fault - one copter loses its link mid-mission: the square must
# still complete (missions are autonomous) and the link monitor must
# have seen the unit offline.
version: 1
name: demo_fault
transport: fake
operation: ../packages/copter_square.json
verify: {cli: ../tools/verify_wrap.js}
settle_s: 20
fleet:
  - {id: c1, cls: copter}
timeline:
  - {t: 0,  do: mode, unit: c1, mode: GUIDED}
  - {t: 2,  do: deploy}
  - {t: 6,  do: arm, unit: c1}
  - {t: 9,  do: start}
  - {t: 20, fault: link_cut, unit: c1, dur_s: 15}
  - {t: 25, do: wait_journal, unit: c1, re: "mission main completed",
     timeout: 300}
assert:
  - journal_count: {unit: c1, re: "mission main completed", equals: 1}
  - link_offline: {unit: c1, at_least: 1}
  - proc_alive: {unit: c1, proc: de_comm}
teardown: land
```

## Legacy `sim:`+`steps:` tests (tests/ flat files)

The older format — `{version: 1, sim: <sims file>, operation, steps:
[...], assert: [...], teardown}` — still runs through `./sim run`.
`steps` entries are the runner steps above (bare name or `{name: args}`);
`assert` is the same assertion set. New work should use the scenario
format (it carries its own fleet — no sims/ file needed).
