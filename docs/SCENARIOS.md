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
./sim view <file>                           # run + hold with the 3D viewer
./sim suite tests/faults --seeds 1,2        # a directory x seeds
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
transport: fake                # fake | real  (link/clock faults need fake)
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
| `loss` / `delay` / `link_cut` / `commline` | `unit`, `percent`/`ms`/`seconds` — **fake transport only** |
| `write_file` | `unit`, `module`, `path`, `json`, `also: []` (`delay_ms`) |
| `server_up` / `links` / `roster_peers` / `event` / `send` / `clock_offset` | server/console misc |

### `fault:` kinds

| fault | args |
|---|---|
| `link_cut` | `unit`, `dur_s` (30) — sever the server link, fake only |
| `loss` | `unit`, `percent` (50) — seeded packet loss, fake only |
| `delay` | `unit`, `ms` (500) — link delay, fake only |
| `gcs_disconnect` | `dur_s` (30) — drop the operator's GCS session |
| `server_down` | `proc` (all), `dur_s` — kill server procs |
| `kill` | `unit`, `proc` (all) — SIGKILL unit processes |
| `restart` | `unit`, `proc` (all) |
| `module_kill` | `unit`, `module` — kill one module proc |
| `freeze` | `unit`, `proc`\|`module`, `dur_s` — SIGSTOP the process (alive but silent); `dur_s` schedules the SIGCONT, without it the freeze holds until teardown |
| `battery` | `unit`, `volts` (10.5), `cap_mah`, `dur_s`, `ok_volts`/`ok_cap_mah` |
| `gps_off` | `unit`, `dur_s` — SIM_GPS1_FIXTYPE=0 |
| `server_clock` | `offset_ms` (-3600000) — skew the fleet clock, fake only |
| `no_time_sync` | `unit`, `dur_s` — units stop carrying `ts`, fake only |
| `rc_override` | `unit`, `chan`/`channels`+`pwm` (3:1600), `dur_s` (15) |
| `mode` | `unit`, `mode` — force an FC mode |
| `fcb_cut` | `unit`, `dur_s` (30) — sever the de_mavlink↔SITL link via the fcb proxy; reconnects are accepted-then-dropped until expiry |
| `fcb_stall` | `unit`, `ms` or `dur_s` — hold the fcb link silent (sockets stay open) |
| `fcb_corrupt` | `unit`, `byte_drop_pct`/`bit_flip_pct`, `seed`, `dur_s` — seeded frame-aware noise on the fcb link |

`dur_s` auto-restores where it applies. `unit` accepts `all`.

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
| `operator_log_count` | `re`, bounds | the bot's decisions |
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
negative case). `recovers_after_fault` needs `transport: fake` for the
link evidence to be meaningful; on non-FC units it checks the link
only.

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

## Conventions & gotchas

- First line must be a `#` comment — the "what it proves" paragraph.
- Link/clock faults refuse on `transport: real`. Real transport exercises
  the actual comm server (and optional authenticator).
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
