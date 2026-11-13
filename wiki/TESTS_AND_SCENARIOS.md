# Tests, scenarios and the de.scenario/1 format

[← Wiki index](README.md) · [← Studio README](../README.md)

## Tests - the automated library

A `run`/`view`/`suite` of a file with `steps:` is **exclusive**: it downs
every other sim first - that is the CI posture (a test needs a clean
machine), and the interactive commands stay additive (see
[Running several sims in parallel](PARALLEL_SIMS_AND_PORTS.md#running-several-sims-in-parallel)). `--shared` / `SIM_SHARED=1` keeps a
run additive on a shared box.

`tests/` holds everything automation runs: the `de.scenario/1` library in
its feature subdirectories (`faults/`, `fleet_bus/`, `missions/`, `sar/`, `worlds/`, `verify/`, ...).
The nightly suite is `./sim suite tests` (a.k.a. `ci/run_suite.py --roots
tests`); `ci/check_library.py` validates every file on push. For a fast
first pass, `./sim suite --tier smoke` runs only the files marked
`tier: smoke` - one probe per subsystem (bring-up, auth, faults, mission
supervisor, events, modules, world detect, verify gate). Smoke is a
selection heuristic, not evidence - the unmarked files stay the
regression tier.

See the docstring in `engine/runner.py` for every `do:` step and
`docs/SCENARIOS.md` for the format. In short:

- **`do:` steps:** `deploy`, `arm`, `start`, `stop`, `clear`, `fire`, `mode`,
  `land`, `reset`, `wait`, `wait_journal`, `wait_log`, `wait_online`,
  `wait_ready`,
  `kill` / `restart` / `freeze` / `unfreeze` (also `unit: server`;
  `freeze` is SIGSTOP - a hang, not a crash), `cpu_starve` /
  `cpu_unstarve` (a SIGSTOP/SIGCONT duty cycle), `journal_unwritable` /
  `journal_writable` (unit journal dir read-only, auto-restored on
  `dur_s` and always by `sim down`), `clock_step` (per-process clock skew
  on a `faketime: true` unit's de_comm via libfaketime's
  `FAKETIME_TIMESTAMP_FILE`), `reboot` (de_comm under a new
  boot id), `server_up`, `loss` (fake server only) / `delay` /
  `link_cut` / `link_clear`
  (fake server, or real with `link_proxy: true`), `link` (`{unit, set}` -
  raw link-model sub-lines:
  `stall`, `ge`, `bw`, `jitter`, `loss_tx`, `loss_rx`, `profile`; loss*
  is fake-only),
  `commline`, `send`, `udp_chunks` (`{unit, module,
  drop_pct, reorder, dup_pct, seed}` - knobs on the unit's
  module↔de_comm chunk proxy), `write_file` (`{unit, module,
  path, json}` - a JSON file into `var/run/<unit>/<module>/`, for module file
  seams like de_rpi_gpio's `gpio_inputs.json` or de_precland's
  `precland_scene.json`; `also: [{path, json, delay_ms?}]`
  writes extra files inside the same call — an entry with `delay_ms`
  sleeps N ms before its write, keeping the order);
- **`.deop` operations (TASK-23):** `operation:` may name a
  de.operation/2 source (`ops/x.deop`) - the runner compiles it
  on demand with tools/compile_op.js into
  `var/build/packages/<key>/` (source+compiler+ctx hash; edits
  recompile, unchanged runs reuse). Its `<x>.ctx.json` sidecar
  supplies the binding + capability registry; without one the
  binding derives from the fleet (`role:` tags, then fleet order)
  and the dir's single `*.unit.caps.json` feeds --caps;
- **fleet ops (P4A-09):** when `operation:` names a deploy spec (a top-level
  `roles` map - see `packages/p4a05.deploy.json`), `deploy` runs the
  two-phase DE_OP_CONTROL (1094) PREPARE per role, and the run goes up
  with `opstart` / down with `opabort` (`opstatus`, `opmsg` for probes).
  A fleet package refuses the plain `start`;
- **step tokens** (`send`/`opmsg`): `{RUN}`, `{OP_RUN}`, `{PKG:<file>}`,
  `{HASH:<file>}` (paths resolve as given, under `packages/` and the
  on-demand build's `var/build/packages/by-name/`; test fixtures use
  the explicit `tests/packages/<file>` path), `{WM_TID:<unit>}`;
- **assertions:** `journal_count`, `module_log_count` (same bounds over
  `logs/<module>.log` - module stdout, for facts that never reach the
  journal like de_precland's `EVENT_OUT` lines), `journal_order`,
  `journal_time_delta`, `mission_current`, `clock_spread`;
- **teardown:** `land` (default) | `reset` | `none`.

`unit:` takes a unit name, its 1-based position, `block:<name>`, a comma
list, or `all`.

`land` brings each vehicle type down its own way: copter LAND, rover
HOLD + disarm. A plane cannot land in place, so it gets a `reset`: a power-cycle
of SITL (fresh, at home), de_comm and its modules.

Results land in `var/results/<timestamp>-<test>/`: journals, logs, the test file
and the sim manifest.

## Scenarios - human-run

`scenarios/` is the person's side of the library: `demos/` (things to
watch, like the Mavlink3DMap canopy viewer), `examples/` (small commented
starters), and wherever you drop your own files. `./sim new <name>`
scaffolds one; `./sim validate <file>` checks it without starting
anything. **docs/SCENARIOS.md is the authoring reference** - paste it to
an LLM and it can write a scenario for you.

## The scenario format (de.scenario/1)

One versioned file rehearses a whole operation on the desk - fleet,
operation, operator policy, scripted timeline, faults, assertions:

```
./sim run tests/faults/link_cut.yaml                    # one scenario - the
                                                        # file's `seed:` applies
./sim run tests/faults/link_cut.yaml --seed 7           # a seed run (overrides
                                                        # the file's `seed:`)
./sim suite tests/faults --seeds 1,2                    # suite x seeds
./sim view scenarios/demos/world_canopy_viewer.yaml     # sim + Mavlink3DMap
                                                        # viewer, held till Ctrl-C
```

(`run_scenario.py` still works - a shim over the same engine.)

Format (strict - unknown keys fail against
`engine/de_scenario.schema.json`):

- `version`, `name`, `seed`, `realtime`, `transport` (`fake` | `real`),
  `home`, `slot_base`, `world` (P5-08 `worlds/*.yaml`: seeded ground
  truth - entities, motion, attachments, transmitters, clutter,
  camera and SDR models; P5-10 adds `obstacles` - canopy crowns and
  buildings as vertical prisms the stubs attenuate/occlude through
  (`transmittance`, `rf_loss_db`) -, a camera `lens:` option
  (`rectilinear` the default since P5-18 - what a real pinhole lens and
  the product projector do; `angle_linear` kept for comparison) and
  optional Brown `k1`/`k2` distortion,
  ground-anchored clutter (`anchor: ground` - a false positive is a
  fixed ground point, not a sliding pixel); P5-11 adds per-entity
  `behaviour:` (seeded `stationary`/`wander`/`trail`/`seek` tracks that
  avoid buildings), `trails:`/`features:`/`environment:` (sun,
  visibility, precip, wind - the wind reaches every FC unit's
  sim.parm as `SIM_WIND_SPD`/`SIM_WIND_DIR`/`SIM_WIND_TURB`, and the
  camera model dims at dusk unless `kind: thermal`), plus a `state`
  field on the truth rows; P5-12 adds `areas:` - named search
  polygons the coverage asserts grade; PL-08 adds `cls: pad`
  precision-landing boards (`yaw_deg`, `size_m`, `tags_min_m`/
  `tags_max_m` tag ladder, `occlude` world-second windows) and the
  `sensors.precland` downward tag-camera model (FOV, `min_px_tag`,
  `max_rmse_px`, `noise_m`, `p_drop`) - `stub_ai_box` never reports a
  pad as a detection class). Consumed by the `stub_*`
  modules and the truth recorder, which writes
  `var/run/<sim>/truth.jsonl` at 1 Hz plus per-detection rows),
  `operation`, `verify` (P5-06 CLI, absent = recorded skip),
  `operator` (the bot's standing approve/confirm policy), `fleet`,
  `timeline`, `assert`, `teardown`, `settle_s`, `render` (P5-13 opt-in:
  `{mode: viewer|cameras|physics|full, headless}` opens a
  `de.worldstream/1` WebSocket server inside the runner on the leased
  `render.stream` port - hello snapshot + 10 Hz entity/clutter ticks +
  `cmd payload_release` + a 1 Hz `overlay` block (bot world-model
  targets, per-tid seen/miss/occluded states, recent detection
  markers, P5-14), with client `event` frames appended to
  truth.jsonl as `src: render` - and gives every FC unit a
  `--serial4 udpclient` MAVLink fan-out to the leased `render.mav`
  port for Mavlink3DMap. Absent = today's headless behaviour,
  unchanged requests and sim hash; `engine/worldstream_probe.py` is
  the test client.
  P5-14: `mode: viewer|full` with `headless: false` also launches
  Mavlink3DMap's `udp2websocket.js` (the `render.mav` UDP fan-out ->
  ws `render.mav_ws`, prefer 8811) and a vite dev server on
  `render.web` (prefer 8080)
  from `$MAVLINK3DMAP_DIR` (sim.local.env), pid-tracked under
  `var/run/<sim>/render/` - `./sim up` prints the viewer URL
  `http://127.0.0.1:<web>/?world=ws://127.0.0.1:<stream>`
  and opens no browser. `headless: true` streams without them.
  The world stream lives inside the runner process: `./sim view
  <scenario>` (or `./sim run <scenario> --hold`) parks the run after
  the timeline so the stream and the viewer stay live until Ctrl-C,
  then assertions and teardown run as usual.
  P5-15: `mode: cameras|full` adds the rendered-camera farm on
  `blocks/copter_cam3d` units (real `de_yolo_ai` + `de_camera`):
  headless Chromium (`$CHROMIUM_BIN`) renders each unit's
  hello-advertised camera into `[32-byte header][JPEG]` frames on the
  leased `render.sink` binary WebSocket; `render_sink.js` demuxes them
  into one ffmpeg per unit writing that unit's `DE-SRC-<cam_index>`
  v4l2loopback device. de_yolo_ai detects on it and writes annotated
  frames to `DE-AI-<cam_index>`, which de_camera streams. A 16-cell
  frame-number strip in the bottom rows lets `engine/render_probe.py`
  (one per unit) decode frames back off the device and post
  `src: render_probe` `frame_seen` rows; every 5th frame a
  quarter-res ID pass recolours truth entities to unique flats and
  posts `src: render_vis` `visibility` rows (px + normalized box) -
  the honest "was it actually visible" truth for
  `detection_recall {src: render_vis}`, `visible_px`,
  `detection_precision_render` (IoU vs the 1078 boxes) and
  `render_latency` (pose -> device p95). Needs one-time
  `sudo tools/setup_v4l2loopback.sh <n>` (two labelled loopbacks per
  camera unit) - `sim up` fails with that hint when the labels are
  missing. Statistical tier, not byte-deterministic: nightly/
  on-demand, never the pure gates).
- `fleet:` entries take `id`, `cls` (`copter|plane|rover|static|
  comm_only` or a `blocks/` name), `role`, absolute `home`/`base`,
  relative `at`, `modules`, `params`, `config`, `account`. `modules`
  can add catalog entries such as the P5-08 world stubs
  (`stub_ai_box`, `stub_sdr`, `stub_payload`, `stub_precland`):
  python databus modules
  in `stubs/` that read the vehicle pose (SITL serial5 on FC units,
  the intermodule 9102 feed on statics), run the world's seeded
  camera/SDR/payload models, and emit the real advert/event contracts
  (1077/1078 AI messages, `sdr.signal`, `payload.released`).
  `stub_precland` (PL-08) stands in for the real `de_precland` where
  there is no AprilTag renderer: it decides pad visibility from the
  pose + world geometry and publishes the real PRECLAND_TARGET (6537)
  / PRECLAND_STATUS (6538) wire messages plus the `precision_landing`
  capability advert (`stubs/precland_advert.json`, byte-identical to
  `de_precland --print-advert`) and facts (`stubs/precland_events.py`
  ports CPreclandEventModel, pinned by the module's golden vectors).
  `blocks/copter_precland` wires it to a copter with the FC `PLND_*`
  overlay and the de_mavlink `precland` gate open; a scenario whose
  `world:` is `worlds/pad_field.yaml` gives it a pad to land on.
- `timeline:` steps are `{t: <s>, do: <step>, ...}` or
  `{t: <s>, fault: <kind>, ...}`; `t` is seconds from scenario start and
  may be negative (runs at once). `{on: {unit, re}}` arms a journal
  watcher instead; `once` defaults true; regex groups substitute as
  `{M1}`..`{M9}`. The run lasts `last_t + settle_s`.
- `do:` names either a runner.py step (`arm`, `mode`, `takeoff`,
  `param`, `param_get`, `wait`, `link_clear`, ...) or an operator action
  (`deploy`, `start`, `abort`,
  `operator act: approve|deny|confirm|dismiss|fire|<1086>`).
  `wait_journal` takes `any: true` to finish on the first of its units
  that journals the line (a line only a runtime-chosen unit writes, e.g.
  the takeover adopter); `start` takes `expect_fail: [unit, ...]` for
  units whose START refusal is the expected outcome. `on:` watchers are
  polled between steps only - a long blocking wait starves them.
- The operator bot mirrors the WebClient's 1093 resend watch: it keeps
  every decision of the run and re-sends it unchanged (2..30 s backoff)
  while the units' roster `wm` digests disagree.
- `fault:` kinds: `link_cut`, `link_stall`, `link_profile`, `loss`,
  `delay`, `gcs_disconnect`,
  `server_down`, `kill`, `restart`, `module_kill`, `param`,
  `mode`, `rc_override`, `server_clock`, `no_time_sync` (fake transport
  for the link/clock ones) plus the FC param presets `battery`,
  `gps_off`, `gps_jam`, `gps_glitch`, `gps_degrade`, `motor_out`,
  `rc_fail`, `compass_fail`, `vibration`. `param` writes a `set:
  {NAME: value}` map through the verified serial0 param path; every
  param fault (presets included) reads each value before writing, so
  `dur_s` restores what the FC actually reported - never a hardcoded
  default. `dur_s` auto-restores where it applies.
  default. `udp_chunks` runs a seeded drop/reorder/dup
  chunk proxy on the unit's module↔de_comm s2s link - works on any
  transport; the loader wires the `udp_proxy_<module>` proc itself.
  `dur_s` auto-restores where it applies.
- `assert:` adds `proc_alive`/`proc_dead`, `link_offline`,
  `roster_value`, `operator_log_count`, `telemetry_value`, `fc_mode`,
  `distance_to` (incl. `detected_cls:` against the bot's seen targets),
  `coverage_complete` (P5-12 gains an `area:`/`poly:` form: the tlog's
  camera-unit poses - now including ATTITUDE - replayed through the
  world camera/LOS model onto a `cell_m:` grid; `min:` the covered
  fraction; `coverage.json` + `coverage.geojson` land in the run dir),
  `pod_mean` (mean cumulative POD over the area, `min:`) and `pod_at`
  (the POD of the cell holding a truth entity at `t_s:`/run end -
  `min:`/`max:`; paired with `detection_recall` it splits planning
  failures - low POD - from sensor/chain failures - high POD, no
  find), `no_sep_violation`, `detection_precision`,
  `detection_recall` (P5-10: the fraction of the named truth entities
  that were found - `src: ai|sdr|bot` (bot = the whole product chain),
  `match_m:` for the bot src, `max: 0` is the "must stay hidden"
  negative control, the detail line lists each miss's dominant why),
  `time_to_detect` (P5-10: seconds from the `start` step - the world
  epoch when none ran - to the first find, `max_s:`;
  realtime-sensitive), `signals_in_range`,
  `stub_deterministic` to the journal assertion set.
  Run-wide invariants (SL-0011): a 1 Hz sampler records every tracked
  process's pid/RSS/fds into `results/samples.jsonl`;
  `no_unexpected_restart` (pid changes only inside `fault:` kill/
  restart/module_kill/server_down windows - `do: kill` fails it),
  `rss_growth` (least-squares MB/h after `warmup_s`), `fd_growth`,
  `journal_size`, `recovers_after_fault` (link+telemetry back within
  `within_s` of each timed fault restore), `no_stuck_state` (a `re`
  state line must change within `max_s`).
  `journal_count` takes `total: true` to compare the sum over its units
  instead of every unit's own count; `module_log_count` accepts the same
  `total:` flag (B4-01 - for a line exactly one runtime-chosen unit
  prints).
  `realtime: true` gates timing asserts - they SKIP with a visible
  reason when SITL speed-up is not ~1. `telemetry_value` reads the last
  sample by default; `pick: min|max` reads the extreme of the recorded
  series (a sag that recovers by disarm still counts).

Scenario runs fly SITL at `--speedup 3` by default (only SITL's physics
clock scales; modules, the world truth tick and rendered camera feeds
stay wall-clock). A scenario pins its own rate with `speedup:` -
`realtime: true` (timing asserts) or `speedup: 1` (render-farm/real
cameras, world-driven stubs on a moving SITL, closed module<->FC
loops). `sim run --speedup N`/`SIM_SPEEDUP` overrides, and a `sim:`
file's own `speedup:` still wins over the default.

Each run gets a seed-derived port block (no cross-seed collisions) and
`var/results/<utc>/<name>/`: journals, per-unit and server logs,
`links.jsonl` (link up/down transitions the monitor saw),
`telemetry.jsonl` (per-unit FC samples = the tlog), `operator.log` (the
bot's GCS session journal), `truth.jsonl` when a world supplies it,
`verify.json`, `summary.json` (PASS/FAIL/SKIP + commit ids + speed-up)
and `junit.xml`.

**SITL capacity at speed-up 1 (this machine, 14 cores):** 24 copter
SITLs ran together for 75 s with measured speed-up 0.999-1.002 per
unit (see `speedup` in `summary.json` - FC `time_boot_ms` vs wall).
Beyond the tested 24 the FC loops may fall behind; `realtime: true`
scenarios then report a visible SKIP instead of a timing FAIL.

## Operator console

`./sim ctl` with no arguments prints the full list. `./sim ctl -s <sim>`
pins a command to one sim when several are live. The most used:

```
./sim ctl status | fc all | arm all | mode copter2 RTL | land all | reset plane1
./sim ctl deploy packages/copter_square.json | start | fire area.clear | stop | clear
./sim ctl journal copter1 | roster 10 3 | kill server de_server | server_up
./sim logs copter1 de_comm | ./sim logs server gcs_bridge
```

## Verified (2026-10-01, this machine)

| test | sim / server | result |
|---|---|---|
| `smoke_copters` | 3 copters, real server | 5/5 |
| `smoke_copters_auth` | 3 copters, real server + authenticator | 5/5 |
| `server_restart` | 3 copters, real server killed + restarted | 2/2 |
| `fault_link_loss` | 3 copters, fake server (40% loss, 45 s cut) | 5/5 |
| `mixed_bringup` | copter, plane and rover each arm and come down their own way; ground node stays online | steps pass |

Not yet run: `copter_and_cameras`. The `de_camera` binary isn't built on this
machine, so `sim up` stops with a clear "missing binaries" error.

## Notes

- **The real server does not stamp `sd`.** It routes whatever sender id the
  client wrote (`js_chat_routing.js` only injects `p`). The fake server
  stamps `sd`, so a forged-sender test that passes on the fake server
  means nothing on the real one. The bridge sets `sd: GCS_FLEET` itself.
- de_comm journals `comm line offline/online` only while a v2 package is
  loaded, and `nav adapter hello` only once per de_comm process. Tests and
  `reset` are written around both.
- The authenticator allows 10 logins/min per IP. All units share 127.0.0.1,
  so an auth sim with more than ~9 units will see some de_comm logins retried.
- SITL needs ~1-2 min for GPS/EKF before `arm` succeeds. `arm` waits for it.
- **P5-18 lens flip (release note):** the projection model changed from
  angle-linear to rectilinear in `de_common::geo_projection` and the
  harness together. Detection ground positions move by up to ~2.5 % of
  altitude off-axis, so positions/uncertainties recorded before the fix
  are **not directly comparable** to post-fix replays (the stub goldens
  were re-recorded for this reason).
