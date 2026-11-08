# droneengage_studio — stub sensors

## What a stub is

A stub is a simulated DE module written in Python. It launches like a
real module through the catalog (`'<module>' -c '<generated config>'`),
talks the same databus protocol and sends the same messages. Instead
of hardware it reads the **world truth** (`worlds/*.yaml`): it answers
"what would the real sensor report, given where every entity actually
is". Downstream modules (de_mavlink, the supervisor, the webclient)
see ordinary protocol traffic — they cannot tell a stub from hardware.

Shipped stubs live in `stubs/`:

| Stub | Simulates | Emits |
|---|---|---|
| `stub_ai_box.py` | camera AI (de_yolo_ai / de_ir_camera) | `AI_OBSERVATIONS` (6546) boxes + `AI_Recognition_STATUS` (1077); honors class select on 1076 |
| `stub_sdr.py` | `de_sdr` spectrum scan | `sdr.signal` events {freq, rssi_dbm, pos, radius_m, snr_db} + de.cap/1 advert |
| `stub_precland.py` | `de_precland` target detections | precland target reports |
| `stub_payload.py` | a payload-bay module | payload capability/actions |

Helper files beside them (`precland_events.py`, `model_import.py`,
`fake_proc_stat.py`) are inputs/builders, not modules.

## How a stub is wired

One block folder carries everything — no `engine/` edits:

```
blocks/<name>/
  block.yaml          # extends a base block, adds the stub module
  catalog.yaml        # block-local module registration (stub: true)
  sensor.yaml         # the sensor contract
  stub_<name>.py      # the model, on stubs/stub_base.py
```

`stub: true` in the catalog entry is the key flag. On a scenario with
`world:` the engine injects into the module's config:

```json
"stub": {"world": "<world.yaml>", "seed": 1,
         "unit": "<unit>", "pose_port": <sitl serial5 port>}
```

From that the stub reads the world file plus, under `var/run/<sim>/`:

- `world_epoch.json` — scenario t0; absent → the stub idles
- `world_overrides.jsonl` — react knots the truth tick writes (SL-0013)
- `stubs/<unit>.log` — its own "one line per sent message" record
- `truth.jsonl` — **it appends** one row per report: the evidence every
  `sensor_error`/`signals_in_range` assert consumes

Pose comes from SITL serial5 (ATTITUDE + GLOBAL_POSITION_INT) on FC
units — never the pilot/fcb ports — or from 9102 intermodule location
reports on static units (their only source; de_comm broadcasts the
configured `location` at 1 Hz).

## How to use one in a test

```yaml
# tests/<area>/my_test.yaml
world: ../worlds/forest_a.yaml        # seeded truth entities
fleet:
  - {id: probe, cls: copter_ai}       # a block whose catalog adds stub_ai_box
assert:
  - sensor_error: {src: ai_box, unit: probe, mean_le: 5, p95_le: 10, min_count: 5}
```

Rules that apply:

- A scenario with `world:` needs `realtime: true` / `speedup: 1` —
  world-driven stubs run on wall clock.
- Tune the model per scenario via `world.sensors.<key>` in the world
  file — it merges over the contract's `model:` defaults; the engine
  never sees the names.
- Every assert that judges the sensor works off its own
  `truth.jsonl` rows (`sensor_error` measures reported lat/lng vs the
  truth entity's position at that `t_ms`). Unmatched rows are skipped.

## Example — stub_sdr end to end

`tests/worlds/world_sdr_mast.yaml` puts a static mast with `tx_dbm`
emitters in the world and runs `stub_sdr` on a probe:

- the contract `stubs/stub_sdr.sensor.yaml` declares
  `truth: {pose: true, field: tx_dbm}` — only emitter entities feed
  the model;
- the model computes a log-distance RSSI per emitter plus a seeded
  noise floor, debounces like the real `CSDRSignalDetector`, and emits
  `sdr.signal` only while a transmitter is inside modelled range;
- the test asserts `signals_in_range` on the `signal` truth rows.

`stubs/test_stubs.py` runs the same models against scripted poses and
proves two same-seed runs are byte-identical.

## Where to read next

- `docs/SENSORS.md` — the full sensor contract (the source of truth).
- `agents_stub_dev.md` — writing a new stub (scaffold, StubBase API,
  determinism rules, verification).
- `docs/WORLDS.md` — authoring the world file the stub reads.
