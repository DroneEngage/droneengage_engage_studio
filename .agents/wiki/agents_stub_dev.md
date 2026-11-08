# droneengage_studio — developing a stub sensor

How to write a new world-driven sensor. Read `agents_stubs.md` first
for what a stub is; `docs/SENSORS.md` is the contract's source of truth.

## Scaffold — never write the files by hand

```bash
./sim new --sensor demo_range
# writes: blocks/demo_range/{block.yaml,catalog.yaml,sensor.yaml,
#          stub_demo_range.py} + scenarios/demo_range.yaml
./sim validate scenarios/demo_range.yaml
./sim run scenarios/demo_range.yaml --seed 1
```

The scaffolded model passes on first run (its scenario asserts
`sensor_error` against its own reports). Edit the model + contract,
not the plumbing.

## The three files you actually edit

**`sensor.yaml`** — the contract `stub_base.StubBase` loads (named
after the stub file: `stub_x.py` → `stub_x.sensor.yaml`; plain
`sensor.yaml` is the block fallback):

```yaml
sensor:
  key: demo_range            # world.sensors.<key> overrides model:
  truth:
    pose: true               # model needs the unit pose feed (always wired)
    cls: person              # OR: only entities of this class
    field: tx_dbm            # OR: only entities carrying this field
  model:                     # defaults; scenario-tunable
    rate_hz: 2.0
    range_m: 150.0
    noise_m: 1.5
  output:
    kind: events             # what lands on the bus (de.cap/1 events)
    ns: demo_range
    emits: [demo_range.hit]
    truth_rows: hit          # truth.jsonl {type: detection} rows
```

**`catalog.yaml`** — registers the module beside the block (its name
must not collide with `modules/catalog.yaml` — that is a load error):

```yaml
stub_demo_range:
  repo: droneengage_studio
  bin: stub_demo_range.py     # resolves inside the block dir
  template: stub.json         # falls back to modules/templates/
  start_delay: 3
  stub: true                  # the world injector feeds stub:cfg
  # pose: true                # add when the block rides an FC unit
  local: {file: stub_demo_range.local}
```

**`stub_<name>.py`** — subclass `stub_base.StubBase`:

```python
class StubDemoRange(StubBase):
    def reset_run(self):            # per-run state; a reused sim re-runs
        self.seen = {}
    def tick(self, t_s, frame):     # called ~20 Hz
        for e in self.truth_entities():
            ...                     # model math -> self.send_*(...)
```

The API surface:

- `self.truth_entities()` — world entities matching `truth.cls`/`field`
  (all when unset), each with pose/attrs at time `t_s`.
- `self.pose_feed` — the unit's own pose (SITL serial5 on FC units,
  the 9102 feed on statics).
- `self.sensor_params()` — `model:` merged under `world.sensors[<key>]`.
- `self.rng(<parts...>)` — the seeded RNG. **Never `random.random()`** —
  determinism is the contract's core promise.
- Send helpers write both the bus message and the `truth.jsonl` row.

## Determinism rules (hard requirements)

- Draw every random quantity from `self.rng(...)`, seeded by
  `stub.seed` — same seed, byte-identical output.
- Read the clock from `t_s` (scenario time), never `time.time()`.
- `reset_run()` holds all per-run state — a warm sim re-runs with a
  new epoch on the same process.
- `--selftest <pose.jsonl>` must replay scripted poses on the virtual
  clock; two same-seed selftest runs are byte-identical —
  `stub_deterministic` asserts it, `stubs/test_stubs.py` runs it.

## Evidence a stub must produce

Every report appends a `truth.jsonl` detection row:

```json
{"type": "detection", "src": "<key>", "unit": "<unit>", "tid": "<ent>",
 "lat": ..., "lng": ..., "t_ms": ..., "sent": true}
```

Asserts then compare reports against truth:

- `sensor_error {src, unit, mean_le, p95_le, max_le, min_count}` —
  distance error stats from the rows.
- `signals_in_range` — sdr-style event assertions.
- `stub_deterministic` — the selftest byte-identity check.

## Verify

```bash
python3 stubs/test_stubs.py                      # determinism over all stubs
./sim validate <scenario.yaml>
./sim run scenarios/<name>.yaml --seed 1         # end to end
./sim run tests/worlds/world_sdr_mast.yaml       # the worked reference
```

`tests/worlds/world_sdr_mast.yaml` + `stubs/stub_sdr.py` +
`stubs/stub_sdr.sensor.yaml` is the canonical example — copy its
shape.
