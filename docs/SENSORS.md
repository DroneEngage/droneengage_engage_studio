# SENSORS.md — the sensor contract (SL-0030)

A simulated sensor is **one block folder** — no `engine/` edits. The
folder carries everything the engine needs to find, launch and feed it:

```
blocks/<name>/
  block.yaml          # extends a base block, adds the stub module
  catalog.yaml        # the module's own catalog fragment (merged by
                      # simlib.load_catalog - modules/catalog.yaml is
                      # untouched)
  sensor.yaml         # the sensor contract (below)
  stub_<name>.py      # the world-driven model, on stubs/stub_base.py
```

`./sim new --sensor <name>` writes all of it from
`blocks/_template_sensor/` plus a `scenarios/<name>.yaml` that
exercises it end to end.

## The contract — `<stem>.sensor.yaml` beside the stub

`stub_base.StubBase` loads the contract named after the stub file
(`stub_sdr.py` -> `stub_sdr.sensor.yaml`; a block folder may use the
plain `sensor.yaml` fallback). It declares three things:

```yaml
sensor:
  key: sdr                     # world.sensors.<key> overrides model:
  truth:                       # what world truth the model reads
    pose: true                 #   unit pose feed (SITL serial5 on FC
                               #   units, the 9102 feed on statics)
    cls: person                #   entities of this class only
    field: tx_dbm              #   ...or entities carrying a field
  model:                       # defaults a scenario may override
    rate_hz: 2.0
    range_m: 150.0
    noise_m: 1.5
  output:                      # the seam toward the rest of the system
    kind: events               # de.cap/1 capability events
    ns: sdr
    emits: [sdr.signal]
    truth_rows: signal         # truth.jsonl {type: detection} rows
```

### truth

`truth:` is declarative — the stub reads it through
`self.truth_entities()`, which returns the world entities matching
`cls`/`field` (all entities when unset). `pose: true` just documents
that the model needs `self.pose_feed`; the pose feed is always wired.

### model

`self.sensor_params()` returns `model:` merged under
`world.sensors[<key>]`: the shipped defaults apply everywhere, and a
world file can retune the sensor per scenario:

```yaml
sensors:
  demo_range: {range_m: 80, p_drop: 0.1}
```

`world.py` passes every `sensors.<key>` through to `world.sensors`
unchanged — the engine does not know the sensor's parameter names.

### output

`output:` documents what the stub puts on the bus and into
`var/run/<sim>/truth.jsonl`. The detection rows are the evidence:

```json
{"type": "detection", "src": "<key>", "unit": "...", "tid": "...",
 "lat": ..., "lng": ..., "t_ms": ..., "sent": true, ...}
```

Any check can then compare *what the sensor reported* against *where
the truth actually was*. `sensor_error` does this for every detection
row of a `src`:

```yaml
assert:
  - sensor_error: {src: demo_range, unit: probe,
                   mean_le: 5, p95_le: 10, min_count: 5}
```

For each row it measures the distance from the reported `lat`/`lng` to
the truth entity's position at that time, and reports `n` / `mean_m` /
`p95_m` / `max_m` into `summary.json`; `mean_le` / `p95_le` / `max_le`
bound them (each optional). Rows that can't be matched to a truth
entity are skipped, not counted.

## The catalog fragment

A block's own `catalog.yaml` registers its module — the block name is
the fleet `cls:` and the module lands in `add_modules:` via
`block.yaml`:

```yaml
stub_demo_range:
  repo: droneengage_simulation_land
  bin: stub_demo_range.py        # resolves inside this block dir
  template: stub.json            # falls back to modules/templates/
  port_base: 62000
  start_delay: 3
  stub: true                     # the world injector feeds it stub:cfg
  # pose: true                   # add if it rides an FC unit
  local: {file: stub_demo_range.local}
```

`stub: true` replaces the old hardcoded stub list: on a `world:`
scenario the runner injects `{world, seed, unit, pose_port}` into the
module's config so the stub can read truth and write `truth.jsonl`.
`pose: true` additionally requests the SITL serial5 pose port for FC
units. Two modules must not define the same catalog name — a block
fragment colliding with `modules/catalog.yaml` is a load error.

## The model

The stub subclasses `stub_base.StubBase` and implements
`tick(t_s, frame)` (~20 Hz). Determinism is the same rule as every
world consumer: draw from `self.rng(<parts...>)` — seeded by
`stub.seed` — never `random.random()`. `--selftest <pose.jsonl>`
replays scripted poses on the virtual clock; two same-seed selftest
runs are byte-identical (`stub_deterministic` asserts it).

Per-run state goes in `reset_run()` — a reused sim keeps the process
and re-runs with a new epoch.

## Reference: stub_sdr

`stubs/stub_sdr.py` + `stubs/stub_sdr.sensor.yaml` are the worked
example. The contract declares its inputs (`field: tx_dbm` emitters +
pose), the full model surface (path-loss exponent, debounce/throttle,
noise floor, sweep timing, radius clamps — all overridable via
`world.sensors.sdr`) and the output seam (`sdr.signal` events +
`signal` truth rows). `tests/p5/world_sdr_mast.yaml` exercises it and
asserts `signals_in_range`; `stubs/test_stubs.py` covers determinism.

## Try it

```
./sim new --sensor demo_range
# edit blocks/demo_range/sensor.yaml + stub_demo_range.py model
./sim validate scenarios/demo_range.yaml
./sim run scenarios/demo_range.yaml --seed 1
```

The scaffolded scenario sits a static probe in `worlds/forest_a.yaml`
and asserts `sensor_error` against the stub's own reports, so the
default model passes on first run.
