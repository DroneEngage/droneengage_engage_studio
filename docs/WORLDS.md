# WORLDS.md — add a ground-truth world (de.world/1)

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

A `worlds/*.yaml` file describes **what is really out there**: the
people, phones, pads and aircraft. Stub sensors turn this truth into
the messages real modules would send. The runner loads it once per run.
It writes the scenario epoch into `var/run/<sim>/world_epoch.json`. It
records 1 Hz truth rows into `var/run/<sim>/truth.jsonl`. Every stub
reads the same file + epoch. Truth and detections always agree.

The field-by-field contract lives in the `engine/world.py` docstring.
It is the spec. This file is the workflow. `worlds/forest_canopy.yaml`
is the reference example.

## The recipe

1. **Pick an origin.** `origin: [lat, lng]` is the world's flat-earth
   frame. Entities, obstacle polys, trails and areas are absolute
   `[lat, lng]` coordinates. Put the origin at or near the sim's home.
   Then the north/east offsets in the header math stay small (the
   existing worlds sit ~100 m NE of the ArduPilot SITL default
   `-35.363262, 149.165237`).

2. **Write the header first.** Every world opens with a comment block.
   It states what the world studies. It shows the math a reviewer can
   check by hand: entity N/E offsets vs origin, the attenuation/gate
   numbers the scenario asserts. `forest_canopy` shows the convention:
   one line per entity, then the derived quantities (e.g. the sdr range
   with and without crown loss). If you cannot state why an entity sits
   where it does, the world is not done.

3. **Entities.** `{id, cls, pos|path|attach, ...}`:
   - `pos: [lat,lng]` — static.
   - `path: [[lat,lng],...] + speed_mps` — piecewise-linear walk.
     `loop: true` patrols.
   - `attach: <entity_id>` + `offset_m: [n,e]` — rides another entity
     (a phone in a person's crown, a pad on a vehicle). `alt_m` sets
     the height above the home plane.
   - `behaviour:` — seeded motion on top of the anchor
     (`stationary`, `wander`, `trail`, `seek`).
   - `react:` — ONE-authority triggers (`near: {unit|unit_cls, dist_m,
     then: {flee|freeze|seek}}`). The runner's truth tick latches them.
   - Special classes: `pad` (precision-landing AprilTag board, PL-08
     keys), `aircraft` (ADS-B intruder, SL-0016 keys), `phone`
     (tx_dbm/freq_hz for the sdr model), `temp_c` for thermal.
   - `size_m` — the truth footprint the pixel checks grade.

4. **Obstacles** — vertical prisms `{id, kind, poly, height_m, ...}`.
   `canopy` attenuates Beer-Lambert style (`base_m`, `gap_fraction`,
   `rf_db_per_m`, `rf_max_db`). `building` is opaque (`rf_loss_db`).
   Defaults are ESTIMATES. Override them with your scene's magnitudes.

5. **Areas / trails / features.** `areas:` names search polygons. The
   coverage assertions (`coverage_complete`, `pod_mean`, `pod_at`) grid
   over them. `trails:`/`features:` feed the `trail`/`seek` behaviours.

6. **Clutter.** `clutter: {cls, rate_per_km2_min, lifetime_s, anchor}`
   — seeded false positives. `anchor: ground` re-projects a fixed
   ground point each frame (occlusion applies). `anchor: image` puts it
   on the image plane.

7. **Environment.** `environment:` is static per run:
   `time_of_day`/`sun_elev_deg` (light), `visibility_m`, `precip`,
   `ambient_c` (thermal background), `wind` (pushed to every FC's
   `SIM_WIND_*` at bring-up).

8. **Sensors.** `sensors.camera` (lens `rectilinear` default /
   `angle_linear`, FOV, mount angles, `p_detect_*`, `max_range_m`,
   `pos_noise_px`, `size_m` per cls, `kind: thermal` for thermal),
   `sensors.sdr` (freq, path-loss exponent, reference/noise/threshold,
   `rf_precip_db_per_km`), `sensors.precland` (the downward tag
   camera). These feed the stub models AND the world stream. The viewer
   renders the same geometry.

## Wire it up

```yaml
world: ../../worlds/my_world.yaml        # file-relative, like operation:
```

`./sim validate <scenario.yaml>` resolves the reference. `./sim list`
and `check_library` enforce the header paragraph. Run it:

```bash
./sim run tests/worlds/my_test.yaml        # full pipeline + truth.jsonl
./sim view scenarios/demos/my_demo.yaml    # + Mavlink3DMap world stream
```

## Determinism and gotchas

- All randomness derives from `(scenario seed, purpose, unit, entity,
  tick)`. Same seed gives identical model output. A world with no
  obstacles and default sensor keys replays byte-identically
  (`stubs/golden`, `engine/test_golden.py`).
- A `react:` world is **not** byte-identical. The truth tick latches
  triggers at wall-clock. It is statistically reproducible only. Pin
  `seed:` for stable behaviour. Assert bounds, not exact times.
- `pos_at` is pure given the file. Stubs never mutate the world. The
  runner's `world_overrides.jsonl` is the only side-channel. Only
  `react:` writes it.
- wander/seek never enter a `building` prism. `canopy` is walkable.
- `pos:` is absolute `[lat, lng]`. Check it by hand: at the SITL
  default, 1e-5 deg lat ~= 1.1 m north, 1e-5 deg lng ~= 0.9 m east (cos
  35.36 deg). The header's N/E offsets exist so nobody has to redo that
  math.
