# scenarios/precland/plan — PL-09 precision landing in mission plans

TASK-PL-09: precision landing proven as part of compiled
`de.operation/2` plans, not just standalone module runs
(`scenarios/precland/standalone/` covers those). Two groups:

- **Group A** — a static unit runs the real `de_precland` against
  test-scene files; the plan gates on the module's actual facts.
- **Group B** — copters fly the closed loop on SITL: `stub_precland`
  locks a world pad, `de_mavlink` gates `LANDING_TARGET` (PL-06), the
  FC precision-lands. Touchdown error is measured against truth rows.

| file | group | what it proves |
|---|---|---|
| `pl_plan_gate_stabilized.yaml` | A | `gate {event: precision_landing.stabilized}` releases on the module's fact, not the enable command; `disabled` (emitted inside the invoke) releases via the sticky path |
| `pl_plan_command_vs_fact.yaml` | A | a `do` invocation is not the fact — the gate waits for the real `enabled` event |
| `pl_plan_trigger_lost.yaml` | A | `precision_landing.lost` (reason `no_target` on an empty scene) triggers a sensor mission twice across a near/empty cycle |
| `pl_plan_set_target.yaml` | A | `set_target` → `target_changed` fact releases the gate |
| `pl_plan_set_target_enum.yaml` | A | negative: `target: 7` outside the advert enum → `CAP-002` at verify, deploy never happens (exit 1 by design) |
| `pl_plan_calibrating_reject.yaml` | A | offline boundary: `calibrate` is refused outright on a test source ("requires a live camera source") while `enable` still works |
| `pl_copter_land_nav.yaml` | B | full closed loop: `nav: land` puts AUTO on `MAV_CMD_NAV_LAND`, the PL-06 gate opens, `LANDING_TARGET` pulls touchdown to 0.0 m of `pad_1` (4.0 m from the nominal nav point) |
| `pl_copter_end_land.yaml` | B | same loop ended by `end: land` — NavMode LAND opens the mode gate instead of the PL-06 seq gate; 0.0 m on `pad_1` |
| `pl_copter_no_pad.yaml` | B | no pad in the world: gate times out (`on_timeout: continue`), dumb landing 0.0 m of the nominal point |
| `pl_copter_dropout.yaml` | B | occlude window drops the tag mid-descent: `lost` fact, `watch` emits `pad.alarm` exactly once (`once_per: run`), re-acquire and touchdown 0.0 m |
| `pl_copter_fleet_scope.yaml` | B | two copters on `pad_1`/`pad_2`: the peer's `stabilized` arrives fleet-wide but `when: ${event.src == self.party}` scopes `watch` to each unit's own lock; both land 0.0 m on their own pads |

Key contract: `precision_landing.*` events the plan waits on are declared
`persist: "run"` (sticky facts) with advert-copied payload schemas —
synchronous emissions inside a `do` invoke (e.g. `disabled`,
`target_changed`) would be missed by an edge-only gate.

Ops live in `ops/` (one `.deop` + generated `.ctx.json` caps registry
each — `pl.unit.caps.json` for the real module, `pl.copter.caps.json`
for the stub) and compile to `packages/` via:

```bash
node scenarios/support/compile_op.js scenarios/precland/plan/ops/<op>.deop \
    --out-dir packages --caps scenarios/precland/plan/ops/pl.unit.caps.json \
    --fleet --binding node=node            # group A

node scenarios/support/compile_op.js scenarios/precland/plan/ops/<op>.deop \
    --out-dir packages --caps scenarios/precland/plan/ops/pl.copter.caps.json \
    --fleet --binding lander=cam1          # group B (land_a=c1;land_b=c2 for fleet)
```

Worlds: `worlds/pad_field.yaml` (+`pad_field_ref`/`pad_none` markers,
`pad_occluded` dropout window). Group B needs a SITL build
(`cls: copter_precland`); every copter scenario arms in GUIDED before
`start`. Terminal `nav: land` disarms before "mission completed" can
journal — `vehicle disarmed` is the touchdown marker.

Measured touchdown error (seed 1, both runs): 0.0 m from `pad_1` /
`pad_2` for the four closed-loop landings, 0.0 m from `lz_nominal`
for the no-pad dumb landing.

```bash
./run_scenario.py scenarios/precland/plan/pl_copter_land_nav.yaml
```
