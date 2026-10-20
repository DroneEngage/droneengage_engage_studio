# Testing drone missions like software

Drone projects test the part that doesn't matter and demo the part
that does. The motor driver gets unit tests; the search-and-rescue op
that coordinates six aircraft gets a field day and a prayer. The excuse
is always the same: missions are physical, humans are in the loop,
nothing is reproducible.

`de.scenario/1` is the format that removes the excuse. One versioned
YAML file rehearses an entire operation — fleet, mission package,
operator, timeline, faults, and the grading — the way one test file
rehearses a code path:

```yaml
fleet:     # which units, what class, which module stubs, where parked
operation: # the compiled mission package to deploy
operator:  # a bot that replays the human: approvals, confirms, vetoes
timeline:  # scripted steps: arm, start, wait for journal lines, fire
           # events — and inject faults at t seconds
assert:    # what must be true when it ends
```

Every piece of the "you can't test that" list is in the file.

**The fleet** is simulated units — ArduPilot SITL behind a real
de_comm + de_mavlink stack, or a fake transport when the physics is not
the point. A scenario declares `cls: copter` the way a test declares a
fixture, and blocks (`unit: block:copter`) make the same file work when
the fleet grows.

**The operation** is the real artifact — the compiled `.deploy.json`
the webclient would ship, not a re-described mission. If the package
format changes, the test breaks, which is the point.

**The operator** is the piece everyone forgets to test. `operator:
approve: {launch: {after_s: 2}}` replays a human clicking "approve"
two seconds after the request. Denials, slow responses, and no-shows
are scripts too — the SAR suite's `p4b10_b_auto_veto` through
`p4b10_f_drop_denied` variants exist precisely because the human's
answer changes the mission, so the human is in the test.

**Faults** are timeline steps, not chaos monkeys: `fault: battery,
volts: 10.5, cap_mah: 50, dur_s: 200` lands at a scripted `t`, inside a
run whose every other event is deterministic. You can point at the
exact moment the declared `battery_low` handler should fire and assert
that it fired *once*.

**The grading** is assertions on journal lines, telemetry series,
world truth and coverage grids — the same vocabulary a reviewer uses to
judge a real flight log.

Two properties make it behave like software rather than a rig:

- **Determinism.** A `seed:` pins the world; the fake transport pins
  the link model; scenario runs default to a 3× SITL speedup, and
  `realtime: true` opts into wall-clock when the timing itself is under
  test. The same file gives the same result in CI tonight as on your
  desk this afternoon.
- **Ownership.** `./sim run` is exclusive — it refuses to take over a
  sim another session owns, and stale manifests from dead processes
  can't block a fresh run. The suite command sweeps `tests/` by
  feature folder, excludes `soak/` and `experimental/` by default, and
  emits machine-readable `--json` with a `RESULT` summary path.

The library it produced looks like a test suite because it is one:
`tests/missions/`, `tests/faults/`, `tests/sar/`, `tests/fleet_bus/`,
`tests/worlds/`, `tests/launch/`, `tests/precland/` — a couple of
hundred files, each with a header line saying what it proves. The good
ones are promoted into `scenarios/demos/` for humans to watch.

Missions will never be as cheap to test as pure functions — SITL still
has to boot. But "expensive and deterministic" is the whole job of a
test harness. The remaining step is cultural: treat the mission, the
operator and the failure modes as code — version them, review them,
and let CI fly them while you sleep.
