# Dev-call demo script (~10 min) — ArduPilot weekly call

All commands are real paths in the repo; rehearse on the demo machine
first (`./sim doctor` all-green). Demos marked *long* are the fallback
if the call runs over — have the pre-canned `summary.json` from a
rehearsal ready to show.

## 0. Cold open (30 s)

"A scenario is one YAML file: fleet, mission package, operator,
timeline, faults, assertions. It replays deterministically. Here's one
running."

```bash
./sim list scenarios          # the gallery: 10 demos, one-line stories
```

## 1. The viewer (3 min)

```bash
./sim view scenarios/demos/world_canopy_viewer.yaml
```

Browser opens on the Mavlink3DMap viewer; the status overlay walks
connecting → waiting → pose → ready. Narrate: "the copter is sweeping
a forest; the world — people, canopy, a hut — streams to the map over
the local `de.worldstream/1` protocol, no Mapbox, no network."

Point out: the yellow pin when `person_open` is found; `person_hut`
staying an occluded outline (it *should* stay hidden — it's the
negative control).

## 2. Ground truth grading (3 min)

While the run finishes, show the scorecard it will print:

- `coverage_complete` ≈ 0.95 of the sector — the route was flown;
- `pod_mean` ≈ 0.8 — mean probability of detection over the same cells;
- `pod_at person_open` ≈ 1.0, `person_canopy` ≈ 0.5–0.9, `person_hut`
  = 0.0.

Punchline: "covered is not searched — the same path scores 0.95 on one
grader and 0.8 on the other. That's what a camera actually buys you
under canopy."

## 3. Faults as test cases (3 min, *long* — or show the summary)

```bash
./sim run scenarios/demos/battery_low_rtl.yaml     # ~80 s at 3x
```

At t=20 s the battery collapses; the op's declared `battery_low`
handler fires `rtl` once and the run pauses — the assert block proves
it fired exactly once and the NavMode changed. "The operator behavior
is a script too — approvals, vetoes, slow answers are all in the
scenario file."

## 4. Close (1 min)

- ~200 scenario files in `tests/` by feature (missions, faults, sar,
  fleet_bus, worlds, launch, precland); `./sim suite` runs the library.
- Monthly challenge: `docs/CHALLENGE.md` — fixed seed, verified by
  replay.
- LLM authoring: `docs/SCENARIOS.md` is written so an LLM can draft a
  valid scenario; `./sim validate` is the gate.

Backup if the viewer is cranky: everything except beat 1 also shows in
`./sim run` console output; the run's RESULT dir has
`coverage_*.geojson` to open in any GIS viewer.
