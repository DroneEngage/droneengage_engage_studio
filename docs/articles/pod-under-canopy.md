# Why covered != searched: POD under canopy

A lawn-mower survey that reports "95 % area covered" sounds like a
mission that worked. It usually isn't. Coverage counts *looks*; a look
through leaves is not a search. Under canopy the difference between the
two is exactly where missing persons hide.

This is the gap the simulation world exists to measure. The
`world_canopy_sweep` scenario (promoted to
`scenarios/demos/world_canopy_viewer.yaml` for watching live) flies a
single camera copter over `worlds/forest_canopy.yaml` — a field with
three people in it:

- `person_open` — standing in the open;
- `person_canopy` — under the tree line, reachable only through gaps in
  the clutter;
- `person_hut` — inside a hut. Fully occluded, on purpose: the negative
  control.

The world model raycasts every camera look through the clutter and
obstacle layers. A look either lands on the entity or it doesn't — and
when it doesn't, the world says *why* ("occluded"), not just "no
detection". Probability of detection is then measurable, per cell and
per person, instead of asserted by hope.

The scenario's assertion block is the whole argument in four lines:

```yaml
- coverage_complete: {area: sector_a, cell_m: 5, min: 0.95}
- pod_mean:          {area: sector_a, cell_m: 5, min: 0.8}
- pod_at:            {truth: person_open,   min: 0.9}
- pod_at:            {truth: person_canopy, min: 0.5}
- pod_at:            {truth: person_hut,    max: 0}
- detection_recall:  {truth: person_hut,    max: 0}
```

Read top to bottom: the sweep really does cover 95 % of the sector —
and the mean probability of detection over those same cells is only
80 %. The open-ground person is essentially guaranteed to be seen
(POD ≥ 0.9). The person under canopy is a coin flip (≥ 0.5 — the assert
floors it, but the honest number hovers there). The person in the hut
must produce *zero* detections, and the recall check double-locks that.

The first two lines are the lesson. `coverage_complete` and `pod_mean`
grade the *same* flown path against the *same* grid and disagree by 15
points, because coverage treats every visited cell as searched while
POD weights each look by whether anything could actually have been seen
through the canopy from that pose.

That is also why the two checks exist as separate graders in
`engine/checks.py` rather than one "search quality" score: they answer
different questions. Coverage tells you the plan was *flown*. POD tells
you the plan was *worth flying*. A survey tuned only for coverage will
happily skim the treetops, log a perfect grid, and leave the person in
the hut exactly where they started.

If you take one thing into mission planning: coverage is a property of
the *route*; probability of detection is a property of the route *and
the world*. Any tool that reports only the first is grading the easy
half.
