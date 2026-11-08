# Course assignment — search under canopy

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

**Aim:** write a drone search pattern that beats **POD 0.8 under
canopy**. POD is probability of detection, not coverage. The world is
a fixed ground-truth world. The assignment fits robotics courses and
GSoC warm-ups. It runs end to end on a laptop. No hardware is
necessary.

## Setup

```bash
./sim doctor                        # one-time environment check
./sim run scenarios/challenges/entries/baseline/scenario.yaml --seed 11
```

The baseline is the reference entry of the monthly challenge
(`docs/CHALLENGE.md`). It flies three lawnmower legs at 40 m over
`worlds/challenge_sar.yaml`. Watch what it does before you write your
own.

## The world (starter file)

`worlds/challenge_sar.yaml` is fixed. Treat it as the exam paper, not
the worksheet:

- `person_meadow` — open ground. Any competent pattern finds them.
- `person_wood` — under a canopy crown. The crown is a 2–12 m band
  with `gap_fraction: 0.35`. A look through the crown transmits at
  ~35 % head-on. It transmits less at oblique angles. This person is
  the subject of the assignment.
- `person_barn` — indoors. POD is 0 by geometry. A "find" here means
  your detector is hallucinating. It will cost precision.
- `clutter` — seeded person-look-alikes. They keep re-projecting as
  false positives.

The graded surface is `areas.search_block` (~75 × 115 m). The camera
points at nadir. Its field of view is 62.2° × 48.8°. Detection
probability falls from 0.95 at nadir to 0.6 at the frame edge. The
math is documented in `docs/SENSORS.md` and `docs/WORLDS.md`.

## Your task

Clone the entry. Change only `operation:`. Improve the score:

```bash
cp -r scenarios/challenges/entries/baseline \
      scenarios/challenges/entries/<you>
# fix world:/verify.cli: paths are already right (same depth);
# rename `name:` to your own while you're in there
# write your own .deploy.json + .pkg.json, point operation: at them
./sim run --seed 11 scenarios/challenges/entries/<you>/scenario.yaml
python3 tools/verify_challenge.py scenarios/challenges/entries/<you>
```

## Thresholds

| grade | requirement |
|---|---|
| pass | `pod_at person_wood` ≥ 0.8 **and** `detection_precision` ≥ 0.9 |
| merit | baseline's score (90.7) beaten |
| distinction | score ≥ 95 — i.e. near-perfect POD *and* sub-40 s TTD |

The verifier prints `pod_mean`, `ttd`, `precision` and the weighted
score. The scoreboard formula is in `docs/CHALLENGE.md`.

## Reference solution walkthrough

`p5_canopy_sweep` is the baseline op. It reaches **pod_at person_wood
= 0.900**. That is past the 0.8 bar. It scores **90.7** on seed 11:

1. **Legs over the cell.** The serpentine lays ~36 m-wide nadir
   footprints across `search_block`. This gives 99.1 % coverage.
   Coverage is the entry ticket. POD cannot exceed it.
2. **Eleven looks on the wood cell.** The `person_wood` cell collected
   11 looks through the crown. POD 0.900 means each look contributed
   ~0.19 independent detection probability through the 35 % gap. The
   math is `1 − (1 − p)¹¹ ≈ 0.90`. Volume of looks is how you buy POD
   through gaps.
3. **…and still missed.** With 11 looks at 0.9 POD, the person is
   *usually* found. Seed 11 drew the 10 %. Recall on `person_wood` is
   0. The wood TTD fails. That is the real lesson. A POD number is a
   distribution, not a guarantee. The margin between 0.90 and 0.99 is
   what your pattern buys.
4. **The honest zero.** The `person_barn` cell absorbed 8 looks. POD
   stayed 0.000. The building eats every ray. You cannot score here.
   You can only *lose* here. A reported barn find costs points.

To beat it, you have options:

1. Revisit the wood cell. A second pass at a different heading changes
   the oblique path through the crown.
2. Fly lower over it. Does a shorter slant give more transmit length
   through gaps? No. Lower altitude gives *shorter* crown thickness
   only when the geometry cooperates. Check the math in `world.py`'s
   docstring.
3. Accept more wall time per unit of POD near the hard cells. Sprint
   the easy ones.

## Deliverable

Put the deliverable in `scenarios/challenges/entries/<you>/`. It
contains `scenario.yaml`, your op files, and the `summary.json` of a
`verify_challenge.py --replay` VERIFIED run. Write up: what you
changed, which assert moved, and one number that surprised you.

## Instructor notes

- A run takes ~4 min wall time. `realtime: true` is set. The world
  truth tick is wall-clock. Do not raise the speedup. A raised speedup
  under-samples the look stream. That failure is itself teachable.
- The seed is fixed at 11 with `--seed`. A different seed is a
  different exam.
- Extension ideas: add a second copter that splits `search_block`.
  Claim mechanics are in `tests/fleet_bus/`. Or raise the clutter rate
  in a *copy* of the world. Or use `react:` entities that flee. See
  `worlds/sar_forest_flee.yaml`.
