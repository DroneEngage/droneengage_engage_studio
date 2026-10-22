# Course assignment — search under canopy

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

**Aim:** write a drone search pattern that beats **POD 0.8 under canopy**
— probability of detection, not coverage — against a fixed ground-truth
world. Suitable for robotics courses and GSoC warm-ups; runs end to end
on a laptop, no hardware.

## Setup

```bash
./sim doctor                        # one-time environment check
./sim run scenarios/challenges/entries/baseline/scenario.yaml --seed 11
```

The baseline is the reference entry of the monthly challenge
(`docs/CHALLENGE.md`): three lawnmower legs at 40 m over
`worlds/challenge_sar.yaml`. Watch what it does before writing your own.

## The world (starter file)

`worlds/challenge_sar.yaml` — fixed; treat it as the exam paper, not
the worksheet:

- `person_meadow` — open ground; any competent pattern finds them.
- `person_wood` — under a canopy crown (2–12 m band, `gap_fraction:
  0.35`): a look through the crown transmits at ~35 % head-on and worse
  obliquely. This is the person the assignment is about.
- `person_barn` — indoors. POD 0 by geometry; a "find" here means your
  detector is hallucinating and will cost precision.
- `clutter` — seeded person-look-alikes that keep re-projecting as
  false positives.

The graded surface is `areas.search_block` (~75 × 115 m). The camera is
nadir, 62.2° × 48.8°, detection probability falling from 0.95 at nadir
to 0.6 at the frame edge — the math is documented in
`docs/SENSORS.md` and `docs/WORLDS.md`.

## Your task

Clone the entry, change only `operation:`, and improve the score:

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
score; the scoreboard formula is in `docs/CHALLENGE.md`.

## Reference solution walkthrough

`p5_canopy_sweep` (the baseline op) reaches **pod_at person_wood =
0.900** — past the 0.8 bar — and scores **90.7** on seed 11:

1. **Legs over the cell.** Its serpentine lays ~36 m-wide nadir
   footprints across `search_block` → 99.1 % coverage. Coverage is the
   entry ticket: POD can't exceed it.
2. **Eleven looks on the wood cell.** `person_wood`'s cell collected 11
   looks through the crown. POD 0.900 means each look contributed
   ~0.19 independent detection probability through the 35 % gap —
   `1 − (1 − p)¹¹ ≈ 0.90`. Volume of looks is how you buy POD through
   gaps.
3. **…and still missed.** With 11 looks at 0.9 POD the person is
   *usually* found — seed 11 drew the 10 %: recall on `person_wood` is
   0 and the wood TTD fails. That is the real lesson: a POD number is
   a distribution, not a guarantee. The margin between 0.90 and 0.99 is
   what your pattern is buying.
4. **The honest zero.** `person_barn`'s cell absorbed 8 looks and POD
   stayed 0.000 — the building eats every ray. You cannot score here;
   you can only *lose* here if your stream reports a barn find anyway.

To beat it: revisit the wood cell (a second pass at a different heading
changes the oblique path through the crown), fly lower over it
(shorter slant → more transmit length through gaps? no — *shorter*
crown thickness at lower altitude only if geometry cooperates; check
the math in `world.py`'s docstring), or accept more wall time per unit
of POD near the hard cells and sprint the easy ones.

## Deliverable

`scenarios/challenges/entries/<you>/` containing `scenario.yaml`, your
op files, and the `summary.json` of a `verify_challenge.py --replay`
VERIFIED run. Write up: what you changed, which assert moved, and one
number that surprised you.

## Instructor notes

- Run is ~4 min wall (`realtime: true` — the world truth tick is
  wall-clock; don't raise the speedup or the look stream under-samples,
  which is itself a teachable failure).
- Seed is fixed at 11 via `--seed`; a different seed is a different
  exam.
- Extension ideas: second copter splitting `search_block` (claim
  mechanics in `tests/fleet_bus/`), clutter rate raised in a *copy* of
  the world, or `react:` entities that flee (`worlds/sar_forest_flee.yaml`).
