# The monthly SAR challenge

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

The world is fixed. The seed is fixed. The operation is yours. Write a
search package. It must find the people in `worlds/challenge_sar.yaml`
faster and more honestly than the reference sweep. Then prove it by
replay.

## The contract

- **World:** `worlds/challenge_sar.yaml`. One person is in the open.
  One person is under a canopy crown. One person is indoors. The indoor
  person is the honest zero. A seeded clutter field of
  person-look-alikes is also present. Camera search only.
- **Seed:** `11`. It is recorded in
  `scenarios/challenges/sar_challenge.yaml`.
- **What you may change:** exactly one line, `operation:`. Everything
  else in the scenario is the contract. This includes the world, the
  fleet, the timeline, the scorecard asserts, the seed and the
  teardown. The contract is verified.

## Entering

```bash
mkdir -p scenarios/challenges/entries/<your-handle>
cp scenarios/challenges/sar_challenge.yaml \
   scenarios/challenges/entries/<your-handle>/scenario.yaml
# the entry dir is two levels deeper - add ../.. to world: and
# verify.cli:
#   world:   ../../../../worlds/challenge_sar.yaml
#   verify:  {cli: ../../../../tools/verify_wrap.js}
# drop your .deploy.json (+ .pkg.json files) next to scenario.yaml and
# point `operation:` at it — then:
./sim run scenarios/challenges/entries/<your-handle>/scenario.yaml
# copy the RESULT summary.json into your entry dir
python3 tools/verify_challenge.py \
    scenarios/challenges/entries/<your-handle>
```

> `./sim run` takes the seed from the file's `seed:`. The contract's
> `seed: 11` applies on its own. Use `--seed N` to override it when you
> want a different draw.

A submission directory contains `scenario.yaml`, your op files, and
`summary.json`. The verifier checks that the scenario is the official
one. Only `name:` and `operation:` may differ. It also checks that the
seed is 11. Then it prints your score. `--replay` runs your scenario
again. It compares every graded value. VERIFIED means your numbers
reproduce. The replay first downs any same-name sim. The run then
always starts cold. A reused sim keeps its world epoch. That shifts
every seeded draw. The replay runs additive
(`--shared --slot-base any`). It works beside other live sims.

## Scoring (0–100)

| component | weight | source assert |
|---|---|---|
| POD over `search_block` | 60 | `pod_mean` × 60 |
| Time to find `person_meadow` | 25 | `max(0, 1 − ttd/120 s)` × 25 |
| Stream honesty | 15 | `detection_precision` × 15 |

Precision is where hopeful detectors lose points. Every clutter ghost
costs real points. Every "found" `person_barn` costs real points.
Per-person POD (`pod_at`), recall and the second TTD are reported for
information. Ties break on `person_wood`'s POD, then on
`person_meadow`'s TTD.

## The baseline

`p5_canopy_sweep.deploy.json` is the reference entry
(`<handle>` = `baseline`). It flies three lawnmower legs at 40 m. Beat
it.

## Scoreboard

`docs/SCOREBOARD.md` is the scoreboard. Append your verifier output
row after a `--replay` run comes back VERIFIED.
