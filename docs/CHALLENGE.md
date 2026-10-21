# The monthly SAR challenge

Fixed world, fixed seed, your operation. Write a search package that
finds the people in `worlds/challenge_sar.yaml` faster and more honestly
than the reference sweep — then prove it by replay.

## The contract

- **World:** `worlds/challenge_sar.yaml` — one person in the open, one
  under a canopy crown, one indoors (the honest zero), plus a seeded
  clutter field of person-look-alikes. Camera search only.
- **Seed:** `11` (recorded in `scenarios/challenges/sar_challenge.yaml`).
- **What you may change:** exactly one line — `operation:`. Everything
  else in the scenario (world, fleet, timeline, the scorecard asserts,
  seed, teardown) is the contract and is verified.

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
./sim run --seed 11 \
    scenarios/challenges/entries/<your-handle>/scenario.yaml
# copy the RESULT summary.json into your entry dir
python3 tools/verify_challenge.py \
    scenarios/challenges/entries/<your-handle>
```

> `./sim run` takes the seed from `--seed` (default 1) — the file's
> `seed: 11` marks the contract; the flag is what applies it.

A submission directory is `scenario.yaml` + your op files +
`summary.json`. The verifier checks the scenario is the official one
(modulo `name:`/`operation:`), that the seed is 11, and prints your
score. `--replay` reruns your scenario and compares every graded value
— VERIFIED means your numbers reproduce. The replay first downs any
same-name sim so it always starts cold (a reused sim keeps its world
epoch, which shifts every seeded draw), and runs additive
(`--shared --slot-base any`) so it works beside other live sims.

## Scoring (0–100)

| component | weight | source assert |
|---|---|---|
| POD over `search_block` | 60 | `pod_mean` × 60 |
| Time to find `person_meadow` | 25 | `max(0, 1 − ttd/120 s)` × 25 |
| Stream honesty | 15 | `detection_precision` × 15 |

Precision is where hopeful detectors lose: every clutter ghost and
every "found" `person_barn` costs real points. Per-person POD
(`pod_at`), recall and the second TTD are reported for information —
ties break on `person_wood`'s POD, then `person_meadow`'s TTD.

## The baseline

`p5_canopy_sweep.deploy.json` — three lawnmower legs at 40 m — is the
reference entry (`<handle>` = `baseline`). Beat it.

## Scoreboard

`docs/SCOREBOARD.md` — append your verifier output row once a
`--replay` run comes back VERIFIED.
