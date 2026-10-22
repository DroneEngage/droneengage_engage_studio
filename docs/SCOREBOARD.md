# SAR challenge scoreboard

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

Season world `worlds/challenge_sar.yaml`, seed 11 — see
`docs/CHALLENGE.md` for the contract and scoring. Entries are appended
after a `tools/verify_challenge.py --replay` VERIFIED. Only a
`--replay` VERIFIED counts as verified: a bare `verify_challenge.py`
run (STRUCTURE OK) checks structure and self-reported numbers only and
never earns a row.

| handle | pod_mean | ttd meadow | precision | score | verified |
|---|---|---|---|---|---|
| baseline | 0.985 | 40.4 s | 1.0 | 90.7 | yes |

Baseline notes: `person_wood` (POD 0.924 over 12 looks through the
canopy gap) was never detected — a 0.92-POD cell still misses ~8 % of
the time; `person_barn` stayed invisible as designed (POD 0.000, all
looks terminate inside the building). The winning margin is in the
wood: more oblique looks, lower transit, or a second pass lifts that
POD above what a single 40 m sweep buys.
