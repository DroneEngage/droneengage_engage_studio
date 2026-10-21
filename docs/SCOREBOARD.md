# SAR challenge scoreboard

Season world `worlds/challenge_sar.yaml`, seed 11 — see
`docs/CHALLENGE.md` for the contract and scoring. Entries are appended
after a `tools/verify_challenge.py --replay` VERIFIED.

| handle | pod_mean | ttd meadow | precision | score | verified |
|---|---|---|---|---|---|
| baseline | 0.984 | 40.4 s | 1.0 | 90.6 | yes |

Baseline notes: `person_wood` (POD 0.905 over 11 looks through the
canopy gap) was never detected — a 0.9-POD cell still misses ~10 % of
the time; `person_barn` stayed invisible as designed (POD 0.000, all
looks terminate inside the building). The winning margin is in the
wood: more oblique looks, lower transit, or a second pass lifts that
POD above what a single 40 m sweep buys.
