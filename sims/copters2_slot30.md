# copters2_slot30 — how to run

Two quad copters on a spawned real `droneengage_server` — identical to
`copters2` but at slot_base 30 (pilot ports 6060+), for when the
developer's own SITL sessions or another sim already hold the default
5760+10k / slot-40 range. Units also get `sv_accel_s: 8` (the default
3 s allowance only covers spin-up; SITL copters decelerate ~5 s on
waypoint approach at 10 m/s).

## Run

```bash
./sim up sims/copters2_slot30.yaml
./sim ctl -s copters2_slot30 status
./sim ctl -s copters2_slot30 web --print
```

## Teardown

```bash
./sim down copters2_slot30
```

## Ports

slot_base 30: copter1 sysid 30 (pilot 6060, fcb 7960, de_comm 62300,
de_mavlink 63300); copter2 +10. Server wss 62008, control 62009.
