# copters2 — how to run

Two quad copters (copter1..copter2, 20 m apart) on a spawned real
`droneengage_server` in local mode — no authenticator. The smaller
copters3 for scenarios that only need a pair.

## Run

```bash
./sim up sims/copters2.yaml
./sim ctl -s copters2 status
./sim status -s copters2 --open-web
```

SITL needs ~1–2 min of GPS/EKF warm-up before `arm` succeeds.

## Teardown

```bash
./sim down copters2
```

## Ports

slot_base 40: copter1 sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400); copter2 +10. Server wss 62008, control 62009. Cannot
run alongside another slot-40 sim — use `--with slot30` in that case.
