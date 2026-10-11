# copters3 — how to run

Three quad copters (copter1..copter3, 20 m apart) on a spawned real
`droneengage_server` in local mode — no authenticator. The default fleet
for mission / supervisor tests; most `tests/` scenarios either extend it
or assume this shape.

## Run

```bash
./sim up sims/copters3.yaml
./sim ctl -s copters3 status
./sim ctl -s copters3 web --print     # WebClient deep link (needs the
                                    # dev server on https://localhost:3000)
```

SITL needs ~1–2 min of GPS/EKF warm-up before `arm` succeeds:

```bash
./sim ctl -s copters3 arm copter1
./sim ctl -s copters3 mode copter1 GUIDED
./sim ctl -s copters3 takeoff copter1 20
```

## Teardown

```bash
./sim down copters3        # or ./sim ctl -s copters3 land all first:
                           # an airborne fleet reused next run drains
                           # battery and trips battery_low checks
```

## Ports

slot_base 40: copter1 sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400); copter2 +10; copter3 +20. Server wss 62008, control
62009.
