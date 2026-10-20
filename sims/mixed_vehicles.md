# mixed_vehicles — how to run

One of each vehicle type plus a ground node, on the spawned real
server: copter1 (quad), plane1 (fixed-wing, 60 m east), rover1 (40 m
west), ground1 (de_comm only, 30 m north). For fleet-mix scenarios and
the mixed-vehicle bring-up regression.

## Run

```bash
./sim up sims/mixed_vehicles.yaml
./sim ctl -s mixed_vehicles status
./sim status -s mixed_vehicles --open-web
```

Arm differs per vehicle — the plane arms in MANUAL, the rover arms in
HOLD; `land` on the plane resets its SITL (it cannot land in place).

## Teardown

```bash
./sim down mixed_vehicles
```

## Ports

slot_base 40: copter1 sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400); plane1 +10; rover1 +20; ground1 de_comm 62430. Server
wss 62008, control 62009.
