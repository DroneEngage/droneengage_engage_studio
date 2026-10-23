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

Every port is leased from the machine-wide allocator at `up` — `./sim
ports -s mixed_vehicles` lists what was actually bound.
`slot_base: 40` is identity only: copter1/plane1/rover1 get sysids
40/41/42; ground1 has no FC.
