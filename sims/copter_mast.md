# copter_mast — how to run

One quad copter plus a comm-only static unit (`mast1`, "SDR mast", 30 m
north) on the real server — the TASK-P4A-11 acceptance-9 fleet: a flying
role next to a static sensor-lane role. mast1 has no FC, never arms and
is never sent a nav command; its package is a sensor mission only.

## Run

```bash
./sim up sims/copter_mast.yaml
./sim ctl -s copter_mast status
./sim status -s copter_mast --open-web
```

## Teardown

```bash
./sim down copter_mast
```

## Ports

Every port is leased from the machine-wide allocator at `up` — `./sim
ports -s copter_mast` lists what was actually bound. `slot_base: 40`
is identity only (copter1 sysid 40; mast1 has no FC).
