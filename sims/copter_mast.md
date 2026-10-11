# copter_mast — how to run

One quad copter plus a comm-only static unit (`mast1`, "SDR mast", 30 m
north) on the real server — the TASK-P4A-11 acceptance-9 fleet: a flying
role next to a static sensor-lane role. mast1 has no FC, never arms and
is never sent a nav command; its package is a sensor mission only.

## Run

```bash
./sim up sims/copter_mast.yaml
./sim ctl -s copter_mast status
./sim ctl -s copter_mast web --print
```

## Teardown

```bash
./sim down copter_mast
```

## Ports

slot_base 40: copter1 sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400); mast1 de_comm 62410. Server wss 62008, control 62009.
