# copters3_static_faults — how to run

Three quad copters plus a de_comm-only static unit (`static1`, 30 m
north — no FC, never arms) on the fake server: seeded per-link
loss / delay / cut plus the clock-offset controls. The fleet for the
P4A-09 two-phase deploy test — the watcher role needs a static unit and
the clock skew needs the fake server.

## Run

```bash
./sim up sims/copters3_static_faults.yaml
./sim ctl -s copters3_static_faults status
./sim ctl -s copters3_static_faults link_cut static1 30
```

## Teardown

```bash
./sim down copters3_static_faults
```

## Ports

slot_base 40: copter1 sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400); copter2 +10; copter3 +20; static1 de_comm 62430.
Fake server wss 62008, control 62009.
