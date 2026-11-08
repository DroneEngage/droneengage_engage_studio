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

Every port is leased from the machine-wide allocator at `up` — `./sim
ports -s copters2` lists what was actually bound. `slot_base: 40` is
identity only (sysids 40..41); two slot-40 sims can now run side by side
since ports never clash — use `--with slot30` only if you also want
distinct sysids.
