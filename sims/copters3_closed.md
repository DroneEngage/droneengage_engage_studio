# copters3_closed — how to run

Three quad copters in a **closed environment** (P4A-10 scenario 4), on
the fake server:

- No GPS on any SITL — `SIM_GPSx_ENABLE 0`, `GPSx_TYPE 0`, the FC flies
  on the SIM AHRS (`AHRS_EKF_TYPE 10`).
- The fleet server stamps a clock 3 h wrong (`clock_offset_ms:
  10800000`).
- copter3 never receives a clock stamp (`noclock: [copter3]`) so it
  stays tq=0 from boot.

slot_base 30 keeps clear of SITL instances on the default 5760+10k
range.

## Run

```bash
./sim up sims/copters3_closed.yaml
./sim ctl -s copters3_closed status
```

Arming needs the usual warm-up, and with no GPS expect MANUAL-style
arming only (no GPS-dependent modes).

## Teardown

```bash
./sim down copters3_closed
```

## Ports

slot_base 30: copter1 sysid 30 (pilot 6060, fcb 7960, de_comm 62300,
de_mavlink 63300); copter2 +10; copter3 +20. Fake server wss 62008,
control 62009.
