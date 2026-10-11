# copters3_faults — how to run

The copters3 fleet on the **fake** fleet server
(`engine/fake_fleet_server.py`) — seeded per-link loss / delay / cut
that the real server cannot inject. Base for the link-fault tests.

## Run

```bash
./sim up sims/copters3_faults.yaml
./sim ctl -s copters3_faults status
./sim ctl -s copters3_faults web --print
```

Fake-server-only ctl commands work here (they are refused on `type:
real`):

```bash
./sim ctl -s copters3_faults link_cut copter2 30   # in-band cut, 30 s
./sim ctl -s copters3_faults loss copter2 25       # 25 % message loss
./sim ctl -s copters3_faults kick copter2
./sim ctl -s copters3_faults link_clear copter2
./sim ctl -s copters3_faults clock offset 5000     # server clock skew
./sim ctl -s copters3_faults links                 # active rules
```

Caveat: the fake server stamps `sd` itself, so it is not proof for
sender-identity behaviour — use a `type: real` sim for that.

## Teardown

```bash
./sim down copters3_faults
```

## Ports

slot_base 40: copter1 sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400); copter2 +10; copter3 +20. Fake server wss 62008,
control 62009.
