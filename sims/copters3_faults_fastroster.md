# copters3_faults_fastroster — how to run

`copters3_faults` with a faster roster cadence: `sv_roster_hz: 1` +
`sv_peer_lost_s: 8` — a cut unit turns LOST ~8–10 s after silence
instead of ~30 s, so lost-member / peer_lost gate tests do not have to
outwait a whole loiter.

## Run

```bash
./sim up sims/copters3_faults_fastroster.yaml
./sim ctl -s copters3_faults_fastroster status
```

Then fault as usual:

```bash
./sim ctl -s copters3_faults_fastroster link_cut copter2 60
./sim ctl -s copters3_faults_fastroster roster 20 3   # watch LOST flip
```

## Teardown

```bash
./sim down copters3_faults_fastroster
```

## Ports

slot_base 40: copter1 sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400); copter2 +10; copter3 +20. Fake server wss 62008,
control 62009.
