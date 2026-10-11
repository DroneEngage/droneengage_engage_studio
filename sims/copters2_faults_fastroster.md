# copters2_faults_fastroster — how to run

`copters2` on the fake server with the fast roster cadence
(`sv_roster_hz: 1` + `sv_peer_lost_s: 8`): a cut unit turns LOST ~8–10 s
after silence instead of ~30 s, so the P4A-11 peer_lost pair test does
not have to outwait a whole loiter.

## Run

```bash
./sim up sims/copters2_faults_fastroster.yaml
./sim ctl -s copters2_faults_fastroster status
./sim ctl -s copters2_faults_fastroster link_cut copter2 60
```

## Teardown

```bash
./sim down copters2_faults_fastroster
```

## Ports

slot_base 40: copter1 sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400); copter2 +10. Fake server wss 62008, control 62009.
