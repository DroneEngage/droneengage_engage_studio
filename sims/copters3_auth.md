# copters3_auth — how to run

The copters3 fleet through the full login path: a spawned
`droneengage_authenticator` (single account `sim@sim.droneengage.com` /
`sim1234`) plus the real `droneengage_server`. Every unit's de_comm and
the GCS bridge log in. Use it when the feature under test involves the
auth handshake rather than the local-mode shortcut.

## Run

```bash
./sim up sims/copters3_auth.yaml
./sim ctl -s copters3_auth status
./sim ctl -s copters3_auth web --print
```

The deep link carries `?email&accesscode&authserver=...` — the WebClient
does the normal `/w/wl` login against the spawned authenticator.

Logins are limited to 10/min per IP and every unit is 127.0.0.1 —
repeated `up`/restarts in a tight loop can hit the limiter; give it a
minute.

## Teardown

```bash
./sim down copters3_auth
```

## Ports

slot_base 40: copter1 sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400); copter2 +10; copter3 +20. Server wss 62008, control
62009, authenticator 62018.
