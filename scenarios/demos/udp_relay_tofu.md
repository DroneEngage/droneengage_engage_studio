# Public UDP relay (TOFU) — real-GCS walkthrough

One copter SITL behind the real DroneEngage servers with the public
UDP relay in **tofu** mode. You drive a real QGC / Mission Planner
through the whole operator story: first lock, the flickering badge,
approve, switch back, reject, reject longer, a GCS restart, the 30 s
idle takeover, kick, and change port.

## Before you start

- `./sim run scenarios/demos/udp_relay_tofu.yaml` — the sim stays up
  for two hours. The banner prints the webclient deep link
  (`webclient:` line) and the unit comes up online + telemetry resumed.
- **Check the comm server's public_host first**: `server.config.local`
  must advertise an address the unit can dial (for this sim everything
  is loopback, so `127.0.0.1` is fine; on real deployments it must be
  the public IP or the unit never reaches comm).
- The relay's GCS port (socket2) shows in
  `./sim ctl -s udp_relay_tofu relay list` — point QGC's UDP (MAVLink)
  connection at `127.0.0.1:<socket2>`.
- Open the webclient deep link and log in with
  `sim@sim.droneengage.com` / `sim1234` — the udp-proxy gadget is on
  the copter panel.

## The walkthrough

Each step: what to do, what you should see, and where the screenshot
goes. Steps that need a real GCS are marked **[HUMAN]** — the agent
could not run them (no QGC on this machine).

| # | Step | Expected | Status |
|---|------|----------|--------|
| 1 | Connect QGC #1 to socket2 | First source locks the pair (tofu); QGC gets telemetry. | [HUMAN] not run — no real QGC here |
| 2 | Connect QGC #2 (second machine or second address, e.g. `127.0.0.2`) | Badge flickers in the webclient (source IP + `+N more`); QGC #2 gets nothing. **Screenshot: badge flicker.** | [HUMAN] not run |
| 3 | Click **Approve** | QGC #2 takes the lock; QGC #1 goes quiet. **Screenshot: approve.** | [HUMAN] not run |
| 4 | Click **Approve** again when QGC #1's attempt shows | The lock swings back to QGC #1. | [HUMAN] not run |
| 5 | Click **Reject** on a knocking source | Attempt stops for ~10 s, then a fresh attempt appears. **Screenshot: reject.** | [HUMAN] not run |
| 6 | Reject with the **longer** checkbox | Attempts suppressed for ~10 min. | [HUMAN] not run |
| 7 | Restart QGC (same machine, new source port) | Same-IP return takes the lock automatically — quiet `same_ip` line, no badge. | [HUMAN] not run |
| 8 | Close QGC #1 entirely; keep QGC #2 knocking | After ~30 s silence the lock auto-switches — takeover notice + **Kick** shows. **Screenshot: takeover notice + Kick.** | [HUMAN] not run |
| 9 | Click **Kick** | Lock frees; the next knock (the original GCS if restarted) takes it back. | [HUMAN] not run |
| 10 | Click **Change port** (confirm) | Relay rebinds socket2; the stray keeps talking to the dead port; the first GCS on the **new** port locks in. **Screenshot: change port.** | [HUMAN] not run |

## Notes for the human to fill in

- **Real heartbeat rate**: QGC sends ___/s, Mission Planner sends ___/s
  (the fake sends 1 Hz — a faster real rate changes how quickly
  attempts and takeovers appear).
- **Restart behaviour**: does a real QGC restart keep the same UDP
  source port (then it counts as the *same* peer and stays locked) or
  take a new one (then it is a same-IP takeover)?
- **When dropped** (rejected / port changed): does QGC retry quietly,
  pop an error, or hammer?

## Related

- Automated versions of every case here live in
  `tests/udp_relay/relay_*.yaml` (`./sim run <file>`) — same flows with
  scriptable fakes.
