# udp_relay scenarios

Each scenario is a file in `tests/udp_relay/`. All use the tofu relay
(`gcs_peer_policy: tofu`) with one real copter, except where noted.
Each entry names the case that the scenario proves.

## Idle takeover, kick and restart (SL-0181)

| File | Case it proves |
|---|---|
| `relay_idle_takeover.yaml` | A locked GCS that is quiet for the idle-release window loses the lock to the next knock (reason `idle`). Telemetry moves to the new GCS. |
| `relay_short_silence.yaml` | A GCS that is quiet for less than the idle window keeps the lock. A stranger only raises an attempt. Telemetry resumes to the locked GCS when it returns. |
| `relay_takeover_in_flight.yaml` | The same idle takeover while the copter is armed. The switch is automatic and the flight is not affected. |
| `relay_kick_takeback.yaml` | After a takeover, a kick frees the lock and the original QGC takes it back. |
| `relay_restart_locked.yaml` | The relay restarts with a live GCS. The lock survives, a stranger gets nothing, and telemetry resumes. |
| `relay_restart_free.yaml` | The relay restarts with the GCS gone. The restored pin counts as just heard, so the lock frees one idle window after the restart. The waiting stranger then takes it. |

## Lock, approve, reject and timeout (SL-0180)

| File | Case it proves |
|---|---|
| `relay_lock_stranger.yaml` | The first GCS locks the pair. A stranger gets no telemetry and cannot arm the drone or write a mission. |
| `relay_same_ip_return.yaml` | The locked GCS restarts on the same IP with a new source port. The lock moves to it at once (reason `same_ip`) without an operator. |
| `relay_approve_switchback.yaml` | The operator approves a knocking stranger. Telemetry moves and the old GCS is cut. The old GCS asks again and is approved back. |
| `relay_attempt_timeout.yaml` | Nobody answers a stranger for the 15 s request window. The locked GCS keeps telemetry and the stranger gets nothing. |
| `relay_reject_10s.yaml` | The operator rejects a knocking GCS. No new attempt is raised while the block holds, also not from a new port on the same IP. A fresh attempt appears after the block ends. |
| `relay_reject_long.yaml` | "Reject longer" blocks attempts for the set duration, also from a new source port on the same IP. |

## Flood and change port (SL-0182)

| File | Case it proves |
|---|---|
| `relay_flood_msgs.yaml` | A flood of sources from many 127.0.0.x IPs raises one waiting request with a growing `others` count and few operator messages. The locked GCS keeps telemetry. |
| `relay_flood_reject_bound.yaml` | The reject list stays within its maximum while a flood keeps the pair rejecting expired attempts. |
| `relay_change_port_stray.yaml` | After `change_port`, the stray source talks to a dead port and raises no more attempts. |
| `relay_change_port_lock.yaml` | After `change_port`, the first GCS on the new port is locked with no IP restriction. The drone-side link is not affected and telemetry keeps flowing. |

## Cross account (SL-0184)

| File | Case it proves |
|---|---|
| `relay_cross_account.yaml` | A GCS of another account never hears attempt or switch pushes. Its approve returns `not_allowed`, and the lock of the real account stays. |

## Earlier relay scenarios

| File | Case it proves |
|---|---|
| `relay_telemetry.yaml` | One copter's MAVLink reaches the GCS through the spawned relay. The unit advertises a port in the relay port range, and FC HEARTBEATs flow back to the GCS. |
| `relay_reject_injection.yaml` | With `gcs_peer_policy: allowlist`, a stranger cannot inject MAVLink (ARM command and mission upload) through the GCS side. The relay drops and counts the datagrams. |
