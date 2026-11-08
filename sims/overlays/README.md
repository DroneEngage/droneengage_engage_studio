# sims/overlays — one twist applied on top of a base sim

An overlay is a partial sim file applied with
`./sim <show|up|run|view> <sim.yaml> --with a,b` (or `SIM_WITH=a,b`, or a
spec's own `with:`, or a scenario's `sim: {base: <sim>, with: [...]}`).
The overlay wins: `server:`/`home:`/`slot_base:`/`render:`/`speedup:` are
deep-merged, `units:` are appended, `config:` deep-merges into every unit
entry's `config:`, `fc:` likewise into every `fc:`, and `with:` pulls
more overlays (applied after the overlay's own keys, in file order).

Each applied overlay name appends to the sim's derived name
(`copters3 --with faults` → `copters3_faults`) and the overlay keys are
part of the sim hash, so a variant gets its own `var/run/` dir and never
reuses a different variant's warm sim.

| overlay | changes |
|---|---|
| `auth` | `server: {type: real, auth: true}` + the shared sim account — the full login path: authenticator + real server; every de_comm and the GCS bridge log in. |
| `closed` | closed environment (P4A-10): no GPS on any SITL (`SIM_GPSx_ENABLE 0`, `GPSx_TYPE 0`, `AHRS_EKF_TYPE 10`), fake server stamps a clock 3 h ahead (`clock_offset_ms`), `copter3` gets no clock stamp (`noclock` → tq=0), `slot_base: 30`. |
| `fastroster` | `config:` push on every unit: `de_comm.sv_roster_hz 1` + `sv_peer_lost_s 8` — a cut unit turns LOST ~8–10 s after silence instead of ~30 s. |
| `faults` | `server: {type: fake, seed: 1337}` — the fake fleet server: seeded per-link loss / delay / cut for link-fault tests (the real server cannot inject those). |
| `slot30` | `slot_base: 30` for distinct sysids when another slot-40 sim shares the machine (ports never clash — the allocator leases them), plus `de_comm.sv_accel_s 8` (SITL copters decelerate ~5 s on waypoint approach). |
| `static` | `units:` append — a `comm_only` static (`static1`, no FC, 30 m south) for tests that need a watcher role (e.g. the P4A-09 two-phase deploy). |

Replacements for the removed near-duplicate sims:

| removed file | use instead |
|---|---|
| `sims/copters2_faults_fastroster.yaml` | `copters2 --with faults,fastroster` |
| `sims/copters2_slot30.yaml` | `copters2 --with slot30` |
| `sims/copters3_auth.yaml` | `copters3 --with auth` |
| `sims/copters3_closed.yaml` | `copters3 --with closed` |
| `sims/copters3_faults.yaml` | `copters3 --with faults` |
| `sims/copters3_faults_fastroster.yaml` | `copters3 --with faults,fastroster` |
| `sims/copters3_static_faults.yaml` | `copters3 --with static,faults` |
