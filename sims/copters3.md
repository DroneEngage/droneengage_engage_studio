# copters3 — how to run

Three quad copters (copter1..copter3, 20 m apart) on a spawned real
`droneengage_server` in local mode — no authenticator. The default fleet
for mission / supervisor tests; most `tests/` scenarios either extend it
or assume this shape.

## Run

```bash
./sim up sims/copters3.yaml
./sim ctl -s copters3 status
./sim status -s copters3 --open-web   # WebClient deep link (needs the
                                      # dev server on https://localhost:3000)
```

SITL needs ~1–2 min of GPS/EKF warm-up before `arm` succeeds:

```bash
./sim ctl -s copters3 arm copter1
./sim ctl -s copters3 mode copter1 GUIDED
./sim ctl -s copters3 takeoff copter1 20
```

## Teardown

```bash
./sim down copters3        # or ./sim ctl -s copters3 land all first:
                           # an airborne fleet reused next run drains
                           # battery and trips battery_low checks
```

## Ports

Every port is leased from the machine-wide allocator at `up` — `./sim
ports -s copters3` (or `var/run/copters3/sim.json`) lists what was
actually bound, and released ports are preferred back on the next run.
`slot_base: 40` is identity only: copter1..3 get sysids 40..42 and never
affect the leased ports.

## Variants (`--with` overlays)

The old near-duplicate sim files are now `sims/overlays/<name>.yaml`
twists on this base — apply with `./sim <cmd> sims/copters3.yaml --with
a,b` (comma list, order matters: later overlays win; the derived name
appends them, e.g. `copters3_faults_fastroster`):

- `faults` — fake fleet server (seeded link loss/delay/cut)
- `fastroster` — faster roster loss detection (~8–10 s)
- `static` — adds `static1`, a de_comm-only unit (watcher role)
- `closed` — closed environment: no GPS, clock 3 h off, slot_base 30
- `auth` — real server + authenticator (full login path)
- `slot30` — sysids 30..32 when the default ones clash (identity only;
  ports are still leased)

A scenario picks the same fleet with `sim: {base: copters3, with:
[faults]}` instead of a `fleet:` block. See
[sims/overlays/README.md](overlays/README.md) for the full table.
