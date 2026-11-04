# DroneEngage Studio — User Guide

Fly simulated drones on your own computer. No drone, no radio, no risk.

This tool builds a pretend world: simulated vehicles (copter, plane,
rover) running the **real** DroneEngage software, talking to a **real**
DroneEngage server — all on one machine. You can watch the fleet, fly
missions, break things on purpose, and check that everything behaves.

You only need to remember one command: `./sim`.

---

## 1. What it is, in plain words

Three ideas, from small to big:

| Piece | Think of it as | Where it lives |
|---|---|---|
| **Block** | A recipe for *one* vehicle — "a copter with a camera", "a ground antenna". | `blocks/` |
| **Sim** | A fleet — "3 copters + 1 fixed camera, talking to my server". | `sims/` |
| **Test / Scenario** | A script — "take off, fly a square, then come home" — plus checks that it worked. | `tests/` (automatic), `scenarios/` (for humans) |

When you start a sim, the tool launches real programs: the ArduPilot
simulator (called **SITL** — "software in the loop") for each vehicle,
the DroneEngage modules each vehicle carries, and a DroneEngage server.
Everything talks over your machine's local network (127.0.0.1), so it
needs no internet and no root access.

---

## 2. Before you start — the shopping list

The tool itself needs no install, but it launches things that must
already exist on your machine:

| You need | How to get it | Needed for |
|---|---|---|
| Python 3 packages | `pip install websockets pymavlink pyyaml` | everything |
| ArduPilot SITL built | in your ardupilot checkout: `./waf configure --board sitl && ./waf copter plane rover` | any flying vehicle |
| `de_comm` + `de_ardupilot` binaries | `./build.sh` in their repos (`droneengage_communication`, `droneengage_mavlink`) | every unit |
| Rust toolchain + `cargo build --release` | via rustup, then in `droneengage_server` (and `droneengage_authenticator` for login tests) | the server |

Other DroneEngage modules (camera, tracker, ...) are only needed if a
block uses them. If something is missing, `./sim up` stops early and
tells you exactly which binary it could not find.

---

## 3. One-time setup (2 minutes)

```bash
cp sim.local.env.example sim.local.env
```

Open `sim.local.env` and check two paths:

- `DE_CODE` — the folder that holds your DroneEngage checkouts
  (default `~/de_code`).
- `ARDUPILOT_DIR` — your ardupilot checkout with SITL built
  (default `~/code/ardupilot`).

Everything else in that file is optional overrides — repo paths that
live somewhere unusual, extra binaries (camera, GPIO, precision
landing), the 3D viewer path, and login accounts for your own servers.
The file is git-ignored, so it stays yours.

Now run `./sim doctor` once. It prints one OK / MISSING / OPTIONAL line
per thing the sim needs — binaries, repos, Python packages, ports —
and for every MISSING line it tells you the one command that fixes it.
`./sim doctor sims/<name>.yaml` checks only what that sim actually
needs (e.g. no camera checks for a copters-only sim). When every line
is OK or OPTIONAL, you are ready.

---

## 4. Try it in 5 minutes

```bash
./sim studio sims/copters3.yaml     # the human session: fleet + planner + 3D viewer
```

`studio` is the everyday way to *sit at* a sim. One command brings the
fleet up (reusing it if it is already running), opens the WebClient
planner logged into the sim, starts the Mavlink3DMap 3D viewer when it
is installed (it prints a line and simply skips the viewer when it is
not), and prints one status block — links, units online, record file —
then parks until Ctrl-C. The fleet stays up afterwards; add `--down` to
tear it down on exit, `--world <name>` to seed a world for the viewer
(`worlds/<name>.yaml`), or `--record <file.yaml>` to write the session
out as a replayable scenario (see §10).

To watch an automated test run instead:

```bash
./sim list                          # see everything you can run
./sim run tests/smoke_copters.yaml  # the "hello world": 3 copters fly a square
./sim down                          # stop everything when done
```

What you will see: the tool builds a fleet of 3 simulated copters,
starts a real DroneEngage server, sends each copter a square mission,
arms them, starts the mission, and waits for each one to finish. At the
end it prints a PASS/FAIL summary and lands the fleet.

**Be patient on the first run:** a simulated vehicle needs about 1–2
minutes of "GPS warm-up" before it can arm, just like a real drone.
Later runs on the same fleet reuse it, so they start faster.

---

## 5. The `./sim` command — everyday cheatsheet

| Command | Plain words |
|---|---|
| `./sim list` | Show every runnable file with a one-line description. |
| `./sim blocks` | List the vehicle recipes you can use. |
| `./sim show sims/copters3.yaml` | Preview a fleet (units, ports) without starting anything. |
| `./sim studio <sim>` | The human session: fleet up + webclient planner + 3D viewer (+ `--world` / `--record` / `--down`) until Ctrl-C. |
| `./sim up sims/copters3.yaml` | Start a fleet and leave it running. |
| `./sim sims` | Which fleets are up and how many processes are alive. |
| `./sim status` | Live processes + which units the server sees online. |
| `./sim ctl ...` | Operator console — remote-control the fleet by hand (see §7). |
| `./sim run <file>` | Run a test or scenario to the end, then tear down. **Owns the machine** — stops other sims first. |
| `./sim run <file> --keep` | Same, but leave the fleet up afterwards so you can poke it. |
| `./sim view <file>` | Run a demo **and** open the 3D viewer; stays up until Ctrl-C. Same shared/exclusive rules as `run`. |
| `./sim suite tests` | Run a whole folder of tests (the nightly run). Owns the machine. |
| `./sim new my_demo` | Create a starter scenario file you can edit. |
| `./sim record <sim> -o <file>` | Record a live WebClient session into a replayable scenario (see §10). |
| `./sim validate <file>` | Check a file for mistakes — starts nothing. |
| `./sim logs <unit> <module>` | Tail a unit's log, e.g. `./sim logs copter1 de_comm`. |
| `./sim down` | Stop all sims. `./sim down copters3` stops just one. |
| `./sim clean` | Delete old run/results folders (add `--dry-run` to preview). |

Running `./sim` alone prints this same list.

**On a shared machine:** interactive commands are additive by default —
`up`, `studio` and a `run`/`view` of a bare `sims/*.yaml` never stop a
fleet that is already running. Only a `run`/`view`/`suite` of a file
with `steps:` owns the machine (stops the other sims first), because a
test needs a clean machine — that is the CI posture. Pass `--shared` on
the command line, or export `SIM_SHARED=1`, and even scenario runs stay
additive next to live sims.

---

## 6. The three ways people use it

### a) "I want to watch drones fly" — demos

```bash
./sim view scenarios/demos/world_canopy_viewer.yaml
```

This runs a search demo — a copter sweeping a forest looking for a
hiker — and opens the Mavlink3DMap 3D viewer in your browser. It prints
the URL to open (`http://127.0.0.1:8070/...` — the port is `8070` plus
the sim's slot base). Press Ctrl-C when done; the tool lands everything
cleanly.

The viewer needs `MAVLINK3DMAP_DIR` set in `sim.local.env` to a
Mavlink3DMap checkout.

### b) "I want my own fleet, up and waiting" — `up`

```bash
./sim up sims/quadcopter.yaml      # start it
./sim sims                         # is it alive?
./sim ctl status                   # who is online?
./sim down quadcopter              # stop it
```

You can run **several fleets at once** — they never fight over ports
(`./sim up` refuses to start one that clashes), and unit names may even
repeat between fleets: each sim spawns its own server, so `quad1` in one
sim is invisible to the other. Names still must be unique when fleets
share a server stack *you* run (`server: {type: external}`) or when
either fleet renders cameras (the virtual video devices are
machine-global). The ready-made
`sims/quadcopter.yaml`, `sims/rover.yaml` and `sims/plane.yaml` are
built for this; they connect to a server stack *you* run (see §8).

### c) "I want to check something works" — tests

```bash
./sim run tests/smoke_copters.yaml        # do 3 copters complete a mission?
./sim run tests/fault_link_loss.yaml      # does the fleet survive a bad link?
./sim suite tests --seeds 1,2,3           # everything, 3 times each
```

Exit code 0 = everything passed. Results (logs, journals, a summary)
are saved under `var/results/<timestamp>-<name>/` so you can dig into
what happened after the fact.

---

## 7. Remote-control a running fleet (`./sim ctl`)

While a sim is up, `ctl` is your hand on the controls — the same things
a GCS operator would do:

```bash
./sim ctl status                 # overview: units, ports, online state
./sim ctl arm copter1            # arm one unit
./sim ctl arm all                # arm the whole fleet
./sim ctl mode copter2 RTL       # change flight mode
./sim ctl land all               # bring everything down
./sim ctl journal copter1        # read what a unit has been doing
./sim ctl deploy packages/copter_square.json   # upload a mission
./sim ctl start                  # start it
./sim logs copter1 de_comm       # watch a module's log file
```

Run `./sim ctl` with no arguments for the full list. A *unit name* can
be the name (`copter1`), its position (`2`), `block:copter` (every
copter), a comma list, or `all`. With several sims up, `-s <sim>`
chooses one: `./sim ctl -s quadcopter status`.

---

## 7b. The public UDP relay — drive it with a real GCS

`scenarios/demos/udp_relay_tofu.yaml` brings one copter up with the
real servers and the public UDP relay in **tofu** mode and then just
stays up, so you can plug a real QGC or Mission Planner into the
relay's GCS port and walk the whole operator story: first lock, the
flickering badge, approve, switch back, reject, reject longer, a GCS
restart, the 30-second idle takeover, kick, and change port.

```bash
./sim run scenarios/demos/udp_relay_tofu.yaml
./sim ctl -s udp_relay_tofu relay list     # the socket2 port to point QGC at
```

The runbook `scenarios/demos/udp_relay_tofu.md` is the step-by-step
checklist (what to connect, what you should see, where screenshots
go, and the notes to fill in on real heartbeat rates and restart
behaviour). The same flows are automated in
`tests/udp_relay/relay_*.yaml`.

## 8. Simulating on your own DroneEngage server

By default a sim spawns its own private server — perfect for tests,
invisible to the outside world. But if you already run a DroneEngage
server stack (e.g. `servers/sh_start_servers.sh`) and want simulated
vehicles to show up in **your real WebClient**, use a sim with
`server: {type: external}` — that's what `sims/quadcopter.yaml`,
`sims/rover.yaml` and `sims/plane.yaml` are.

```bash
# start your servers first, then:
./sim up sims/quadcopter.yaml --user me@example.com --password secret
```

- `--user` / `--password` make every unit (and the sim's control
  bridge) log in with that account. The account must exist on your
  authenticator.
- `--auth-server <host[:port]>` points at a specific authenticator.
- Instead of the command line you can put `SIM_AUTH_USER` /
  `SIM_AUTH_CODE` (and `SIM_UNIT_AUTH_*` for the units) in
  `sim.local.env`.
- **Same team rule:** the unit accounts and the account you use in the
  WebClient must be in the same team, or the drones stay invisible.
- A unit account (unit permission only) can't be used as the GCS/bridge
  login — that one needs the GCS permission.

Now open your WebClient — the simulated quad appears like real hardware,
and you can fly it from the normal UI.

---

## 9. Writing your own scenario (the friendly version)

A scenario is one YAML file: a fleet + a timeline of things that
happen + checks at the end.

```bash
./sim new my_first                     # creates scenarios/my_first.yaml
./sim validate scenarios/my_first.yaml # check it (starts nothing)
./sim run scenarios/my_first.yaml      # run it
```

A minimal working file looks like this — one copter flies a square on
the real server:

```yaml
# my_first - one copter flies a square mission and lands.
version: 1
name: my_first
transport: real                        # use the real comm server
operation: ../packages/copter_square.json
settle_s: 20
fleet:
  - {id: c1, cls: copter}              # one copter named c1
timeline:
  - {t: 0, do: mode, unit: c1, mode: GUIDED}  # must be GUIDED to load missions
  - {t: 2, do: deploy}                  # send the mission
  - {t: 6, do: arm, unit: c1}           # arm (waits for GPS warm-up)
  - {t: 9, do: start}                   # go!
  - {t: 12, do: wait_journal, unit: c1, re: "mission main completed",
     timeout: 300}                      # wait until it finishes
assert:
  - journal_count: {unit: c1, re: "mission main completed", equals: 1}
teardown: land                          # always land at the end
```

Read it like a recipe: `t` is seconds after the start, `do` is the
action, `assert` is what must be true at the end. There are many more
steps (kill a process, cut a link, fire an event) and many more
assertions — `docs/SCENARIOS.md` is the complete reference, written so
you can also paste it into an AI assistant and ask it to draft a
scenario for you.

Rules of thumb:

- The first line must be a `#` comment saying what the file proves.
- Flying scenarios end with `teardown: land` — a fleet left flying
  drains its battery and breaks the next run.
- Link tricks (packet loss, cutting the connection) need
  `transport: fake` — the fake server exists only for that.
- Want `de_mavlink` to reach the vehicle over a serial device like
  real hardware? Put `fcb_link: {kind: pty, baud: 57600}` on the unit —
  the sim bridges a virtual serial port into SITL and paces it at the
  real baud rate (`docs/SCENARIOS.md` has the details).
- Put your files in `scenarios/`; `tests/` is for cases the nightly
  automation runs.

---

## 10. Turn your session into a test — `./sim record`

You just flew a sim by hand in the WebClient — deployed a mission,
armed, hit start, watched it go. `./sim record` writes *that session*
down as a scenario file you can replay with `./sim run`:

```bash
./sim record sims/copters3.yaml -o scenarios/my_recording.yaml
# ... drive the WebClient (the command prints your deep links) ...
# Ctrl-C when done -> the scenario + a .md runbook are written
./sim validate scenarios/my_recording.yaml
./sim run scenarios/my_recording.yaml      # replays your session
```

What it captures: every command your WebClient sent, stamped at the
comm server the moment it arrived. On Ctrl-C those frames become
timeline steps — deploy/arm/mode/start and friends map to the same
verbs you'd write by hand (`./sim verbs` lists them); a deploy also
saves the package it carried next to the yaml. Anything with no verb
stays in the file as a `# unmapped:` comment so nothing is silently
lost — the step just doesn't replay until you give it one.

How it works, in one breath: recording adds a tiny capture plugin to
the *sim's own* comm server config (`command_plugin`, the seam TASK-24
chose) and restarts it — so it works on `server: {type: real}` sims.
Two limits to know:

- **`fake` server:** refused — it has no plugin seam.
- **`external` server (§8, e.g. `sims/quadcopter.yaml`):** the sim
  doesn't own your server. `record` writes the plugin to
  `var/run/<sim>/server/record_plugin.js` and prints the one line to
  add to *your* server's `command_plugin` — restart it and frames start
  landing. Don't? Then the file comes out empty.

You can also record *around* a scripted run — `--record` on
`run`/`view` captures whatever you do in the WebClient during the run
(including its `--hold` park) and writes the file when it ends:

```bash
./sim run scenarios/my_first.yaml --hold --record scenarios/what_i_did.yaml
```

The output is a normal scenario — edit it, fill in the `assert:` TODO
block (the one thing a recording can't guess is *what you were checking
for*), and commit it like any other test.

---

## 11. When things go wrong

| Symptom | Likely cause / fix |
|---|---|
| `missing binaries` on `sim up` | A module isn't built. The error names which one — run `./build.sh` in its repo. |
| `arm` waits forever / times out | Normal cold boot: SITL needs 1–2 min for GPS/EKF. `arm` already waits; make your `timeout` generous. |
| `port already in use` / refused start | Another sim (or your own server) owns the port. `./sim sims` shows what's live; `./sim down` clears it, or pick a different `slot_base` in the sim file. |
| Login fails on an `external` sim | Account doesn't exist on the authenticator, wrong team, or you hit the 10-logins/minute limit — wait a minute and retry. |
| `API endpoint not found` from the authenticator | The account name isn't valid — use alphanumeric or a real e-mail address. |
| Test fails with `battery_low` | A previous run left the fleet airborne. Always use `teardown: land`, or `./sim down` and re-`up`. |
| Everything looks stuck | `./sim down` then `./sim up <file> --fresh` gives you a clean slate. Check `./sim logs <unit> <module>` for the real error. |
| Leftover junk | `./sim clean --dry-run` to preview, `./sim clean` to delete old run/results folders (never touches a live sim). |

Where to look when debugging:

- `var/run/<sim>/` — everything the sim generated: configs, launch
  scripts, per-process **logs**, and each unit's **journal** (a
  timestamped diary of what it did — the first place to look).
- `var/results/<test>/` — a copy of everything for a finished test:
  journals, logs, `summary.json` (the verdict), `telemetry.jsonl` (the
  flight recording).

---

## 12. Words you'll bump into

| Word | Meaning |
|---|---|
| **SITL** | The ArduPilot simulator — a fake flight controller that flies like a real one. |
| **de_comm** | The DroneEngage module every unit runs; connects it to the server. |
| **de_mavlink** | The module that translates between SITL/the flight controller and DroneEngage. |
| **GCS** | Ground Control Station — the operator side (your WebClient, or the sim's built-in operator bot). |
| **Journal** | A per-unit diary file of important events; tests read it to check what happened. |
| **Block / sim / scenario** | Unit recipe / fleet / scripted rehearsal — see §1. |
| **slot_base** | The number that decides a sim's ports; give each parallel sim its own. |
| **fcb_link / pty** | A unit key that makes de_mavlink open a virtual serial device (into SITL through the fcb proxy) instead of a TCP connection — for testing serial-link behavior without real hardware. |
| **package / operation** | A compiled mission or job sent to the fleet (`packages/*.json`). |

---

## 13. Going deeper

- `README.md` — the full technical reference (blocks, config layers,
  ports, extending the engine).
- `docs/SCENARIOS.md` — every timeline step, fault, and assertion.
- `./sim ctl` — the operator console's own help.
