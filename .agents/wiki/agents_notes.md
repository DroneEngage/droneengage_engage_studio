# droneengage_studio — invariants, gotchas & verification

## Invariants and gotchas

- **Only kill our own pids.** `simlib.pid_is_ours` matches only command
  lines containing `RUN_DIR`/`ENGINE_DIR`. Never `pkill` by binary name:
  the Phase-4A fleet and the developer's own servers use the same binaries.
- **The engine stays unprivileged.** The only root step is the one-time
  camera pool (`tools/setup_v4l2loopback.sh` / `./sim vcam --pool`) -
  `sim up` never sudo's; camera sims fail fast naming the script on a
  host without the pool.
- **Feeds never touch real devices.** ffmpeg producers only stream into
  sim-managed devices (pool/loopback/missing indices); a real webcam or
  capture card is never written to, so `--cam 0` is safe.
- **Feed before camera.** An `exclusive_caps` loopback advertises
  CAPTURE only while a producer streams, so every bound index's feed
  reaches CAPTURE before its de_camera instance launches (cold-start
  and `add_module` alike).
- **SITL must stay a child of its launch shell.** SITL exits when its
  parent dies, so `launch.sitl.sh` runs it with `&` and `wait`s, never
  `exec`. Every other launch script `exec`s and writes its own pidfile.
- **Real servers run from their repos with a generated
  `--config=<relative path>`.** The common config loader joins the path
  onto the repo dir. Never edit or depend on the repos' own
  `server.config(.local)`.
- **The real server does not stamp `sd`.** It routes the client's
  `sd`. The bridge sets `sd: GCS_FLEET`. The fake server stamps `sd`, so it
  is not proof for sender-identity behaviour.
- de_comm journals `comm line offline/online` only while a v2 package is
  loaded, and `nav adapter hello` once per de_comm process. That is why
  `reset` restarts SITL + de_comm + modules together.
- Authenticator: the account must be alphanumeric or an e-mail with a TLD;
  a bad one comes back as "API endpoint not found". Logins are limited to
  10/min per IP and every unit is 127.0.0.1.
- Link faults split by transport. On `server: {type: real,
  link_proxy: true}` the per-unit proxies take the byte-level faults:
  `link_cut`, `kick`, `link ... stall|ge|bw|jitter|delay|profile|cut|
  clear`, `link_clear`, `links` (GE `drop` mode kills the session - a
  byte stream cannot skip bytes). Fake-only, refused by `ctl` on real
  with or without proxies: message-level `loss`/`loss_tx`/`loss_rx` and
  profiles carrying loss knobs (`uplink_dead`), plus the server-stamp
  faults `clock`/`noclock` (`server_clock`, `no_time_sync`). On
  `transport: real` without `link_proxy` every link fault refuses.
  `link_proxy` needs `auth: false` (the authenticator hands de_comm the
  server address at login). The GCS bridge/operator always connects
  direct - a unit's link fault never touches it.
- **A downed sim keeps its manifest.** `var/run/<sim>/sim.json` stays for
  post-mortem logs/journals, so a dead sim's units can share names with a
  live one. `simlib.find_unit` and ctl resolution always prefer *live*
  sims (`live_sim_names`); never iterate `all_manifests()` when the
  intent is the running fleet. Live sims on disjoint server namespaces
  may share unit names (`check_conflicts` allows it) - a bare name that
  hits several live sims is ambiguous: `find_unit` raises and ctl
  requires `-s <sim>` rather than guessing.
- `server: {type: external}` spawns no server - the sim's units and GCS
  bridge join a server stack the user runs (`./sim up de_servers`
  starts it: the four servers from their repos' own configs, tracked
  under var/servers/ - infrastructure, not a sim, so `down`/`run`/
  `clean` never sweep it; only `./sim down de_servers` stops it).
  `./sim up de_webclient` is the same kind of devstack for the
  webclient's `npm start` dev server (var/webclient/, `--open-web`
  opens SIM_WEBCLIENT_URL). Its
  `account` (or
  `SIM_AUTH_USER`/`SIM_AUTH_CODE` in `sim.local.env`, or `./sim --user`/
  `--password`/`--auth-server` on the command line - they win over both)
  must exist on that authenticator; with `account_storage_type: db`
  create it in the webadmin first.
- **Scenario runs fly SITL at `--speedup 3` by default.** Only SITL's
  physics clock scales; modules, the 1 Hz world truth tick, rendered
  camera feeds and journal stamps stay wall-clock. A scenario pins
  `realtime: true` (timing asserts - also arms the SKIP gate) or
  `speedup: 1` (render/real cameras, world-driven stubs on a moving
  SITL, closed module↔FC loops like precland). Precedence: `realtime`
  > scenario `speedup:` > `sim:` file's `speedup:` > `SIM_SPEEDUP`/
  `--speedup` > 3. A direct `sim up sims/*.yaml` still defaults 1.
- **Suite tiers:** `tier: smoke` marks the fast first-pass set (one probe
  per subsystem); `./sim suite --tier smoke` runs only those. Absent =
  regression. Smoke is a selection, not evidence — keep it fast and
  dependency-free.
- `land`: copter LAND, rover HOLD + disarm, plane → `reset` (it cannot land
  in place). Always tear down: an airborne reused fleet drains its battery
  and the next test trips `battery_low`.
- `ensure` decides reuse by the sim hash. Any change to a block, sim or
  server settings changes the hash and forces a fresh `up`. That is
  intended.
- The world stream (`de.worldstream/1`) is a thread inside the scenario
  runner - it dies when the run exits. `./sim view` / `run --hold`
  parks the run after the timeline so a viewer session survives; the
  render procs (vite, mav_bridge) are pid-tracked daemons like units,
  so a leftover viewer is stopped by `sim down`, not by the run ending.

## Verifying engine changes

Minimum before handing back:

    python3 -c "import ast,glob;[ast.parse(open(f).read()) for f in glob.glob('engine/*.py')]"
    python3 engine/test_checks.py                      # assertions, journal reader, bot, ports
    python3 stubs/test_stubs.py                        # stub sensor models (needs the databus checkout)
    for f in sims/*.yaml; do ./sim show $f; done      # resolution, no side effects
    ./sim run tests/smoke_copters.yaml                 # real server path
    ./sim run tests/fault_link_loss.yaml               # if fake server / ctl faults touched
    ./sim run tests/smoke_copters_auth.yaml            # if login / bridge / server gen touched
    ./sim run tests/mixed_bringup.yaml                 # if FC_KINDS / pilot commands touched
    ./sim down
