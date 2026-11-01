# droneengage_studio — how to use

    cp sim.local.env.example sim.local.env    # once per machine; edit paths
    ./sim blocks                              # what unit types exist
    ./sim show sims/copters3.yaml             # resolved units, ports, positions (no side effects)
    ./sim run tests/smoke_copters.yaml        # up if needed -> steps -> asserts -> teardown
    ./sim try tests/smoke_copters.yaml --module de_mavlink=$HOME/de_code/droneengage_mavlink/bin/de_ardupilot
        # the build->try loop: run a scenario against a fresh module build
        # on the warm sim - only that proc restarts, no SITL warm-up
    ./sim view scenarios/demos/world_canopy_viewer.yaml  # sim + Mavlink3DMap viewer
                                              # (= run --hold) until Ctrl-C
    ./sim ctl status                          # live procs + units seen online by the GCS
    ./sim logs copter1 de_comm                # tail a process log
    ./sim clean --days 7                      # prune var/run/ + var/results/ (never a live sim)
    ./sim down

- `./sim up` is additive: it restarts only the same-named sim and refuses
  ports that clash with a live sim. Unit names may repeat across sims
  that spawn their own server (fake/real - private party registries);
  they stay unique on a shared `server: {type: external}` stack and for
  rendered-camera fleets (machine-global v4l2 labels). `./sim down <name>`
  stops one sim, `./sim down` all of them. `./sim run` is exclusive - a
  test downs every other sim first. `SIM_SHARED=1 ./sim run ...` keeps
  it additive (same rules as `up`) when other sessions share the machine.
  **Agents: never down the developer's sims to run a test.** If a port or
  name clash (or a live sim you did not start) blocks the run, do NOT stop
  and ask, and do NOT `sim down` it - rerun additively on a free port
  block: `./sim run <test> --shared --slot-base any`. `--slot-base any`
  re-resolves the sim on the lowest slot whose whole port set (units +
  spawned server/auth) is free vs live manifests and bound host ports;
  unit names may repeat because each sim's own server has a private party
  registry. Only `server: {type: external}` tests (shared registry) and
  rendered-camera fleets (machine-global v4l2 labels) cannot share the
  machine - ask then. A shifted slot changes the sim hash, so the run does
  a fresh `up` (one SITL warm-up) and never reuses the warm fleet.
- `./sim ctl -s <sim>` pins a command to one sim; without it commands
  that need a single sim use the only live one, and unit lookups search
  live sims first.
- `./sim ctl add_module <unit> de_camera [--cam dev] [--ffmpeg src]`
  hot-adds a camera instance on a live unit without restarting SITL
  (`remove_module` reverses). Run `./sim vcam --list` first to pick a
  free device: `--cam` takes an index (`4`), `videoN`, `/dev/videoN` or
  a card label; `--ffmpeg <src>` feeds only the newly bound index and
  is refused on a real device.
- `./sim run` reuses the running sim only when its hash matches and every
  process is alive; otherwise `up --fresh`. Run tests on the same sim back to
  back to skip the 1-2 min SITL/EKF warm-up.
- `./sim run <test> --keep` skips the teardown so you can inspect the fleet
  with `./sim ctl ...`.
- `./sim try <test> --module <name>=<bin>` is the way to validate a module
  change: it reruns the scenario against the fresh binary on the warm sim
  - only that module proc restarts on each unit (no SITL warm-up, no
  teardown). The override is part of the sim hash and recorded in
  summary.json; dropping `--module` swaps back to the default build the
  same way.
- Exit code 0 = every step and assertion passed. Read the printed
  PASS/FAIL lines and `var/results/<ts>-<test>/` (journals + logs), not just the
  exit code.
- Machine consumers: every read-only command takes `--json`, and a run's
  last stdout line is `RESULT <dir>/summary.json` — shapes and exit-code
  meanings are in `docs/MACHINE_API.md`. Never scrape the human text.
- Before claiming a change works, run the relevant test(s) and quote the
  `N/M assertions passed` line.
