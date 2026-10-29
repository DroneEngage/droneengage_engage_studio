# MAP.md - the one-page model map

Three axes, one tree. **COMPOSE** is what exists (things you build),
**EXECUTE** is why you run it (run files), **SUBSTRATE** is where it
runs (host, dev stacks, hardware). `./sim help` groups its commands by
the same intents (DISCOVER / COMPOSE / LIVE / EXECUTE / CAPTURE /
HARDWARE / HOUSE).

## The noun diagram

```
                 modules/catalog.yaml        how each DE module launches
                          |
                          v
                    blocks/<name>/           ONE unit type:
                    (block.yaml + overlays)  which fc, which modules,
                          |                  which config
                          v
                     sims/<name>.yaml        a FLEET: blocks composed,
                          |                  placement, server choice
                          v
        +-----------------+-----------------+
        |                                   |
   tests/**/*.yaml                 scenarios/{demos,examples,
   (machine-checked)               challenges}/**.yaml
        |                          (human-run)
        +-----------------+-----------------+
                          |
              a de.scenario/1 RUN FILE: fleet -> timeline -> assert
                          |
        +-----------------+-----------------+
        |                 |                 |
   worlds/*.yaml    packages/*.json    dev stacks (engine/devstack.py)
   (ground truth    (ops that          de_servers: the real auth/comm/
    a run seeds)     `do: deploy`       storage/udp-relay servers;
                    sends)              de_webclient: npm dev server
                          |
                          v
               SUBSTRATE: host SITL (default) - ./sim up <stack> -
               tools/rpi + tools/hil hardware - android phones
```

A running fleet lives under `var/run/<sim>/` (generated: sim.json,
configs, launch scripts, logs, pids); results land in `var/results/`.
Neither is committed.

## Folder -> axis

| folder | axis | holds |
|---|---|---|
| `blocks/` | COMPOSE | unit types (block.yaml + module/fc overlays) |
| `sims/` | COMPOSE | fleet definitions (`overlays/` = one-twist variants) |
| `modules/` | COMPOSE | module catalog + base templates |
| `worlds/` | COMPOSE | seeded ground-truth worlds |
| `packages/` | COMPOSE | hand-written ops (demo/example fixtures) |
| `templates/` | COMPOSE | what `./sim new` scaffolds |
| `tests/` | EXECUTE | the machine-checked de.scenario/1 library |
| `scenarios/` | EXECUTE | human-run run files (demos/examples/challenges) |
| `engine/` | EXECUTE | the sim engine (loader, runner, ctl, devstack, registry) |
| `ci/` | EXECUTE | suite driver + library checks (`run_suite.py`, `check_library.py`) |
| `tools/` | SUBSTRATE | `rpi/` + `hil/` hardware helpers, verify/compile tools |
| `stubs/` | SUBSTRATE | world-driven sensor stubs + their tests |
| `docker/` | SUBSTRATE | the all-in-one image (SITL + binaries + engine + viewer) |
| `var/` | SUBSTRATE | runtime state: `run/`, `results/`, stack pids/logs (gitignored) |
| `docs/` | meta | authoring references (SCENARIOS.md, SENSORS.md, this file) |
| `wiki/` | meta | user docs (USER_GUIDE.md - the plain-English start) |

Top-level files: `sim` (the CLI), `run_scenario.py` (compat shim ->
`./sim run`), `sim.local.env.example`, `de_scenario.schema.json`-adjacent
engine modules under `engine/`.

## Glossary

- **a block** (`blocks/<name>/`) - one unit *type*: which flight
  controller (or none), which DE modules, which config overlays.
  Composed via `extends:`. Compose axis.
- **sims/** - the *files* defining fleets: which blocks, how many, where
  placed, which server. Compose axis.
- **a sim** - a *running* fleet: the live processes + `var/run/<sim>/`
  manifest you get from `./sim up sims/x.yaml`. Distinct from the
  sims/ file it was generated from.
- **a run file** - one de.scenario/1 yaml under `tests/` (machine-checked,
  CI) or `scenarios/` (human-run). Its kind is inferred from the path;
  `tasks:`/`tier:` select it. Execute axis.
- **a stack** - a developer server set, not a sim: `de_servers` (real
  auth/comm/storage/udp-relay from their repos) or `de_webclient` (npm
  dev server), started with `./sim up <stack>` for `server: {type:
  external}` sims to join. Substrate axis.
- **a world** (`worlds/<name>.yaml`) - seeded ground truth a run file
  loads (moving targets, coverage, truth recorder). Compose axis.
- **a package** (`packages/*.json`, `tests/packages/`) - a hand-written
  DE operation a run file deploys; `.deop` sources compile on demand.
  Compose axis.
- **a module** - one DE process type (`de_comm`, `de_mavlink`, ...)
  described in `modules/catalog.yaml`: how it launches, ports, readiness.
- **an overlay** (`sims/overlays/<name>.yaml`) - a one-twist variant
  applied with `--with <name>`; not a new fleet file.
```
