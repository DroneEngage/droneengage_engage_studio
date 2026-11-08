# MAP.md - the one-page model map

There are three axes and one tree. **COMPOSE** is what exists. These
are the things you build. **EXECUTE** is why you run it. These are the
run files. **SUBSTRATE** is where it runs. This is the host, the dev
stacks and the hardware. `./sim help` groups its commands by the same
intents (DISCOVER / COMPOSE / LIVE / EXECUTE / CAPTURE /
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

A running fleet lives under `var/run/<sim>/`. The directory is
generated. It holds sim.json, configs, launch scripts, logs and pids.
Results land in `var/results/`. Neither directory is committed.

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

Top-level files: `sim` (the CLI), `run_scenario.py` (compat shim to
`./sim run`), `sim.local.env.example`. The
`de_scenario.schema.json`-adjacent engine modules live under
`engine/`.

## Glossary

- **a block** (`blocks/<name>/`) - one unit *type*. It says which
  flight controller (or none), which DE modules, and which config
  overlays. Compose with `extends:`. Compose axis.
- **sims/** - the *files* that define fleets. They say which blocks,
  how many, where placed, and which server. Compose axis.
- **a sim** - a *running* fleet. It is the live processes plus the
  `var/run/<sim>/` manifest. You get it from `./sim up sims/x.yaml`.
  It is not the same as the sims/ file it was generated from.
- **a run file** - one de.scenario/1 yaml under `tests/`
  (machine-checked, CI) or `scenarios/` (human-run). Its kind is
  inferred from the path. `tasks:` and `tier:` select it. Execute axis.
- **a stack** - a developer server set, not a sim. It is `de_servers`
  or `de_webclient`. `de_servers` is the real auth, comm, storage and
  udp-relay servers from their repos. `de_webclient` is an npm dev
  server. Start a stack with `./sim up <stack>`. `server: {type:
  external}` sims then join it. Substrate axis.
- **a world** (`worlds/<name>.yaml`) - seeded ground truth that a run
  file loads. It holds moving targets, coverage and the truth
  recorder. Compose axis.
- **a package** (`packages/*.json`, `tests/packages/`) - a
  hand-written DE operation that a run file deploys. `.deop` sources
  compile on demand. Compose axis.
- **a module** - one DE process type (`de_comm`, `de_mavlink`, ...).
  `modules/catalog.yaml` describes it: how it launches, its ports, its
  readiness.
- **an overlay** (`sims/overlays/<name>.yaml`) - a one-twist variant.
  Apply it with `--with <name>`. It is not a new fleet file.
```
