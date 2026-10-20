# Vehicle-stack adapter interface (de.stack/1)

`de.scenario/1` — the scenario core — is deliberately vehicle-stack
agnostic. The timeline, the world model, ground truth, and every
grader in `engine/checks.py` only need a narrow surface from whatever
flies the aircraft. That surface is the **adapter seam**:
`engine/adapter.py` defines `VehicleAdapter`, and the DroneEngage path
is one implementation (`DEAdapter`), not hard-coded into the runner.

## What the core hands the adapter

`ctx` — the run context dict built by `run_scenario`:

| key | contents |
|---|---|
| `manifest` | the sim's `var/run/<sim>/sim.json` — units, ports, pids |
| `sim` | sim name |
| `scn` | the loaded scenario (`file`, `operation`, `fleet`, `operator`…) |
| `units_by_name` | `{name: unit}` lookup for `unit:` args → party_id |
| `marks` | per-unit journal marks for `wait_journal`-style steps |
| `bot` | the operator bot session (None until `bot.start()`) |
| `server` | transport server record (`type`, port) |
| `proc_events` | append `{t_ms, unit, proc}` when a step re-creates pids |
| `start_ms` | set by the first `start` step — the TTD epoch |
| `results_dir`, `operator_log`, `samples_file` | output paths |
| `adapter` | the adapter itself |

A `do:` step reaches the adapter as
`adapter.do_step(name, args, step, scn, ctx) -> truthy`.
`args` is the step minus `t/on/do/fault/once`; `step` is raw (an `on:`
trigger's `unit:` default lives on it). Falsy return = step failure =
run failure.

## Required surface (what a non-DE adapter MUST implement)

Each `do:` verb the scenarios use, mapped to the stack's primitive:

| verb family | verbs | required behavior |
|---|---|---|
| mode/arm | `mode`, `arm`, `disarm`, `fc` | set flight mode; arm with the stack's own readiness semantics |
| lifecycle | `reset`, `land`, `stop`, `wait_ready`, `wait_online`, `wait_journal`, `wait_log` | process/mission lifecycle + journal waits |
| navigation | `guided`, `goto`, `takeoff`… (registry `do` names) | passthrough to the stack's command channel |
| mission flow | `deploy`, `prepare`, `start`, `abort` | load the operation onto units; start/abort the run |
| operator | `operator` (approve/deny/confirm/dismiss/fire/activate/pause/resume/stop/clear) | replay the human's session |
| comms | `link_cut`, `link_stall`, `link_clear`… | exercised through the transport's link model — the adapter only needs the unit-facing half |
| fire/params | `fire`, `param`, `fc_param`… | event fire + parameter get/set |

Verb names are the public contract — `./sim verbs` lists the registry.
An adapter may implement a verb as a no-op *only* where the semantics
genuinely don't exist (e.g. `deploy` under a stack with no package
format — but then scenarios using `deploy` can't run).

## Required inputs (data the core consumes)

| input | shape | producer |
|---|---|---|
| telemetry rows | `telemetry.jsonl`: `{unit, lat, lng, alt_m/rel_alt_m, armed, mode, hdg, roll/pitch/yaw, t_ms, speedup, ekf_ok…}` | the stack's telemetry feed, thinned to ~1 Hz |
| detections | `truth.jsonl` `kind: detection` rows: `{tid, cls, t_ms, lat, lng, vis, why…}` + the 1078-style box stream the stubs emit | world model + sensor stubs (stack-agnostic) |
| journals | per-unit text journal grep-able by `wait_journal`/`journal_*` asserts | the stack's own logging |
| op state | `op_state.json` `{run: …}` — "is a run live" for the smart `start`/`abort` path | adapter |

## Optional hooks

- **faults** — `fault:` timeline entries land through
  `_fc_param_fault` (param read-modify-write over the FC link) and the
  server-side link model. An adapter without MAVLink params can
  implement `battery`/param faults as its own hook or the scenario
  simply can't use them; `link_*` faults ride the transport, not the
  adapter.
- **start/abort hooks** — `op_state.json` lets `start` carry overrides
  (`lead_ms`, `targets`, `expect_fail`, `{role: mission}` map) when a
  run is already live; absent the file, `start` is a plain fleet-wide
  signal.
- **operator bot** — `ctx["bot"]` is a GCS session that drives
  approvals/denies/confirms/fires. A non-DE adapter supplies its own
  bot object implementing `decide/confirm/fire/action/op_deploy/
  op_start/op_abort/op_authorize/op_revoke_launch` — or leaves
  `operator:` verbs unsupported.

## The DEAdapter mapping (what the seam wraps today)

| scenario verb | DE path |
|---|---|
| `deploy`/`prepare` (deploy-spec op) | `bot.op_deploy(spec, timeout, authorize)` — launch grants ride the deploy |
| `deploy` (compiled package / per-unit `package:`) | `runner.run_ctl("deploy", op)` / `runner.run_step` |
| `.deop` source | `opbuild.build_deop` compiles at run start or step time |
| `start` (run live) | `bot.op_start(overrides, timeout)` |
| `start` (cold) | `runner.run_ctl("start")` |
| `abort` | `bot.op_abort(run, why)` |
| `authorize` / `revoke_launch` | `bot.op_authorize` / `bot.op_revoke_launch` (launch grants, P4F-07) |
| `operator` | `bot.decide/confirm/fire/action` over the bot's GCS session |
| everything else | `runner.run_step({name: args})` — the ctl verb set |
| fleet bus (1090) | inside ctl `fire`/`deploy` — targets emit over the unit bus, no separate seam |

## Verifying the seam

Same code path either way — the refactor only moved the dispatch:

```bash
./sim run tests/smoke_copters.yaml              # real transport, 3 SITL
./sim run tests/fleet_bus/p4a05_catchup.yaml    # bot + bus + operator path
```
