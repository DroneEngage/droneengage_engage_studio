# Vehicle-stack adapter interface (de.stack/1)

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

`de.scenario/1` is the scenario core. It is vehicle-stack agnostic by
design. The timeline, the world model, ground truth, and every grader
in `engine/checks.py` need only a narrow surface. The surface comes
from whatever flies the aircraft. That surface is the **adapter seam**.
`engine/adapter.py` defines `VehicleAdapter`. The DroneEngage path is
one implementation (`DEAdapter`). It is not hard-coded into the runner.

## What the core hands the adapter

`ctx` is the run context dict. `run_scenario` builds it:

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
`args` is the step minus `t/on/do/fault/once`. `step` is raw. An `on:`
trigger's `unit:` default lives on `step`. A falsy return means step
failure. Step failure means run failure.

## Required surface (what a non-DE adapter MUST implement)

Every `do:` verb in the registry reaches the adapter. `./sim verbs`
lists them under "do: steps" and "do: probes". The list below is
complete as of this document. `./sim verbs` is the authority if the
list drifts.

| verb family | verbs | required behavior |
|---|---|---|
| mission flow | `deploy`, `prepare`, `start`, `abort`, `opstart`, `opabort`, `authorize`, `revoke_launch` | load the operation onto units; start/abort the run; launch grants |
| operator | `operator` (approve/deny/confirm/dismiss/fire/activate/pause/resume/stop/clear/deploy/start/abort) | replay the human's session |
| flight | `arm`, `disarm`, `land`, `mode`, `takeoff`, `fc`, `param`, `param_get` | FC commands with the stack's own readiness semantics |
| mission control | `stop`, `pause`, `resume`, `clear` | mission-level control passthrough |
| lifecycle | `reset`, `reboot`, `kill`, `restart`, `freeze`, `unfreeze`, `kick`, `journal`, `server_up` | process/session lifecycle |
| timing | `wait`, `delay`, `clock_offset`, `clock_step` | timeline sleep; clock skew/step |
| waits/probes | `wait_journal`, `wait_log`, `wait_ready`, `wait_online`, `opstatus`, `opmsg`, `beacon`, `roster_peers`, `links`, `fcb_stats` | observe-only reads of journal/roster/op state |
| comms | `link_cut`, `link_clear`, `link`, `loss`, `commline`, `udp_chunks` | transport link model; adapter needs only the unit-facing half |
| fc-bus faults | `fcb_cut`, `fcb_stall`, `fcb_corrupt`, `fcb_drop`, `fcb_inject`, `fcb_clear` | FC link fault injection (MAVLink stacks) |
| host faults | `cpu_starve`, `cpu_unstarve`, `journal_unwritable`, `journal_writable`, `write_file` | host-level fault injection |
| events | `fire`, `event`, `send` | event fire / raw message send |

Verb names are the public contract. `./sim verbs` lists the registry.
An adapter may implement a verb as a no-op *only* where the semantics
do not exist. Example: `deploy` under a stack with no package format.
Scenarios that use `deploy` then cannot run.

## Required inputs (data the core consumes)

| input | shape | producer |
|---|---|---|
| telemetry rows | `telemetry.jsonl`: `{unit, lat, lng, alt_m/rel_alt_m, armed, mode, hdg, roll/pitch/yaw, t_ms, speedup, ekf_ok…}` | the stack's telemetry feed, thinned to ~1 Hz |
| detections | `truth.jsonl` `kind: detection` rows: `{tid, cls, t_ms, lat, lng, vis, why…}` + the 1078-style box stream the stubs emit | world model + sensor stubs (stack-agnostic) |
| journals | per-unit text journal grep-able by `wait_journal`/`journal_*` asserts | the stack's own logging |
| op state | `op_state.json` `{run: …}` — "is a run live" for the smart `start`/`abort` path | adapter |

## Optional hooks

- **faults** — `fault:` timeline entries go through `_fc_param_fault`
  and the server-side link model. `_fc_param_fault` does a param
  read-modify-write over the FC link. An adapter without MAVLink params
  can implement `battery`/param faults as its own hook. Otherwise the
  scenario cannot use them. `link_*` faults ride the transport, not the
  adapter.
- **start/abort hooks** — `op_state.json` lets `start` carry overrides
  when a run is already live. The overrides are `lead_ms`, `targets`,
  `expect_fail`, and a `{role: mission}` map. Without the file,
  `start` is a plain fleet-wide signal.
- **operator bot** — `ctx["bot"]` is a GCS session. It sends
  approvals, denies, confirms and fires. A non-DE adapter supplies its
  own bot object. The bot implements
  `decide/confirm/fire/action/op_deploy/op_start/op_abort/op_authorize/op_revoke_launch`.
  Without a bot, the adapter leaves `operator:` verbs unsupported.

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

## How to verify the seam

The code path is the same in both cases. The refactor only moved the
dispatch:

```bash
./sim run tests/smoke_copters.yaml              # real transport, 3 SITL
./sim run tests/fleet_bus/p4a05_catchup.yaml    # bot + bus + operator path
./sim run tests/sar/p4b10_a_nominal.yaml        # SAR: 32 PASS, 0 FAIL
```
