# CHECKS.md — add a grader (assert: check)

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

An `assert:` entry in a de.scenario/1 scenario is `{name: {args}}` —
the runner calls it after the timeline and grades the run evidence.
Adding one is three small edits; the heavy lifting (unit resolution,
journal/truth readers, verdict shape) is shared.

## Where the pieces live

| piece | file | role |
|---|---|---|
| grader fn | `engine/checks.py` | `_<name>(name, args, ctx) -> _r(...)` |
| dispatch | `engine/checks.py` `DISPATCH` | name -> function |
| unit gate | `engine/checks.py` `UNIT_ASSERTS` | names that resolve `unit:` (0 units = FAIL) |
| chaos gate | `engine/checks.py` `INVARIANT_ASSERTS` | run-wide invariants a `chaos:` scenario must carry |
| registry | `engine/registry.py` `V(name,"check",...)` | `./sim verbs` docs + `validate`'s arg check |
| user docs | `docs/SCENARIOS.md` | the assert: section |
| tests | `engine/test_checks.py` | the grader's unit tests |

## 1. Write the grader

```python
def _my_check(name, a, ctx):
    ...
    return _r(name, "PASS" if ok else "FAIL", value, detail)
```

- `_r(name, status, value, detail)` is the result shape —
  `{"name","status","value","detail"}`. `SKIP` is a verdict for
  missing evidence/unsupported realtime, never a silent pass.
- `a` is the args mapping; `ctx` carries `manifest`, `marks` (journal
  marks from the timeline), `sim`, `results_dir`, `samples`,
  `proc_events`, `link_events`, `restore_events`, `world`,
  `world_epoch_ms`, `telemetry`, `bot`, `operator_log`, `coverage_grids`,
  `baseline`, `run_start_ms`, `start_ms`, `hold_ms`, `file`,
  `realtime_required`, `realtime_ok`, `realtime_note`.
- Reuse the readers instead of parsing files: `_truth_rows`,
  `simlib.journal_match`/`journal_lines` (with `ctx["marks"]`),
  `ctl.resolve` for `unit:` (via `_units(arg, ctx)`), `_haversine`.
- Time-critical checks belong in `REALTIME_SENSITIVE` (they SKIP when
  SITL didn't hold ~1x) or honour `arg["realtime"]`.

## 2. Register it

```python
DISPATCH = {
    ...
    "my_check": _my_check,
}
```

- If it takes `unit:` add the name to `UNIT_ASSERTS` — the empty-unit
  case then fails instead of vacuously passing.
- If it grades run evidence rather than a scripted step (restart
  counting, resource growth), it is an *invariant*: add it to
  `INVARIANT_ASSERTS` so a `chaos:` scenario can satisfy the "at least
  one invariant" rule with it.

## 3. Register the verb

`engine/registry.py` — every user-facing key is a `V()`:

```python
V("my_check", "check", req=["unit"], opt=["min", "max"],
  doc="one line for ./sim verbs",
  eg="{my_check: {unit: c1, min: 2}}"),
```

`./sim verbs` prints the doc/eg; `validate` rejects unknown args
against req/opt — keep them accurate.

## 4. Document + test

- Describe the check in the `checks.py` docstring evidence list (what
  it reads: journals, telemetry.jsonl, truth rows, stub logs...) and in
  `docs/SCENARIOS.md`'s assert section.
- `engine/test_checks.py` exercises graders against canned fixtures —
  add cases there; a scenario in `tests/` exercises the wiring end to
  end.

## Example: `udp_proxy_rejects` (negative / security check)

`udp_proxy_rejects {unit, from_ip, secs, expect}` proves a stranger cannot
drive the FC through the UDP proxy pair. It sends an ARM COMMAND_LONG and a
one-item mission upload to the unit's 1071 `{a,p}` from a socket bound to
`from_ip` (default `127.0.0.2`; the allowlisted GCS is the bridge at
`127.0.0.1`), then reads the FC back on serial0 (armed bit, mission
signature).

- PASS: FC still disarmed, mission unchanged, and on a `server: {udp_relay}`
  sim `./sim ctl relay stats` shows `rejected.gcs_not_allowed >= 1`
  (`gcs_unpinned` alone does not count).
- FAIL: relay sim whose 1071 port is outside `udp_port_range` - same rule as
  `udp_proxy_mavlink`; nothing is sent.
- Registered in `engine/registry.py`, documented in `docs/SCENARIOS.md`,
  tests in `engine/test_checks.py` (`TestUdpProxyRejects`).

## Example: the fake-GCS relay checks (SL-0178)

`./sim ctl relay gcs start <name> --ip <src> [--port <n>]` spawns a fake
GCS: a MAVLink heartbeat once a second into the relay pair's socket2
from `<src>` (the single pair's port by default, `--port` to pick), the
telemetry counted on the way back and reported in
`relay gcs status [name]` / the proc's state file. `stop`, `silence`
(hold the heartbeat, keep counting), `resume`, `cmd <name> <arm|disarm>`
round it out; `./sim down` kills them like every other sim proc. Three
checks read that state:

- `relay_gcs_telemetry {gcs, secs, at_least}` — PASS when the named fake
  GCS's telemetry counter grows by `at_least` (default 1) inside `secs`
  (default 3); FAIL when it is not running or the count stalls.
- `relay_gcs_no_telemetry {gcs, secs}` — PASS when it receives nothing
  for the window (the lock moved elsewhere or the pair is silent).
- `relay_gcs_locked {unit, gcs}` — PASS when the pair's `list` row
  `pinned.gcs {ip,port}` equals the named fake GCS's own source address.
