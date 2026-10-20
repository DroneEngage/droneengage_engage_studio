# CHECKS.md — add a grader (assert: check)

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
