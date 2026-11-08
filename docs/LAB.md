# Measure your hardware with lab tests

Lab tests (`audience: lab`) measure your own machines. They run only
when asked:

```bash
./sim suite --tier lab --target pi@192.168.1.20
```

`--target user@host` (or `SIM_TARGET` in sim.local.env) names the
machine under test - no lab test hard-codes a host.

## Preflight

Before a lab run the engine checks: SSH reaches the target
(BatchMode - key login) and the modules the test drives are present.
A missing item stops the test with a message naming it; a missing
target SKIPs (requires: [rpi_target]).

## The report

Every lab run writes `var/reports/<test>-<utc>.json` and `.txt`:

| field | meaning |
|---|---|
| host | target, machine arch, model, cpu count, memory |
| versions | the workspace repo commits the run used |
| metrics | measured rows - per-step load, rates, counts |
| limits | the boundary the test found (or "survived N") |

## Examples

```bash
# the 1000-item plan path - upload time, FC item rate, overflow limit
./sim run tests/capacity/plan_1kwp.yaml

# de_comm load ramp on a Pi - CPU/RSS/bandwidth/loss per step
./sim suite --tier lab --target pi@<pi-host> tests/capacity
```

A lab test never changes the target permanently: no leftover
processes or files after the run.
