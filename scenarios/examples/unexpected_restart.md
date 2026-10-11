# examples/unexpected_restart — how to run

A **negative** example: `do: kill` stops de_comm with no fault window
around it, so the `no_unexpected_restart` invariant must FAIL the run.
Use it to check the invariant fires — it is not part of any passing
suite.

## Run

```bash
./sim run scenarios/examples/unexpected_restart.yaml
echo $?    # non-zero = the invariant caught the kill (that IS the pass)
```

Expected outcome: FAIL on `no_unexpected_restart` — a passing run here
means the invariant is broken.
