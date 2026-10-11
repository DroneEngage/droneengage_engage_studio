# scenarios/ — human-run scenarios

This directory holds scenarios meant to be **run by a person**: demos to
watch, examples to copy, experiments you drive yourself. Everything
automated (the CI / nightly regression library) lives in `../tests/`.

```
demos/       showcase runs - viewers, render farm, things to look at
examples/    small, commented starter files - copy one and edit
```

Every scenario yaml has a `<name>.md` runbook next to it (what it does,
how to run it, what to watch); the yaml's header points at it. Keep the
pair when adding a scenario — same convention as `../sims/`.

Run any file with the usual commands:

```bash
./sim run scenarios/demos/world_canopy_viewer.yaml   # timeline + asserts
./sim view scenarios/demos/world_canopy_viewer.yaml  # park with the
                                                     # Mavlink3DMap viewer
                                                     # live until Ctrl-C
./sim list scenarios                                 # what is here
```

## Writing your own

```bash
./sim new my_flight            # scaffold scenarios/my_flight.yaml from
                               # templates/scenario.yaml
./sim validate scenarios/my_flight.yaml   # schema + path check, no sim
./sim run scenarios/my_flight.yaml
```

`docs/SCENARIOS.md` is the full authoring reference - it is written so
you can also paste it (or point an LLM at it) to have a scenario written
for you. The format is strict: unknown keys fail against
`engine/de_scenario.schema.json`, so `./sim validate` after every edit.
