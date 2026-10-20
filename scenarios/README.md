# scenarios/ — human-run scenarios

This directory holds scenarios meant to be **run by a person**: demos to
watch, examples to copy, experiments you drive yourself. Everything
automated (the CI / nightly regression library) lives in `../tests/`.

```
demos/       showcase runs - viewers, render farm, things to look at
examples/    small, commented starter files - copy one and edit
```

The `demos/` gallery - each yaml is promoted unchanged from a passing
`tests/` scenario, with a one-line story and a runbook:

| demo | the story |
|---|---|
| `world_canopy_viewer` | forest search watched live in Mavlink3DMap - the open-ground hiker is found, the one in the hut stays hidden |
| `sdr_orbit_viewer` | two-drone find-and-look: the SDR scout cues the camera ship into an orbit, live in the viewer |
| `sar_fleet_a` | the reference SAR op end to end - scouts find, searchers claim sectors, the ambulance drops |
| `decoy_clutter` | the SAR story against look-alike decoys and a detector that calls people `human` |
| `react_flee` | a reactive world - the person flees once the drone closes inside its react radius |
| `precision_land` | AprilTag pad landing - the stub's stabilized fact releases the FC into LAND on target |
| `battery_low_rtl` | the battery collapses mid-search and the op's declared handler flies it home |
| `link_stall` | a 30 s congested-cell blackout, then the held queue flushes and the unit rejoins |
| `link_catchup` | one scout loses its link mid-emit and still activates the shared find on catch-up |
| `mixed_fleet` | copter + plane + rover + comm mast boot, arm and come down on one sim |

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
