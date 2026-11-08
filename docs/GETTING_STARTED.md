# Getting started with core sims

From clone to a green suite on a clean Linux host:

```bash
cp sim.local.env.example sim.local.env   # fill in DE_CODE, ARDUPILOT_DIR
./sim doctor                             # the readiness table
./sim suite                              # core only - green on SITL alone
```

## The four tiers

- **core** - SITL only, always green, the `./sim suite` default. A
  failure here is a real fault, never missing hardware.
- **extended** - needs a stated capability (`requires:` in the file:
  camera, v4l2loopback, android_adb, mavlink3dmap, ...). Run with
  `./sim suite --tier extended`; a missing capability shows SKIP with
  the reason, never FAIL. `./sim doctor --caps` shows what this host
  has.
- **lab** - measures your own hardware (capacity, load, bandwidth).
  Run with `./sim suite --tier lab --target user@host` - see
  docs/LAB.md.
- **mylab** - private developer content, kept out of the repo under
  `SIM_LAB_DIR` in sim.local.env. Its sims/tests/overlays/worlds
  extend the search path; `./sim list` labels them `[mylab]`.

To see why a test skipped: the suite row and summary.md print
`needs <capability> (<reason>)`; `./sim doctor --caps` gives the same
map for the whole host.
