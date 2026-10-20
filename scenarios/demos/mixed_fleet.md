# mixed_fleet — every vehicle type at once

Copter, plane, rover and a comm-only mast all come up on one sim,
arm (or the type-equivalent), and are brought down the way each
vehicle allows.

Run:

    ./sim run scenarios/demos/mixed_fleet.yaml

    # real transport: needs ArduPilot SITL for copter+plane+rover.
    # For a human watch-along: ./sim view keeps the sim up.

What happens:

1. `sim up` brings four vehicle units online — three SITLs plus the
   ground node.
2. The timeline arms the copter block, rover block and plane block at
   t=0 (the runner resolves each block's arm semantics per type).
3. `fc` + `land all` end each craft the way its type allows (copter
   LAND, rover HOLD + disarm, plane SITL reset); `wait_online`
   confirms all four, ground node included, stay registered.

The simplest full-fleet smoke a new machine can run.
