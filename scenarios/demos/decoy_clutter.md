# decoy_clutter — decoys and class aliases

The same SAR story, but the world is salted with look-alike decoys and
every camera stub reports the person class under the detector's own
label (`human` instead of `person`).

Run:

    ./sim run scenarios/demos/decoy_clutter.yaml   # realtime, ~15 min

What happens:

1. `class_labels: {person: human}` remaps every stub box; the op's
   detect decl lists `classes: [person, human]`, so aliased boxes still
   fuse into the canonical `person` target.
2. Decoy entities in `sar_forest_decoy` produce boxes that match
   nothing declared — they keep dropping instead of false-confirming.
3. The rest of the reference op runs as in `sar_fleet_a`: claims,
   person confirm, ambulance launch + drop.

The story for anyone tuning a real detector whose label set won't
match the op's classes.
