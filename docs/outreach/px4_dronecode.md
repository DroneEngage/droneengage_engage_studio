# PX4/Dronecode outreach draft — post for AFTER the mavlink adapter lands

> Hold until the MAVLink adapter work (SL-0043/SL-0042 seam) is real;
> this draft assumes a vehicle-stack adapter exists that speaks plain
> MAVLink instead of the DroneEngage bus.

Hi all — we've built a mission-rehearsal playground and the interesting
part for this community is that the *core is vehicle-stack agnostic*.

The scenario format (`de.scenario/1`) describes a fleet, an operation,
an operator, a timeline, faults, and assertions. The world model —
ground-truth entities, canopy/building occlusion, camera/SDR/thermal
sensor models, clutter, seeded determinism — doesn't know or care
what's flying. What it needs from a vehicle stack is a narrow adapter:
telemetry rows (pose + attitude), a detections stream
(cls/pos/confidence), start/abort hooks, and optionally fault hooks.
The adapter contract is written down in `docs/` (adapter interface
spec); the DroneEngage path — packages, fleet bus, operator bot — is
one implementation of it.

What that buys a PX4 user: the same scenarios fly a PX4 SITL fleet.
The same graders apply — coverage vs POD under canopy, time-to-detect,
precision against a seeded clutter field. The same monthly challenge
scores your search pattern the way it scores ours.

**Try it** (same commands, adapter flag/config TBD when the adapter
lands):

```bash
./sim doctor
./sim view scenarios/demos/world_canopy_viewer.yaml
./sim run scenarios/challenges/entries/baseline/scenario.yaml --seed 11
```

What we'd love from this community: which part of a PX4 stack should
own the adapter — is a uXRCE-DDS bridge the right edge, or a plain
MAVLink router shim? And which graders matter to you that we don't
have (wind models, GPS degradation, gimbal dynamics)?

*(Fill in adapter name/config flag and repo links when the seam is
merged — do not post before then.)*
