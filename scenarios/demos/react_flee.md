# react_flee — the world fights back

A reactive world: `person_1` patrols a line in `sar_forest_flee` and
flees when the drone closes inside its 30 m react radius — truth is
the single authority, the camera can only watch it move.

Run:

    ./sim run scenarios/demos/react_flee.yaml

What happens:

1. A stub_ai_box copter loiters 40 m over the person's path midpoint.
2. The 1 Hz truth tick (sole evaluator — per-stub reactions would
   diverge truth from detections) latches the flee knot once the copter
   closes inside 30 m.
3. The person sprints its flee path; the box stream and the truth rows
   both show the track move.

The demo for reactive scenarios: ground truth moves on its own, and
the grading still holds.
