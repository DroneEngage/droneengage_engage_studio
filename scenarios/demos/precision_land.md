# precision_land — AprilTag pad landing

Closed-loop precision landing: a copter descends onto an AprilTag pad
with the stub_precland module publishing target poses into the FC's
LANDING_TARGET gate.

Run:

    ./sim run scenarios/demos/precision_land.yaml

What happens:

1. The copter flies its leg; the compiled op's start action invokes
   `precision_landing.enable` on the unit's stub_precland.
2. The flight-lane gate holds at 20 m over the nominal point until the
   stub's `stabilized` fact releases it.
3. The package ends without a nav:land item — the supervisor's
   `end: land` puts the FC in NavMode LAND, opening the precland gate
   through allowed_modes (LAND|RTL).
4. The LANDING_TARGET stream steers touchdown onto `pad_1` (measured
   error 0.0 m on seed 1).

The pad demo: vision stub, gate and FC landing loop all in one shot.
