# battery_low_rtl — low battery mid-mission

One copter flying a survey loses its battery mid-mission; the op's
declared `battery_low` handler fires `rtl`, not an operator.

Run:

    ./sim run scenarios/demos/battery_low_rtl.yaml

What happens:

1. `c1` deploys the p2_10_handlers search package and gets airborne.
2. At t=20 s the battery fault collapses capacity to 50 mAh plus a
   10.5 V sag — the FC-derived remaining % crosses `sv_battery_low_pct`.
3. The supervisor journals "handler battery_low fired ... -> rtl" and
   pauses the run; the FC takes the RTL backstop — declared policy,
   not an annotation.

Asserts prove the handler fired exactly once, the pause happened, a
NavMode change was confirmed, and the telemetry sag was real (min
bat_v < 11 V).
