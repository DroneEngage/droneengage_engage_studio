# link_catchup — two scouts, one drops, still catches up

Three copters share a find over the fleet bus. `copter2`'s link is cut
for 60 s mid-emit — `copter3` activates live; `copter2` must activate
within seconds of its link returning (retry/catch-up), and never
twice.

Run:

    ./sim run scenarios/demos/link_catchup.yaml

What happens:

1. `copter1` emits a target with a 300 s TTL over the unit bus.
2. `copter2` loses its link for 60 s; `copter3` sees the target live
   and activates on it.
3. When `copter2`'s link returns, the bus retry delivers the still-live
   target — it activates once, on catch-up.

Asserts pin exactly-once activation on both late and live units — the
fleet-bus delivery guarantee, not just connectivity.
