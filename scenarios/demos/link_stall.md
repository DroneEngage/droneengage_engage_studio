# link_stall — congested-cell blackout

A flying copter's link hard-stalls for 30 s — every byte buffered, the
socket still up. A blackout, not a cut: when the window ends the held
queue flushes in order and the unit comes back by itself.

Run:

    ./sim run scenarios/demos/link_stall.yaml    # realtime run

What happens:

1. `c1` is airborne on a probe mission when the stall lands at t=20 s.
2. ~10 s in, the unit's ping watchdog drops the ws — "comm line
   offline"; server-side the unit vanishes from online.
3. Its re-auth sits in the held tx queue; at the flush it lands, the
   conn re-registers and the journal writes "comm line online".
4. A probe event fired at t=62 s latches — commands flow again.

Asserts pin the offline->online order, the ~10-60 s offline span,
recovery within 90 s, and that the mission still completed.
