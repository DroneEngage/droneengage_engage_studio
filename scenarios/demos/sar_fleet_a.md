# sar_fleet_a — the reference SAR op end to end

Six copters run the full search-and-rescue reference op over the
sar_forest world: camera + SDR scouts find, three searchers claim
sectors, and the ambulance auto-launches to drop on the confirmed
person.

Run:

    ./sim run scenarios/demos/sar_fleet_a.yaml    # realtime, ~15 min

What happens:

1. `sdr1`'s wide search passes inside the ~600 m phone gate — the stub
   fires `sdr.signal` and every unit journals `WM_NEW cls=phone`.
2. Searchers `s1,s2,s3` bid; the coordinator splits the grid into one
   claimed sector each.
3. A camera box over the person becomes `WM_NEW cls=person`; the
   scripted operator confirms it ~3 s later.
4. `amb1` (start:"ground", auto_launch) asks for a launch approval,
   gets it, and flies the drop leg.

Read tests/sar/README.md for the full op narrative; the asserts grade
confirm/deny timing, claim coverage and the drop.
