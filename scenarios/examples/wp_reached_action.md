# wp_reached_action — a waypoint's on:'reached' action fires on arrival, not before

MP-0412 proof for the guided step popup (MP-0411): the mission
supervisor really does dispatch a nav step's `actions[]` when the FC
reaches that waypoint.

## What it does

Deploys `packages/p2_10_handlers.json` on one bare copter. That package
anchors `camera.capture` on `reached` at w1, w2, w3 and w4 (seqs 2, 3,
4, 8). The copter flies w1 → w2 → w3, then sits at the `clear.zone`
hold gate — which is never fired, so it times out to `rtl` and w4 is
never reached.

Because the unit has no `de_camera`, each `camera.capture` cannot ride
a capability invoke; the supervisor falls back to the Phase-2 legacy
bridge (`ACTION <id> via legacy bridge`). The dispatch is what this
scenario proves — a camera module is not needed to show the anchor
fires at the right moment.

## Run

    ./sim run scenarios/examples/wp_reached_action.yaml

## What to watch

In `var/results/<ts>-wp_reached_action/c1/journal/`:

- `action camera.capture id=<run>:1:2`, `:1:4`, `:1:6` — one capture
  per reached waypoint, in order, spread across the flight.
- No `id=<run>:1:11` — w4's anchored capture never fires because the
  gate times out and w4 is never reached. 'reached' means reached.
- `gate g timed out -> rtl`, then `run ... completed` after the RTL
  landing disarms the copter.
