# How ground truth caught three viewer bugs

A 3D map with a drone frozen at the origin looks like a rendering
glitch. Three times in one week it was three different bugs in three
different layers — and each one fell out not from staring at the map,
but from the world stream and the deterministic ground truth behind it.
When you can compare "what the viewer shows" against "what the world
knows", every mismatch becomes a bug report instead of a shrug.

## 1. The Infinity hello

The world stream's hello packet describes the environment, including
`visibility_m`. The default in `worlds/` YAML is "no haze" — which
Python happily represents as `float('inf')`, and `json.dumps` happily
writes as `Infinity`. Strict JSON has no such literal; the browser's
`JSON.parse` rejected the *entire hello*, silently. No hello meant no
origin, no origin meant no coordinate converter — and every vehicle
pinned at `[0, 0, 0]`.

The fix (`8883a2f`): the emitter sends strict JSON, mapping non-finite
values to `null`. The lesson outlasted the fix — a dropped hello now
fails loudly, and the viewer's status overlay (added later, on the same
reasoning) tells a human *which* phase the stream died in instead of
rendering a parked drone and calling it telemetry.

## 2. The stream-rate rename

Second bug, same symptom — drone at origin. This time the hello parsed
fine; the vehicle simply never got pose packets. The sim had been
setting `SR4_*` stream-rate parameters on the serial channel bridged to
the viewer. ArduPilot had renamed that family to `MAVn_*` — and because
stream rates are only read at boot, the *old* parameters were accepted
into the parameter table and ignored: serial4 sent a heartbeat and
nothing else.

Same commit, second fix: render sims set the `MAVn_*` names. The
follow-up (`fb28c02`) hardened it further — rates are raised on the
*computed* serial channel index rather than hard-coded "4", so whatever
other link lands on channel 4 doesn't get its rates stomped.

## 3. The single-client bridge

The third bug hid in the UDP→WebSocket bridge that feeds the map. The
bridge held one client slot: a second browser tab opening the viewer
took the slot, the first tab's auto-reconnect took it back, and the two
fought over the feed forever. Multi-vehicle sims had the mirror-image
problem — UDP peers were tracked as "last speaker", so replies went to
whichever drone had spoken most recently.

`d89bcc0` in Mavlink3DMap replaced the slot with a client `Set` (dead
sockets reaped on send/close/error), keyed UDP peers by MAVLink sysid
so browser→vehicle messages unicast to `target_system` and only
broadcast when unknown, and — the belt-and-suspenders part — prods each
new peer with `REQUEST_DATA_STREAM` (msgid 66, all streams at 10 Hz)
every 30 s. That last piece means the viewer no longer depends on the
sim's `MAVn_*` guess at all; even a drone whose parameters were never
set gets talked into streaming.

## The pattern

None of these were found by opening the map and looking. They were
found by asserting against ground truth — the scenario knows where the
drone is, so "viewer shows origin" is a diff, and diffs get bisected
into protocol layers: hello dropped (world stream), telemetry absent
(serial params), feed stolen (bridge topology). Three layers, three
commits, one symptom.

Ground truth is usually sold as a grading input — did the mission work.
It's also the best debugging instrument the viewer has: a map that can
be compared against a known world is a map whose bugs describe
themselves.
