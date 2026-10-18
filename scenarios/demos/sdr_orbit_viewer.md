# sdr_orbit_viewer — SDR cue + camera orbit demo

Two-drone "find and look" chain, watched live in the Mavlink3DMap
viewer.

Run:

    ./sim view scenarios/demos/sdr_orbit_viewer.yaml

What happens:

1. `sdr1` (copter + `stub_sdr`) takes off and lawnmowers the field while
   its sensor mission sweeps 868–870 MHz. The casualty's phone
   (`phone_1`, attached to `person_1` ~5 m away) gates the receiver at
   ~60 m slant — a survey leg has to pass almost overhead before
   `sdr.signal` fires.
2. The signal fuses a `phone` world-model target (pos = the scout's own
   pose at detection, `unc_m` = the modelled range). `sys.target_new`
   activates `cam1`'s `orbit_found` template.
3. `cam1` (copter + `stub_ai_box`, `start:"ground"` + `auto_launch`)
   raises `APPROVAL_REQ launch`; the operator bot approves, the launch
   stepper arms it and sends it to the orbit entry point.
4. `cam1` flies a 3-turn orbit around the reported point, radius =
   `clamp(target.unc_m, 30, 70)` at 40 m. Its camera is a fixed
   side-looking mount (yaw +90°, pitch −42° in the world file), so the
   optical axis faces the orbit centre for the whole circle — the sim
   stand-in for a gimbal ROI (the op language has no ROI item). If the
   person ends up in that side frame, `detect.detected` raises a second
   `person` target pin.

Asserts: the phone detection reached both units' world models, the
reported position is within 80 m of `phone_1`'s truth, the launch
approval round-tripped, and the orbit mission activated on `cam1`.

Files:

- `ops/sdr_person_orbit.deop` — the operation (compiled on demand by
  `engine/opbuild.py`).
- `ops/sdr_person_orbit.ctx.json` — binding + capability adverts for
  compile-time/verify checks.
- `../../worlds/sdr_person_orbit.yaml` — the world (person + phone,
  side-looking camera mount, SDR sensor params).
