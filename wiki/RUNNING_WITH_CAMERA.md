# Running a sim with a camera (`de_camera`)

`de_camera` is the DroneEngage camera module — it grabs frames from a
video device on the vehicle and streams them to the WebClient. It is
already in the module catalog (`modules/catalog.yaml`), so getting it
running is two things: **give a unit the module** and **give the module
a video device**. Both are covered below.

---

## Before you start

**The binary.** The engine resolves `de_camera` from
`DE_CAMERA_BIN` in `sim.local.env`, falling back to
`$DE_CAMERA_DIR/compiled_bin/de_camera`. Point it at a real build — the
checked-in `compiled_bin/de_camera` may be older than the repo (the
WebRTC gn/ninja builds land in `compiled_bin_*/`).

```bash
# sim.local.env
DE_CAMERA_BIN=~/de_code/droneengage_camera_2025/compiled_bin_x86_release/de_camera
```

**The check.** `./sim doctor sims/<name>.yaml` prints one line per
requirement — including the camera binary and video devices — for just
that sim. Run it once before bringing a camera sim up.

**A video device.** `de_camera` opens a V4L2 device. You have three
sources:

| Source | How | When |
|---|---|---|
| Named-label pool | `./sim vcam --pool` (sudo, once) | `cameras:` counts, label-bound cameras, hot-added instances — the usual choice |
| Per-sim devices | `./sim vcam sims/<name>.yaml` (sudo) | a sim that names fixed `/dev/videoN` indices |
| A real webcam | pass `--cam <dev>` | you want real frames on a single-camera unit |

`./sim vcam --pool` runs `tools/setup_v4l2loopback.sh` and creates the
named labels once: `DE-CAM1..10`, `DE-SRC`/`DE-AI` render pairs. The
indices are kernel-picked so real webcams are never collided with.
`--persist` keeps them across reboots. This is the only `sudo` path in
the engine — `sim up` itself stays unprivileged.

---

## Way 1 — declare it in the sim (cold start)

Pick a block that already carries the module:

- `copter_camera` — the `copter` block + `de_camera`
- `camera_fixed` — `de_comm` + `de_camera` at a fixed position, no
  flight controller (`fc: null`); a camera post

```yaml
units:
  - {block: copter_camera, name: camcopter}
  - block: camera_fixed
    name: tower1
    cameras: [DE-CAM2]                      # pin to a pool label
    at: {north_m: -50, east_m: 30}
```

Or bolt it onto any block yourself:

```yaml
# blocks/my_block/block.yaml
extends: copter
add_modules: [de_camera]
```

`cameras:` on a unit runs that many `de_camera` instances, each bound
to a device: `cameras: 2` auto-binds two free `DE-CAM*` labels;
`cameras: [DE-CAM1, 0, {label: DE-CAM3, name: front}]` mixes labels,
indices and named entries. A camera unit with no `cameras:` selection
defaults to one auto pool label.

Then bring it up as usual — `./sim up sims/copter_and_cameras.yaml` —
and `./sim show <sim>` first if you want to see the resolved units and
ports without side effects. `sim up` waits for `de_camera`'s
`Communicator Server Found` stdout line before calling the unit ready.

---

## Way 2 — hot-add on a live sim (`add_module`)

The sim is already up and you want a camera on a running unit — no
restart, no SITL warm-up:

```bash
./sim ctl add_module camcopter de_camera                  # auto-picks a free DE-CAM label
./sim ctl add_module camcopter de_camera --cam /dev/video2   # or pin a device/label
./sim ctl remove_module camcopter de_camera               # reverses it
```

Under the hood (`engine/ctl.py:774`) it picks the next free instance
name and port (a second camera becomes `de_camera2`), generates the
config, allocates a free `DE-CAM*` label (or honours `--cam`), and —
important — **starts the video feed and waits for a real CAPTURE read
before exec'ing the module**. `v4l2loopback`'s `exclusive_caps` means a
device only advertises CAPTURE once a producer is streaming into it, so
`de_camera` would not even see the device otherwise.

The hot-added process is recorded in the sim's manifest under
`units[].hot_added`. The spec hash does not change, so a later
`./sim up` on the same sim reuses the live fleet and keeps your camera.

`add_module` is an operator-console command only — it is not available
as a `do:` step inside tests.

---

## Feeding the virtual camera frames

A loopback device with no producer shows a black screen. Put
`camera_feed: testsrc2` in the sim yaml and the engine starts an ffmpeg
test-pattern producer on every bound `/dev/videoN` **before** `de_camera`
scans — which is also what makes the devices enumerable at all
(`exclusive_caps` again). `sims/quadcopter_cam.yaml` and
`sims/copter_and_cameras.yaml` both do this; copy the pattern.

---

## Ports and verification

`de_camera` talks to its unit's `de_comm` over the s2s UDP bus on
`64000 + 10 * slot` (slot = `slot_base + unit index`), the same scheme
as every catalog module — e.g. slot 80 → `64800`.

```bash
./sim show sims/quadcopter_cam.yaml     # resolved unit, port, camera binding
./sim up sims/quadcopter_cam.yaml
./sim logs -s <sim> camquad1 de_camera  # tail the module's stdout
./sim ctl status                        # is the unit/module online
```

Then open the WebClient URL `sim up` prints — the video stream appears
on the unit card.

Ready-made examples to steal from:

| File | Shows |
|---|---|
| `sims/quadcopter_cam.yaml` | one camera copter on your own external server stack |
| `sims/copter_and_cameras.yaml` | camera copter + fixed camera post, label pinning, `camera_feed` |
| `blocks/copter_camera/` | minimal `add_modules: [de_camera]` block |
| `blocks/copter_cam3d/` | rendered camera: `de_yolo_ai` + `de_camera` on `DE-SRC`/`DE-AI` render pairs |
| `tests/camera/` | the automated camera scenarios (capture, events, ops) |
