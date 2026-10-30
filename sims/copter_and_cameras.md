# copter_and_cameras — how to run

Camera fleet: a camera copter (`camcopter`, copter + de_camera) and a
fixed camera post (`tower1`, de_comm + de_camera, no FC) on the spawned
real server.

## Setup

- `de_camera` binary reachable — `DE_CAMERA_DIR`/`DE_CAMERA_BIN` in
  `sim.local.env` (the camera repo builds via its `local/sh_do_all.sh`
  WebRTC pipeline, not `./build.sh`).
- A V4L2 device per camera unit. Once:
  `./sim vcam sims/copter_and_cameras.yaml` derives the indices each
  unit's de_camera opens (camcopter scans 2–3, tower1 is pinned to 3)
  and `sudo modprobe`s the missing v4l2loopback devices itself —
  the only sudo step; `sim up` stays unprivileged. Real devices are
  never touched. (`--print` shows the modprobe line, `--reload`
  replaces an already-loaded module's devices.)
- The sim file declares `camera_feed: testsrc2`, so `sim up` also
  spawns pid-tracked ffmpeg test-pattern producers on every configured
  index *before* the units start — required because `exclusive_caps`
  only advertises CAPTURE while a producer streams. `sim down` kills
  the feeders; their logs land under `var/run/<sim>/feed/logs/`.
  Alternatives: real cameras on those indices, or hand-run
  `video_clock.sh N` feeders (drop `camera_feed` then).
- Devices missing at `sim up` fail fast naming them (`./sim vcam` +
  `camera_feed` hint) instead of hanging in `wait_ready`.

## Run

```bash
./sim up sims/copter_and_cameras.yaml
./sim ctl -s copter_and_cameras status
./sim status -s copter_and_cameras --open-web   # camera feeds show in the GCS
```

## Teardown

```bash
./sim down copter_and_cameras
```

## Ports

slot_base 40: camcopter sysid 40 (pilot 6160, fcb 8060, de_comm 62400,
de_mavlink 63400, de_camera 64400); tower1 de_comm 62410, de_camera
64410. Server wss 62008, control 62009.
