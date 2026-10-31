# copter_and_cameras — how to run

Camera fleet: a camera copter (`camcopter`, copter + de_camera) and a
fixed camera post (`tower1`, de_comm + de_camera, no FC) on the spawned
real server.

## Setup

- `de_camera` binary reachable — `DE_CAMERA_DIR`/`DE_CAMERA_BIN` in
  `sim.local.env` (the camera repo builds via its `local/sh_do_all.sh`
  WebRTC pipeline, not `./build.sh`).
- The named V4L2 pool. **Once per boot** (the only sudo step):
  `sudo tools/setup_v4l2loopback.sh` — also reachable as
  `./sim vcam --pool`. It creates the whole pool in one modprobe with
  kernel-picked indices and `exclusive_caps`: `DE-CAM1`..`DE-CAM10`,
  `DE-SRC-*`/`DE-AI-*` render pairs, plus the camera repo's named-VC
  labels (`DE-TRK`, `DE-RPI`, `DE-THERMAL`, `DE-AI`, `DE-GIMBAL`,
  `SIM-CAM1`) — a superset of
  `droneengage_camera_2025/scripts/sh_camera_create_named_vc.sh`, so
  either can be loaded first. `--persist` writes the
  `/etc/modules-load.d` + `/etc/modprobe.d` entries so the pool
  survives reboot. Camera sims fail fast on a host without the pool
  until it runs once.
- The sim file declares `camera_feed: testsrc2`, so `sim up` also
  spawns pid-tracked ffmpeg test-pattern producers on every bound
  loopback *before* the units start — required because
  `exclusive_caps` only advertises CAPTURE while a producer streams.
  Real devices never get a feed (`--cam 0` on a real webcam is safe).
  `sim down` kills the feeders; their logs land under
  `var/run/<sim>/feed/logs/`.
- Missing pool labels fail fast naming `tools/setup_v4l2loopback.sh`
  instead of hanging in `wait_ready`.
- `./sim up sims/copter_and_cameras.yaml --cam 0` pins a single-camera
  sim's instance to a real device (or `--cam DE-CAM5` a pool label).
- Live add: `./sim ctl add_module camcopter de_camera` adds another
  camera instance on the next free DE-CAM label (feed included);
  `remove_module` reverses it.
- Media: each instance saves stills/recordings under
  `/tmp/<unit>_<module>_media` (`/tmp/camcopter_de_camera_media`,
  `/tmp/tower1_de_camera_media`) - durable outside `var/run`.

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
64410. Server wss 62008, control 62009. Extra camera instances stride
+2 (de_camera2 = 64402, its udp_proxy on +1, max 5).
