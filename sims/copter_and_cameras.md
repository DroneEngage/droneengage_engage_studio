# copter_and_cameras — how to run

Camera fleet: a camera copter (`camcopter`, copter + de_camera) and a
fixed camera post (`tower1`, de_comm + de_camera, no FC) on the spawned
real server.

## Setup

- `de_camera` binary reachable — `DE_CAMERA_DIR`/`DE_CAMERA_BIN` in
  `sim.local.env` (the camera repo builds via its `local/sh_do_all.sh`
  WebRTC pipeline, not `./build.sh`).
- A V4L2 device per camera unit —
  `droneengage_camera/scripts/create_virtual_videos.sh`, or the named
  devices via `SIM_V4L2_SRC_LABELS`/`SIM_V4L2_AI_LABELS` in
  `sim.local.env`. tower1 is pinned to `/dev/video3`
  (`camera_start_index = camera_end_index = 3`).
- `video_clock.sh` / `video_colors.sh` test-pattern feeders exist under
  the camera repo's `scripts/` if no real camera is attached.

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

Every port is leased from the machine-wide allocator at `up` — `./sim
ports -s copter_and_cameras` lists what was actually bound.
`slot_base: 40` is identity only (camcopter sysid 40). The render feeds
are leased too; the v4l2 device labels stay machine-global, so this sim
still cannot share the machine with another rendered-camera fleet.
