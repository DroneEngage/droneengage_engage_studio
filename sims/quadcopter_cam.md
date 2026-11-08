# quadcopter_cam — how to run

Sim: ONE quad box with an onboard camera — ArduCopter SITL
(raw binary over TCP, like sitl_tcp.sh) + de_comm + de_mavlink +
de_camera — on **your own** server stack.
Add `--with viewer` for the Mavlink3DMap mav_bridge + vite dev page
comes up with the sim.

## Setup

- Start your servers first (`servers/sh_start_servers.sh`):
  authenticator on 19408, comm server on 9966. Nothing server-side is
  spawned — only this sim's GCS bridge, which logs in like a GCS client.
- Accounts: the GCS login needs the GCS permission bit, the unit login
  the unit bit, both on the same team — set `SIM_AUTH_*` /
  `SIM_UNIT_AUTH_*` in `sim.local.env`, or pass
  `./sim up sims/quadcopter_cam.yaml --user me@x.com --password secret
  --auth-server 203.0.113.9:19408`. Per-unit `account:` with `{name}`
  expansion is supported in the yaml.
- `de_camera` binary reachable — `DE_CAMERA_DIR`/`DE_CAMERA_BIN` in
  `sim.local.env` (the camera repo builds via its `local/sh_do_all.sh`
  WebRTC pipeline, not `./build.sh`).
- The named V4L2 pool. **Once per boot** (the only sudo step):
  `sudo tools/setup_v4l2loopback.sh` — also reachable as
  `./sim vcam --pool`. It creates `DE-CAM1`..`DE-CAM10`, `DE-SRC-*`/
  `DE-AI-*` render pairs and the camera repo's named-VC labels
  (`DE-TRK`, `DE-RPI`, `DE-THERMAL`, `DE-AI`, `DE-GIMBAL`, `SIM-CAM1`)
  in one modprobe with kernel-picked indices and `exclusive_caps`;
  `--persist` survives reboot. `camquad1` binds `DE-CAM1` via the
  unit's default `cameras: 1` — this sim fails fast on a host without
  the pool until it runs once.
- The sim file declares `camera_feed: testsrc2`, so `sim up` also
  spawns pid-tracked ffmpeg test-pattern producers on every bound
  loopback — exclusive_caps loopbacks only advertise CAPTURE while a
  producer streams. A real device bound with `--cam 0` never gets a
  feed written into it.
- Media: stills/recordings land in `/tmp/camquad1_de_camera_media`
  (per-instance `/tmp/<unit>_<module>_media`), durable outside the
  disposable `var/run` dir.
- Live add: `./sim ctl add_module camquad1 de_camera` hot-adds a second
  camera instance on the next free DE-CAM label.
- `MAVLINK3DMAP_DIR` in `sim.local.env` for the viewer.

## Run

```bash
./sim up sims/quadcopter_cam.yaml
./sim ctl -s quadcopter_cam status        # prints the viewer + webclient links
./sim status -s quadcopter_cam --open-web
./sim ctl -s quadcopter_cam arm camquad1  # after ~1-2 min GPS/EKF warm-up
```

Runs in parallel with `quadcopter.yaml` / `plane.yaml` / `rover.yaml` —
each sim has its own slot_base.

## Teardown

```bash
./sim down quadcopter_cam
```

## Ports

slot_base 80: camquad1 sysid 80 (pilot 6560, fcb 8460, de_comm 62800,
de_mavlink 63800, de_camera 64800), ctl 62089. External server wss 9966,
auth 19408.
