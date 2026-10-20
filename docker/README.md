# Docker quickstart image

One image with everything a scenario needs: ArduPilot SITL, the DE
binaries (`de_comm`, `de_mavlink`), the sim engine, the vendored databus
python client (`tools/rpi/de_databus_py`) and the Mavlink3DMap viewer.
Nothing is mounted from the host — every upstream dependency is fetched
inside the build at a pinned ref (`ARG *_REF` at the top of the
`Dockerfile`).

## Status

The image builds and brings the whole stack up — fake server, SITL,
`de_comm`, `de_mavlink`, the stub AI, the viewer (URL printed, page
serves). Verified with `sim doctor` (all required checks pass) and a
live `sim view` run.

Two gaps are upstream, not in the image:

- **The public `DroneEngage/droneengage_communication` and
  `droneengage_mavlink` mirrors predate the mission-supervisor /
  nav-adapter protocol.** de_mavlink's catalog `ready_re` waits for
  `nav adapter hello`, a line only a supervisor-capable `de_comm`
  journals — so `sim up` currently times out at `wait_ready` and the
  deploy/timeline never fires. The pinned refs are the newest public
  ones; the missing code (de_mission_supervisor, nav_adapter) is not
  published on any public ref or tag.
- **Op verification is skipped.** `tools/verify_wrap.js` needs the
  webclient's `verify/` pipeline, which the public webclient mirror
  does not ship. The wrapper is dockerignored out of the image, so the
  runner records `"P5-06 CLI not yet available"` in `verify.json` and
  proceeds — the designed skip path (deploy-spec ops are skipped by
  the wrapper anyway).

Once the public mirrors carry those pieces, the full
`<5 min to a flying drone` path should work with no image changes —
the pins already resolve to the newest public refs.

## Build

From the repository root:

```bash
docker build -f docker/Dockerfile -t de-sim .
```

The first build compiles SITL and the two C++ modules, so it takes a
while (~15 min on a typical desktop, most of it the ArduPilot copter
build). Rebuilds reuse the layer cache; a `--build-arg` bump only
redoes what changed, e.g.:

```bash
docker build -f docker/Dockerfile -t de-sim \
    --build-arg ARDUPILOT_REF=Copter-4.6.3 .
```

## Run

```bash
# the canopy-search demo: fleet comes up, the copter flies the sweep,
# the run parks with the viewer live, Ctrl-C finishes + grades
docker run -it --network host de-sim

# the same, spelled out
docker run -it --network host de-sim \
    view scenarios/demos/world_canopy_viewer.yaml

# any other sim command works too
docker run --network host de-sim list
docker run --network host de-sim doctor scenarios/demos/world_canopy_viewer.yaml
docker run -it --network host de-sim bash     # a shell inside the image
```

When the sim is up the console prints the viewer URL:

    render viewer: http://127.0.0.1:8080/?world=ws://127.0.0.1:61010&mavlink=ws://127.0.0.1:8811

Open that in a browser. `Ctrl-C` releases the hold and the run ends
with the graded assertions.

### Why `--network host`

The sim binds dozens of per-slot loopback ports (SITL TCP links, the
fake server, the world-stream and MAVLink WebSockets, the vite dev
server on `127.0.0.1:8080`) and the printed viewer URL assumes the
browser reaches them on localhost. Host networking keeps the whole
model intact and co-exists with sims running on the host (the slot
allocator skips occupied port blocks). Linux only; on macOS/Windows
run the container inside a Linux VM or use the native setup
(`sim.local.env`).

### What the image does *not* cover

Scenarios needing hardware-adjacent pieces stay host-side: anything
with `v4l2loopback` camera devices (`de_camera`/`de_yolo_ai` units,
`render: {mode: cameras|full}`), the real `droneengage_server`/
authenticator stack (`transport: real` sims), and libfaketime fault
scenarios. Everything else — world scenarios on the fake transport,
the stubs, the graders, the viewer — runs in the image.

## Files

- `Dockerfile` — the multi-stage build (SITL, DE modules, viewer;
  runtime on Ubuntu 24.04 + Node 22).
- `Dockerfile.dockerignore` — BuildKit picks it up next to the
  Dockerfile; keeps `var/`, `.git`, host files and the unpublished
  verify wrapper out of the image.
- `fetch-repo.sh` — shallow fetch of a pinned ref inside the build,
  including submodule init.
- `entrypoint.sh` — `cd` into the repo and pass through to `./sim`;
  `bash`/`sh` give a shell, no args runs the canopy demo.
