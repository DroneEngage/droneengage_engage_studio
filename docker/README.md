# Docker quickstart image

One image with everything a scenario needs: ArduPilot SITL, the DE
binaries (`de_comm`, `de_mavlink`), the sim engine, the vendored databus
python client (`tools/rpi/de_databus_py`) and the Mavlink3DMap viewer.
Nothing is mounted from the host — every upstream dependency is fetched
inside the build at a pinned ref (`ARG *_REF` at the top of the
`Dockerfile`).

## Build

```bash
docker build -f docker/Dockerfile -t de-sim .          # from the repo root
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
docker run -it --network host de-sim
```

That is the whole quickstart. The default `CMD` runs
`view docker/canopy_quickstart.yaml`: the fake server, SITL copter,
`de_comm`, `de_mavlink`, the stub AI and the viewer all come up, the
timeline uploads a lawnmower mission over the forest-canopy world,
arms and flies it, then the run parks with the viewer live. The
console prints the URL to open in a browser:

    render viewer: http://127.0.0.1:8080/?world=ws://127.0.0.1:61010&mavlink=ws://127.0.0.1:8811

(the port block shifts if another sim holds it — copy the line the
console prints). Watch the copter fly the sweep in 3D, then `Ctrl-C`
to release the hold; the run ends with the graded assertions
(coverage, POD, stub detections, precision).

Other entry points:

```bash
docker run --network host de-sim doctor docker/canopy_quickstart.yaml
docker run --network host de-sim list
docker run -it --network host de-sim \
    view scenarios/examples/mission_square.yaml   # another scenario
docker run -it --network host de-sim bash         # a shell in the image
```

### Why `--network host`

The sim binds dozens of per-slot loopback ports (SITL TCP links, the
fake server, the world-stream and MAVLink WebSockets, the vite dev
server) and the printed viewer URL assumes the browser reaches them on
localhost. Host networking keeps the whole model intact and co-exists
with sims running on the host — the slot allocator skips occupied port
blocks and prints the shifted viewer URL. Linux only; on macOS/Windows
run the container inside a Linux VM or use the native setup
(`sim.local.env`).

## What runs inside, and what does not

**In:** the canopy quickstart flies the full search pattern for real —
the mission is uploaded through the classic `Upload_DE_Mission`
(mt 1075) path, coverage/POD graders and stub-AI detection assertions
all pass, and the viewer shows the live 3D flight.

**Gated upstream (public mirrors), not in the image:**

- **Supervised op deploys.** The public `de_comm`/`de_mavlink` mirrors
  predate the mission-supervisor / nav-adapter protocol, so scenarios
  with `do: deploy` (operation packages, e.g.
  `scenarios/demos/world_canopy_viewer.yaml`) cannot run in the image.
  The image patches two public-build gaps at build time: the catalog
  ready marker (`nav adapter hello` is never journaled →
  `ready_log_re: "Communicator Server Found"`) and the databus python
  client's missing message ids (the public `droneengage_databus`
  mirror predates 6544/6545/6546 → symlinked to the vendored
  `tools/rpi/de_databus_py` copy). The quickstart scenario therefore
  grades detections at the stub (`src: ai`) rather than the
  operator-facing `detect.detected` surface, which is produced by the
  private geo projector.
- **Op verification is skipped.** `tools/verify_wrap.js` needs the
  webclient's `verify/` pipeline, which the public webclient mirror
  does not ship. The wrapper is dockerignored out of the image, so the
  runner records `"P5-06 CLI not yet available"` in `verify.json` —
  the designed skip path.
- **Hardware-adjacent scenarios stay host-side:** `v4l2loopback`
  camera units (`de_camera`/`de_yolo_ai`, `render: {mode: cameras|
  full}`), the real `droneengage_server`/authenticator stack
  (`transport: real`), and libfaketime fault scenarios.

When the public mirrors catch up, `view
scenarios/demos/world_canopy_viewer.yaml` works in the image with no
changes — the deploy path is the same binary protocol.

## Files

- `Dockerfile` — the multi-stage build (SITL, DE modules, viewer;
  runtime on Ubuntu 24.04 + Node 22), all upstream refs pinned.
- `canopy_quickstart.yaml` — the image's default scenario: the
  forest-canopy sweep flown via the classic mission upload, no
  supervisor needed.
- `Dockerfile.dockerignore` — BuildKit picks it up next to the
  Dockerfile; keeps `var/`, `.git`, host files and the unpublished
  verify wrapper out of the image.
- `fetch-repo.sh` — shallow fetch of a pinned ref inside the build,
  including submodule init.
- `entrypoint.sh` — `cd` into the repo and pass through to `./sim`;
  `bash`/`sh` give a shell, no args runs the quickstart.
