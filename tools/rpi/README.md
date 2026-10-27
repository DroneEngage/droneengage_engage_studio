# tools/rpi — RPI platform hardware test scripts

Shell-level tests that run **on a real Pi** against real `de_*` binaries
and `v4l2loopback` devices — the hardware counterpart of the `tests/`
simulation library (which stays de.scenario/1 only). Written for the
`rpi5.local` build+test host; device defaults match its named loopback
set (`DE-AI`=/dev/video7, `DE-CAM2`=/dev/video3, `DE-TRK`=/dev/video4).

Everything is self-contained: `bus_agent.py` (module-bus tap + capability
invoke), `verify_log.py` (CA-0015 contract checks) and `de_databus_py/`
(the python bus client) ship in this folder.

## tracker_rss.sh — de_tracker RSS + reseed harness

Builds the real pipeline and measures the tracker's memory behaviour:

```
person clip --ffmpeg--> VDEV_IN --> de_perceive (onnx, 6546 obs)
    --> VDEV_OUT --> de_tracker --> VDEV_TRK
                       ^
                       +-- visual_tracker.lock {class:"person"} via bus
```

Sequence: obs warm-up -> lock -> `LOCK_WINDOW_S` locked window (the clip
blanks the person periodically, so natural re-acquires happen) ->
`CYCLES` x unlock/relock -> `verify_log.py`. `VmRSS` is sampled every 2 s
into `rss.csv`; events and per-cycle values go to `events.csv` /
`cycles.csv`. Exit 0 = no failures (a dead tracker mid-cycle fails).

### Requirements on the Pi

- `ffmpeg`, `python3`, the named v4l2loopback devices (or set VDEV_* to
  real paths).
- `de_comm`, `de_perceive` binaries + a yolo `.onnx` model + a person
  test image (`TEST_IMAGE`).
- `de_tracker` binary + its config/local files.
- If `de_perceive` was built for the onnx backend it needs
  `libonnxruntime.so.*` — point `ORT_LIB` at the dir:

      ORT_LIB=/opt/onnxruntime/lib ./tracker_rss.sh

  (`LD_LIBRARY_PATH` is prepended only for the perceive process.)

### Env knobs (defaults are the rpi5 layout)

| Var | Default | Meaning |
|---|---|---|
| `RUN_DIR` | `/tmp/tracker_rss_run` | logs + captures land here |
| `DE_COMM_BIN` | `~/drone_engage/de_comm/de_comm` | broker binary |
| `PERCEIVE_BIN` | `~/drone_engage/de_perceive/de_perceive` | obs producer |
| `TRACKER_BIN` | `~/de_code/droneengage_visual_tracker/bin/de_tracker` | module under test |
| `TRACKER_CONFIG` / `TRACKER_LOCAL` | `~/drone_engage/de_tracking/...` | production config (auto algo) |
| `MODEL` | `~/drone_engage/de_perceive/yolo11n.onnx` | detection model |
| `TEST_IMAGE` | `~/de_code/droneengage_yolo_ai/test-image-640x640.jpg` | clip source |
| `BUS_PORT` | `60000` | de_comm module-bus port |
| `VDEV_IN` / `VDEV_OUT` / `VDEV_TRK` | `/dev/video7` / `3` / `4` | feeder->perceive->tracker chain |
| `LOCK_WINDOW_S` | `300` | locked RSS window |
| `CYCLES` | `10` | unlock/relock count |
| `DE_DATABUS_PY` | this folder's `de_databus_py/` | python bus client |
| `ORT_LIB` | empty | onnxruntime C++ lib dir |

### Outputs (`$RUN_DIR`)

`rss.csv` (ts,rss_kb), `events.csv` (phase markers), `cycles.csv`
(per-cycle RSS), `bus.jsonl` (every intermodule message), `de_tracker.log`
(seed lines, FATAL/abort count), `invoke_*.out`, `de_comm/de_perceive`
logs. The final summary prints the locked-window band means, +30s/end
delta, and the reseed series.

### Forcing an algorithm for the RSS run

`tracker_rss.sh` takes the tracker config from `TRACKER_CONFIG`, so any
`tracker_algorithm_index` / `tracker_auto_csrt_min_mem_mb` combination can
be measured without touching the production config:

```bash
# generate a variant config, then:
TRACKER_CONFIG=/tmp/de_tracker.kcf.json LOCK_WINDOW_S=90 CYCLES=0 \
    ORT_LIB=~/opt/onnxruntime-linux-aarch64-1.18.1/lib \
    ./tracker_rss.sh
```

e.g. `tracking: {tracker_algorithm_index: -1, tracker_auto_csrt_min_mem_mb:
99999}` forces the AUTO->KCF path on a large-RAM board — the same code
path a 1GB board takes.

## tracker_auto_probe.sh — AUTO algorithm-selection probe

Checks `tracker_algorithm_index=-1` (AUTO, CA-0044) resolution: starts
`de_tracker` briefly per generated config variant and asserts the
`tracker_algorithm_index AUTO: MemTotal=.. threshold=.. -> X` startup
line. Expectations are derived from the host's real MemTotal, so the same
script expects CSRT on a big board and KCF on a 1GB one. No bus or video
chain needed — the AUTO line is printed before the camera opens.

```
./tracker_auto_probe.sh        # PASS/FAIL per variant, exit !=0 on failure
```

Variants: `auto` (default threshold), `forced_kcf` (threshold=MemTotal+1),
`forced_csrt` (threshold=1), `bad_thresh` (threshold=0 must be ignored),
`explicit` (index 7 must print no AUTO line).

Env knobs: `TRACKER_BIN`, `RUN_DIR`, `VDEV_SRC`, `VDEV_OUT`, `BUS_PORT`,
`PROBE_S` (per-variant run seconds, default 6).

### Gotchas learned on rpi5 (CA-0042)

- `v4l2loopback` `exclusive_caps=Y`: the node only shows capture caps
  once a producer actually streams — the script gates the tracker start
  on a real read, not on a log marker. Starting earlier fails with
  "could not open camera".
- `de_perceive` onnx backend: pip's `onnxruntime` wheel is statically
  linked and does **not** ship `libonnxruntime.so.*` — fetch the
  official `onnxruntime-linux-aarch64-<ver>.tgz` release tarball.
- The `de_snd` vendored python `de_common` is stale for this client —
  use `de_databus_py/` here (it carries the 6544/6545/6546 constants).

## Bluetooth FC link
The Pi's Bluetooth can be tested against SITL with
[`../hil/bt_spp_relay`](../hil/bt_spp_relay/README.md): run the relay on the
SITL host, `rfcomm connect /dev/rfcomm0 <HOST_MAC> 22` on the Pi, and point
de_mavlink's `fcb_connection_uri` at `/dev/rfcomm0` (serial).
