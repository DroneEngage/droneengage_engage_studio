#!/usr/bin/env bash
#
# tracker_rss.sh — de_tracker RSS / reseed harness for RPI platforms.
#
#   looped person clip -> v4l2loopback -> de_perceive (onnx)
#   -> loopback -> de_tracker (locked via visual_tracker.lock)
#
# Samples the de_tracker VmRSS at ~2s through a locked window plus
# N unlock/relock cycles, then runs verify_log.py (CA-0015 contract:
# obs stream, lock invoke, 1043 fields, vanish/reacquire, unlock).
#
# Written for the CA-0042 measurement on rpi5.local; device names match
# the named v4l2loopback set on that host (~/scripts/sh_camera_create_named_vc.sh).
# Every path/knob is an env override - see README.md in this folder.
#
set -u
cd "$(dirname "$0")"

RUN_DIR="${RUN_DIR:-/tmp/tracker_rss_run}"
DE_COMM_BIN="${DE_COMM_BIN:-$HOME/drone_engage/de_comm/de_comm}"
PERCEIVE_BIN="${PERCEIVE_BIN:-$HOME/drone_engage/de_perceive/de_perceive}"
TRACKER_BIN="${TRACKER_BIN:-$HOME/de_code/droneengage_visual_tracker/bin/de_tracker}"
TRACKER_CONFIG="${TRACKER_CONFIG:-$HOME/drone_engage/de_tracking/de_tracker.config.module.json}"
TRACKER_LOCAL="${TRACKER_LOCAL:-$HOME/drone_engage/de_tracking/de_tracker.config.local}"
MODEL="${MODEL:-$HOME/drone_engage/de_perceive/yolo11n.onnx}"
TEST_IMAGE="${TEST_IMAGE:-$HOME/de_code/droneengage_yolo_ai/test-image-640x640.jpg}"
BUS_PORT="${BUS_PORT:-60000}"
VDEV_IN="${VDEV_IN:-/dev/video7}"      # DE-AI   (feeder -> perceive)
VDEV_OUT="${VDEV_OUT:-/dev/video3}"    # DE-CAM2 (perceive -> tracker input)
VDEV_TRK="${VDEV_TRK:-/dev/video4}"    # DE-TRK  (tracker output)
CLIP="$RUN_DIR/clip.mp4"
LOCK_WINDOW_S="${LOCK_WINDOW_S:-300}"
CYCLES="${CYCLES:-10}"
# de_databus python client (contains de_module.py) for bus_agent.py
DE_DATABUS_PY="${DE_DATABUS_PY:-$HOME/droneengage_simulation_land/tools/rpi/de_databus_py}"
# onnxruntime C++ lib dir for de_perceive (libonnxruntime.so.*) - empty if
# the binary already resolves its libs.
ORT_LIB="${ORT_LIB:-}"

export DE_COMM_PORT=$BUS_PORT
export DE_DATABUS_PY

PIDS=()
OVERALL=0
say()  { echo "[tracker_rss] $*"; }
fail() { echo "[tracker_rss][FAIL] $*" >&2; OVERALL=1; }

cleanup() {
    say "stopping processes"
    for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
    sleep 0.5
    for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill -9 "$p" 2>/dev/null; done
    pkill -9 -f "$RUN_DIR/de_comm.config.json" 2>/dev/null
    pkill -9 -f "$RUN_DIR/de_perceive.config.json" 2>/dev/null
    pkill -9 -f "bus_agent.py tap $RUN_DIR" 2>/dev/null
    say "logs in $RUN_DIR"
}
trap cleanup EXIT

wait_log_for() { # file, pattern, timeout_s
    local f="$1" pat="$2" t="$3"
    local end=$((SECONDS + t))
    while [ $SECONDS -lt $end ]; do
        [ -f "$f" ] && grep -q "$pat" "$f" && return 0
        sleep 0.3
    done
    return 1
}

wait_obs_person() { # timeout_s -> first confirmed person trk
    local end=$((SECONDS + $1)) trk=""
    while [ $SECONDS -lt $end ]; do
        trk=$(python3 - "$RUN_DIR/bus.jsonl" <<'EOF' 2>/dev/null
import json,sys
try:
    for line in open(sys.argv[1]):
        try: r = json.loads(line)
        except Exception: continue
        if r.get("mt") != 6546: continue
        for o in (r.get("ms") or {}).get("obs") or []:
            if o.get("cls") == "person" and "trk" in o:
                print(o["trk"]); sys.exit(0)
except FileNotFoundError:
    pass
EOF
)
        [ -n "$trk" ] && echo "$trk" && return 0
        sleep 0.5
    done
    return 1
}

rss_kb() { awk '/VmRSS/{print $2}' "/proc/$1/status" 2>/dev/null; }

mark() { echo "$(date +%s.%N),$1" >> "$RUN_DIR/events.csv"; }

rss_sampler() { # pid -> ts,rss_kb rows while alive
    local pid=$1
    while kill -0 "$pid" 2>/dev/null; do
        local r; r=$(rss_kb "$pid")
        [ -n "$r" ] && echo "$(date +%s.%N),$r" >> "$RUN_DIR/rss.csv"
        sleep 2
    done
}

# ---------------- preflight ----------------
for b in "$DE_COMM_BIN" "$PERCEIVE_BIN" "$TRACKER_BIN" "$MODEL" "$TEST_IMAGE" "$TRACKER_CONFIG"; do
    [ -e "$b" ] || { echo "missing: $b"; exit 2; }
done
for c in ffmpeg python3; do command -v "$c" >/dev/null || { echo "missing tool: $c"; exit 2; }; done
for d in "$VDEV_IN" "$VDEV_OUT" "$VDEV_TRK"; do [ -e "$d" ] || { echo "missing device $d"; exit 2; }; done
[ -f "$DE_DATABUS_PY/de_module.py" ] || { echo "missing de_databus python client at $DE_DATABUS_PY (set DE_DATABUS_PY)"; exit 2; }
mkdir -p "$RUN_DIR"; rm -f "$RUN_DIR"/bus.jsonl "$RUN_DIR"/*.log "$RUN_DIR"/*.csv
say "RUN_DIR=$RUN_DIR  window=${LOCK_WINDOW_S}s cycles=$CYCLES"
say "tracker: $TRACKER_BIN"

# ---------------- clip ----------------
if [ ! -s "$CLIP" ]; then
    say "generating clip (motion + timed person-vanish window)"
    ffmpeg -y -loglevel error -loop 1 -i "$TEST_IMAGE" -filter_complex \
 "[0:v]scale=800:800,zoompan=z='1.0+0.08*sin(2*PI*on/240)':x='iw/2-(iw/zoom/2)+30*sin(2*PI*on/90)':y='ih/2-(ih/zoom/2)':d=1:s=640x640:fps=15[p]; \
  [p]drawbox=x=0:y=60:w=450:h=580:c=black:t=fill:enable='lt(mod(t\,16)\,2)'[v]" \
        -map "[v]" -t 32 -c:v mpeg4 -q:v 3 -pix_fmt yuv420p "$CLIP" \
        || { echo "clip generation failed"; exit 2; }
fi

# ---------------- configs ----------------
cat > "$RUN_DIR/de_comm.config.json" <<EOF
{
    "module_id": "MT_1",
    "unit_type": "control_unit",
    "s2s_udp_listening_ip": "0.0.0.0",
    "s2s_udp_listening_port": "$BUS_PORT",
    "s2s_udp_packet_size": "8192",
    "use_unix_socket": false,
    "auth_ip": "127.0.0.1",
    "auth_port": 19408,
    "auth_verify_ssl": false,
    "use_local_comm_server": false,
    "userName": "demo@demo.droneengage.com",
    "accessCode": "demo1234",
    "unitID": "demo1",
    "groupID": "1",
    "unitDescription": "tracker_rss measurement unit",
    "logger_enabled": false,
    "logger_debug": false,
    "ping_server_rate_in_ms": 1500,
    "max_allowed_ping_delay_in_ms": 5000,
    "max_offline_count": 10000,
    "led_pins_enabled": false,
    "buzzer_pins_enabled": false,
    "sv_force_paged": false,
    "sv_comm_lost_s": 10,
    "sv_battery_low_pct": 25
}
EOF

read -r -d '' CLASS_NAMES <<'EOF' || true
["person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"]
EOF

cat > "$RUN_DIR/de_perceive.config.json" <<EOF
{
    "module_id": "AI-Generic-Detection",
    "s2s_udp_target_ip": "127.0.0.1",
    "s2s_udp_target_port": "$BUS_PORT",
    "s2s_udp_listening_ip": "127.0.0.1",
    "s2s_udp_listening_port": "61001",
    "s2s_udp_packet_size": "8192",
    "use_unix_socket": false,
    "auto_enable_detection": true,
    "backend": "onnx",
    "cam_id": "TRACKER-RSS-CAM",
    "source_video_device": "$VDEV_IN",
    "output_video_device": "$VDEV_OUT",
    "model_path": "$MODEL",
    "class_names": $CLASS_NAMES,
    "track": {"confirm_hits": 2, "max_misses": 60, "iou_min": 0.3, "min_conf": 0.25},
    "advanced_tracking": {"frame_to_skip_between_track_process": 1,
                          "overlay_frames_between_updates": 5,
                          "de_camera_draw": false}
}
EOF

cat > "$RUN_DIR/de_comm.local" <<EOF
{"module_key": "tracker-rss-de_comm"}
EOF
cat > "$RUN_DIR/de_perceive.config.local" <<EOF
{"module_key": "tracker-rss-de_perceive"}
EOF

# ---------------- processes ----------------
say "starting ffmpeg feeder -> $VDEV_IN"
ffmpeg -loglevel error -re -stream_loop -1 -i "$CLIP" \
    -f v4l2 -vcodec rawvideo -pix_fmt yuv420p "$VDEV_IN" \
    > "$RUN_DIR/ffmpeg.log" 2>&1 &
PIDS+=($!)
sleep 1

say "starting de_comm (module bus :$BUS_PORT)"
(cd "$RUN_DIR" && exec "$DE_COMM_BIN" -c "$RUN_DIR/de_comm.config.json" \
    -b "$RUN_DIR/de_comm.local" > "$RUN_DIR/de_comm.log" 2>&1) &
PIDS+=($!)
sleep 2

say "starting de_perceive (onnx, $VDEV_IN -> $VDEV_OUT)"
(cd "$RUN_DIR" && \
    { [ -n "$ORT_LIB" ] && export LD_LIBRARY_PATH="${ORT_LIB}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" || true; } && \
    exec "$PERCEIVE_BIN" -c "$RUN_DIR/de_perceive.config.json" \
    --bconfig "$RUN_DIR/de_perceive.config.local" > "$RUN_DIR/de_perceive.log" 2>&1) &
PIDS+=($!)
sleep 2

# v4l2loopback exclusive_caps flips the node to capture mode only once the
# producer actually streams; opening earlier fails with "could not open
# camera". Gate the tracker start on a real read, not a log line.
say "waiting for $VDEV_OUT to become capturable (v4l2loopback exclusive_caps)"
end=$((SECONDS + 45))
while [ $SECONDS -lt $end ]; do
    if timeout 4 ffmpeg -loglevel error -f v4l2 -i "$VDEV_OUT" -frames:v 1 -f null - 2>/dev/null; then
        say "$VDEV_OUT is producing"
        break
    fi
    sleep 1
done
[ $SECONDS -ge $end ] && { fail "$VDEV_OUT never became capturable"; tail -20 "$RUN_DIR/de_perceive.log"; exit 1; }

say "starting de_tracker ($VDEV_OUT -> $VDEV_TRK)"
(cd "$RUN_DIR" && exec "$TRACKER_BIN" -c "$TRACKER_CONFIG" \
    --bconfig "$TRACKER_LOCAL" > "$RUN_DIR/de_tracker.log" 2>&1) &
TRACKER_PID=$!
PIDS+=($TRACKER_PID)
sleep 2
kill -0 "$TRACKER_PID" || { fail "de_tracker exited early"; tail -20 "$RUN_DIR/de_tracker.log"; exit 1; }
say "de_tracker pid=$TRACKER_PID"

say "starting bus tap -> bus.jsonl"
python3 bus_agent.py tap "$RUN_DIR/bus.jsonl" > "$RUN_DIR/tap.log" 2>&1 &
PIDS+=($!)

say "starting RSS sampler (2s)"
echo "ts,rss_kb" > "$RUN_DIR/rss.csv"
echo "ts,event" > "$RUN_DIR/events.csv"
rss_sampler "$TRACKER_PID" &
PIDS+=($!)
mark "stack_up"

# ---------------- sequence ----------------
say "waiting for first confirmed person obs (6546)..."
TRK=$(wait_obs_person 90) || fail "no person obs with trk within 90s"
[ -n "${TRK:-}" ] && say "first person trk: $TRK"
mark "first_obs"

say "accumulating obs stream (~45s)..."
sleep 45
mark "pre_lock"

say "invoke visual_tracker.lock class=person"
mark "lock_invoke"
python3 bus_agent.py invoke visual_tracker lock '{"class":"person"}' 8 \
    | tee "$RUN_DIR/invoke_lock.out" || fail "lock invoke failed"

say "locked RSS window ${LOCK_WINDOW_S}s (clip vanish windows included)..."
sleep "$LOCK_WINDOW_S"
mark "window_end"

say "reseed cycles: $CYCLES x (unlock -> lock class=person)"
: > "$RUN_DIR/cycles.csv"
for i in $(seq 1 "$CYCLES"); do
    python3 bus_agent.py invoke visual_tracker unlock '{}' 6 > /dev/null 2>&1
    mark "cycle${i}_unlock"
    sleep 4
    python3 bus_agent.py invoke visual_tracker lock '{"class":"person"}' 6 > /dev/null 2>&1
    mark "cycle${i}_lock"
    sleep 6
    r=$(rss_kb "$TRACKER_PID")
    if [ -z "$r" ]; then
        fail "de_tracker died during cycle $i"
        mark "cycle${i}_DEAD"
        break
    fi
    echo "$i,$r" >> "$RUN_DIR/cycles.csv"
    say "cycle $i: RSS=${r}KB"
done

say "final unlock"
python3 bus_agent.py invoke visual_tracker unlock '{}' 8 \
    | tee "$RUN_DIR/invoke_unlock.out" || fail "unlock invoke failed"
sleep 3
mark "run_end"

# ---------------- verify + summarize ----------------
say "verifying captured bus log (CA-0015 checks)"
python3 verify_log.py "$RUN_DIR" && say "DEMO: PASS" || fail "verification failed"

say "RSS summary:"
python3 - "$RUN_DIR" <<'EOF'
import os, sys, statistics
run = sys.argv[1]
lines = open(os.path.join(run, "rss.csv")).read().splitlines()
rss = [(float(t), int(v)) for t, v in
       (l.split(",") for l in lines[1:] if l.strip())]
ev = {n.strip(): float(t) for t, n in
      (l.split(",", 1) for l in
       open(os.path.join(run, "events.csv")).read().splitlines()[1:] if l.strip())}

def at(t0):
    return min(rss, key=lambda r: abs(r[0] - t0))[1]

print(f"  samples: {len(rss)}  span: {rss[-1][0]-rss[0][0]:.0f}s")
if "pre_lock" in ev: print(f"  pre-lock RSS:             {at(ev['pre_lock'])} KB")
tL = ev.get("lock_invoke"); tE = ev.get("window_end")
if tL:
    rise = [v for t, v in rss if tL <= t <= tL + 20]
    if rise: print(f"  lock-rise peak (<=20s):     {max(rise)} KB")
if tL and tE:
    w = [(t, v) for t, v in rss if tL <= t <= tE]
    print(f"  locked window: {tE-tL:.0f}s  peak {max(v for _, v in w)} KB")
    for a, b in ((30, 90), (90, 150), (150, 210), (210, 270), (270, int(tE-tL))):
        seg = [v for t, v in w if tL + a <= t < tL + b]
        if seg:
            print(f"    [{a:>3},{b:>3}]s mean={statistics.mean(seg):7.0f} KB  min={min(seg)}  max={max(seg)}")
    for off in (30, 60, 120, 180, 240, 300):
        if rss[-1][0] >= tL + off - 1:
            print(f"    RSS at +{off:>3}s after lock: {at(tL+off)} KB")
    print(f"    +30s -> end delta:        {at(tE) - at(tL+30):+d} KB")
cpath = os.path.join(run, "cycles.csv")
if os.path.isfile(cpath):
    rows = [l.split(",") for l in open(cpath).read().splitlines() if l.strip()]
    if rows:
        print(f"  reseed cycles: {len(rows)}  per-cycle RSS KB: {[r[1] for r in rows]}")
        print(f"    first={rows[0][1]}  last={rows[-1][1]}  delta={int(rows[-1][1])-int(rows[0][1]):+d} KB")
EOF

exit $OVERALL
