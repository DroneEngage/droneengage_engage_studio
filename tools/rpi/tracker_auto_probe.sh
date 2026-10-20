#!/usr/bin/env bash
#
# tracker_auto_probe.sh — de_tracker AUTO algorithm-selection probe.
#
# Exercises tracker_algorithm_index=-1 (TRACKER_ALGORITHM_AUTO, CA-0044):
# the module resolves the algorithm at startup from /proc/meminfo
# MemTotal against tracking.tracker_auto_csrt_min_mem_mb (default 3072 MB)
# and logs "tracker_algorithm_index AUTO: MemTotal=.. threshold=.. -> X".
#
# No bus, no video chain — the AUTO line is emitted while the config is
# read, before the camera is opened. Each variant runs de_tracker for a
# few seconds and the log line is checked against the expectation derived
# from this host's real MemTotal, so the same script is correct on a 1GB
# board (expects KCF) and an 8GB board (expects CSRT).
#
# Variants:
#   auto        -1, no threshold            -> CSRT iff MemTotal >= 3072
#   forced_kcf  -1, threshold = MemTotal+1  -> KCF
#   forced_csrt -1, threshold = 1           -> CSRT
#   bad_thresh  -1, threshold = 0           -> ignored, behaves as auto
#   explicit    7                           -> no AUTO line (unchanged path)
#
# For the memory-behaviour side of the same feature see tracker_rss.sh
# (README has the forced-KCF TRACKER_CONFIG recipe).
#
set -u
cd "$(dirname "$0")"

RUN_DIR="${RUN_DIR:-/tmp/tracker_auto_probe}"
TRACKER_BIN="${TRACKER_BIN:-$HOME/de_code/droneengage_visual_tracker/bin/de_tracker}"
VDEV_SRC="${VDEV_SRC:-/dev/video7}"      # any existing node; never read to completion
VDEV_OUT="${VDEV_OUT:-/dev/video4}"
BUS_PORT="${BUS_PORT:-"64000"}"          # nothing listens - AUTO is decided before bus use
PROBE_S="${PROBE_S:-6}"                  # per-variant run time
DEFAULT_THRESHOLD_MB=3072

PIDS=()
OVERALL=0
say()  { echo "[auto_probe] $*"; }
fail() { echo "[auto_probe][FAIL] $*" >&2; OVERALL=1; }

cleanup() {
    for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill "$p" 2>/dev/null; done
    sleep 0.3
    for p in "${PIDS[@]:-}"; do [ -n "$p" ] && kill -9 "$p" 2>/dev/null; done
    pkill -9 -f "$RUN_DIR/de_tracker" 2>/dev/null
}
trap cleanup EXIT

# ---------------- preflight ----------------
[ -e "$TRACKER_BIN" ] || { echo "missing: $TRACKER_BIN (set TRACKER_BIN)"; exit 2; }
MEM_KB=$(awk '/^MemTotal:/{print $2}' /proc/meminfo)
[ -n "$MEM_KB" ] || { echo "cannot read MemTotal from /proc/meminfo"; exit 2; }
MEM_MB=$((MEM_KB / 1024))
say "MemTotal=${MEM_MB}MB  tracker: $TRACKER_BIN"
mkdir -p "$RUN_DIR"; rm -f "$RUN_DIR"/probe_*.log "$RUN_DIR"/*.json

cat > "$RUN_DIR/de_tracker.local" <<EOF
{"module_key": "auto-probe-de_tracker"}
EOF

mk_cfg() { # $1=outfile $2=extra tracking fields
cat > "$1" <<EOF
{
    "module_id": "VIS_TRK_PRO",
    "s2s_udp_target_ip": "127.0.0.1",
    "s2s_udp_target_port": "$BUS_PORT",
    "s2s_udp_listening_ip": "127.0.0.1",
    "s2s_udp_listening_port": "64001",
    "s2s_udp_packet_size": "8192",
    "use_unix_socket": false,
    "camera": {
        "source_video_device": "$VDEV_SRC",
        "output_video_device": "$VDEV_OUT"
    },
    "tracking": {
        $2
        "camera_orientation": 0,
        "camera_flipped": false,
        "tracking_camera_direction": 4,
        "crosshair": false,
        "de_camera_draw": false
    }
}
EOF
}

mk_cfg "$RUN_DIR/de_tracker.auto.json"        '"tracker_algorithm_index": -1,'
mk_cfg "$RUN_DIR/de_tracker.forced_kcf.json"  "\"tracker_algorithm_index\": -1, \"tracker_auto_csrt_min_mem_mb\": $((MEM_MB + 1)),"
mk_cfg "$RUN_DIR/de_tracker.forced_csrt.json" '"tracker_algorithm_index": -1, "tracker_auto_csrt_min_mem_mb": 1,'
mk_cfg "$RUN_DIR/de_tracker.bad_thresh.json"  '"tracker_algorithm_index": -1, "tracker_auto_csrt_min_mem_mb": 0,'
mk_cfg "$RUN_DIR/de_tracker.explicit.json"    '"tracker_algorithm_index": 7,'

# ---------------- probe ----------------
# Runs one variant, then checks:
#   expect=CSRT|KCF  -> AUTO line present and resolves to it
#   expect=NONE      -> no AUTO line at all (explicit index)
#   thresh=N         -> logged threshold must be N (validation check)
probe() { # $1=cfg $2=tag $3=expect $4=expected_threshold_mb_or_empty
    local cfg="$1" tag="$2" expect="$3" exp_th="${4:-}"
    timeout "$PROBE_S" "$TRACKER_BIN" -c "$cfg" \
        --bconfig "$RUN_DIR/de_tracker.local" > "$RUN_DIR/probe_$tag.log" 2>&1 &
    local pid=$!; PIDS+=($pid); wait "$pid" 2>/dev/null
    local line; line=$(grep -o 'tracker_algorithm_index AUTO:.*' "$RUN_DIR/probe_$tag.log" | head -1 | sed 's/\x1b\[[0-9;]*m//g')

    if [ "$expect" = "NONE" ]; then
        if [ -z "$line" ]; then say "$tag: PASS (explicit index, no AUTO line)"; return 0
        else fail "$tag: explicit index produced AUTO line: $line"; return 1; fi
    fi
    [ -n "$line" ] || { fail "$tag: no AUTO line"; tail -5 "$RUN_DIR/probe_$tag.log"; return 1; }

    local got_algo got_th
    got_algo=$(echo "$line" | sed 's/.*-> *//')
    got_th=$(echo "$line" | grep -o 'threshold=[0-9-]*' | cut -d= -f2)
    [ "$got_algo" = "$expect" ] || { fail "$tag: resolved $got_algo, expected $expect ($line)"; return 1; }
    if [ -n "$exp_th" ] && [ "$got_th" != "$exp_th" ]; then
        fail "$tag: threshold=$got_th, expected $exp_th ($line)"; return 1
    fi
    say "$tag: PASS ($line)"
    return 0
}

auto_expect=KCF; [ "$MEM_MB" -ge "$DEFAULT_THRESHOLD_MB" ] && auto_expect=CSRT

say "running variants (PROBE_S=${PROBE_S}s each)"
probe "$RUN_DIR/de_tracker.auto.json"       auto        "$auto_expect" "$DEFAULT_THRESHOLD_MB"
probe "$RUN_DIR/de_tracker.forced_kcf.json" forced_kcf  "KCF"          "$((MEM_MB + 1))"
probe "$RUN_DIR/de_tracker.forced_csrt.json" forced_csrt "CSRT"        "1"
probe "$RUN_DIR/de_tracker.bad_thresh.json" bad_thresh  "$auto_expect" "$DEFAULT_THRESHOLD_MB"
probe "$RUN_DIR/de_tracker.explicit.json"   explicit    "NONE"

if [ "$OVERALL" -eq 0 ]; then say "ALL PASS - logs in $RUN_DIR"; else say "FAILURES - logs in $RUN_DIR"; fi
exit $OVERALL
