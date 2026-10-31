#!/bin/bash
# setup_v4l2loopback.sh [cam_count [pair_count]] [--force] [--persist]
# - one-time root setup creating the whole named v4l2loopback pool in
# one modprobe. Labels are the contract; indices are left to the kernel
# (no video_nr) so the pool never collides with a real webcam or
# capture card.
#
#   sudo tools/setup_v4l2loopback.sh              # default pool
#   sudo tools/setup_v4l2loopback.sh 12 3         # 12 DE-CAM, 3 SRC/AI pairs
#   sudo tools/setup_v4l2loopback.sh --persist    # also survives reboot
#   sudo tools/setup_v4l2loopback.sh --force      # reload over a foreign set
#
# Default label set (cam_count=10, pair_count=2 = 20 devices):
#   DE-CAM1..DE-CAM10        de_camera instances (the sim's camera pool)
#   DE-SRC-k / DE-AI-k       render farm -> de_yolo_ai pairs (k < pair_count)
#   DE-TRK DE-GIMBAL DE-THERMAL DE-RPI DE-AI SIM-CAM1
#                            reserved / the camera repo's named-vc set
#   (strict superset of droneengage_camera_2025/scripts/
#    sh_camera_create_named_vc.sh - there is only one v4l2loopback
#    module per host, the two scripts must not wipe each other)
#
# The sim engine itself never needs root: `sim up` checks the labels
# exist and points here when they don't.

CAMS="${1:-10}"
PAIRS="${2:-2}"
FORCE=0
PERSIST=0
for a in "$@"; do
    case "$a" in
        --force)   FORCE=1 ;;
        --persist) PERSIST=1 ;;
    esac
done
case "$CAMS"  in ''|*[!0-9]*) echo "usage: $0 [cam_count [pair_count]] [--force] [--persist]"; exit 2;; esac
case "$PAIRS" in ''|*[!0-9]*) echo "usage: $0 [cam_count [pair_count]] [--force] [--persist]"; exit 2;; esac

# ---- wanted label set -------------------------------------------------
LABELS=""
for ((k=1; k<=CAMS; k++)); do  LABELS="${LABELS}DE-CAM${k},"; done
for ((k=0; k<PAIRS; k++)); do  LABELS="${LABELS}DE-SRC-${k},DE-AI-${k},"; done
LABELS="${LABELS}DE-TRK,DE-GIMBAL,DE-THERMAL,DE-RPI,DE-AI,SIM-CAM1"
DEVICES=$(echo "$LABELS" | tr ',' '\n' | wc -l)
CAPS=$(printf '1,%.'0s $(seq $DEVICES) | sed 's/,$//')

in_set() {   # label $1 in csv $2 ?
    echo ",$2," | grep -q ",$1,"
}

# ---- loaded-set classification ----------------------------------------
if lsmod | grep -q ^v4l2loopback; then
    cur=$(cat /sys/module/v4l2loopback/parameters/card_label 2>/dev/null)
    missing=""
    for l in $(echo "$LABELS" | tr ',' ' '); do
        in_set "$l" "$cur" || missing="$missing $l"
    done
    if [ -z "$missing" ]; then
        echo "already loaded with every wanted label - nothing to do"
        exit 0
    fi
    # missing wanted labels: reload is allowed only when the loaded set
    # is a subset of ours (an earlier pool or the named-vc set) - a set
    # carrying foreign labels is refused without --force
    foreign=""
    for l in $(echo "$cur" | tr ',' ' '); do
        in_set "$l" "$LABELS" || foreign="$foreign $l"
    done
    if [ -n "$foreign" ] && [ "$FORCE" != "1" ]; then
        echo "loaded with foreign label(s)${foreign} - refusing to reload"
        echo "(missing:${missing} )  rerun with --force to replace"
        exit 1
    fi

    # never drop a device in use - name the holders
    holders=""
    for nr in $(ls /sys/class/video4linux/ | sed 's/^video//'); do
        [ -d "/sys/devices/virtual/video4linux/video${nr}" ] || continue
        pids=$(fuser "/dev/video${nr}" 2>/dev/null)
        [ -n "$pids" ] && holders="$holders /dev/video${nr}:$pids"
    done
    if [ -n "$holders" ]; then
        echo "loopback device(s) in use - refusing to reload:$holders"
        exit 1
    fi
    echo "upgrading loaded set (missing:${missing} ) - reloading"
    modprobe -r v4l2loopback || {
        echo "could not unload v4l2loopback"; exit 1; }
fi

modprobe v4l2loopback devices=${DEVICES} \
         card_label="${LABELS}" exclusive_caps="${CAPS}" || {
    echo "modprobe failed - install v4l2loopback-dkms first"; exit 1; }

# ---- persist across reboots -------------------------------------------
if [ "$PERSIST" = "1" ]; then
    echo v4l2loopback > /etc/modules-load.d/v4l2loopback.conf
    echo "options v4l2loopback devices=${DEVICES} card_label=\"${LABELS}\" exclusive_caps=\"${CAPS}\"" \
         > /etc/modprobe.d/de-vcam.conf
    echo "persisted: /etc/modules-load.d/v4l2loopback.conf + /etc/modprobe.d/de-vcam.conf"
fi

echo "pool labels:"
for d in /sys/class/video4linux/video*; do
    [ -d "/sys/devices/virtual/video4linux/$(basename "$d")" ] || continue
    printf "  /dev/%s  %s\n" "$(basename "$d")" "$(cat "$d/name" 2>/dev/null)"
done
