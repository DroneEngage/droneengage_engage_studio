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

FORCE=0
PERSIST=0
FILL=0
ARGS=()
for a in "$@"; do
    case "$a" in
        --force)   FORCE=1 ;;
        --persist) PERSIST=1 ;;
        --fill)    FILL=1 ;;
        *)         ARGS+=("$a") ;;
    esac
done
CAMS="${ARGS[0]:-10}"
PAIRS="${ARGS[1]:-2}"
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

in_set() {   # label $1 in csv $2 ?
    echo ",$2," | grep -q ",$1,"
}

# --fill: invoked from the /etc/modprobe.d install hook --persist writes
# when the kernel cap forces ctl-added devices - the hook hands us the
# load, so bypass it (--ignore-install), then add the rest
if [ "$FILL" = "1" ]; then
    modprobe --ignore-install v4l2loopback devices=1 \
             card_label="$(echo "$LABELS" | cut -d, -f1)" \
             exclusive_caps="1" || exit 1
    for l in $(echo "$LABELS" | tr ',' ' ' | cut -d' ' -f2-); do
        v4l2loopback-ctl add -n "$l" -x 1 || exit 1
    done
    exit 0
fi

# ---- loaded-set classification ----------------------------------------
if lsmod | grep -q ^v4l2loopback; then
    # read the labels the loaded DEVICES actually carry, from sysfs -
    # NOT the card_label modparam: the kernel-cap ctl-fill path below
    # (devices=1 + v4l2loopback-ctl add) leaves that param holding only
    # the first label (or absent entirely), which made every rerun see
    # the full pool as missing and reload it - orphaning live producers.
    cur=""
    for d in /sys/class/video4linux/video*; do
        n=$(basename "$d")
        [ -d "/sys/devices/virtual/video4linux/$n" ] || continue
        l=$(cat "$d/name" 2>/dev/null)
        [ -n "$l" ] && cur="${cur:+$cur,}$l"
    done
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

    # never drop a device in use - name the holders; this gate precedes
    # ANY device drop (modprobe -r here is the only one today). fuser
    # first; when it reports no holders at all a single /proc fd sweep
    # confirms before dropping - a restricted sudo env or missing
    # psmisc must not let a silent fuser slip a reload through while a
    # producer streams.
    holders=""
    for nr in $(ls /sys/class/video4linux/ | sed 's/^video//'); do
        [ -d "/sys/devices/virtual/video4linux/video${nr}" ] || continue
        pids=$(fuser "/dev/video${nr}" 2>/dev/null)
        [ -n "$pids" ] && holders="$holders /dev/video${nr}:$pids"
    done
    if [ -z "$holders" ]; then
        for fd in /proc/[0-9]*/fd/*; do
            t=$(readlink "$fd" 2>/dev/null) || continue
            case "$t" in
                /dev/video*)
                    n=$(basename "$t")
                    [ -d "/sys/devices/virtual/video4linux/$n" ] \
                        || continue
                    pid=${fd#/proc/}
                    holders="$holders $t:${pid%%/*}" ;;
            esac
        done
    fi
    if [ -n "$holders" ]; then
        echo "loopback device(s) in use - refusing to reload:$holders"
        exit 1
    fi
    echo "upgrading loaded set (missing:${missing} ) - reloading"
    modprobe -r v4l2loopback || {
        echo "could not unload v4l2loopback"; exit 1; }
fi

# one modprobe when the kernel build takes the whole list; v4l2loopback
# has a compile-time device cap (EINVAL above it), so a bigger pool
# falls back to a single loaded device plus v4l2loopback-ctl add for
# the rest - same labels, same exclusive_caps
FIRST=$(echo "$LABELS" | cut -d, -f1)
CTL_FILL=0
if ! modprobe v4l2loopback devices=${DEVICES} \
        card_label="${LABELS}" exclusive_caps="${CAPS}"; then
    if ! command -v v4l2loopback-ctl >/dev/null; then
        echo "modprobe devices=${DEVICES} rejected and no "
        echo "v4l2loopback-ctl to add the rest - shrink the pool: "
        echo "$0 <cam_count> <pair_count>"
        exit 1
    fi
    lsmod | grep -q ^v4l2loopback || \
        modprobe v4l2loopback devices=1 card_label="${FIRST}" \
                 exclusive_caps="1" || {
        echo "modprobe failed - install v4l2loopback-dkms first"; exit 1; }
    for l in $(echo "$LABELS" | tr ',' ' ' | cut -d' ' -f2-); do
        v4l2loopback-ctl add -n "$l" -x 1 || {
            echo "v4l2loopback-ctl add $l failed"; exit 1; }
    done
    CTL_FILL=1
    echo "kernel device cap below ${DEVICES} - pool filled via v4l2loopback-ctl"
fi

# ---- persist across reboots -------------------------------------------
if [ "$PERSIST" = "1" ]; then
    echo v4l2loopback > /etc/modules-load.d/v4l2loopback.conf
    if [ "$CTL_FILL" = "1" ]; then
        # an options line hits the same cap at boot - the install hook
        # reproduces the full pool through this script's --fill mode
        echo "install v4l2loopback $0 --fill" \
             > /etc/modprobe.d/de-vcam.conf
    else
        echo "options v4l2loopback devices=${DEVICES} card_label=\"${LABELS}\" exclusive_caps=\"${CAPS}\"" \
             > /etc/modprobe.d/de-vcam.conf
    fi
    echo "persisted: /etc/modules-load.d/v4l2loopback.conf + /etc/modprobe.d/de-vcam.conf"
fi

echo "pool labels:"
for d in /sys/class/video4linux/video*; do
    [ -d "/sys/devices/virtual/video4linux/$(basename "$d")" ] || continue
    printf "  /dev/%s  %s\n" "$(basename "$d")" "$(cat "$d/name" 2>/dev/null)"
done
