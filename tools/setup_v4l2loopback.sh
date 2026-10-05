#!/bin/bash
# setup_v4l2loopback.sh N - one-time root setup for P5-15 rendered
# cameras. Creates 2N v4l2loopback devices labelled DE-SRC-k / DE-AI-k
# for k in 0..N-1: the render sink's ffmpeg writes the Mavlink3DMap
# render to DE-SRC-k, de_yolo_ai_generic reads it and writes annotated
# output to DE-AI-k, which de_camera streams to the WebClient.
#
#   sudo tools/setup_v4l2loopback.sh 4        # 4 camera units worth
#
# The sim engine itself never needs root: `sim up` checks the labels
# exist and points here when they don't.

N="${1:-4}"
case "$N" in ''|*[!0-9]*) echo "usage: $0 <camera_unit_count>"; exit 2;; esac

LABELS=""
NRS=""
CAPS=""
for ((k=0; k<N; k++)); do
    LABELS="${LABELS}DE-SRC-${k},DE-AI-${k},"
    NRS="${NRS}$((2*k)),$((2*k+1)),"
    CAPS="${CAPS}1,1,"
done
LABELS="${LABELS%,}"
NRS="${NRS%,}"
CAPS="${CAPS%,}"

DEVICES=$((2*N))
echo "v4l2loopback: ${DEVICES} devices, labels ${LABELS}"

if lsmod | grep -q v4l2loopback; then
    cur=$(cat /sys/module/v4l2loopback/parameters/card_label 2>/dev/null)
    if [ "$cur" = "$LABELS" ]; then
        echo "already loaded with these labels - nothing to do"
        exit 0
    fi
    echo "loaded with different labels (${cur}) - reloading"
    modprobe -r v4l2loopback || {
        echo "could not unload (a device is open)"; exit 1; }
fi

modprobe v4l2loopback devices=${DEVICES} video_nr="${NRS}" \
         card_label="${LABELS}" exclusive_caps="${CAPS}" || {
    echo "modprobe failed - install v4l2loopback-dkms first"; exit 1; }

echo "labels now:"
for nr in $(echo "$NRS" | tr ',' ' '); do
    printf "  /dev/video%s  %s\n" "$nr" \
           "$(cat /sys/class/video4linux/video${nr}/name 2>/dev/null)"
done
