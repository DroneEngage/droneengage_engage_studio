#!/usr/bin/env python3
"""render_probe.py - P5-15 V4L2 readback for one rendered camera unit.

Opened as a `render` owner proc by simlib.gen_render (one per
de_yolo_ai unit): connects to the scenario's world stream to learn the
unit's camera dims from hello.units[*].camera, then reads the unit's
DE-SRC-k loopback device through ffmpeg and decodes the 16-cell frame
number strip the render farm draws into the bottom 8 pixel rows
(frontend frame_meta.js drawStrip). Every decoded frame is posted
back as an event - truth.jsonl rows {src: render_probe, unit,
frame_no, t_unix_ms} - so the render_latency assert can join them with
the farm's frame_meta events on (unit, frame_no).

    python3 engine/render_probe.py --device /dev/video6 --unit cam1 \
        --stream ws://127.0.0.1:61010
"""

import argparse
import json
import subprocess
import sys
import time

STRIP_CELLS = 16
STRIP_ROWS = 8


def read_strip(luma, width, height):
    """16-bit frame_no out of the bottom STRIP_ROWS rows - the JS
    drawStrip layout: cell i carries bit i, LSB first, white=1."""
    cell_w = width // STRIP_CELLS
    y_mid = height - STRIP_ROWS // 2
    frame_no = 0
    for i in range(STRIP_CELLS):
        x_mid = i * cell_w + cell_w // 2
        if luma[y_mid * width + x_mid] > 127:
            frame_no |= 1 << i
    return frame_no


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", required=True, help="/dev/videoN to read")
    ap.add_argument("--unit", required=True, help="unit name for rows")
    ap.add_argument("--stream", required=True,
                    help="world stream ws:// url (hello supplies dims)")
    ap.add_argument("--width", type=int, default=0,
                    help="override dims (else hello.units camera)")
    ap.add_argument("--height", type=int, default=0)
    a = ap.parse_args()

    from websockets.sync.client import connect
    # the probe launches with the render procs during `sim up` - the
    # world stream only opens inside the scenario runner afterwards,
    # so wait for it instead of dying on the first refused connect
    ws = None
    deadline = time.time() + 120
    while ws is None:
        try:
            ws = connect(a.stream)
        except OSError:
            if time.time() > deadline:
                print("render_probe: world stream never opened",
                      file=sys.stderr)
                return 1
            time.sleep(0.5)
    hello = json.loads(ws.recv(timeout=15))
    if hello.get("t") != "hello":
        print(f"render_probe: first frame not hello: {hello.get('t')}",
              file=sys.stderr)
        return 1
    cam = next((u["camera"] for u in hello.get("units", [])
                if u.get("name") == a.unit and u.get("camera")), None)
    w = a.width or (cam or {}).get("img_w", 640)
    h = a.height or (cam or {}).get("img_h", 480)
    print(f"render_probe {a.unit}: {a.device} {w}x{h}", file=sys.stderr)

    # v4l2loopback serves yuv420p (what the sink's ffmpeg writes); read
    # raw luma frames - the strip needs no chroma
    proc = subprocess.Popen(
        ["ffmpeg", "-loglevel", "error",
         "-f", "v4l2", "-input_format", "yuv420p",
         "-video_size", f"{w}x{h}", "-i", a.device,
         "-f", "rawvideo", "-pix_fmt", "gray", "-"],
        stdout=subprocess.PIPE)
    frame_bytes = w * h
    try:
        while True:
            buf = b""
            while len(buf) < frame_bytes:
                chunk = proc.stdout.read(frame_bytes - len(buf))
                if not chunk:
                    return 0           # device/ffmpeg ended
                buf += chunk
            fn = read_strip(buf, w, h)
            ws.send(json.dumps(
                {"t": "event", "src": "render_probe",
                 "kind": "frame_seen", "unit": a.unit,
                 "frame_no": fn,
                 "t_unix_ms": int(time.time() * 1000)}))
    finally:
        proc.terminate()


if __name__ == "__main__":
    sys.exit(main())
