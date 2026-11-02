#!/usr/bin/env python3
"""render_probe.py - P5-15 frame readback for one rendered camera unit.

Opened as a `render` owner proc by simlib.gen_render (one per rendered
camera unit): connects to the scenario's world stream for dims and the
event channel, and taps the render sink's per-unit frame stream
(ws://.../tap/<unit_idx>) - a copy of the same [32-byte header][JPEG]
frames the sink's producer ffmpeg writes into the unit's DE-SRC-k
loopback.

DE-0052: the probe used to read that device through ffmpeg itself, but
an exclusive_caps v4l2loopback carries a producer + exactly ONE
consumer - the unit's de_camera/de_yolo_ai already holds it, so the
probe's open/streamon took the slot and the module's StartCapture came
back EBUSY. The tapped JPEG is decoded per frame through ffmpeg and
the 16-cell frame-number strip the render farm draws into the bottom
8 pixel rows (frontend frame_meta.js drawStrip) is read off the real
pixels. Every decoded frame posts back as an event - truth.jsonl rows
{src: render_probe, unit, frame_no, t_unix_ms} - so the render_latency
assert can join them with the farm's frame_meta on (unit, frame_no).

    python3 engine/render_probe.py --unit cam1 \\
        --sink ws://127.0.0.1:61020/tap/0 \\
        --stream ws://127.0.0.1:61010 --mark var/run/<sim>/render
"""

import argparse
import json
import struct
import subprocess
import sys
import time

STRIP_CELLS = 16
STRIP_ROWS = 8
FRAME_HEADER_BYTES = 32


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


def connect_retry(url, what):
    """the probe launches with the render procs during `sim up` - the
    world stream only opens inside the scenario runner and the sink's
    WS listener comes up with the render_sink proc, so wait for them
    instead of dying on the first refused connect."""
    from websockets.sync.client import connect
    deadline = time.time() + 120
    while True:
        try:
            return connect(url)
        except OSError:
            if time.time() > deadline:
                print(f"render_probe: {what} never opened",
                      file=sys.stderr)
                sys.exit(1)
            time.sleep(0.5)


def decode_gray(jpeg, mark):
    """One JPEG frame -> gray luma bytes via ffmpeg. Per-frame decode
    (not a persistent child): image2pipe holds a frame until the next
    image or EOF, and a long-lived child orphans on `sim down`
    (DE-0052). -metadata stamps the run dir into argv so the stray-pid
    sweep can claim a child that outlives us."""
    r = subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-nostdin",
         "-f", "image2pipe", "-vcodec", "mjpeg", "-i", "-",
         "-f", "rawvideo", "-pix_fmt", "gray",
         "-metadata", f"comment={mark}", "-"],
        input=jpeg, capture_output=True, timeout=30)
    return r.stdout


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sink", required=True,
                    help="render_sink tap ws:// url (/tap/<unit_idx>)")
    ap.add_argument("--unit", required=True, help="unit name for rows")
    ap.add_argument("--stream", required=True,
                    help="world stream ws:// url (hello dims + events)")
    ap.add_argument("--mark", default="",
                    help="run dir stamped into the decode child's argv "
                         "so the sim's stray-pid sweep can claim it")
    ap.add_argument("--width", type=int, default=0,
                    help="override dims (else frame header, else hello)")
    ap.add_argument("--height", type=int, default=0)
    a = ap.parse_args()

    stream = connect_retry(a.stream, "world stream")
    hello = json.loads(stream.recv(timeout=15))
    if hello.get("t") != "hello":
        print(f"render_probe: first frame not hello: {hello.get('t')}",
              file=sys.stderr)
        return 1
    cam = next((u["camera"] for u in hello.get("units", [])
                if u.get("name") == a.unit and u.get("camera")), {})
    hint_w = a.width or cam.get("img_w") or 640
    hint_h = a.height or cam.get("img_h") or 480

    sink = connect_retry(a.sink, "render sink")
    print(f"render_probe {a.unit}: {a.sink} ~{hint_w}x{hint_h}",
          file=sys.stderr)
    for msg in sink:
        if not isinstance(msg, (bytes, bytearray)) \
                or len(msg) < FRAME_HEADER_BYTES:
            continue
        (_uidx, stream_id, _fno, w, h, jpeg_len) = \
            struct.unpack_from("<HHIHHI", msg, 0)
        if stream_id != 0:
            continue
        w = a.width or w or hint_w
        h = a.height or h or hint_h
        try:
            luma = decode_gray(
                bytes(msg[FRAME_HEADER_BYTES:
                          FRAME_HEADER_BYTES + jpeg_len]), a.mark)
        except (subprocess.SubprocessError, OSError):
            continue
        if len(luma) < w * h:
            continue                 # decode miss or dims mismatch
        fn = read_strip(luma, w, h)
        stream.send(json.dumps(
            {"t": "event", "src": "render_probe",
             "kind": "frame_seen", "unit": a.unit,
             "frame_no": fn,
             "t_unix_ms": int(time.time() * 1000)}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
