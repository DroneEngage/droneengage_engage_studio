#!/usr/bin/env python3
"""worldstream_probe.py - a de.worldstream/1 test/debug client (P5-13).

Connects to a running scenario's world stream, prints the hello
summary and the measured tick rate, and can inject a fake event
(which must land in truth.jsonl with src render):

    python3 engine/worldstream_probe.py --port 61010 --seconds 3
    python3 engine/worldstream_probe.py --port 61010 \
        --inject '{"kind":"payload_landed","unit":"cam1","lat":1.0,
                   "lng":2.0,"alt_m":0,"hit":"ground","impact_mps":7.9}'

Exit 0 when a hello arrived and the tick rate was >= 9 Hz, 1
otherwise - the live render_feed_smoke acceptance checks use it.
"""

import argparse
import asyncio
import json
import sys
import time


async def probe(port, seconds, inject):
    import websockets
    uri = f"ws://127.0.0.1:{port}"
    async with websockets.connect(uri) as ws:
        raw = await asyncio.wait_for(ws.recv(), 10)
        hello = json.loads(raw)
        if hello.get("t") != "hello":
            print(f"first frame is not hello: {hello.get('t')}")
            return 1
        print(json.dumps({
            "schema": hello.get("schema"), "sim": hello.get("sim"),
            "entities": len(hello.get("entities", [])),
            "obstacles": len(hello.get("obstacles", [])),
            "units": len(hello.get("units", []))}))
        if inject is not None:
            ev = dict(inject)
            ev["t"] = "event"
            ev.setdefault("t_unix_ms", int(time.time() * 1000))
            await ws.send(json.dumps(ev))
            print(f"injected: {json.dumps(inject)}")
        ticks, cmds = 0, 0
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                raw = await asyncio.wait_for(
                    ws.recv(), max(0.05, seconds - (time.time() - t0)))
            except asyncio.TimeoutError:
                break
            msg = json.loads(raw)
            if msg.get("t") == "tick":
                ticks += 1
            elif msg.get("t") == "cmd":
                cmds += 1
                print(f"cmd: {json.dumps(msg)}")
        dt = time.time() - t0
        rate = ticks / dt if dt else 0.0
        print(json.dumps({"seconds": round(dt, 1), "ticks": ticks,
                          "tick_hz": round(rate, 1), "cmds": cmds}))
        return 0 if rate >= 9.0 else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--inject", default=None,
                    help="JSON event payload to send once "
                         "(kind/unit/...)")
    a = ap.parse_args()
    inject = json.loads(a.inject) if a.inject else None
    return asyncio.run(probe(a.port, a.seconds, inject))


if __name__ == "__main__":
    sys.exit(main())
