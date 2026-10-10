#!/usr/bin/env python3
"""gcs_bridge.py - one persistent GCS session on the REAL droneengage_server.

Started by `sim up` when the sim uses `server: {type: real}`. It logs in as
the GCS party GCS_FLEET (through droneengage_authenticator when the sim sets
`auth: true`, else straight to the comm server's local mode), keeps the
websocket up (re-connects on drop) and serves the same newline control
protocol as fake_fleet_server.py on the sim's control port, so ctl.py talks
to either backend the same way:

    list                         units heard recently ("d:<party>") + "g:GCS_FLEET"
    send / to / upload / action / event / commline   as in fake_fleet_server
    listen <secs> <mt[,mt..]>    stream received frames of those types (JSON lines)
    req <secs> <reply_mt> <json> send <json>, stream <reply_mt> replies until
                                 ms.more is false or <secs> pass

Link faults (link / kick / clock / stats) are not the bridge's business:
ctl.py sends them to the fake server, or - with server:
{type: real, link_proxy: true} (SL-0009) - to the affected unit's own
link_proxy; a raw line here still answers "ERR ...". The bridge's GCS
session is never proxied.

Unlike the fake server, the real server does NOT stamp `sd` - it routes
the sender id the client wrote (checked 2026-10-01, js_chat_routing.js
only injects `p`). So the bridge sets "sd": GCS_FLEET itself, and an
explicit "sd" (`event`, forged `send`) passes through unchanged.
"""

import asyncio
import json
import os
import ssl
import sys
import time
import urllib.parse
import urllib.request

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import simlib
import fake_fleet_server as ffs        # build_message / pretty_rx only

ffs.PRETTY = True
PARTY = simlib.GCS_PARTY  # the sim's GCS party (main() sets it from the manifest)
SEEN_S = 60               # a unit counts as online if heard this recently
MAVLINK_MT = 6502           # TYPE_AndruavMessage_MAVLINK - raw MAVLink payload
ADSB_MSGID = 246            # MAVLINK_MSG_ID_ADSB_VEHICLE

ws_conn = None              # current websocket (None while reconnecting)
seen = {}                   # party -> last rx time
listeners = []              # [(set(mt), asyncio.Queue)]
last_id_req = 0.0
_mav = None                 # lazy pymavlink parser for binary frames


def _ssl_ctx():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


def login(srv):
    """-> (host, port, key). Auth mode does the web login (/w/wl) like the
    webclient; local mode connects straight to the comm server."""
    if not srv["auth"]:
        return srv["ip"], srv["port"], "gcs_bridge"
    body = urllib.parse.urlencode({
        "acc": srv["account"]["user"], "pwd": srv["account"]["code"],
        "gr": "1", "app": "de", "ex": "de", "ver": "sim"}).encode()
    req = urllib.request.Request(
        f"https://{srv['auth_ip']}:{srv['auth_port']}/w/wl", data=body,
        headers={"Content-Type": "application/x-www-form-urlencoded"})
    with urllib.request.urlopen(req, context=_ssl_ctx(), timeout=10) as r:
        rep = json.loads(r.read())
    if rep.get("e") not in (0, None) or "cs" not in rep:
        raise RuntimeError(f"login refused: {rep.get('em', rep)}")
    cs = rep["cs"]
    return cs["g"], int(cs["h"]), cs["f"]


def _mav_parse(payload):
    """Decode the MAVLink frames of an mt-6502 payload (lazy parser -
    pymavlink is a sim dependency, not every bridge run needs it)."""
    global _mav
    try:
        if _mav is None:
            from pymavlink import mavutil
            _mav = mavutil.mavlink.MAVLink(None)
        return _mav.parse_buffer(bytearray(payload)) or []
    except Exception:
        return []


def _binary_rx(raw):
    """A binary intermodule frame: `JSON\\0` header + payload. mt 6502
    wraps the raw MAVLink bytes a unit forwards to the GCS; ADSB_VEHICLE
    inside one is logged as a one-line JSON `[adsb] {...}` row - the
    SL-0016 display-path evidence: a world aircraft injected through the
    fcb proxy surfaces at the GCS only here (the webclient's own
    ADSB_VEHICLE case is disabled, and nothing else consumes it)."""
    nul = raw.find(b"\0")
    if nul <= 0:
        return
    try:
        hdr = json.loads(raw[:nul])
    except ValueError:
        return
    if hdr.get("mt") != MAVLINK_MT:
        return
    for m in _mav_parse(raw[nul + 1:]):
        if m is None or m.get_msgId() != ADSB_MSGID:
            continue
        row = {"t_ms": int(time.time() * 1000), "sd": hdr.get("sd"),
               "icao": "%06x" % (m.ICAO_address & 0xFFFFFF),
               "callsign": m.callsign,
               "lat": round(m.lat * 1e-7, 7),
               "lng": round(m.lon * 1e-7, 7),
               "alt_m": round(m.altitude * 0.001, 1),
               "heading_deg": round(m.heading * 0.01, 1),
               "spd_mps": round(m.hor_velocity * 0.01, 1),
               "emitter_type": m.emitter_type}
        print("[adsb] " + json.dumps(row), flush=True)


async def send(msg):
    if ws_conn is None:
        raise RuntimeError("not connected to the comm server")
    await ws_conn.send(json.dumps(msg))


async def request_ids():
    """Ask every unit for its ID (1005 {C:1004}) so `list` sees units
    that have nothing else to tell the GCS yet."""
    global last_id_req
    if time.time() - last_id_req < 3 or ws_conn is None:
        return
    last_id_req = time.time()
    try:
        await send({"ty": "i", "mt": 1005, "tg": "_AGN_",
                    "sd": PARTY, "ms": {"C": 1004}})
    except Exception:
        pass


async def session(srv):
    """Keep one GCS websocket up forever."""
    global ws_conn
    import websockets
    while True:
        # a clean close (server restart) ends `async for` without an
        # exception - the retry delay must exist on that path too
        delay = 5
        try:
            host, port, key = await asyncio.get_running_loop().run_in_executor(
                None, login, srv)
            async with websockets.connect(f"wss://{host}:{port}", ssl=_ssl_ctx(),
                                          compression=None, max_size=None,
                                          ping_interval=20) as ws:
                await ws.send(json.dumps({"ty": "s", "mt": "de_auth", "f": key,
                                          "s": PARTY, "at": "g"}))
                # the real server pushes 9007 (connected + clock) before
                # the ack - skip frames until de_auth_ack
                t_end = time.time() + 10
                while True:
                    raw = await asyncio.wait_for(
                        ws.recv(), max(0.1, t_end - time.time()))
                    if not isinstance(raw, str):
                        continue    # binary intermodule frame (e.g. mt 6502)
                    ack = json.loads(raw)
                    if ack.get("mt") == "de_auth_ack":
                        break
                if ack.get("r") != "ok":
                    raise RuntimeError(f"de_auth refused: {ack}")
                ws_conn = ws
                print(f"[bridge] GCS {PARTY} connected to "
                      f"wss://{host}:{port}", flush=True)
                await request_ids()
                async for raw in ws:
                    if not isinstance(raw, str):
                        # binary intermodule frame (e.g. mt 6502): sniff
                        # ADSB_VEHICLE for the SL-0016 evidence path
                        _binary_rx(raw)
                        continue
                    try:
                        msg = json.loads(raw)
                    except ValueError:
                        continue
                    sd = msg.get("sd")
                    if sd and sd != PARTY:
                        seen[sd] = time.time()
                    mt = msg.get("mt")
                    if mt not in (9005, 9007, 1004):
                        ffs.pretty_rx(mt, msg.get("ms", {}), sd, msg.get("tg", ""))
                    for mts, q in listeners:
                        if mt in mts:
                            q.put_nowait(msg)
        except Exception as e:
            # 429: the authenticator allows ~10 logins/min per IP - a 5 s
            # retry keeps the limiter saturated forever, so back off past
            # its window. Anything else retries fast.
            delay = 65 if isinstance(e, urllib.error.HTTPError) \
                and e.code == 429 else 5
            print(f"[bridge] session down: {e!r} - retrying in {delay} s",
                  flush=True)
        ws_conn = None
        await asyncio.sleep(delay)


async def control(reader, writer):
    def say(text):
        writer.write((text + "\n").encode())

    async def stream(mts, secs, first=None, stop_on_last=None):
        if stop_on_last is None:
            stop_on_last = first is not None and not isinstance(first, list)
        q = asyncio.Queue()
        ent = (mts, q)
        listeners.append(ent)
        try:
            if first is not None:
                for one in (first if isinstance(first, list) else [first]):
                    one.setdefault("sd", PARTY)
                    await send(one)
            end = time.time() + secs
            while time.time() < end:
                try:
                    m = await asyncio.wait_for(q.get(), end - time.time())
                except asyncio.TimeoutError:
                    break
                say(json.dumps(m))
                await writer.drain()
                if stop_on_last and not (m.get("ms") or {}).get("more"):
                    break
        finally:
            listeners.remove(ent)

    try:
        while True:
            raw = await reader.readline()
            if not raw:
                return
            line = raw.decode().strip()
            if not line:
                continue
            print(f"[ctrl] {time.strftime('%H:%M:%S')} {line[:120]}", flush=True)
            try:
                if line == "list":
                    await request_ids()
                    if ws_conn is not None:
                        say(f"g:{PARTY}")
                    now = time.time()
                    for p, t in sorted(seen.items()):
                        if now - t < SEEN_S:
                            say(f"d:{p}")
                elif line.startswith("listen "):
                    _, secs, mts = line.split(None, 2)
                    await stream({int(x) for x in mts.split(",")}, float(secs))
                elif line.startswith("req "):
                    _, secs, mt, body = line.split(None, 3)
                    await stream({int(mt)}, float(secs), json.loads(body))
                elif line.startswith("reqm "):
                    # reqm <secs> <mt[,mt]> <[msg,...]> - send a batch of
                    # frames as the GCS, then stream every matching reply
                    # (P4A-09 op control round-trips: N sends, N replies)
                    _, secs, mts, body = line.split(None, 3)
                    await stream({int(x) for x in mts.split(",")},
                                 float(secs), json.loads(body),
                                 stop_on_last=False)
                elif line.split()[0] in ("link", "kick", "clock", "stats"):
                    # ctl routes these to the unit link proxies itself;
                    # a raw line here means it was sent by hand
                    say("ERR link faults need the fake server "
                        "(sim server: {type: fake}), or "
                        "server: {type: real, link_proxy: true} with "
                        "./sim ctl link <unit> ... (per-unit proxy)")
                else:
                    _tg, msg = ffs.build_message(line)
                    if msg is None:
                        say(f"ERR unknown command {line.split()[0]!r}")
                    else:
                        msg.setdefault("sd", PARTY)
                        await send(msg)
                        say("OK")
            except Exception as e:
                say(f"ERR {e}")
            await writer.drain()
    except Exception as e:
        print(f"[ctrl] handler error: {e!r}", flush=True)
    finally:
        writer.close()


async def main():
    global PARTY
    srv = simlib.load_manifest(sys.argv[1] if len(sys.argv) > 1 else None)["server"]
    PARTY = srv.get("gcs_party") or simlib.GCS_PARTY
    # control lines carry whole packages (`send <1085>`) - the default
    # 64 KiB stream limit resets the conn on a big plan
    ctl = await asyncio.start_server(control, "127.0.0.1",
                                     srv["control_port"], limit=16 << 20)
    print(f"[bridge] control on 127.0.0.1:{srv['control_port']} "
          f"(server {srv['port']}, auth={srv['auth']})", flush=True)
    async with ctl:
        await asyncio.gather(session(srv), ctl.serve_forever())


if __name__ == "__main__":
    asyncio.run(main())
