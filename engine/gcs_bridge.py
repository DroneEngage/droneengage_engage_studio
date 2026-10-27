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
seen_name = {}              # party -> last UD heard on a 1004 id frame
WATCH = False               # watch mode: print party join/leave/id events
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
                        if sd not in seen and WATCH:
                            print(f"{time.strftime('%H:%M:%S')} [watch] + "
                                  f"{sd}", flush=True)
                        seen[sd] = time.time()
                        if WATCH and msg.get("mt") == 1004:
                            ud = (msg.get("ms") or {}).get("UD") or ""
                            if ud != seen_name.get(sd):
                                seen_name[sd] = ud
                                print(f"{time.strftime('%H:%M:%S')} [watch]   "
                                      f"{sd} UD={ud}", flush=True)
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


async def _watch_reaper():
    """Watch mode: ask units for their ids so quiet-but-up parties stay
    listed, and announce a party as left once it goes silent for SEEN_S."""
    while True:
        await asyncio.sleep(5)
        if ws_conn is None:
            continue
        try:
            await request_ids()
        except Exception:
            pass
        now = time.time()
        for p, t in list(seen.items()):
            if now - t > SEEN_S:
                del seen[p]
                print(f"{time.strftime('%H:%M:%S')} [watch] - {p} "
                      f"{seen_name.pop(p, '')}".rstrip(), flush=True)


def _parse_args(argv):
    import argparse
    p = argparse.ArgumentParser(
        prog="gcs_bridge",
        description="GCS session on a DroneEngage comm server. Manifest mode "
                    "is started by `sim up`; any connection flag switches to "
                    "watch mode for a stack with no sim manifest (e.g. the "
                    "external servers of `./sim up de_servers`) and prints "
                    "party join/leave/id events to stdout.")
    p.add_argument("sim", nargs="?",
                   help="sim name (manifest mode, as used by sim up/ctl)")
    p.add_argument("--watch", action="store_true",
                   help="manifest mode too: also print join/leave/id events")
    p.add_argument("--server-ip", help="comm server host (default 127.0.0.1)")
    p.add_argument("--server-port", type=int,
                   help="comm server wss port (default 9966)")
    p.add_argument("--auth-ip",
                   help="authenticator host for the GCS login")
    p.add_argument("--auth-port", type=int,
                   help="authenticator https port (default 19408)")
    p.add_argument("--account", "--user", dest="user",
                   help="account name - needs GCS permission")
    p.add_argument("--code", "--password", dest="code",
                   help="account access code")
    p.add_argument("--party", help="party id to join as (default GCS_WATCH)")
    p.add_argument("--no-auth", action="store_true",
                   help="skip the authenticator - connect straight to "
                        "--server-ip/--server-port (local_server mode)")
    p.add_argument("--ctl-port", type=int, default=0,
                   help="control port for the list/listen/req protocol; "
                        "0 disables it (default in watch mode)")
    return p.parse_args(argv)


async def main(argv=None):
    global PARTY, WATCH
    a = _parse_args(sys.argv[1:] if argv is None else argv)
    if (a.server_ip or a.server_port or a.auth_ip or a.auth_port or a.user
            or a.code or a.no_auth):
        # external stack: no manifest - build the server dict from flags
        WATCH = True
        srv = {
            "type": "external",
            "ip": a.server_ip or "127.0.0.1",
            "port": a.server_port or 9966,
            "auth": not a.no_auth,
            "auth_ip": a.auth_ip or a.server_ip or "127.0.0.1",
            "auth_port": a.auth_port or 19408,
            "account": {"user": a.user or "", "code": a.code or ""},
            "gcs_party": a.party or "GCS_WATCH",
            "control_port": a.ctl_port,
        }
        # A bad account fails the first login - surface the server's refusal
        # (e.g. e=6 "not a GCS account") instead of hiding it in the retry loop.
        if srv["auth"]:
            try:
                await asyncio.get_running_loop().run_in_executor(
                    None, login, srv)
            except Exception as e:
                sys.exit(f"gcs_bridge: login failed: {e}")
    else:
        WATCH = bool(a.watch)
        srv = simlib.load_manifest(a.sim)["server"]
    PARTY = srv.get("gcs_party") or simlib.GCS_PARTY
    tasks = [session(srv)]
    if srv.get("control_port"):
        # control lines carry whole packages (`send <1085>`) - the default
        # 64 KiB stream limit resets the conn on a big plan
        ctl = await asyncio.start_server(control, "127.0.0.1",
                                         srv["control_port"], limit=16 << 20)
        print(f"[bridge] control on 127.0.0.1:{srv['control_port']} "
              f"(server {srv['port']}, auth={srv['auth']})", flush=True)
        tasks.append(ctl.serve_forever())
    if WATCH:
        tasks.append(_watch_reaper())
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    asyncio.run(main())
