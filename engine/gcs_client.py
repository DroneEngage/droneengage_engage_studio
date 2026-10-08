#!/usr/bin/env python3
"""gcs_client.py - one GCS websocket session for the operator bot
(TASK-P5-07).

Works on every server type the same way: the fake fleet server, the real
droneengage_server in local mode, through the authenticator, or an
external stack - the login path is the one gcs_bridge.py uses. The
session syncs to the server clock from the 9007 "connected" push
(CLOCK.md T2.1): fleet_ms() estimates the fleet clock so operator.log
lines and 1090/1093 "t" fields carry fleet time like the WebClient's.
"""

import asyncio
import json
import ssl
import time
import urllib.parse
import urllib.request

import simlib


def _ssl_ctx():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


def _login_key(srv):
    """-> (host, port, key) for the de_auth frame. Auth mode does the
    web login (/w/wl) like the WebClient/bridge; local + fake connect
    straight to the comm server with a plain key."""
    if not srv.get("auth"):
        return srv["ip"], srv["port"], "gcs_operator"
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


class GCSClient:
    """One bot-owned GCS session. `on_frame(msg)` is invoked for every
    received wire message; `fleet_ms()` is the synced server clock."""

    def __init__(self, srv, party, on_frame=None):
        self.srv = srv
        self.party = party
        self.on_frame = on_frame
        self.ws = None
        self._offset_ms = None        # server_ms - local_unix_ms
        self._offset_mono = 0.0
        self.connected = asyncio.Event()
        self.hold_until = 0.0         # gcs_disconnect: no reconnect until

    def fleet_ms(self):
        if self._offset_ms is None:
            return int(time.time() * 1000)
        return int(self._offset_ms +
                   (time.monotonic() - self._offset_mono) * 1000.0)

    async def connect(self):
        import websockets
        host, port, key = await asyncio.get_running_loop().run_in_executor(
            None, _login_key, self.srv)
        ws = await websockets.connect(
            f"wss://{host}:{port}", ssl=_ssl_ctx(), compression=None,
            max_size=None, ping_interval=20)
        try:
            await ws.send(json.dumps(
                {"ty": "s", "mt": "de_auth", "f": key,
                 "s": self.party, "at": "g"}))
            # the server pushes 9007 (connected + clock) around the ack
            t_end = time.time() + 15
            got_ack = False
            while time.time() < t_end and not got_ack:
                raw = await asyncio.wait_for(
                    ws.recv(), max(0.1, t_end - time.time()))
                if not isinstance(raw, str):
                    continue        # binary intermodule frame (e.g. mt 6502)
                msg = json.loads(raw)
                mt = msg.get("mt")
                if mt == 9007 and isinstance(msg.get("ms"), dict) \
                        and msg["ms"].get("ts"):
                    self._offset_ms = msg["ms"]["ts"]
                    self._offset_mono = time.monotonic()
                elif mt == "de_auth_ack":
                    if msg.get("r") != "ok":
                        raise RuntimeError(f"de_auth refused: {msg}")
                    got_ack = True
            if not got_ack:
                raise RuntimeError("no de_auth_ack from the server")
        except BaseException:
            # a refused/failed handshake leaves the ws's keepalive task
            # behind - close it before the exception propagates
            try:
                await ws.close()
            except Exception:
                pass
            raise
        self.ws = ws
        self.connected.set()
        return ws

    async def send(self, msg):
        if self.ws is None:
            raise RuntimeError("not connected to the comm server")
        msg.setdefault("sd", self.party)
        await self.ws.send(json.dumps(msg))

    async def run(self):
        """Reconnect loop: holds the session, dispatches on_frame. Runs
        until cancelled. hold_until > now suppresses the reconnect
        (the gcs_disconnect fault)."""
        while True:
            try:
                while time.time() < self.hold_until:
                    await asyncio.sleep(1)
                ws = await self.connect()
                async for raw in ws:
                    try:
                        msg = json.loads(raw)
                    except ValueError:
                        continue
                    if self.on_frame:
                        self.on_frame(msg)
            except asyncio.CancelledError:
                raise
            except Exception:
                pass
            finally:
                ws = self.ws
                self.ws = None
                self.connected.clear()
                if ws is not None:
                    try:
                        await ws.close()
                    except Exception:
                        pass
            await asyncio.sleep(3)

    async def close(self):
        if self.ws is not None:
            try:
                await self.ws.close()
            except Exception:
                pass
