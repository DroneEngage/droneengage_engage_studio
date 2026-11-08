#!/usr/bin/env python3
"""world_stream.py - the de.worldstream/1 render feed (TASK-P5-13).

A scenario that opts in with `render:` gets a per-run WebSocket server
INSIDE the scenario runner (D2 - one process owns the truth, no second
evaluator to drift). It serves one snapshot + a 10 Hz tick of the
shared World object to renderers (Mavlink3DMap in P5-14, the camera
farm in P5-15, the physics loop in P5-16), and takes `event` frames
back into the same truth.jsonl the assertions read (D6).

    {"t":"hello","schema":"de.worldstream/1","sim":"..","seed":7,
     "origin":[lat,lng],"epoch_unix_ms":0,
     "obstacles":[...], "areas":{...}, "trails":{...},
     "environment":{...},
     "entities":[{"id","cls","size_m"}],
     "units":[{"name","sysid","block","camera":{..},"payload":{..}}]}
    {"t":"tick","t_s":12.3,"t_unix_ms":0,
     "entities":[{"id","lat","lng","alt_m","heading_deg","moving"}],
     "clutter":[{"id","lat","lng","cls"}],
     "overlay":{"targets":{tid:{cls,pos}},        # every 10th tick
               "states":{tid:"seen|miss|occluded"},
               "detections":[{tid,cls,lat,lng,alt_m,t_ms}]}}
    {"t":"cmd","cmd":"payload_release","unit":"cam1","t_unix_ms":0}

    {"t":"event","kind":"payload_landed|visibility|collision|
      frame_meta", ...}

Ticks carry t_s (scenario seconds from the epoch) and t_unix_ms;
events carry the t_unix_ms of the pose sample they were computed
from (D7). Unknown `t` values are ignored on both sides (additive
evolution). The socket binds 127.0.0.1 only, like every sim port.

headless runs never reach here: scenario_runner only starts a
WorldStream when sim.render is set (D1).
"""

import asyncio
import json
import math
import os
import threading
import time

SCHEMA = "de.worldstream/1"
TICK_HZ = 10.0
OVERLAY_EVERY_TICKS = 10        # overlay rides every 10th tick (1 Hz)
MAX_DETECTIONS = 40             # recent-detection marker cap

# catalog modules that make a unit a camera / a payload carrier in
# hello.units (the world supplies the camera model; the renderer needs
# to know which unit owns the lens)
CAMERA_MODULES = ("stub_ai_box", "de_camera")
PAYLOAD_MODULES = ("stub_payload",)
# blocks whose FC drives a real gripper servo (the physics rig watches it)
PAYLOAD_BLOCKS = ("copter_gripper",)


def hello_frame(world, manifest, epoch_ms):
    """The snapshot a client gets on connect (D5: one origin)."""
    units = []
    for u in manifest["units"]:
        row = {"name": u["name"],
               "sysid": (u["fc"] or {}).get("sysid", 1),
               "block": u["block"]}
        if any(m in u["modules"] for m in CAMERA_MODULES):
            cam = dict(world.sensors["camera"])
            cam["fps"] = TICK_HZ
            row["camera"] = cam
        pay = dict(world.sensors.get("payload") or {})
        if any(m in u["modules"] for m in PAYLOAD_MODULES) \
                or u["block"] in PAYLOAD_BLOCKS:
            row["payload"] = pay
        if u["fc"] and pay.get("collision_radius_m") is not None:
            row["collision_radius_m"] = pay["collision_radius_m"]
        units.append(row)
    return {"t": "hello", "schema": SCHEMA,
            "sim": manifest["name"], "seed": world.seed,
            "origin": list(world.origin),
            "epoch_unix_ms": epoch_ms,
            "obstacles": world.obstacles_json(),
            "areas": {k: dict(v) for k, v in world.areas.items()},
            "trails": {k: dict(v) for k, v in world.trails.items()},
            "environment": dict(world.environment),
            "entities": [{"id": e.id, "cls": e.cls,
                          "size_m": world.size_m(e)}
                         for e in world.entities],
            "units": units}


def _finite(v):
    """Browsers' JSON.parse rejects Infinity/NaN (e.g. the default
    environment visibility_m = inf) and then drops the whole frame -
    send them as null."""
    if isinstance(v, float) and not math.isfinite(v):
        return None
    if isinstance(v, dict):
        return {k: _finite(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_finite(x) for x in v]
    return v


def dumps(frame):
    return json.dumps(_finite(frame), allow_nan=False)


def tick_frame(world, epoch_ms):
    """One 10 Hz tick: entity truth at t_s plus the live ground-
    anchored clutter field (P5-10 D8 points, now world objects)."""
    now_ms = int(time.time() * 1000)
    t_s = (now_ms - epoch_ms) / 1000.0
    ents = []
    for e in world.entities:
        lat, lng, alt = e.pos_at(t_s, world)
        st = e.state_at(t_s, world)
        ents.append({"id": e.id,
                     "lat": round(lat, 7), "lng": round(lng, 7),
                     "alt_m": round(alt, 2),
                     "heading_deg": round(st.get("heading_deg", 0.0), 1),
                     "moving": bool(st.get("moving", False))})
    return {"t": "tick", "t_s": round(t_s, 3), "t_unix_ms": now_ms,
            "entities": ents, "clutter": world.clutter_at(t_s, TICK_HZ)}


def overlay_frame(bot, states, detections):
    """P5-14 D5: the belief layer, sent once a second. targets is the
    operator's world model (reported pos + cls - the viewer draws the
    uncertainty ring), states the latest truth-row outcome per tid
    (seen/miss/occluded), detections the recent sent look rows."""
    targets = {}
    if bot is not None:
        for tid, r in (bot.targets() or {}).items():
            targets[tid] = {"cls": r.get("cls"), "pos": r.get("pos")}
    return {"targets": targets, "states": dict(states),
            "detections": list(detections)}


class WorldStream:
    """The per-run de.worldstream/1 server. start() spawns a daemon
    thread with its own asyncio loop; stop() shuts it down. Client
    connects/disconnects and event counts append to
    var/run/<sim>/worldstream.log (collected into var/results/)."""

    def __init__(self, world, manifest, epoch_ms, port, truth_path,
                 log_path, bot=None):
        self.world = world
        self.manifest = manifest
        self.epoch_ms = epoch_ms
        self.port = int(port)
        self.truth_path = truth_path
        self.log_path = log_path
        self.bot = bot                      # overlay.targets (P5-14)
        self._clients = set()
        self._thread = None
        self._loop = None
        self._closing = None
        self._counts = {"events": 0, "ticks": 0, "cmds": 0}
        self._states = {}                   # tid -> seen|miss|occluded
        self._detections = []               # recent sent look rows
        self._stop = threading.Event()
        # SL-0051 — set once the socket is actually bound, so a caller
        # that connects immediately can synchronize instead of racing
        # the thread's loop start-up
        self._listening = threading.Event()

    # ------------------------------------------------------- lifecycle

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def wait_ready(self, timeout=10.0):
        """Block until the port is bound and accepting; False on
        timeout (or if _serve died before listening — the 'error'
        row in the log says why)."""
        return self._listening.wait(timeout)

    def stop(self):
        self._stop.set()
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._begin_close)
        if self._thread is not None:
            self._thread.join(5)
        self._log("stop", dict(self._counts))

    def _begin_close(self):
        if self._closing is not None and not self._closing.done():
            self._closing.set_result(True)

    def _run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._serve())
        except Exception as e:
            self._log("error", f"{type(e).__name__}: {e}")
        finally:
            self._loop.close()

    async def _serve(self):
        import websockets
        self._closing = self._loop.create_future()
        async with websockets.serve(self._client, "127.0.0.1",
                                    self.port, compression=None,
                                    ping_interval=None):
            self._listening.set()
            self._log("listening", {"port": self.port})
            tick = asyncio.ensure_future(self._tick_loop())
            tail = asyncio.ensure_future(self._truth_tail())
            await self._closing
            tick.cancel()
            tail.cancel()

    # ---------------------------------------------------------- loops

    async def _tick_loop(self):
        import websockets
        while True:
            frame = tick_frame(self.world, self.epoch_ms)
            if self._counts["ticks"] % OVERLAY_EVERY_TICKS == 0:
                frame["overlay"] = overlay_frame(
                    self.bot, self._states, self._detections)
            msg = dumps(frame)
            self._counts["ticks"] += 1
            if self._clients:
                websockets.broadcast(self._clients, msg)
            await asyncio.sleep(1.0 / TICK_HZ)

    async def _truth_tail(self):
        """payload_drop truth rows -> cmd payload_release (a release
        from the real provider or the stub looks the same to the
        renderer). Only rows appended after the stream starts count."""
        pos = 0
        try:
            pos = os.path.getsize(self.truth_path)
        except OSError:
            pass
        while True:
            await asyncio.sleep(0.25)
            try:
                with open(self.truth_path) as f:
                    f.seek(pos)
                    lines = f.readlines()
                    pos = f.tell()
            except OSError:
                continue
            import websockets
            for ln in lines:
                try:
                    row = json.loads(ln)
                except ValueError:
                    continue
                self._observe_row(row)
                if row.get("cls") != "payload_drop":
                    continue
                cmd = {"t": "cmd", "cmd": "payload_release",
                       "unit": row.get("unit"),
                       "t_unix_ms": row.get("t_ms")}
                self._counts["cmds"] += 1
                self._log("cmd", cmd)
                if self._clients:
                    websockets.broadcast(self._clients,
                                         dumps(cmd))

    def _observe_row(self, row):
        """truth.jsonl look rows -> overlay state: a stub row with a
        tid and a sent/why verdict is the LATEST outcome for that
        entity (D4 - the viewer draws occluded entities as outlines),
        and every sent row carries the reported position for a
        detection marker."""
        tid = row.get("tid")
        if not tid:
            return
        if row.get("sent"):
            self._states[tid] = "seen"
            if row.get("lat") is not None:
                self._detections.append(
                    {"tid": tid, "cls": row.get("cls"),
                     "lat": row["lat"], "lng": row.get("lng"),
                     "alt_m": row.get("alt_m"),
                     "t_ms": row.get("t_ms")})
                del self._detections[:-MAX_DETECTIONS]
        elif row.get("why") in ("miss", "occluded"):
            self._states[tid] = row["why"]

    # --------------------------------------------------------- client

    async def _client(self, ws):
        import websockets
        self._clients.add(ws)
        self._log("connect", {"n": len(self._clients)})
        try:
            await ws.send(dumps(
                hello_frame(self.world, self.manifest, self.epoch_ms)))
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                if msg.get("t") == "event":
                    self._intake_event(msg)
        except websockets.exceptions.ConnectionClosed:
            # a client that dies mid-stream is routine (ctl stop kills the
            # render client) - log the disconnect below, not a traceback
            pass
        finally:
            self._clients.discard(ws)
            self._log("disconnect", {"n": len(self._clients)})

    def _intake_event(self, msg):
        """A client event frame -> a truth row (D6): src render,
        the event's own t_unix_ms as t_ms (D7)."""
        row = {k: v for k, v in msg.items() if k != "t"}
        row["type"] = "event"
        row.setdefault("src", "render")
        row["t_ms"] = int(msg.get("t_unix_ms")
                          or time.time() * 1000)
        try:
            with open(self.truth_path, "a") as f:
                f.write(json.dumps(row) + "\n")
        except OSError:
            pass
        self._counts["events"] += 1
        self._log("event", row)

    def _log(self, ev, detail):
        try:
            with open(self.log_path, "a") as f:
                f.write(json.dumps({"t_ms": int(time.time() * 1000),
                                    "ev": ev,
                                    "detail": detail}) + "\n")
        except OSError:
            pass


def log_path(sim):
    import simlib
    return os.path.join(simlib.run_dir(sim), "worldstream.log")
