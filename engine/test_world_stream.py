#!/usr/bin/env python3
"""test_world_stream.py - pure P5-13 gates (no SITL, no fleet).

  - hello contents from forest_canopy + a two-unit manifest: origin,
    obstacles, entities and the per-unit camera/payload blocks;
  - tick entity positions equal Entity.pos_at at the tick's own t_s;
  - render port allocation rides the slot block (no collision across
    seeds) and absent render changes nothing: no serial4, no SR4_*
    params, identical sim hash and used_ports;
  - a live in-process WebSocket round-trip: hello + ticks on the wire,
    an event lands in truth.jsonl with src render, and a payload_drop
    truth row comes back as a cmd payload_release.
"""

import asyncio
import json
import os
import socket
import sys
import tempfile
import time
import unittest

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

import simlib              # noqa: E402
import world as world_mod  # noqa: E402
import world_stream        # noqa: E402

WORLD = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "worlds", "forest_canopy.yaml")


def _sim(render=None, slot_base=10):
    spec = {"version": 1, "name": "ws_test", "slot_base": slot_base,
            # the scenario layer rides the server port on the slot
            # block too (scenario._server_spec) - mirror it here
            "server": {"type": "fake",
                       "port": simlib.SERVER_PORT + slot_base},
            "units": [{"block": "copter", "name": "cam1",
                       "add_modules": ["stub_ai_box", "stub_payload"]},
                      {"block": "comm_only", "name": "mast1"}]}
    if render is not None:
        spec["render"] = render
    return simlib.load_sim(spec)

# --------------------------------------------- de.worldstream/1 contract
# Frame shapes from docs/WORLDSTREAM.md (SL-PG-22), checked field by
# field: required keys, JSON types, strict-JSON (no inf/nan). Extra
# keys are legal (additive evolution), missing/mistyped ones are not.

_NUM = (int, float)


def _is_num(v):
    return isinstance(v, _NUM) and not isinstance(v, bool)


def _need(errs, where, obj, key, check, what):
    if key not in obj:
        errs.append(f"{where}: missing {key}")
    elif not check(obj[key]):
        errs.append(f"{where}.{key}: expected {what}, got {obj[key]!r}")


def _latlng(errs, where, o):
    _need(errs, where, o, "lat", _is_num, "number")
    _need(errs, where, o, "lng", _is_num, "number")


def _finite_json(errs, where, v):
    if isinstance(v, float) and v != v or v in (float("inf"),
                                                float("-inf")):
        errs.append(f"{where}: non-finite {v!r}")
    elif isinstance(v, dict):
        for k, x in v.items():
            _finite_json(errs, f"{where}.{k}", x)
    elif isinstance(v, list):
        for i, x in enumerate(v):
            _finite_json(errs, f"{where}[{i}]", x)


def validate_frame(frame):
    """Return the list of contract violations for one frame (empty =
    valid). Unknown `t` values are valid by rule 2 and return []."""
    errs = []
    t = frame.get("t")
    if t == "hello":
        _need(errs, "hello", frame, "schema",
              lambda v: v == "de.worldstream/1", "de.worldstream/1")
        _need(errs, "hello", frame, "sim", lambda v: isinstance(v, str),
              "str")
        _need(errs, "hello", frame, "seed", _is_num, "number")
        _need(errs, "hello", frame, "origin",
              lambda v: isinstance(v, list) and len(v) == 2
              and all(_is_num(x) for x in v), "[lat, lng]")
        _need(errs, "hello", frame, "epoch_unix_ms", _is_num, "number")
        for k in ("obstacles", "units", "entities"):
            _need(errs, "hello", frame, k, lambda v: isinstance(v, list),
                  "list")
        for k in ("areas", "trails", "environment"):
            _need(errs, "hello", frame, k, lambda v: isinstance(v, dict),
                  "object")
        for i, e in enumerate(frame.get("entities") or []):
            w = f"hello.entities[{i}]"
            _need(errs, w, e, "id", lambda v: isinstance(v, str), "str")
            _need(errs, w, e, "cls", lambda v: isinstance(v, str), "str")
            _need(errs, w, e, "size_m", _is_num, "number")
        for i, u in enumerate(frame.get("units") or []):
            w = f"hello.units[{i}]"
            _need(errs, w, u, "name", lambda v: isinstance(v, str), "str")
            _need(errs, w, u, "sysid", lambda v: isinstance(v, int),
                  "int")
            _need(errs, w, u, "block", lambda v: isinstance(v, str),
                  "str")
            for k in ("camera", "payload"):
                if k in u and not isinstance(u[k], dict):
                    errs.append(f"{w}.{k}: expected object")
            if "collision_radius_m" in u \
                    and not _is_num(u["collision_radius_m"]):
                errs.append(f"{w}.collision_radius_m: expected number")
    elif t == "tick":
        _need(errs, "tick", frame, "t_s", _is_num, "number")
        _need(errs, "tick", frame, "t_unix_ms", _is_num, "number")
        _need(errs, "tick", frame, "entities",
              lambda v: isinstance(v, list), "list")
        _need(errs, "tick", frame, "clutter",
              lambda v: isinstance(v, list), "list")
        for i, e in enumerate(frame.get("entities") or []):
            w = f"tick.entities[{i}]"
            _need(errs, w, e, "id", lambda v: isinstance(v, str), "str")
            _latlng(errs, w, e)
            _need(errs, w, e, "alt_m", _is_num, "number")
            _need(errs, w, e, "heading_deg", _is_num, "number")
            _need(errs, w, e, "moving", lambda v: isinstance(v, bool),
                  "bool")
        for i, c in enumerate(frame.get("clutter") or []):
            w = f"tick.clutter[{i}]"
            _need(errs, w, c, "id", lambda v: isinstance(v, str), "str")
            _latlng(errs, w, c)
            _need(errs, w, c, "cls", lambda v: isinstance(v, str), "str")
        if "overlay" in frame:
            ov = frame["overlay"]
            _need(errs, "overlay", ov, "targets",
                  lambda v: isinstance(v, dict), "object")
            _need(errs, "overlay", ov, "states",
                  lambda v: isinstance(v, dict)
                  and all(x in ("seen", "miss", "occluded")
                          for x in v.values()), "tid -> seen|miss|occluded")
            _need(errs, "overlay", ov, "detections",
                  lambda v: isinstance(v, list)
                  and len(v) <= world_stream.MAX_DETECTIONS,
                  f"list (max {world_stream.MAX_DETECTIONS})")
            for i, d in enumerate(ov.get("detections") or []):
                w = f"overlay.detections[{i}]"
                _need(errs, w, d, "tid", lambda v: isinstance(v, str),
                      "str")
                _latlng(errs, w, d)
                _need(errs, w, d, "t_ms", _is_num, "number")
    elif t == "cmd":
        _need(errs, "cmd", frame, "cmd", lambda v: isinstance(v, str),
              "str")
        _need(errs, "cmd", frame, "unit", lambda v: isinstance(v, str),
              "str")
        _need(errs, "cmd", frame, "t_unix_ms", _is_num, "number")
    elif t == "event":
        _need(errs, "event", frame, "kind", lambda v: isinstance(v, str),
              "str")
        _need(errs, "event", frame, "t_unix_ms", _is_num, "number")
    _finite_json(errs, t or "frame", frame)
    return errs


class T(unittest.TestCase):

    def test_hello_contents(self):
        w = world_mod.load(WORLD, 7)
        sim = _sim(render={"mode": "viewer"})
        hello = world_stream.hello_frame(w, sim, 123456)
        self.assertEqual(hello["schema"], world_stream.SCHEMA)
        self.assertEqual(hello["origin"], w.origin)
        self.assertEqual(hello["epoch_unix_ms"], 123456)
        self.assertEqual(hello["sim"], "ws_test")
        self.assertEqual([o["id"] for o in hello["obstacles"]],
                         ["wood_n", "hut_1"])
        self.assertEqual(sorted(hello["areas"]), ["sector_a"])
        self.assertEqual({e["id"] for e in hello["entities"]},
                         {"person_open", "person_canopy", "person_hut",
                          "phone_canopy"})
        units = {u["name"]: u for u in hello["units"]}
        self.assertEqual(units["cam1"]["sysid"], 10)      # slot
        self.assertIn("camera", units["cam1"])            # stub_ai_box
        self.assertEqual(units["cam1"]["camera"]["hfov_deg"], 62.2)
        self.assertIn("payload", units["cam1"])           # stub_payload
        self.assertNotIn("camera", units["mast1"])

    def test_hello_is_strict_json(self):
        # the default visibility_m is inf - a browser JSON.parse drops
        # the whole hello on a bare Infinity token
        w = world_mod.load(WORLD, 7)
        hello = world_stream.hello_frame(w, _sim(render={"mode": "viewer"}),
                                         123456)
        txt = world_stream.dumps(hello)
        self.assertNotIn("Infinity", txt)
        self.assertNotIn("NaN", txt)
        back = json.loads(txt, parse_constant=lambda c: self.fail(c))
        self.assertIsNone(back["environment"]["visibility_m"])

    def test_tick_positions_equal_pos_at(self):
        w = world_mod.load(WORLD, 7)
        frame = world_stream.tick_frame(w,
                                        int(time.time() * 1000) - 5000)
        t_s = frame["t_s"]
        ents = {e["id"]: e for e in frame["entities"]}
        for e in w.entities:
            lat, lng, alt = e.pos_at(t_s, w)
            row = ents[e.id]
            self.assertAlmostEqual(row["lat"], lat, places=7)
            self.assertAlmostEqual(row["lng"], lng, places=7)
            self.assertAlmostEqual(row["alt_m"], alt, places=1)
            st = e.state_at(t_s, w)
            self.assertEqual(row["moving"], st["moving"])

    def test_tick_clutter_is_ground_anchored(self):
        w = world_mod.load(WORLD, 7)
        pts = w.clutter_at(60.0)
        for p in pts:
            self.assertIn("lat", p)
            self.assertIn("cls", p)
        # deterministic: a fresh world at the same seed sees the
        # same field at the same tick
        w2 = world_mod.load(WORLD, 7)
        self.assertEqual(pts, w2.clutter_at(60.0))

    def test_render_ports_and_serial4(self):
        on = _sim(render={"mode": "viewer"}, slot_base=10)
        # the resolved values are the prefer hints - lease_commit may
        # still move them, but the hints ride the slot block
        r = on["render"]
        self.assertEqual(r["stream_port"], simlib.STREAM_PORT_BASE + 10)
        self.assertEqual(r["mav_port"], simlib.MAV_PORT_BASE + 10)
        self.assertEqual(r["mav_ws_port"], simlib.MAV_WS_PORT_BASE + 10)
        self.assertEqual(r["web_port"], simlib.WEB_PORT_BASE + 10)
        self.assertEqual(r["sink_port"], simlib.SINK_PORT_BASE + 10)
        ports = simlib.used_ports(on)
        for key in ("stream_port", "mav_port", "mav_ws_port", "web_port",
                    "sink_port"):
            self.assertIn(r[key], ports)
        cam1 = next(u for u in on["units"] if u["name"] == "cam1")
        self.assertEqual(cam1["fc"]["mav_port"], r["mav_port"])
        self.assertEqual(cam1["fc"]["params"]["SR4_POSITION"], 10)
        self.assertEqual(cam1["fc"]["params"]["MAV4_POSITION"], 10)
        self.assertEqual(cam1["fc"]["params"]["SERIAL4_PROTOCOL"], 2)
        mast = next(u for u in on["units"] if u["name"] == "mast1")
        self.assertIsNone(mast["fc"])                # statics: no fc

    def test_render_stream_rates_follow_the_channel(self):
        # a MAVLink serial3 makes serial4 channel 5: only MAV5_*/SR5_*
        # are set - a fixed MAV4_* would raise rates on serial3's link
        spec = {"version": 1, "name": "ws_chan", "slot_base": 10,
                "server": {"type": "fake",
                           "port": simlib.SERVER_PORT + 10},
                "render": {"mode": "viewer"},
                "units": [{"block": "copter", "name": "c1",
                           "fc": {"params": {"SERIAL3_PROTOCOL": 2}}}]}
        params = simlib.load_sim(spec)["units"][0]["fc"]["params"]
        self.assertEqual(params["MAV5_POSITION"], 10)
        self.assertEqual(params["SR5_POSITION"], 10)
        self.assertFalse([k for k in params
                          if k.startswith(("MAV4_", "SR4_"))], params)

    def test_read_parm_files_layers(self):
        d = tempfile.mkdtemp()
        a, b = os.path.join(d, "a.parm"), os.path.join(d, "b.parm")
        with open(a, "w") as f:
            f.write("# c\nSERIAL1_PROTOCOL 2\nSERIAL3_PROTOCOL,5  # gps\n")
        with open(b, "w") as f:
            f.write("SERIAL1_PROTOCOL -1\n")
        got = simlib.read_parm_files([a, b, os.path.join(d, "missing")])
        self.assertEqual(got, {"SERIAL1_PROTOCOL": "-1",
                               "SERIAL3_PROTOCOL": "5"})
        self.assertEqual(simlib.mav_chan_index(got, 4), 3)

    def test_absent_render_is_identical(self):
        off = _sim(render=None, slot_base=10)
        self.assertIsNone(off["render"])
        ports = simlib.used_ports(off)
        for p in (simlib.STREAM_PORT_BASE + 10, simlib.MAV_PORT_BASE + 10,
                  simlib.MAV_WS_PORT_BASE + 10, simlib.WEB_PORT_BASE + 10):
            self.assertNotIn(p, ports)
        for u in off["units"]:
            if u["fc"]:
                self.assertNotIn("mav_port", u["fc"])
                for k in u["fc"]["params"]:
                    self.assertFalse(k.startswith("SR4_"), k)
                    self.assertFalse(k.startswith("MAV4_"), k)
                    self.assertFalse(k.startswith("SERIAL4_"), k)
        # the sim hash is generated from the unit specs (+ the --module
        # overrides in force - none here), so a headless sim hashes
        # exactly like a pre-P5-13 one
        self.assertEqual(off["hash"], simlib.sim_hash(off, {}))

    def test_slot_blocks_do_not_collide(self):
        a = _sim(render={"mode": "viewer"}, slot_base=10)
        b = _sim(render={"mode": "physics"}, slot_base=35)
        self.assertFalse(simlib.used_ports(a) & simlib.used_ports(b))

    def test_hello_and_tick_validate_against_contract(self):
        w = world_mod.load(WORLD, 7)
        hello = json.loads(world_stream.dumps(world_stream.hello_frame(
            w, _sim(render={"mode": "viewer"}), 123456)))
        self.assertEqual(validate_frame(hello), [])
        tick = json.loads(world_stream.dumps(world_stream.tick_frame(
            w, int(time.time() * 1000) - 5000)))
        self.assertEqual(validate_frame(tick), [])
        tick["overlay"] = json.loads(world_stream.dumps(
            world_stream.overlay_frame(
                None, {"person_open": "seen"},
                [{"tid": "person_open", "cls": "person", "lat": 1.0,
                  "lng": 2.0, "alt_m": 0.0, "t_ms": 5}])))
        self.assertEqual(validate_frame(tick), [])

    def test_validator_rejects_broken_frames(self):
        # the validator must actually bite, else the gates above are
        # vacuous
        w = world_mod.load(WORLD, 7)
        hello = world_stream.hello_frame(w, _sim(render={"mode": "viewer"}),
                                         1)
        self.assertTrue(validate_frame(hello))      # raw inf not stripped
        hello = json.loads(world_stream.dumps(hello))
        hello["schema"] = "de.worldstream/2"
        del hello["origin"]
        errs = validate_frame(hello)
        self.assertEqual(len(errs), 2, errs)
        tick = {"t": "tick", "t_s": "12", "t_unix_ms": 0,
                "entities": [{"id": "x", "lat": 1.0}], "clutter": []}
        self.assertGreaterEqual(len(validate_frame(tick)), 3)
        self.assertEqual(validate_frame({"t": "future_frame", "x": 1}), [])
        self.assertEqual(validate_frame(
            {"t": "cmd", "cmd": "payload_release", "unit": "cam1",
             "t_unix_ms": 1}), [])
        self.assertTrue(validate_frame({"t": "cmd", "cmd": "x"}))

    def test_roundtrip_ws(self):
        """In-process: serve on an ephemeral port, read hello + a tick,
        inject an event, see it land in truth.jsonl; append a
        payload_drop row, see the cmd come back."""
        w = world_mod.load(WORLD, 7)
        sim = _sim(render={"mode": "viewer"})
        d = tempfile.mkdtemp()
        truth = os.path.join(d, "truth.jsonl")
        open(truth, "w").close()
        log = os.path.join(d, "worldstream.log")
        epoch = int(time.time() * 1000) - 3000
        # an ephemeral port - never a sim formula value that another
        # sim's lease or a foreign listener could already hold
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        sock.close()

        class _Bot:
            def targets(self):
                return {"tid_42": {"cls": "person",
                                   "pos": {"lat": 1.5, "lng": 2.5}}}
        stream = world_stream.WorldStream(w, sim, epoch, port,
                                          truth, log, bot=_Bot())
        stream.start()
        # the serve coroutine binds on the thread's own loop - connect
        # only once it is actually listening, never on a timer
        self.assertTrue(stream.wait_ready(10))
        try:
            hello, frames = asyncio.run(
                _client(port, truth, seconds=1.4))
        finally:
            stream.stop()
        self.assertEqual(hello["schema"], world_stream.SCHEMA)
        self.assertEqual(hello["sim"], "ws_test")
        # every frame that crossed the wire honours the contract
        for f in [hello] + frames:
            self.assertEqual(validate_frame(f), [], f)
        self.assertEqual({f["t"] for f in frames} | {"hello"},
                         {"hello", "tick", "cmd"})
        ticks = [f for f in frames if f.get("t") == "tick"]
        self.assertGreaterEqual(len(ticks), 8)     # ~10 Hz over 1.4 s
        cmds = [f for f in frames if f.get("t") == "cmd"]
        self.assertEqual(len(cmds), 1)
        self.assertEqual(cmds[0]["cmd"], "payload_release")
        self.assertEqual(cmds[0]["unit"], "cam1")
        # P5-14 overlay: a 1 Hz belief layer rides the same ticks
        ols = [f["overlay"] for f in ticks if f.get("overlay")]
        self.assertTrue(ols)
        last = ols[-1]
        self.assertEqual(last["states"].get("person_open"), "seen")
        self.assertEqual(last["states"].get("person_hut"), "occluded")
        self.assertEqual(last["targets"]["tid_42"]["cls"], "person")
        self.assertEqual(last["detections"][-1]["tid"], "person_open")
        evs = [json.loads(ln) for ln in open(truth)
               if json.loads(ln).get("type") == "event"]
        self.assertEqual(len(evs), 1)
        self.assertEqual(evs[0]["src"], "render")
        self.assertEqual(evs[0]["kind"], "payload_landed")
        # the client event frame itself obeys the contract, and the
        # stored truth row keeps its kind/payload verbatim
        self.assertEqual(validate_frame(
            dict(evs[0], t="event", t_unix_ms=evs[0]["t_ms"])), [])
        # the log saw the client + the event
        evs_log = [json.loads(ln)["ev"] for ln in open(log)]
        self.assertIn("connect", evs_log)
        self.assertIn("event", evs_log)


async def _client(port, truth_path, seconds):
    import websockets
    frames = []
    async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
        hello = json.loads(await asyncio.wait_for(ws.recv(), 5))
        await ws.send(json.dumps(
            {"t": "event", "kind": "payload_landed", "unit": "cam1",
             "lat": 1.0, "lng": 2.0, "alt_m": 0.0, "hit": "ground",
             "impact_mps": 7.9,
             "t_unix_ms": int(time.time() * 1000)}))
        # a stub-style payload_drop truth row -> cmd payload_release
        with open(truth_path, "a") as f:
            f.write(json.dumps(
                {"src": "payload", "type": "detection", "unit": "cam1",
                 "tid": "payload", "cls": "payload_drop",
                 "lat": 1.0, "lng": 2.0, "alt_m": 20.0,
                 "t_ms": int(time.time() * 1000)}) + "\n")
            # P5-14 overlay rows: a sent look (seen + a detection
            # marker) and an occluded one
            f.write(json.dumps(
                {"src": "ai", "type": "detection", "unit": "cam1",
                 "tid": "person_open", "cls": "person", "sent": True,
                 "lat": 1.0, "lng": 2.0, "alt_m": 0.0,
                 "t_ms": int(time.time() * 1000)}) + "\n")
            f.write(json.dumps(
                {"src": "ai", "type": "detection", "unit": "cam1",
                 "tid": "person_hut", "cls": "person", "sent": False,
                 "why": "occluded",
                 "t_ms": int(time.time() * 1000)}) + "\n")
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                frames.append(json.loads(
                    await asyncio.wait_for(ws.recv(), 0.5)))
            except asyncio.TimeoutError:
                pass
    return hello, frames


if __name__ == "__main__":
    unittest.main(verbosity=2)
