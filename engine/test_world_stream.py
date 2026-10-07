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
import sys
import tempfile
import time
import unittest

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
        self.assertEqual(on["render"]["stream_port"], 61010)
        self.assertEqual(on["render"]["mav_port"], 16460)
        self.assertEqual(on["render"]["mav_ws_port"], 8811)
        self.assertEqual(on["render"]["web_port"], 8080)
        ports = simlib.used_ports(on)
        self.assertIn(61010, ports)
        self.assertIn(16460, ports)
        self.assertIn(8811, ports)
        self.assertIn(8080, ports)
        cam1 = next(u for u in on["units"] if u["name"] == "cam1")
        self.assertEqual(cam1["fc"]["mav_port"], 16460)
        self.assertEqual(cam1["fc"]["params"]["SR4_POSITION"], 10)
        self.assertEqual(cam1["fc"]["params"]["MAV4_POSITION"], 10)
        self.assertEqual(cam1["fc"]["params"]["SERIAL4_PROTOCOL"], 2)
        mast = next(u for u in on["units"] if u["name"] == "mast1")
        self.assertIsNone(mast["fc"])                # statics: no fc

    def test_absent_render_is_identical(self):
        off = _sim(render=None, slot_base=10)
        self.assertIsNone(off["render"])
        ports = simlib.used_ports(off)
        for p in (61010, 16460, 8811, 8080):
            self.assertNotIn(p, ports)
        for u in off["units"]:
            if u["fc"]:
                self.assertNotIn("mav_port", u["fc"])
                for k in u["fc"]["params"]:
                    self.assertFalse(k.startswith("SR4_"), k)
                    self.assertFalse(k.startswith("MAV4_"), k)
                    self.assertFalse(k.startswith("SERIAL4_"), k)
        # the sim hash is generated from the unit specs only, so a
        # headless sim hashes exactly like a pre-P5-13 one
        import hashlib
        want = hashlib.sha256(json.dumps(
            [off["server"], off["home"], off["units"]],
            sort_keys=True).encode()).hexdigest()[:16]
        self.assertEqual(off["hash"], want)

    def test_slot_blocks_do_not_collide(self):
        a = _sim(render={"mode": "viewer"}, slot_base=10)
        b = _sim(render={"mode": "physics"}, slot_base=35)
        self.assertFalse(simlib.used_ports(a) & simlib.used_ports(b))

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

        class _Bot:
            def targets(self):
                return {"tid_42": {"cls": "person",
                                   "pos": {"lat": 1.5, "lng": 2.5}}}
        stream = world_stream.WorldStream(w, sim, epoch, 61999,
                                          truth, log, bot=_Bot())
        stream.start()
        try:
            hello, frames = asyncio.run(
                _client(61999, truth, seconds=1.4))
        finally:
            stream.stop()
        self.assertEqual(hello["schema"], world_stream.SCHEMA)
        self.assertEqual(hello["sim"], "ws_test")
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
