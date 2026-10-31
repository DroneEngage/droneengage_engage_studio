#!/usr/bin/env python3
"""Offline unit tests for the P5-15 rendered-camera path: simlib's
camera-render procs + device wiring, and checks.py's render_latency /
visible_px / detection_precision_render / detection_recall(render_vis)
asserts. No GPU, no V4L2, no SITL needed:

    python3 engine/test_render.py
"""
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import checks
import simlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _truth_file(rows):
    f = tempfile.NamedTemporaryFile("w", suffix=".jsonl", delete=False)
    for r in rows:
        f.write(json.dumps(r) + "\n")
    f.close()
    return f.name


def _ctx(file="/x/s.yaml", bot=None):
    c = {"manifest": {"name": "t",
                      "units": [{"name": "cam1", "party_id": "cam1",
                                 "fc": True}],
                      "server_procs": []},
         "marks": {}, "file": file, "results_dir": None}
    if bot is not None:
        c["bot"] = bot
    return c


class Bot:
    def __init__(self, targets):
        self._t = targets

    def targets(self):
        return self._t


def run_assert(a, ctx):
    return checks.check_assertion(a, ctx)


class T(unittest.TestCase):

    def setUp(self):
        # the machine's sim.local.env may re-point the camera labels
        # (SIM_V4L2_*_LABELS) - tests exercise the default scheme unless
        # they set the override themselves
        local = {k: v for k, v in simlib._LOCAL.items()
                 if not k.startswith("SIM_V4L2_")}
        p = mock.patch.object(simlib, "_LOCAL", local)
        p.start()
        self.addCleanup(p.stop)

    # -------------------------------------------------- simlib wiring

    def test_cam3d_block_expands_with_cam_index(self):
        spec = {"units": [{"block": "copter_cam3d", "name": "cam1"}]}
        sim = simlib.load_sim(spec, base_dir=ROOT)
        u = sim["units"][0]
        self.assertIn("de_yolo_ai", u["modules"])
        self.assertIn("de_camera", u["modules"])
        self.assertEqual(u["cam_index"], 0)
        devs = u["config"]["de_camera"]["camera"]["camera_list"]
        self.assertEqual(devs[0]["device_name"], "DE-AI-0")

    def test_second_cam_unit_gets_own_index(self):
        spec = {"units": [{"block": "copter", "name": "u0"},
                          {"block": "copter_cam3d", "name": "cam1"},
                          {"block": "copter_cam3d", "name": "cam2"}]}
        sim = simlib.load_sim(spec, base_dir=ROOT)
        self.assertEqual(sim["units"][1]["cam_index"], 0)
        self.assertEqual(sim["units"][2]["cam_index"], 1)

    def test_render_cameras_mode_and_sink_port(self):
        spec = {"render": {"mode": "cameras"},
                "units": [{"block": "copter_cam3d", "name": "cam1"}]}
        sim = simlib.load_sim(spec, base_dir=ROOT)
        self.assertEqual(sim["render"]["mode"], "cameras")
        self.assertIn("sink_port", sim["render"])
        for p in ("stream_port", "mav_port", "mav_ws_port",
                  "web_port", "sink_port"):
            self.assertIn(sim["render"][p], simlib.used_ports(sim))

    def test_gen_render_cameras_procs(self):
        spec = {"render": {"mode": "cameras"},
                "units": [{"block": "copter_cam3d", "name": "cam1"}]}
        sim = simlib.load_sim(spec, base_dir=ROOT)
        fake_labels = {"DE-SRC-0": "/dev/video6",
                       "DE-AI-0": "/dev/video7"}
        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(simlib, "MAVLINK3DMAP_DIR", td), \
                mock.patch.object(simlib, "run_dir",
                                  lambda n: os.path.join(td, n)), \
                mock.patch.object(simlib, "render_dir",
                                  lambda n: os.path.join(td, n, "render")), \
                mock.patch.object(simlib, "pids_dir",
                                  lambda n: os.path.join(td, n, "pids")), \
                mock.patch.object(simlib, "pid_path",
                                  lambda n, o, p: os.path.join(
                                      td, n, "pids", f"{o}.{p}.pid")), \
                mock.patch.object(simlib, "v4l2_label_map",
                                  lambda *a, **k: dict(fake_labels)):
            os.makedirs(os.path.join(td, "backend", "src"))
            for f in ("udp2websocket.js", "render_sink.js"):
                open(os.path.join(td, "backend", "src", f), "w").close()
            os.makedirs(os.path.join(td, "node_modules", "vite", "bin"))
            open(os.path.join(
                td, "node_modules", "vite", "bin", "vite.js"), "w").close()
            procs = simlib.gen_render(sim)
            self.assertIn("render_sink", procs)
            self.assertIn("chromium", procs)
            self.assertIn("render_probe_0", procs)
            rdir = simlib.render_dir(sim["name"])
            sink_map = json.load(open(os.path.join(rdir, "sink_map.json")))
            self.assertEqual(sink_map["units"][0]["device"], "/dev/video6")
            self.assertEqual(sink_map["units"][0]["unit_idx"], 0)
            cmd = open(os.path.join(rdir, "launch.chromium.sh")).read()
            self.assertIn("render=cameras", cmd)
            self.assertIn("--headless=new", cmd)

    def test_gen_render_missing_device_fails_actionably(self):
        spec = {"render": {"mode": "cameras"},
                "units": [{"block": "copter_cam3d", "name": "cam1"}]}
        sim = simlib.load_sim(spec, base_dir=ROOT)
        with tempfile.TemporaryDirectory() as td, \
                mock.patch.object(simlib, "MAVLINK3DMAP_DIR", td), \
                mock.patch.object(simlib, "run_dir",
                                  lambda n: os.path.join(td, n)), \
                mock.patch.object(simlib, "render_dir",
                                  lambda n: os.path.join(td, n, "render")), \
                mock.patch.object(simlib, "pids_dir",
                                  lambda n: os.path.join(td, n, "pids")), \
                mock.patch.object(simlib, "v4l2_label_map",
                                  lambda *a, **k: {}):
            os.makedirs(os.path.join(td, "backend", "src"))
            open(os.path.join(
                td, "backend", "src", "udp2websocket.js"), "w").close()
            os.makedirs(os.path.join(td, "node_modules", "vite", "bin"))
            open(os.path.join(
                td, "node_modules", "vite", "bin", "vite.js"), "w").close()
            with self.assertRaises(simlib.SimError) as e:
                simlib.gen_render(sim)
            self.assertIn("setup_v4l2loopback.sh", str(e.exception))

    def test_cam_labels_env_override(self):
        """Machines with an existing named-VC setup re-point the sim at
        their labels via SIM_V4L2_SRC_LABELS / SIM_V4L2_AI_LABELS."""
        env = {"SIM_V4L2_SRC_LABELS": "SIM-CAM1,SIM-CAM2",
               "SIM_V4L2_AI_LABELS": "DE-AI"}
        with mock.patch.object(simlib, "_LOCAL", {}), \
                mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(simlib.cam_labels(0), ("DE-SRC-0", "DE-AI-0"))
            self.assertEqual(simlib.cam_labels(2), ("DE-SRC-2", "DE-AI-2"))
        with mock.patch.object(simlib, "_LOCAL", {}), \
                mock.patch.dict(os.environ, env):
            self.assertEqual(simlib.cam_labels(0), ("SIM-CAM1", "DE-AI"))
            # index past the lists falls back to the default scheme
            self.assertEqual(simlib.cam_labels(1), ("SIM-CAM2", "DE-AI-1"))

    def test_prepare_sim_injects_geo_for_yolo_unit(self):
        """de_yolo_ai units get the world camera's geo_projection on
        de_mavlink, same as stub units - the real obs frames (6546)
        project through the same lens."""
        import world
        w = world.load(os.path.join(ROOT, "worlds", "forest_a.yaml"), 7)
        scn = {"world": "w.yaml", "seed": 7,
               "sim": {"slot_base": 40,
                       "units": [{"block": "copter_cam3d",
                                  "name": "cam1"}]}}
        world.prepare_sim(scn, w)
        cfg = scn["sim"]["units"][0]["config"]["de_mavlink"]
        self.assertIn("geo_projection", cfg)

    # --------------------------------------------- render asserts

    def _vis_rows(self, tid="person_open", unit="cam1",
                  frames_px=((1, 64), (2, 0), (3, 200))):
        rows = [{"tid": tid, "cls": "person", "t_ms": 1000,
                 "lat": -35.0, "lng": 149.0, "alt_m": 0.0}]
        for fn, px in frames_px:
            r = {"type": "event", "src": "render_vis",
                 "kind": "visibility", "unit": unit,
                 "frame_no": fn, "tid": tid, "px": px,
                 "t_ms": 5000 + fn}
            if px:
                r["box"] = {"x": 0.5, "y": 0.5, "w": 0.1, "h": 0.1}
            rows.append(r)
        return rows

    def test_render_latency_join(self):
        rows = []
        for fn, t0 in ((1, 100000), (2, 100100), (3, 100200)):
            rows.append({"type": "event", "src": "render",
                         "kind": "frame_meta", "unit": "cam1",
                         "frame_no": fn, "t_ms": t0})
            rows.append({"type": "event", "src": "render_probe",
                         "kind": "frame_seen", "unit": "cam1",
                         "frame_no": fn, "t_ms": t0 + 120})
        f = _truth_file(rows)
        self.addCleanup(os.unlink, f)
        r = run_assert({"render_latency": {"p95_max_ms": 250,
                                            "truth_file": f}},
                       _ctx())
        self.assertEqual((r["status"], r["value"]), ("PASS", 120))
        r = run_assert({"render_latency": {"p95_max_ms": 100,
                                            "truth_file": f}},
                       _ctx())
        self.assertEqual(r["status"], "FAIL")
        # no joined pairs -> SKIP, not a vacuous pass
        rows2 = rows[:2]
        f2 = _truth_file(rows2)
        self.addCleanup(os.unlink, f2)
        r = run_assert({"render_latency": {"p95_max_ms": 250,
                                            "truth_file": f2}},
                       _ctx())
        self.assertEqual(r["status"], "PASS")  # one pair still joins
        f3 = _truth_file([])
        self.addCleanup(os.unlink, f3)
        r = run_assert({"render_latency": {"p95_max_ms": 250,
                                            "truth_file": f3}},
                       _ctx())
        self.assertEqual(r["status"], "SKIP")

    def test_visible_px_bounds(self):
        rows = self._vis_rows()
        f = _truth_file(rows)
        self.addCleanup(os.unlink, f)
        # >=16 px in >=2 frames: only frames 1 and 3 qualify
        r = run_assert({"visible_px": {"truth": "person_open",
                                      "unit": "cam1", "min_px": 16,
                                      "min_frames": 2,
                                      "truth_file": f}},
                       _ctx())
        self.assertEqual((r["status"], r["value"]), ("PASS", 2))
        r = run_assert({"visible_px": {"truth": "person_open",
                                      "unit": "cam1", "min_px": 16,
                                      "min_frames": 3,
                                      "truth_file": f}},
                       _ctx())
        self.assertEqual(r["status"], "FAIL")
        # occluded control: max_px 0 passes only when nothing painted
        hut = self._vis_rows(tid="person_hut",
                             frames_px=((1, 0), (2, 0)))
        f2 = _truth_file(hut)
        self.addCleanup(os.unlink, f2)
        r = run_assert({"visible_px": {"truth": "person_hut",
                                      "unit": "cam1", "max_px": 0,
                                      "truth_file": f2}},
                       _ctx())
        self.assertEqual(r["status"], "PASS")
        # ...but fails if it ever painted
        hut_bad = self._vis_rows(tid="person_hut",
                                 frames_px=((1, 0), (2, 3)))
        f3 = _truth_file(hut_bad)
        self.addCleanup(os.unlink, f3)
        r = run_assert({"visible_px": {"truth": "person_hut",
                                      "unit": "cam1", "max_px": 0,
                                      "truth_file": f3}},
                       _ctx())
        self.assertEqual(r["status"], "FAIL")

    def test_detection_precision_render_iou(self):
        rows = self._vis_rows()  # frames 1,3 paint box at .5/.5/.1/.1
        f = _truth_file(rows)
        self.addCleanup(os.unlink, f)
        bot = Bot({"t1": {"cls": "person", "src": "cam1",
                          "pos": [-35.0, 149.0], "t_ms": 5001,
                          "box": {"x": 0.51, "y": 0.51,
                                  "w": 0.1, "h": 0.1}},
                   "t2": {"cls": "person", "src": "cam1",
                          "pos": [-35.0, 149.0], "t_ms": 5003,
                          "box": {"x": 0.9, "y": 0.9,
                                  "w": 0.05, "h": 0.05}}})
        r = run_assert({"detection_precision_render":
                        {"unit": "cam1", "min": 0.5, "iou": 0.3,
                         "truth_file": f}},
                       _ctx(bot=bot))
        self.assertEqual((r["status"], r["value"]), ("PASS", 0.5))
        r = run_assert({"detection_precision_render":
                        {"unit": "cam1", "min": 0.9, "iou": 0.3,
                         "truth_file": f}},
                       _ctx(bot=bot))
        self.assertEqual(r["status"], "FAIL")

    def test_detection_recall_render_vis_bar(self):
        """src render_vis narrows the denominator to entities the ID
        pass actually painted >= min_px for >= min_frames frames."""
        rows = [
            {"tid": "person_open", "cls": "person", "t_ms": 1000,
             "lat": -35.0, "lng": 149.0, "alt_m": 0.0},
            {"tid": "person_hut", "cls": "person", "t_ms": 1000,
             "lat": -35.1, "lng": 149.1, "alt_m": 0.0},
            {"type": "event", "src": "render_vis", "kind": "visibility",
             "unit": "cam1", "frame_no": 4, "tid": "person_open",
             "px": 500, "t_ms": 8000},
            {"type": "event", "src": "render_vis", "kind": "visibility",
             "unit": "cam1", "frame_no": 4, "tid": "person_hut",
             "px": 0, "t_ms": 8000},
        ]
        f = _truth_file(rows)
        self.addCleanup(os.unlink, f)
        bot = Bot({"env1": {"cls": "person", "pos": [-35.0, 149.0],
                            "t_ms": 8000}})
        # person_hut never painted - excluded from the denominator,
        # person_open found -> recall 1.0
        r = run_assert({"detection_recall":
                        {"truth": ["person_open", "person_hut"],
                         "src": "render_vis", "min_px": 16,
                         "min_frames": 1, "truth_file": f}},
                       _ctx(bot=bot))
        self.assertEqual((r["status"], r["value"]), ("PASS", 1.0))
        # bar too high -> nobody qualifies -> explicit FAIL, not vacuous
        r = run_assert({"detection_recall":
                        {"truth": "person_open", "src": "render_vis",
                         "min_px": 99999, "truth_file": f}},
                       _ctx(bot=bot))
        self.assertEqual(r["status"], "FAIL")
        # visible but not found -> recall 0
        r = run_assert({"detection_recall":
                        {"truth": "person_open", "src": "render_vis",
                         "min_px": 16, "min": 1, "truth_file": f}},
                       _ctx(bot=Bot({})))
        self.assertEqual((r["status"], r["value"]), ("FAIL", 0.0))


if __name__ == "__main__":
    unittest.main(verbosity=1)
