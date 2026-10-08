#!/usr/bin/env python3
"""test_render_viewer.py - pure P5-14 gates (no SITL, no fleet).

  - gen_render() is silent unless render.mode is viewer|full and not
    headless - cameras/physics modes and headless runs launch nothing;
  - the generated launch scripts run the Mavlink3DMap UDP->WS bridge
    on render.mav_port / ws render.mav_ws_port and a vite dev server
    on render.web_port, pid-marked via node --require so sim down sees
    them, under var/run/<sim>/render/;
  - gated smoke (skipped without node + MAVLINK3DMAP_DIR): boot both
    procs and check the ws port accepts and the page serves 200.
"""

import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.request

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

import simlib  # noqa: E402

M3D = os.environ.get("MAVLINK3DMAP_DIR") or simlib.MAVLINK3DMAP_DIR
BRIDGE = os.path.join(M3D, "backend", "src", "udp2websocket.js")


def _sim(render):
    spec = {"version": 1, "name": "rv_test", "slot_base": 10,
            "server": {"type": "fake",
                       "port": simlib.SERVER_PORT + 10},
            "units": [{"block": "comm_only", "name": "mast1"}],
            "render": render}
    return simlib.load_sim(spec)


def _fake_m3d():
    """A fake Mavlink3DMap tree: gen_render validates file existence
    only - the scripts themselves carry the real paths."""
    fake = tempfile.mkdtemp()
    for rel in ("backend/src/udp2websocket.js",
                "node_modules/vite/bin/vite.js"):
        p = os.path.join(fake, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        open(p, "w").close()
    return fake


class T(unittest.TestCase):

    def test_no_render_no_procs(self):
        sim = _sim(None)
        sim["render"] = None
        self.assertEqual(simlib.gen_render(sim), [])

    def test_headless_launches_nothing(self):
        sim = _sim({"mode": "viewer", "headless": True})
        self.assertEqual(simlib.gen_render(sim), [])

    def test_cameras_mode_no_cam_units_is_viewer_only(self):
        # P5-15: mode cameras without a de_yolo_ai unit has nothing to
        # feed loopbacks with - the viewer procs still launch (the page
        # renders the world), the farm does not
        saved = simlib.MAVLINK3DMAP_DIR
        try:
            simlib.MAVLINK3DMAP_DIR = _fake_m3d()
            sim = _sim({"mode": "cameras"})
            self.assertEqual(sorted(simlib.gen_render(sim)),
                             ["mav_bridge", "web"])
        finally:
            simlib.MAVLINK3DMAP_DIR = saved

    def test_viewer_scripts(self):
        saved = simlib.MAVLINK3DMAP_DIR
        try:
            simlib.MAVLINK3DMAP_DIR = _fake_m3d()
            sim = _sim({"mode": "viewer", "headless": False})
            procs = simlib.gen_render(sim)
        finally:
            simlib.MAVLINK3DMAP_DIR = saved
        self.assertEqual(procs, ["mav_bridge", "web"])
        rdir = simlib.render_dir(sim["name"])
        bridge = open(os.path.join(rdir, "launch.mav_bridge.sh")).read()
        self.assertIn("udp2websocket.js -p %d -w %d"
                      % (sim["render"]["mav_port"],
                         sim["render"]["mav_ws_port"]), bridge)
        self.assertIn("render.mav_bridge.pid", bridge)
        web = open(os.path.join(rdir, "launch.web.sh")).read()
        self.assertIn("vite.js", web)
        self.assertIn("--port %d" % sim["render"]["web_port"], web)
        self.assertIn("--strictPort", web)
        self.assertIn("render.web.pid", web)
        for s in (bridge, web):
            # pid_is_ours needs var/run/<sim>/ in the cmdline - the marker
            # preload provides it
            self.assertIn("--require", s)
            self.assertIn("de_sim_marker.cjs", s)
            self.assertIn("echo $$ > ", s)
        self.assertTrue(os.path.isfile(
            os.path.join(rdir, "de_sim_marker.cjs")))

    def test_missing_m3d_is_clear_error(self):
        sim = _sim({"mode": "viewer"})
        saved = simlib.MAVLINK3DMAP_DIR
        try:
            simlib.MAVLINK3DMAP_DIR = "/nonexistent/m3d"
            with self.assertRaises(simlib.SimError) as cm:
                simlib.gen_render(sim)
            self.assertIn("MAVLINK3DMAP_DIR", str(cm.exception))
        finally:
            simlib.MAVLINK3DMAP_DIR = saved

    @unittest.skipUnless(
        shutil.which("node") and os.path.isfile(BRIDGE),
        "needs node + a real MAVLINK3DMAP_DIR checkout")
    def test_smoke_procs(self):
        """The spec's headless page check: boot the two render procs,
        fetch the page (200) and accept on the bridge ws port."""
        sim = _sim({"mode": "viewer", "headless": False})
        simlib.gen_render(sim)
        rdir = simlib.render_dir(sim["name"])
        r = sim["render"]
        procs = []
        for name in ("mav_bridge", "web"):
            log = open(os.path.join(rdir, f"{name}.smoke.log"), "w")
            procs.append((name, log, subprocess.Popen(
                ["bash", os.path.join(rdir, f"launch.{name}.sh")],
                stdout=log, stderr=subprocess.STDOUT)))
        try:
            self.assertTrue(_wait_tcp(r["mav_ws_port"], 15),
                            "mav_bridge ws port never opened")
            self.assertTrue(_wait_tcp(r["web_port"], 60),
                            "vite web port never opened")
            url = (f"http://127.0.0.1:{r['web_port']}"
                   f"/index_4w.html?world=ws://127.0.0.1:"
                   f"{r['stream_port']}")
            code = urllib.request.urlopen(url, timeout=10).status
            self.assertEqual(code, 200)
        finally:
            for name, log, p in procs:
                p.terminate()
                try:
                    p.wait(5)
                except subprocess.TimeoutExpired:
                    p.kill()
                log.close()


def _wait_tcp(port, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            socket.create_connection(("127.0.0.1", port), 0.5).close()
            return True
        except OSError:
            time.sleep(0.5)
    return False


if __name__ == "__main__":
    unittest.main(verbosity=2)
