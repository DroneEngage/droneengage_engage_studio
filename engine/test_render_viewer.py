#!/usr/bin/env python3
"""test_render_viewer.py - pure P5-14 gates (no SITL, no fleet).

  - gen_render() is silent unless render.mode is viewer|full and not
    headless - cameras/physics modes and headless runs launch nothing;
  - the generated launch scripts run the Mavlink3DMap UDP->WS bridge
    on render.mav_port / ws render.mav_ws_port and a vite dev server
    on render.web_port, pid-marked via node --require so sim down sees
    them, under var/run/<sim>/render/;
  - SL-0154: a stack-alive `provides: render_web` replaces the sim's
    own web proc - gen_render returns just mav_bridge, bakes
    render.page_port and render_viewer_url serves it while ?mavlink=
    keeps naming the sim's own bridge port;
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
from unittest import mock

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
import devstack  # noqa: E402

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

    def setUp(self):
        # SL-0154: gen_render asks devstack whether a shared page stack
        # is live - pin it dead so these tests are identical whether or
        # not ./sim up de_mavlink3dmap ran on this box. The shared-page
        # tests re-patch inside their own `with`.
        self._stack_down = mock.patch.object(devstack, "pids",
                                             return_value={})
        self._stack_down.start()

    def tearDown(self):
        self._stack_down.stop()

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

    # ---- SL-0154: a live stack `provides: render_web` replaces the
    # per-sim vite; the sim keeps only its own mav_bridge and bakes the
    # stack's page port into render.page_port

    def test_shared_page_replaces_own_vite(self):
        saved = simlib.MAVLINK3DMAP_DIR
        try:
            simlib.MAVLINK3DMAP_DIR = _fake_m3d()
            with mock.patch.object(simlib, "shared_render_web_port",
                                   return_value=8070):
                sim = _sim({"mode": "viewer", "headless": False})
                stale = os.path.join(
                    simlib.render_dir(sim["name"]), "launch.web.sh")
                if os.path.exists(stale):
                    os.unlink(stale)   # left by the own-vite tests
                procs = simlib.gen_render(sim)
        finally:
            simlib.MAVLINK3DMAP_DIR = saved
        self.assertEqual(procs, ["mav_bridge"])
        self.assertEqual(sim["render"]["page_port"], 8070)
        rdir = simlib.render_dir(sim["name"])
        self.assertFalse(os.path.exists(
            os.path.join(rdir, "launch.web.sh")))

    def test_shared_page_viewer_url(self):
        render = {"mode": "viewer", "web_port": 8080,
                  "stream_port": 61010, "mav_port": 16460,
                  "mav_ws_port": 8811, "page_port": 8070}
        url = simlib.render_viewer_url(render, world_stream=False)
        self.assertTrue(url.startswith("http://127.0.0.1:8070/"))
        # ?mavlink= still selects THIS sim's own bridge
        self.assertIn("mavlink=ws://127.0.0.1:8811", url)
        # no shared page -> the slot-derived port again
        del render["page_port"]
        self.assertTrue(simlib.render_viewer_url(
            render, world_stream=False).startswith(
                "http://127.0.0.1:8080/"))

    def test_shared_page_via_devstack(self):
        """The real lookup: a stack whose provides proc is stack-alive
        hands gen_render its port through devstack.provided_port."""
        stack_port = devstack._port(
            "de_mavlink3dmap",
            next(p for p in devstack.STACKS["de_mavlink3dmap"]["procs"]
                 if p["name"] == "m3d_web"))
        saved = simlib.MAVLINK3DMAP_DIR
        try:
            simlib.MAVLINK3DMAP_DIR = _fake_m3d()
            with mock.patch.object(devstack, "pids",
                                   return_value={"m3d_web": 12345}):
                sim = _sim({"mode": "viewer", "headless": False})
                procs = simlib.gen_render(sim)
        finally:
            simlib.MAVLINK3DMAP_DIR = saved
        self.assertEqual(procs, ["mav_bridge"])
        self.assertEqual(sim["render"]["page_port"], stack_port)

    # ---- DE-0050: virtual cameras - index derivation, feeder procs,
    # the vcam plan and the dead-proc fail-fast

    def _cam_sim(self, feed=None, unit_cfg=None):
        spec = {"version": 1, "name": "vcam_t", "slot_base": 10,
                "server": {"type": "fake",
                           "port": simlib.SERVER_PORT + 10},
                "units": [{"block": "camera_fixed", "name": "cam1"}]}
        if feed is not None:
            spec["camera_feed"] = feed
        if unit_cfg:
            spec["units"][0]["config"] = unit_cfg
        return simlib.load_sim(spec)

    def test_camera_video_indices(self):
        sim = self._cam_sim(unit_cfg={
            "de_camera": {"camera": {"camera_start_index": 3,
                                     "camera_end_index": 3}}})
        self.assertEqual(
            simlib.camera_video_indices(sim["units"][0]), [3])
        # non-camera units derive nothing
        spec = {"version": 1, "name": "vcam_t", "slot_base": 10,
                "server": {"type": "fake",
                           "port": simlib.SERVER_PORT + 10},
                "units": [{"block": "comm_only", "name": "mast1"}]}
        sim2 = simlib.load_sim(spec)
        self.assertEqual(simlib.camera_video_indices(sim2["units"][0]),
                         [])
        self.assertEqual(simlib.camera_missing_nodes(sim2), [])

    def test_camera_feed_gen(self):
        sim = self._cam_sim(feed="testsrc2", unit_cfg={
            "de_camera": {"camera": {"camera_start_index": 3,
                                     "camera_end_index": 3}}})
        self.assertEqual(sim["camera_feed"]["source"], "testsrc2")
        procs = simlib.gen_feeds(sim)
        self.assertEqual(procs, ["cam1_3"])
        body = open(os.path.join(
            simlib.feed_dir(sim["name"]),
            "launch.cam1_3.sh")).read()
        self.assertIn("testsrc2", body)
        self.assertIn("/dev/video3", body)
        self.assertIn("feed.cam1_3.pid", body)
        # pidfile owns the bash wrapper (its cmdline names var/run) -
        # ffmpeg rides & + wait with a TERM trap for `sim down`
        self.assertIn("&", body)
        self.assertIn("trap", body)
        self.assertIn("wait", body)
        # no camera_feed -> no procs
        sim2 = self._cam_sim(unit_cfg={
            "de_camera": {"camera": {"camera_start_index": 3,
                                     "camera_end_index": 3}}})
        self.assertEqual(simlib.gen_feeds(sim2), [])

    def test_vcam_plan(self):
        sim = self._cam_sim(unit_cfg={
            "de_camera": {"camera": {"camera_start_index": 3,
                                     "camera_end_index": 3}}})
        plan = simlib.vcam_plan(sim)
        self.assertEqual(plan["needed"], [3])
        self.assertEqual(sorted(plan["missing"] + plan["real"]
                                + plan["loopback"]), [3])
        if plan["missing"]:
            self.assertEqual(plan["cmd"][0], "modprobe")
            self.assertIn("video_nr=3", plan["cmd"])
            self.assertIn("devices=1", plan["cmd"])
        else:
            self.assertIsNone(plan["cmd"])

    def test_wait_ready_dead_proc_fails_fast(self):
        """A watched proc that already exited returns False at once
        instead of hanging to timeout."""
        import ctl
        u = {"name": "deadcam1", "sim": "vcam_t",
             "modules": ["de_camera"]}
        pf = simlib.pid_path("vcam_t", "deadcam1", "de_camera")
        os.makedirs(os.path.dirname(pf), exist_ok=True)
        # a guaranteed-dead pid: fork + reap
        pid = os.fork()
        if pid == 0:
            os._exit(0)
        os.waitpid(pid, 0)
        with open(pf, "w") as f:
            f.write(str(pid))
        try:
            t0 = time.time()
            self.assertFalse(ctl.wait_ready([u], 30))
            self.assertLess(time.time() - t0, 10)
        finally:
            os.unlink(pf)

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
