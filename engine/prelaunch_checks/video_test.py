"""video_test.py - the video judge, and real WebRTC in a real headless Chrome
when one is on this host. The loopback call is real media: only the camera
and the webclient are replaced. PRELAUNCH_TEST_TURN (json list of ICE servers)
adds a run through a real TURN server."""

import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from prelaunch_checks import video  # noqa: E402
import cdp  # noqa: E402

BROWSER_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "prelaunch", "browser")
CHROME = cdp.find_chrome()


class Judge(unittest.TestCase):
    def test_no_peer_connection(self):
        st, d = video.judge([{"pcs": 0}], False)
        self.assertEqual(st, "FAIL")
        self.assertIn("no peer connection", d)

    def test_frames_that_stop_growing_fail(self):
        s = {"pcs": 1, "frames": 500, "candidate": "host", "ice": ["connected"]}
        st, _ = video.judge([s, s], False)
        self.assertEqual(st, "FAIL")

    def test_good(self):
        st, _ = video.judge([{"pcs": 1, "frames": 200},
                             {"pcs": 1, "frames": 900, "candidate": "host",
                              "ice": ["connected"]}], False)
        self.assertEqual(st, "PASS")

    def test_relay_run_on_a_host_path_fails(self):
        st, d = video.judge([{"pcs": 1, "frames": 200},
                             {"pcs": 1, "frames": 900, "candidate": "host",
                              "ice": ["connected"]}], True)
        self.assertEqual(st, "FAIL")
        self.assertIn("not on the relay", d)

    def test_empty_ice_in_relay_run_names_ice(self):
        st, d = video.judge([{"pcs": 1, "frames": 0, "ice": ["new"],
                              "iceServers": 0}] * 2, True)
        self.assertEqual(st, "FAIL")
        self.assertIn("ICE list is empty", d)


@unittest.skipUnless(CHROME, "no Chrome on this host")
class RealMedia(unittest.TestCase):
    def _run(self, options, sender_ice, seconds=10.0, frames_min=50):
        with cdp.Browser(CHROME, options) as br:
            br.navigate("about:blank")
            with open(os.path.join(BROWSER_DIR, "loopback.js")) as fh:
                br.evaluate(fh.read() + ";true")
            br.evaluate("window.__plLoopback(%s)" % json.dumps(sender_ice))
            return video.judge(video.watch(br, seconds),
                               options.get("forceRelay"), frames_min)

    def test_normal_call_decodes_frames(self):
        st, d = self._run({"iceServers": []}, [])
        self.assertEqual(st, "PASS", d)
        self.assertIn("host", d)

    def test_relay_only_with_empty_ice_fails_and_names_ice(self):
        video.ICE_WAIT_S, keep = 8.0, video.ICE_WAIT_S
        self.addCleanup(setattr, video, "ICE_WAIT_S", keep)
        st, d = self._run({"forceRelay": True, "iceServers": []}, [], 4.0)
        self.assertEqual(st, "FAIL")
        self.assertIn("ICE list is empty", d)

    @unittest.skipUnless(os.environ.get("PRELAUNCH_TEST_TURN"),
                         "PRELAUNCH_TEST_TURN is not set")
    def test_relay_only_through_a_real_turn(self):
        ice = json.loads(os.environ["PRELAUNCH_TEST_TURN"])
        st, d = self._run({"forceRelay": True, "iceServers": ice}, ice, 15.0)
        self.assertEqual(st, "PASS", d)
        self.assertIn("relay", d)

    @unittest.skipUnless(os.environ.get("PRELAUNCH_TEST_TURN"),
                         "PRELAUNCH_TEST_TURN is not set")
    def test_relay_only_with_a_dead_turn_host_fails(self):
        video.ICE_WAIT_S, keep = 10.0, video.ICE_WAIT_S
        self.addCleanup(setattr, video, "ICE_WAIT_S", keep)
        dead = [{"urls": "turn:127.0.0.1:9", "username": "x", "credential": "y"}]
        st, d = self._run({"forceRelay": True, "iceServers": dead}, dead, 4.0)
        self.assertEqual(st, "FAIL")
        self.assertIn("relay-only", d)


if __name__ == "__main__":
    unittest.main()
