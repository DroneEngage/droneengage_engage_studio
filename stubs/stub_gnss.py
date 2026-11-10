#!/usr/bin/env python3
"""stub_gnss.py - a GNSS source on the bus, without a phone (SL-0233).

Reads the SITL truth pose (PoseFeed, serial5) and sends the GNSS fix
type (9103, TYPE_AndruavModule_GNSS_Fix) at 1 Hz, the way the phone app
or a GNSS module does. de_mavlink turns it into GPS_INPUT for the FC.

    {la, ln: degE7, a: alt MSL mm, fix, ns, hd, vd, ha, va, src, gid}

Fix type, satellites, DOP and accuracies come from stub_gnss.sensor.yaml
(`world.sensors.gnss` overrides them). The position is exactly the SITL
truth, so a test can hold the fix to within 1 m of the pose.

Why SIMSTATE: on a MAV-GPS unit the FC has no GPS, so its own
GLOBAL_POSITION_INT is 0,0 until the fix arrives. The stub reads the
SITL truth from SIMSTATE (164: lat/lng degE7) on the same serial5
stream. SIMSTATE has no altitude: "a" is the model's alt_msl_m (SITL
home, 584 m at CMAC) plus the FC relative altitude.

Control: de.cap/1 ns "gnss", actions "stop" and "start" gate the output
(a dead source). Every sent fix goes into the stub log as
"<ms> SEND {"mt": 9103, "ms": {...}}"; a stopped second logs nothing.
"""

import json
import os
import sys

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stub_base import StubBase, PoseFeed, now_ms  # noqa: E402
from messages import TYPE_AndruavModule_GNSS_Fix  # noqa: E402

TICKS_PER_S = 20          # StubBase.run() quantum

ADVERT = {
    "schema": "de.cap/1",
    "ns": "gnss",
    "module": "stub_gnss",
    "ver": "0.1.0",
    "desc": "simulated GNSS source: sends 9103 fixes from the SITL pose",
    "actions": {
        "start": {"desc": "Send fixes again", "timeout_s": 5},
        "stop": {"desc": "Stop sending fixes (a dead source)",
                 "timeout_s": 5},
    },
    "state": {"enabled": {"type": "bool"}},
}


class TruthPoseFeed(PoseFeed):
    """PoseFeed that prefers SITL SIMSTATE (the simulator's own truth)
    over GLOBAL_POSITION_INT, which is 0,0 while the FC has no GPS."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self._truth = {}

    def _run(self):
        from pymavlink import mavutil
        while not self._stop.is_set():
            try:
                m = mavutil.mavlink_connection(
                    f"tcp:127.0.0.1:{self._port}", autoreconnect=False,
                    retries=2, source_system=252)
                hb = m.wait_heartbeat(timeout=5)
                sysid = m.target_system if hb is not None else self._sysid
                for mid in (30, 33, 164):    # ATT, GLOBAL_POS, SIMSTATE
                    m.mav.command_long_send(
                        sysid, 1, 511, 0, mid, 100000, 0, 0, 0, 0, 0)
                while not self._stop.is_set():
                    msg = m.recv_match(type=["ATTITUDE",
                                             "GLOBAL_POSITION_INT",
                                             "SIMSTATE"],
                                       blocking=True, timeout=3.0)
                    if msg is None:
                        raise ConnectionError("pose stream stalled")
                    with self._lock:
                        t = msg.get_type()
                        if t == "ATTITUDE":
                            self._att = {"roll": msg.roll,
                                         "pitch": msg.pitch,
                                         "yaw": msg.yaw, "t_ms": now_ms()}
                        elif t == "SIMSTATE":
                            self._truth = {"lat": msg.lat / 1e7,
                                           "lng": msg.lng / 1e7,
                                           "t_ms": now_ms()}
                        else:
                            self._pos = {"lat": msg.lat / 1e7,
                                         "lng": msg.lon / 1e7,
                                         "alt_m": msg.alt / 1000.0,
                                         "rel_alt_m":
                                             msg.relative_alt / 1000.0,
                                         "t_ms": now_ms()}
            except Exception:
                self._stop.wait(2.0)

    def pose(self):
        p = super().pose()
        with self._lock:
            tr = dict(self._truth)
        if tr and (self.max_age_ms is None
                   or now_ms() - tr["t_ms"] <= self.max_age_ms):
            p = dict(p or {"alt_m": 0.0, "rel_alt_m": 0.0, "roll": 0.0,
                           "pitch": 0.0, "yaw": 0.0, "t_ms": now_ms()})
            p["lat"], p["lng"], p["src"] = tr["lat"], tr["lng"], "simstate"
        return p


class StubGnss(StubBase):
    MODULE_CLASS = "gen"
    MODULE_VERSION = "0.1.0"

    def __init__(self, argv):
        super().__init__(argv, "stub_gnss")
        m = self.sensor_params()
        self.rate_hz = max(0.1, float(m.get("rate_hz", 1.0)))
        self.every = max(1, int(round(TICKS_PER_S / self.rate_hz)))
        self.model = {
            "fix": int(m.get("fix_type", 3)),
            "ns": int(m.get("satellites", 12)),
            "hd": int(m.get("hdop_x100", 90)),
            "vd": int(m.get("vdop_x100", 130)),
            "ha": int(m.get("h_acc_mm", 1500)),
            "va": int(m.get("v_acc_mm", 2500)),
            "src": str(m.get("src", "stub")),
            "gid": int(m.get("gps_id", 0)),
        }
        self.alt_msl_m = float(m.get("alt_msl_m", 584.0))
        self.pose_feed = TruthPoseFeed(self.pose_port,
                                       sysid=self.pose_feed._sysid)
        self.enabled = True

    def reset_run(self):
        self.enabled = True

    def build_fix(self, pose):
        ms = dict(self.model)
        ms["la"] = int(round(pose["lat"] * 1e7))
        ms["ln"] = int(round(pose["lng"] * 1e7))
        # SIMSTATE has no altitude: model home alt + FC relative alt;
        # the pose "alt_m" is used when the FC reports a real one
        alt = pose.get("alt_m", 0.0) or \
            self.alt_msl_m + pose.get("rel_alt_m", 0.0)
        ms["a"] = int(round(alt * 1000.0))
        return ms

    def emit(self, ms):
        self.log.log("SEND", {"mt": TYPE_AndruavModule_GNSS_Fix, "ms": ms},
                     ms=self.clock_ms())
        if self.module is not None:
            self.module.sendJMSG("", ms, TYPE_AndruavModule_GNSS_Fix, True)

    def state(self, changed, full=False):
        self.log.log("STATE", changed, ms=self.clock_ms())
        if self.facade is not None:
            self.facade.publish_state("gnss", changed, full)

    def on_invoke(self, ns, act, params):
        if act in ("start", "stop"):
            self.enabled = act == "start"
            self.log.log("CONTROL", {"act": act}, ms=self.clock_ms())
            self.state({"enabled": self.enabled})
            return {"enabled": self.enabled}, None
        return None, f"unknown action gnss.{act}"

    def tick(self, t_s, frame):
        if not self.enabled or frame % self.every:
            return
        pose = self.pose_feed.pose()
        if pose is None:
            return
        self.emit(self.build_fix(pose))


def main(argv):
    s = StubGnss(argv)
    if s.selftest:
        s.run_selftest()
        return 0
    s.connect(adverts=[json.dumps(ADVERT)])
    s.state({"enabled": True}, full=True)
    s.run()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
