#!/usr/bin/env python3
"""stub_ai_box.py - a world-driven AI box (TASK-P5-08).

What a real camera-AI module does: it looks at the world through the
unit's camera, finds objects and reports the best target's normalized
box as TYPE_AndruavMessage_AI_Recognition_TargetLocation (1078), with a
class list + status over AI_Recognition_STATUS (1077) and honoring a
SEARCH class selection on AI_Recognition_ACTION (1076).

This stub does exactly that - against the de.world/1 truth. Every
fps_boxes frame it takes the unit's pose (SITL serial5 attitude +
position on FC units, the 9102 intermodule feed on statics), projects
each visible entity into the camera frame with the geo_projection port,
rolls the seeded detection probability (nadir vs frame-edge), adds the
modelled pixel noise and sends the 1078. `tm` is the monotonic-us stamp
the de_mavlink pose ring buffer indexes - the real path from box to
detect.detected runs unchanged.

Every emitted box also lands a {"type":"detection", tid, cls, ...} row
in run/<sim>/truth.jsonl (tid "clutter" for the false positives the
clutter model injects) - detection_precision counts them.

Live runs read the scenario clock from run/<sim>/world_epoch.json;
--selftest <poses.jsonl> replays scripted poses on a virtual clock so
same-seed stub logs are byte-identical.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stub_base import (StubBase, ANDRUAV_PROTOCOL_MESSAGE_TYPE,  # noqa: E402
                       ANDRUAV_PROTOCOL_MESSAGE_CMD)
from messages import (  # noqa: E402
    TYPE_AndruavMessage_AI_Recognition_ACTION,
    TYPE_AndruavMessage_AI_Recognition_STATUS,
    TYPE_AndruavMessage_AI_Recognition_TargetLocation)
import geo_projection as gp  # noqa: E402

AI_STATUS_ENABLED = 2
AI_STATUS_CLASS_LIST = 4
AI_ACTION_SEARCH = 1

# the stub plays the whole detection chain on the unit: the camera that
# frames the world plus the tracker the mission may hand a box to - one
# advert per namespace, as the real de_camera / de_tracker pair would
# publish (actions mirroring droneengage_camera's de.cap/1 advert).
ADVERT_CAMERA = {
    "schema": "de.cap/1",
    "ns": "camera",
    "module": "stub_ai_box",
    "ver": "0.1.0",
    "actions": {
        "capture": {
            "desc": "Capture a photo",
            "params": {
                "count": {"type": "int", "min": 1, "max": 100,
                          "default": 1},
                "interval_ms": {"type": "int", "min": 0, "max": 60000,
                                "default": 0, "unit": "ms"},
            },
            "timeout_s": 10,
        },
        "record": {
            "desc": "Start/stop video recording",
            "params": {"on": {"type": "bool"}},
            "timeout_s": 10,
        },
    },
    "events": {
        "captured": {
            "desc": "photo written to storage",
            "payload": {"file": {"type": "string"},
                        "pos": {"type": "geo"}},
        },
    },
    "state": {"recording": {"type": "bool"}},
}

ADVERT_TRACKER = {
    "schema": "de.cap/1",
    "ns": "visual_tracker",
    "module": "stub_ai_box",
    "ver": "0.1.0",
    "actions": {
        "track": {
            "desc": "Track the object inside a normalized box",
            "params": {"x": {"type": "number", "min": 0.0, "max": 1.0},
                       "y": {"type": "number", "min": 0.0, "max": 1.0},
                       "w": {"type": "number", "min": 0.0, "max": 1.0},
                       "h": {"type": "number", "min": 0.0, "max": 1.0}},
            "timeout_s": 10,
        },
        "untrack": {"desc": "Drop the tracked object", "timeout_s": 10},
    },
    "events": {},
    "state": {"tracking": {"type": "bool"}},
}


class StubAIBox(StubBase):
    MODULE_CLASS = "gen"
    MODULE_VERSION = "0.1.0"

    def __init__(self, argv):
        super().__init__(argv, "stub_ai_box")
        cam = (self.world.sensors["camera"] if self.world
               else dict(__import__("world").CAMERA_DEFAULTS))
        self.cam = cam
        self.camera = gp.GeoCameraParams(cam["hfov_deg"], cam["vfov_deg"])
        self.mount = gp.GeoMountParams(cam["mount_pitch_deg"],
                                     cam["mount_yaw_deg"],
                                     cam["mount_roll_deg"])
        self.classes = self.world.classes() if self.world else ["object"]
        self._sel = None                 # 1076 SEARCH index selection
        self._last_frame = -1
        self._clutter = []               # [(die_t_s, u, v)]
        self._status_next = 0.0
        self._recording = False
        self._tracking = False
        self._capture_count = 0

    # ----------------------------------------------------------- invokes

    def on_invoke(self, ns, act, params):
        """handler(ns, act, params) -> (data, err), as the real modules."""
        if ns == "camera" and act == "capture":
            self._capture_count += 1
            pose = self.pose_feed.pose()
            pos = [pose["lat"], pose["lng"]] if pose else None
            result = {"file": f"cap_{self._capture_count}.jpg",
                      "pos": pos}
            self.log.log("CAPTURE", result, ms=self.clock_ms())
            self.state({"captures": self._capture_count})
            return result, None
        if ns == "camera" and act == "record":
            self._recording = bool((params or {}).get("on", True))
            self.log.log("RECORD", {"on": self._recording},
                         ms=self.clock_ms())
            self.state({"recording": self._recording})
            return {"recording": self._recording}, None
        if ns == "visual_tracker" and act == "track":
            self._tracking = True
            self.log.log("TRACK", {"box": params}, ms=self.clock_ms())
            self.state({"tracking": True})
            return {"tracking": True}, None
        if ns == "visual_tracker" and act == "untrack":
            self._tracking = False
            self.log.log("UNTRACK", {}, ms=self.clock_ms())
            self.state({"tracking": False})
            return {"tracking": False}, None
        return None, f"unknown action '{ns}.{act}'"

    # -------------------------------------------------------- databus out

    def state(self, changed, full=False):
        if self.facade is not None:
            self.facade.publish_state("camera", changed, full)

    # ---------------------------------------------------------- messages

    def emit(self, mid, ms):
        """One intermodule message - sent when connected, always logged
        in the deterministic stub log."""
        self.log.log("SEND", {"mt": mid, "ms": ms}, ms=self.clock_ms())
        if self.module is not None:
            self.module.sendJMSG("", ms, mid, True)

    def status(self):
        self.emit(TYPE_AndruavMessage_AI_Recognition_STATUS,
                  {"a": AI_STATUS_ENABLED})
        self.emit(TYPE_AndruavMessage_AI_Recognition_STATUS,
                  {"a": AI_STATUS_CLASS_LIST, "c": list(self.classes)})

    def on_message(self, message, length, jMsg):
        super().on_message(message, length, jMsg)
        try:
            if jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_TYPE) == \
                    TYPE_AndruavMessage_AI_Recognition_ACTION:
                ms = jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_CMD) or {}
                if ms.get("a") == AI_ACTION_SEARCH and \
                        isinstance(ms.get("i"), list):
                    self._sel = [int(i) for i in ms["i"]]
                    self.log.log("SELECT", {"i": self._sel},
                                 ms=self.clock_ms())
        except Exception:
            pass

    def reset_run(self):
        self._sel = None
        self._last_frame = -1
        self._clutter = []
        self._status_next = 0.0

    def _class_ok(self, cls):
        """A SEARCH selection limits detections to the chosen classes."""
        if not self._sel:
            return True
        return any(0 <= i < len(self.classes) and self.classes[i] == cls
                   for i in self._sel)

    # ------------------------------------------------------------- model

    def tick(self, t_s, _frame20):
        if self.world is None:
            return
        if t_s >= self._status_next:
            self._status_next = t_s + 5.0
            self.status()
        fps = float(self.cam["fps_boxes"])
        frame = int(t_s * fps)
        if frame == self._last_frame:
            return
        self._last_frame = frame
        pose = self.pose_feed.pose()
        if pose is None:
            return
        alt = float(pose["rel_alt_m"])
        if alt < 1.0 and not self.pose_port:
            # static mast (SU-04): the camera sits mount_height_m above
            # the flat ground - the same baseline de_comm's static
            # projector uses
            alt = float(self.cam["mount_height_m"])
        if alt < 1.0:
            return                      # still on the ground
        att = gp.GeoAttitude(roll_rad=pose["roll"],
                             pitch_rad=pose["pitch"],
                             yaw_rad=pose["yaw"])
        gp_pose = gp.GeoPose(lat_deg=pose["lat"], lng_deg=pose["lng"],
                             rel_alt_m=alt)
        self._clutter = [c for c in self._clutter if c[0] > t_s]

        for e in self.world.entities:
            if not self._class_ok(e.cls):
                continue
            if self.cam.get("detect_cls") and \
                    e.cls not in self.cam["detect_cls"]:
                continue
            lat, lng, ealt = e.pos_at(t_s, self.world)
            self._maybe_detect(e.id, e.cls, lat, lng, ealt,
                               pose, att, gp_pose, frame, t_s)

        self._clutter_tick(alt, frame, t_s)

    def _maybe_detect(self, tid, cls, lat, lng, ealt,
                      pose, att, gp_pose, frame, t_s):
        cam = self.cam
        uv = gp.geo_ground_to_pixel(lat, lng, ealt, self.camera,
                                    self.mount, att, gp_pose)
        if uv is None:
            return
        u, v, rng_m = uv
        if abs(u) > 0.5 or abs(v) > 0.5:
            return                      # outside the frame
        if rng_m > cam["max_range_m"]:
            return
        # p_detect interpolates nadir -> frame edge by the off-axis
        # fraction of the HALF-frame the point occupies
        edge = max(abs(u), abs(v)) * 2.0
        p = cam["p_detect_nadir"] + \
            (cam["p_detect_edge"] - cam["p_detect_nadir"]) * min(1.0, edge)
        r = self.rng("detect", self.unit, tid, frame)
        if r.random() >= p:
            self.truth.append({"src": "ai", "tid": tid, "cls": cls,
                               "frame": frame, "u": round(u, 4),
                               "v": round(v, 4), "lat": lat, "lng": lng,
                               "alt_m": ealt, "sent": False},
                              ms=self.clock_ms())
            return
        # pixel noise in normalized units (pos_noise_px / image size)
        nu = r.gauss(0.0, cam["pos_noise_px"] / cam["img_w"])
        nv = r.gauss(0.0, cam["pos_noise_px"] / cam["img_h"])
        u_n, v_n = u + nu, v + nv
        # box size: the entity's real size projected at its range
        size = self.world.size_m(self.world.entity(tid)) \
            if self.world.entity(tid) else 0.5
        bw = min(1.0, size / (2.0 * rng_m *
                              math.tan(math.radians(cam["hfov_deg"]) / 2)))
        bh = min(1.0, size / (2.0 * rng_m *
                              math.tan(math.radians(cam["vfov_deg"]) / 2)))
        conf = round(r.uniform(0.75, 0.99), 3)
        box = {"x": round(max(0.0, u_n + 0.5 - bw / 2), 4),
               "y": round(max(0.0, v_n + 0.5 - bh / 2), 4),
               "w": round(bw, 4), "h": round(bh, 4),
               "conf": conf, "tm": self._tm_us()}
        self.emit(TYPE_AndruavMessage_AI_Recognition_TargetLocation,
                  {"b": box})
        self.truth.append({"src": "ai", "tid": tid, "cls": cls,
                           "frame": frame, "u": round(u_n, 4),
                           "v": round(v_n, 4), "lat": lat, "lng": lng,
                           "alt_m": ealt, "conf": conf, "sent": True},
                          ms=self.clock_ms())

    def _tm_us(self):
        """de_mavlink's sampleAt() indexes its pose ring buffer by
        monotonic microseconds - stamp the same clock, a touch in the
        past so a sample already exists. Virtual ms under selftest (no
        live buffer there - the field just stays deterministic)."""
        if self._vt_ms is not None:
            return self._vt_ms * 1000
        import time
        return time.monotonic_ns() // 1000 - 50000

    def _clutter_tick(self, alt, frame, t_s):
        """False positives: spawns at rate_per_km2_min over the frame's
        ground footprint, each lives lifetime_s and reports like a
        detection with the clutter class. tid "clutter" lets
        detection_precision tell them apart. `alt` is the camera height
        tick() resolved (rel alt, or mount_height_m on a static mast)."""
        cam = self.cam
        cl = self.world.clutter
        rate = float(cl.get("rate_per_km2_min") or 0.0)
        if rate <= 0.0:
            return
        fn = math.tan(math.radians(cam["hfov_deg"]) / 2)
        fv = math.tan(math.radians(cam["vfov_deg"]) / 2)
        area_km2 = (2 * alt * fn) * (2 * alt * fv) / 1e6
        fps = float(cam["fps_boxes"])
        r = self.rng("clutter", self.unit, frame)
        # Poisson(lambda) for small lambda per frame
        lam = rate * area_km2 / (60.0 * fps)
        n = 0
        p = r.random()
        cum = math.exp(-lam)
        while p > cum and n < 8:
            n += 1
            cum += math.exp(-lam) * lam ** n / math.factorial(n)
        life = float(cl.get("lifetime_s") or 20.0)
        for _i in range(n):
            self._clutter.append(
                (t_s + life, r.uniform(-0.45, 0.45),
                 r.uniform(-0.45, 0.45)))
        for die, u, v in self._clutter:
            if not self._class_ok(cl["cls"]):
                break
            rng_m = alt / math.cos(
                math.atan(math.hypot(u * 2 * fn, v * 2 * fv)))
            bw = min(1.0, 0.5 / (2.0 * rng_m * fn))
            bh = min(1.0, 0.5 / (2.0 * rng_m * fv))
            conf = round(r.uniform(0.4, 0.7), 3)
            box = {"x": round(max(0.0, u + 0.5 - bw / 2), 4),
                   "y": round(max(0.0, v + 0.5 - bh / 2), 4),
                   "w": round(bw, 4), "h": round(bh, 4),
                   "conf": conf, "tm": self._tm_us()}
            self.emit(TYPE_AndruavMessage_AI_Recognition_TargetLocation,
                      {"b": box})
            self.truth.append({"src": "ai", "tid": "clutter",
                               "cls": cl["cls"], "frame": frame,
                               "u": round(u, 4), "v": round(v, 4),
                               "conf": conf, "sent": True},
                              ms=self.clock_ms())


def main(argv):
    s = StubAIBox(argv)
    if s.selftest:
        s._vt_ms = 0
        s.log.log("SELFTEST", {"poses": s.selftest}, ms=0)
        s.run_selftest()
        return 0
    s.connect(adverts=[__import__("json").dumps(ADVERT_CAMERA),
                       __import__("json").dumps(ADVERT_TRACKER)],
              msg_filter=[TYPE_AndruavMessage_AI_Recognition_ACTION])
    s.status()
    s.run()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
