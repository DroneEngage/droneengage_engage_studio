#!/usr/bin/env python3
"""stub_ai_box.py - a world-driven AI box (TASK-P5-08).

What a real camera-AI module does: it looks at the world through the
unit's camera, finds objects and reports them as an observation frame -
TYPE_AndruavMessage_AI_OBSERVATIONS (6546), one per camera frame, every
visible entity an entry with cls / cls_id / conf / trk / top-left [0..1]
box (SL-0034; it replaced the 1078 best-box wire the deleted
de_yolo_generic_rknn-era path spoke). Status still rides
AI_Recognition_STATUS (1077, enabled state only - the class list moved
to the 'perceive' advert, MP-0214) and a SEARCH class selection on
AI_Recognition_ACTION (1076) is honored.

This stub does exactly that - against the de.world/1 truth. Every
fps_boxes frame it takes the unit's pose (SITL serial5 attitude +
position on FC units, the 9102 intermodule feed on statics), projects
each visible entity into the camera frame with the geo_projection port,
rolls the seeded detection probability (nadir vs frame-edge), adds the
modelled pixel noise and sends the obs frame. `tm_capture` is the
monotonic-us stamp the de_mavlink pose ring buffer indexes - the real
path from box to detect.detected runs unchanged. `trk` is a stable
per-entity id for the run (first-detection order, deterministic per
seed); cam_id defaults to the unit name (world sensors.camera.cam_id
overrides).

Thermal (camera_kind: thermal, TASK-13): a detected entity is reported
as cls "hotspot" (temp_c >= ambient) or "coldspot" (below) with
attrs.temp_c - the de_ir_camera vocabulary - instead of its entity
label; cls_id is omitted since the thermal pair is not a class_names
index.

B4-01 legacy: `--legacy-box`, a unit `config.stub_ai_box.legacy_box` or
the world's `sensors.camera.legacy_box` restores the pre-obs wire - one
1078 TargetLocation {"b": box} per detection, no class fields, no obs
frames. The negative-control rerun of p4b10_k uses it.

B4-02: the detector's LABEL for an entity cls can differ from the cls
itself - a person-detecting model can report the string "human" while
the op's target class stays "person". `class_labels` remaps cls -> label
(world `sensors.camera.class_labels`, per-unit override
`config.stub_ai_box.class_labels`); obs entries then carry the label as
`cls` (and `cls_id` indexes the label list, which is also what the
visual_tracker.start class enum and the 'perceive' advert classes report)
so the op's detect decl `classes: [...]` is what accepts them - p4b10_l.


Every emitted box also lands a {"type":"detection", tid, cls, ...} row
in var/run/<sim>/truth.jsonl (tid "clutter" for the false positives the
clutter model injects) - detection_precision counts them.

P5-11: the world environment scales the model - p_detect picks up
f_light * f_precip, max_range_m is capped by visibility_m and pixel
noise grows 1/f_light in the dark (a unit config camera_kind:
thermal swaps the nadir/edge ramp for p_detect_thermal and ignores
light - the same dusk world can fly both roles). TASK-13 refines
the thermal p with the contrast ESTIMATE: it scales by
|entity temp_c - ambient| so a body sunk to ambient (the dawn/dusk
crossover) reads 0.

Live runs read the scenario clock from var/run/<sim>/world_epoch.json;
--selftest <poses.jsonl> replays scripted poses on a virtual clock so
same-seed stub logs are byte-identical.
"""

import math
import os
import sys

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stub_base import (StubBase, ANDRUAV_PROTOCOL_MESSAGE_TYPE,  # noqa: E402
                       ANDRUAV_PROTOCOL_MESSAGE_CMD)
from messages import (  # noqa: E402
    TYPE_AndruavMessage_AI_OBSERVATIONS,
    TYPE_AndruavMessage_AI_Recognition_ACTION,
    TYPE_AndruavMessage_AI_Recognition_STATUS,
    TYPE_AndruavMessage_Sync_EventFire)
import simlib  # noqa: E402
import geo_projection as gp  # noqa: E402

AI_STATUS_ENABLED = 2
AI_STATUS_CLASS_LIST = 4       # legacy wire only (--legacy-box)
AI_ACTION_SEARCH = 1
# the pre-obs TargetLocation type - gone from the databus constants
# since the projector stopped consuming it; --legacy-box still speaks it
TYPE_AI_TARGET_LOCATION_LEGACY = 1078

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

# SL-0034: the stub plays the detector too - the 'perceive' advert is
# the de_perceive one (de.cap/1, classes[] is the MP-0214 replacement
# for the dropped 1077 class list). classes is filled per unit.
# MP-0215: the namespace is cam-scoped (perceive_<cam_id>, bare
# 'perceive' only when cam_id is empty - the module's perceiveNamespace
# mirror) and the advert offers the runtime configure/restore actions a
# mission 'perceive' step and the operator panel invoke.
CAP_NAME_MAX_LEN = 32


def perceive_namespace(cam_id):
    """de_perceive perceiveNamespace() mirror: 'perceive' on an empty
    cam_id, else 'perceive_<cam>' lowercased, non [a-z0-9_] -> '_',
    capped at CAP_NAME_MAX_LEN."""
    ns = "perceive"
    if not cam_id:
        return ns
    ns += "_"
    for c in str(cam_id):
        if len(ns) >= CAP_NAME_MAX_LEN:
            break
        l = c.lower()
        ns += l if (l.isalnum() and l.isascii()) or l == "_" else "_"
    return ns


ADVERT_PERCEIVE = {
    "schema": "de.cap/1",
    "ns": "perceive",            # adverts() rewrites to the cam-scoped ns
    "module": "stub_ai_box",
    "ver": "0.1.0",
    "actions": {
        "configure": {
            "desc": "apply a runtime perception override: restrict the "
                    "reported classes to the given labels and optionally "
                    "set the confidence gate; an empty classes list "
                    "pauses reporting (perceive nothing)",
            "params": {
                "classes": {"type": "array",
                            "items": {"type": "string"},
                            "desc": "class labels to report; "
                                    "empty = perceive nothing"},
                "min_conf": {"type": "number", "min": 0.0, "max": 1.0,
                             "optional": True,
                             "desc": "track confidence gate; absent "
                                     "keeps the current value"},
                "cam": {"type": "string", "optional": True,
                        "desc": "cam_id routing hint; must match this "
                                "perceiver when present"},
            },
            "timeout_s": 10,
        },
        "restore": {
            "desc": "restore the unit default: every class, the "
                    "configured confidence gate, the startup "
                    "detecting state",
            "params": {
                "cam": {"type": "string", "optional": True,
                        "desc": "cam_id routing hint; must match this "
                                "perceiver when present"},
            },
            "timeout_s": 10,
        },
    },
    "events": {},
    "state": {
        "cam_id":  {"type": "string"},
        "backend": {"type": "string"},
        "classes": {"type": "array", "items": {"type": "string"}},
        "detecting": {"type": "bool"},
        "active_classes": {"type": "array", "items": {"type": "string"}},
        "min_conf": {"type": "number"},
        "fps":     {"type": "number"},
        "hfov":    {"type": "number"},
    },
}


class StubAIBox(StubBase):
    MODULE_CLASS = "gen"
    MODULE_VERSION = "0.1.0"

    def __init__(self, argv):
        super().__init__(argv, "stub_ai_box")
        cam = (self.world.sensors["camera"] if self.world
               else dict(__import__("world").CAMERA_DEFAULTS))
        self.cam = cam
        # P5-11 D5: a unit config camera_kind (rgb|thermal) overrides
        # the world camera's kind - one dusk world flies both roles
        self.cam_kind = str(self.cfg.get("camera_kind")
                            or cam.get("kind", "rgb"))
        if self.cam_kind not in ("rgb", "thermal"):
            raise simlib.SimError(
                f"stub_ai_box: camera_kind {self.cam_kind!r} "
                "(rgb | thermal)")
        self.env_f = (self.world.sensor_factors(self.cam_kind)
                      if self.world
                      else {"light": 1.0, "precip": 1.0, "p_scale": 1.0,
                            "noise_scale": 1.0,
                            "max_range_m": cam["max_range_m"]})
        self.camera = gp.GeoCameraParams(cam["hfov_deg"], cam["vfov_deg"],
                                         cam.get("lens", "rectilinear"),
                                         cam.get("k1", 0.0),
                                         cam.get("k2", 0.0))
        self.mount = gp.GeoMountParams(cam["mount_pitch_deg"],
                                     cam["mount_yaw_deg"],
                                     cam["mount_roll_deg"])
        # PL-08: a cls "pad" entity is a precision-landing board, not an
        # AI detection class - it stays out of the 1077 class list and
        # is never projected as a box
        self.classes = ([c for c in self.world.classes() if c != "pad"]
                        if self.world else ["object"])
        # B4-02: class_labels maps an entity cls -> the detector label
        # the model actually reports ("person" -> "human"). World-wide
        # under sensors.camera.class_labels, per-unit override
        # config.stub_ai_box.class_labels. Truth cls stays untouched -
        # only what goes on the wire (class_name, 1077 list, advert
        # enum) carries the label.
        label_map = dict(cam.get("class_labels") or {})
        label_map.update(self.cfg.get("class_labels") or {})
        self.label_map = label_map
        self.labels = [self.label_map.get(c, c) for c in self.classes]
        # SL-0034: the wire cls table cls_id indexes and the 'perceive'
        # advert reports - the detector labels on rgb, the de_ir_camera
        # vocabulary on thermal
        self.obs_labels = (["hotspot", "coldspot"]
                           if self.cam_kind == "thermal" else self.labels)
        # obs frame identity: cam_id is what de_mavlink's camera
        # registry binds the frame to (unit name by default)
        self.cam_id = str(cam.get("cam_id") or self.unit or "cam")
        # MP-0215: runtime perception configuration. The unit config
        # (stub_ai_box.enabled / classes / min_conf) is the unit default
        # perceive.restore returns to; a configure override scopes the
        # reported labels at the emit seam (the world still sees a
        # dropped detection - its truth row reads sent:false,
        # why:"filtered").
        dflt_classes = self.cfg.get("classes")
        if dflt_classes is not None:
            if not isinstance(dflt_classes, list) or \
                    any(c not in self.obs_labels for c in dflt_classes):
                raise simlib.SimError(
                    f"stub_ai_box: classes {dflt_classes!r} must be a "
                    f"subset of the detector labels {self.obs_labels}")
            dflt_classes = [str(c) for c in dflt_classes]
        self._perceive_defaults = {
            "reporting": bool(self.cfg.get("enabled", True)),
            "classes": dflt_classes,
            "min_conf": float(self.cfg.get("min_conf", 0.0)),
        }
        self._reporting = self._perceive_defaults["reporting"]
        self._perceive_filter = (set(dflt_classes)
                                 if dflt_classes is not None else None)
        self._min_conf = self._perceive_defaults["min_conf"]
        if self._perceive_filter is not None and \
                not self._perceive_filter:
            # classes: [] as the configured default = start paused
            self._reporting = False
        self._perceive_ns = perceive_namespace(self.cam_id)
        self._nonce_ring = []            # Sync_EventFire dedupe
        self._nonce_set = set()
        self._trk_ids = {}               # tid -> stable int trk (run)
        self._frame_obs = []             # obs entries for this frame
        # B4-01: `--legacy-box` restores the pre-obs wire - one 1078
        # TargetLocation per detection, no class fields, no obs frames
        # (the negative-control rerun needs the legacy shape)
        self.legacy_box = bool("--legacy-box" in argv
                               or self.cfg.get("legacy_box")
                               or cam.get("legacy_box"))
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
        if ns == "visual_tracker" and act == "start":
            # B4-02: start(class) picks the detection label set - the
            # stub keeps reporting every class regardless (the world
            # camera is the only gate) but logs the selection
            self.log.log("START", {"class": (params or {}).get("class")},
                         ms=self.clock_ms())
            return {"started": (params or {}).get("class")}, None
        if ns == self._perceive_ns:
            if act == "configure":
                return self._perceive_configure(params, "invoke")
            if act == "restore":
                return self._perceive_restore(params, "invoke")
        return None, f"unknown action '{ns}.{act}'"

    # ------------------------------------------------- perceive (MP-0215)

    def _publish_perceive_state(self):
        if self.facade is not None:
            self.facade.publish_state(self._perceive_ns,
                                      self._perceive_state())

    def _perceive_configure(self, params, cause):
        """perceive.configure - the mission step's invoke AND the
        operator panel's Sync_EventFire land here. Returns
        (result, err) like every invoke handler."""
        params = params or {}
        cam = params.get("cam")
        if isinstance(cam, str) and cam and cam != self.cam_id:
            return None, \
                f"perceive cam '{cam}' not served by '{self.cam_id}'"
        classes = params.get("classes")
        if classes is None:
            classes = []
        if not isinstance(classes, list) or \
                any(not isinstance(c, str) for c in classes):
            return None, "classes must be an array of strings"
        unknown = [c for c in classes if c not in self.obs_labels]
        if unknown:
            return None, f"unknown class(es): {', '.join(unknown)}"
        mc = params.get("min_conf")
        if mc is not None:
            if not isinstance(mc, (int, float)) or \
                    isinstance(mc, bool) or not 0.0 <= mc <= 1.0:
                return None, "min_conf must be a number in [0,1]"
            self._min_conf = float(mc)
        if len(classes) == 0:
            # perceive nothing - reporting paused until the next
            # configure or a restore
            self._reporting = False
            self._perceive_filter = None
        else:
            self._reporting = True
            self._perceive_filter = set(classes)
        active = sorted(self._perceive_filter or [])
        print(f"perceive configure ({cause}): classes={active} "
              f"min_conf={self._min_conf} detecting={self._reporting}",
              flush=True)
        self.log.log("PERCEIVE", {"cmd": "configure", "classes": active,
                     "min_conf": self._min_conf,
                     "detecting": self._reporting, "cause": cause},
                     ms=self.clock_ms())
        self._publish_perceive_state()
        return {"classes": active, "min_conf": self._min_conf,
                "detecting": self._reporting}, None

    def _perceive_restore(self, params, cause):
        """perceive.restore - back to the unit's configured default."""
        cam = (params or {}).get("cam")
        if isinstance(cam, str) and cam and cam != self.cam_id:
            return None, \
                f"perceive cam '{cam}' not served by '{self.cam_id}'"
        self._reporting = self._perceive_defaults["reporting"]
        self._perceive_filter = (
            set(self._perceive_defaults["classes"])
            if self._perceive_defaults["classes"] is not None else None)
        self._min_conf = self._perceive_defaults["min_conf"]
        print(f"perceive restore ({cause}): detecting={self._reporting} "
              f"classes={sorted(self._perceive_filter or [])} "
              f"min_conf={self._min_conf}", flush=True)
        self.log.log("PERCEIVE", {"cmd": "restore",
                     "detecting": self._reporting, "cause": cause},
                     ms=self.clock_ms())
        self._publish_perceive_state()
        return {"detecting": self._reporting}, None

    def _on_perceive_event(self, cmd):
        """A forwarded Sync_EventFire command: {d:'perceive.configure'|
        'perceive.restore'|'<perceive_ns>.<verb>', ed?, m?, n?}. The
        bare 'perceive' name rides the same handler as the cam-scoped
        namespace - de_comm resolves the member at dispatch."""
        d = cmd.get("d")
        if not isinstance(d, str) or "." not in d:
            return
        ns, _, verb = d.partition(".")
        if ns not in ("perceive", self._perceive_ns):
            return
        if verb not in ("configure", "restore"):
            return
        nonce = cmd.get("n")
        if isinstance(nonce, str):
            if nonce in self._nonce_set:
                return
            self._nonce_set.add(nonce)
            self._nonce_ring.append(nonce)
            if len(self._nonce_ring) > 128:
                self._nonce_set.discard(self._nonce_ring.pop(0))
        payload = cmd.get("ed")
        if not isinstance(payload, dict):
            payload = cmd.get("m")
        if not isinstance(payload, dict):
            payload = {}
        if verb == "restore":
            _res, err = self._perceive_restore(payload, "event")
        else:
            _res, err = self._perceive_configure(payload, "event")
        if err:
            print(f"perceive.{verb} event rejected: {err}", flush=True)
            self.log.log("REJECT", {"tag": f"perceive.{verb}: {err}"},
                         ms=self.clock_ms())

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
        if self.legacy_box:
            # the pre-obs wire pair: the class list the 1078 box's
            # class_id indexed (SL-0034 dropped it - the 'perceive'
            # advert carries classes now)
            self.emit(TYPE_AndruavMessage_AI_Recognition_STATUS,
                      {"a": AI_STATUS_CLASS_LIST, "c": list(self.labels)})
        elif self.facade is not None:
            # MP-0214: live perceive state mirrors the advert
            self._publish_perceive_state()

    def _perceive_state(self):
        return {"cam_id": self.cam_id, "backend": "sim",
                "classes": list(self.obs_labels),
                "detecting": self._reporting,
                "active_classes": sorted(self._perceive_filter or []),
                "min_conf": self._min_conf,
                "fps": float(self.cam["fps_boxes"]),
                "hfov": float(self.cam["hfov_deg"])}

    def adverts(self):
        """The serialized de.cap/1 adverts. The tracker one is dynamic:
        start.class is the enum of labels this model reports (B4-02);
        the perceive one (SL-0034) carries the wire cls table - what
        CAP-008 reads off a live registry."""
        tracker = __import__("copy").deepcopy(ADVERT_TRACKER)
        tracker["actions"]["start"] = {
            "desc": "Start detection reporting for a class",
            "params": {"class": {"type": "enum",
                                 "values": list(self.labels)}},
            "timeout_s": 10,
        }
        perceive = __import__("copy").deepcopy(ADVERT_PERCEIVE)
        perceive["ns"] = self._perceive_ns
        for k, v in self._perceive_state().items():
            perceive["state"][k]["default"] = v
        return [__import__("json").dumps(ADVERT_CAMERA),
                __import__("json").dumps(tracker),
                __import__("json").dumps(perceive)]

    def on_message(self, message, length, jMsg):
        super().on_message(message, length, jMsg)
        try:
            mt = jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_TYPE)
            ms = jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_CMD) or {}
            if mt == TYPE_AndruavMessage_AI_Recognition_ACTION:
                if ms.get("a") == AI_ACTION_SEARCH and \
                        isinstance(ms.get("i"), list):
                    self._sel = [int(i) for i in ms["i"]]
                    self.log.log("SELECT", {"i": self._sel},
                                 ms=self.clock_ms())
            elif mt == TYPE_AndruavMessage_Sync_EventFire:
                # MP-0215: the operator panel's perceive.configure /
                # perceive.restore fire lands here (forwarded to this
                # namespace's owner)
                self._on_perceive_event(ms)
        except Exception:
            pass

    def reset_run(self):
        self._sel = None
        self._last_frame = -1
        self._clutter = []
        self._trk_ids = {}
        self._frame_obs = []
        self._status_next = 0.0
        # MP-0215: a new run starts on the unit default, not whatever
        # override the previous run left installed
        self._reporting = self._perceive_defaults["reporting"]
        self._perceive_filter = (
            set(self._perceive_defaults["classes"])
            if self._perceive_defaults["classes"] is not None else None)
        self._min_conf = self._perceive_defaults["min_conf"]
        if self._perceive_filter is not None and \
                not self._perceive_filter:
            self._reporting = False
        self._nonce_ring = []
        self._nonce_set = set()

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
        if not self._reporting:
            # MP-0215: paused - perceive.configure classes:[] or the
            # unit default (stub_ai_box.enabled: false). A paused
            # detector emits no obs frame at all.
            return
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
        self._frame_obs = []

        for e in self.world.entities:
            if e.cls == "pad":
                continue              # PL-08: not a detection class
            if not self._class_ok(e.cls):
                continue
            if self.cam.get("detect_cls") and \
                    e.cls not in self.cam["detect_cls"]:
                continue
            lat, lng, ealt = e.pos_at(t_s, self.world)
            self._maybe_detect(e.id, e.cls, lat, lng, ealt,
                               pose, att, gp_pose, frame, t_s)

        self._clutter_tick(alt, frame, t_s, att, gp_pose)

        if not self.legacy_box:
            # SL-0034: one observation frame per camera frame, every
            # detection an entry - the 6546 the de_mavlink projector
            # consumes (an empty frame is still a frame)
            self.emit(TYPE_AndruavMessage_AI_OBSERVATIONS,
                      {"cam_id": self.cam_id,
                       "tm_capture": self._tm_us(),
                       "obs": self._frame_obs})

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
        # P5-11 D5: fog/haze caps the range; night kills rgb outright
        if rng_m > self.env_f["max_range_m"]:
            return
        ent = self.world.entity(tid)
        if self.cam_kind == "thermal":
            # flat p_detect_thermal, no nadir/edge ramp (D5), scaled
            # by the TASK-13 contrast ESTIMATE: |temp_c - ambient| ->
            # a body at ambient reads 0 (the dawn/dusk crossover)
            p_cam = float(cam.get("p_detect_thermal", 0.85)) * \
                self.world.thermal_factor(ent, t_s)
        else:
            # p_detect interpolates nadir -> frame edge by the off-axis
            # fraction of the HALF-frame the point occupies
            edge = max(abs(u), abs(v)) * 2.0
            p_cam = cam["p_detect_nadir"] + \
                (cam["p_detect_edge"] - cam["p_detect_nadir"]) * \
                min(1.0, edge)
        # p = p_cam * f_env * T: f_env = f_light * f_precip (D5; 1 for
        # a world with no environment block), T the P5-10 obstacle
        # transmittance. The SAME single draw decides, so a world with
        # no obstacles and no environment keeps today's RNG stream and
        # log byte-identical
        size = self.world.size_m(ent) if ent else 0.5
        vis, blockers = self.world.transmittance(
            (gp_pose.lat_deg, gp_pose.lng_deg, gp_pose.rel_alt_m),
            (lat, lng, ealt + size / 2.0))
        p_env = p_cam * self.env_f["p_scale"]
        p = p_env * vis
        r = self.rng("detect", self.unit, tid, frame)
        draw = r.random()
        if draw >= p:
            # why "miss": the camera/environment alone failed the draw
            # (draw >= p_env on a ray that was at least partially open).
            # why "occluded": the obstacles are what killed it - and a
            # fully blocked target (vis 0) is ALWAYS occluded: no draw
            # could ever have hit it, so the draw says nothing about
            # the camera
            self.truth.append({"src": "ai", "tid": tid, "cls": cls,
                               "frame": frame, "u": round(u, 4),
                               "v": round(v, 4), "lat": lat, "lng": lng,
                               "alt_m": ealt, "vis": round(vis, 4),
                               "sent": False,
                               "why": "miss" if draw >= p_env
                               and vis > 0.0 else "occluded",
                               "blockers": blockers},
                              ms=self.clock_ms())
            return
        # pixel noise in normalized units - worse in the dark (D5)
        noise_px = cam["pos_noise_px"] * self.env_f["noise_scale"]
        nu = r.gauss(0.0, noise_px / cam["img_w"])
        nv = r.gauss(0.0, noise_px / cam["img_h"])
        u_n, v_n = u + nu, v + nv
        # box size: the entity's real size projected at its range
        bw = min(1.0, size / (2.0 * rng_m *
                              math.tan(math.radians(cam["hfov_deg"]) / 2)))
        bh = min(1.0, size / (2.0 * rng_m *
                              math.tan(math.radians(cam["vfov_deg"]) / 2)))
        conf = round(r.uniform(0.75, 0.99), 3)
        if conf < self._min_conf:
            # MP-0215: under the confidence gate - detected, not reported
            self.truth.append({"src": "ai", "tid": tid, "cls": cls,
                               "frame": frame, "u": round(u_n, 4),
                               "v": round(v_n, 4), "lat": lat,
                               "lng": lng, "alt_m": ealt,
                               "vis": round(vis, 4), "conf": conf,
                               "sent": False, "why": "conf"},
                              ms=self.clock_ms())
            return
        box = {"x": round(max(0.0, u_n + 0.5 - bw / 2), 4),
               "y": round(max(0.0, v_n + 0.5 - bh / 2), 4),
               "w": round(bw, 4), "h": round(bh, 4),
               "conf": conf, "tm": self._tm_us()}
        if self.legacy_box:
            self.emit(TYPE_AI_TARGET_LOCATION_LEGACY,
                      {"b": box})
        else:
            entry = self._obs_entry(tid, cls, box, t_s)
            if entry is None:
                # MP-0215: the perceive class filter dropped the label
                # at the emit seam - the world saw it, the wire didn't
                self.truth.append({"src": "ai", "tid": tid, "cls": cls,
                                   "frame": frame, "u": round(u_n, 4),
                                   "v": round(v_n, 4), "lat": lat,
                                   "lng": lng, "alt_m": ealt,
                                   "vis": round(vis, 4), "conf": conf,
                                   "sent": False, "why": "filtered"},
                                  ms=self.clock_ms())
                return
            self._frame_obs.append(entry)
        self.truth.append({"src": "ai", "tid": tid, "cls": cls,
                           "frame": frame, "u": round(u_n, 4),
                           "v": round(v_n, 4), "lat": lat, "lng": lng,
                           "alt_m": ealt, "vis": round(vis, 4),
                           "conf": conf, "sent": True},
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

    def _trk_for(self, tid):
        """A stable int trk id per entity for the run - first-detection
        order, so deterministic per seed."""
        trk = self._trk_ids.get(tid)
        if trk is None:
            trk = len(self._trk_ids) + 1
            self._trk_ids[tid] = trk
        return trk

    def _obs_entry(self, tid, cls, box, t_s):
        """One 6546 obs entry: trk (entities only - clutter is a
        transient false positive), the wire cls (the detector label on
        rgb, hotspot/coldspot on thermal - de_ir_camera's vocabulary),
        cls_id indexing obs_labels, conf, the top-left [0..1] box, and
        attrs.temp_c on a thermal camera."""
        ent = self.world.entity(tid) if tid else None
        temp_c = None
        if self.cam_kind == "thermal":
            temp_c = (self.world.temp_c_at(ent, t_s) if ent is not None
                      else self.world.ambient_c() + 25.0)
            wire = ("hotspot" if temp_c >= self.world.ambient_c()
                    else "coldspot")
        else:
            wire = self.label_map.get(cls, cls)
        # MP-0215: perceive.configure's label filter drops the entry at
        # the reporting seam (the caller records it sent:false)
        if self._perceive_filter is not None and \
                wire not in self._perceive_filter:
            return None
        entry = {}
        if tid is not None:
            entry["trk"] = self._trk_for(tid)
        entry["cls"] = wire
        if wire in self.obs_labels:
            entry["cls_id"] = self.obs_labels.index(wire)
        entry["conf"] = box["conf"]
        entry["box"] = [box["x"], box["y"], box["w"], box["h"]]
        if temp_c is not None:
            entry["attrs"] = {"temp_c": round(temp_c, 1)}
        return entry

    def _clutter_tick(self, alt, frame, t_s, att, gp_pose):
        """False positives: spawns at rate_per_km2_min over the frame's
        ground footprint, each lives lifetime_s and reports like a
        detection with the clutter class. tid "clutter" lets
        detection_precision tell them apart. `alt` is the camera height
        tick() resolved (rel alt, or mount_height_m on a static mast).
        P5-10 D8 anchor: "image" keeps today's fixed pixel (u, v) - the
        false positive slides over the ground as the drone moves;
        "ground" fixes the false positive at a seeded ground point in
        the spawn frame's footprint, projects it every frame like an
        entity, and lets the canopy between it and the camera gate it
        - a rock that looks like a person stays where it is. P5-11:
        the ground-anchored FP also takes the environment p_scale on
        its existing draw and the visibility range cap; an
        image-anchored FP is a detector artefact, not a physical
        object - it keeps its own weather-free behaviour (and its
        exact RNG stream)."""
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
            u = r.uniform(-0.45, 0.45)
            v = r.uniform(-0.45, 0.45)
            if cl.get("anchor") == "ground":
                res = gp.geo_project_to_ground(u, v, 0.1, 0.1,
                                               self.camera, self.mount,
                                               att, gp_pose, 0.0, 0.0)
                if not res.ok:
                    continue          # not on the ground plane this frame
                self._clutter.append((t_s + life, res.lat, res.lng))
            else:
                self._clutter.append((t_s + life, u, v))
        for c in self._clutter:
            if not self._class_ok(cl["cls"]):
                break
            row = {"src": "ai", "tid": "clutter", "cls": cl["cls"],
                   "frame": frame, "sent": True}
            if cl.get("anchor") == "ground":
                _die, lat, lng = c
                uv = gp.geo_ground_to_pixel(lat, lng, 0.0, self.camera,
                                            self.mount, att, gp_pose)
                if uv is None:
                    continue          # behind the camera this frame
                u, v, rng_m = uv
                if abs(u) > 0.5 or abs(v) > 0.5:
                    continue          # outside the frame
                if rng_m > self.env_f["max_range_m"]:
                    continue          # past the fog cap
                # like an entity: the crown between the rock and the
                # camera gates this false positive too - and the same
                # draw takes the weather factor
                vis, _blockers = self.world.transmittance(
                    (gp_pose.lat_deg, gp_pose.lng_deg, gp_pose.rel_alt_m),
                    (lat, lng, 0.0))
                if r.random() >= cam["p_detect_edge"] * \
                        self.env_f["p_scale"] * vis:
                    continue
                row.update({"lat": lat, "lng": lng, "alt_m": 0.0,
                            "vis": round(vis, 4)})
            else:
                _die, u, v = c
                rng_m = alt / math.cos(
                    math.atan(math.hypot(u * 2 * fn, v * 2 * fv)))
            bw = min(1.0, 0.5 / (2.0 * rng_m * fn))
            bh = min(1.0, 0.5 / (2.0 * rng_m * fv))
            conf = round(r.uniform(0.4, 0.7), 3)
            if conf < self._min_conf:
                continue          # MP-0215: under the confidence gate
            box = {"x": round(max(0.0, u + 0.5 - bw / 2), 4),
                   "y": round(max(0.0, v + 0.5 - bh / 2), 4),
                   "w": round(bw, 4), "h": round(bh, 4),
                   "conf": conf, "tm": self._tm_us()}
            entry = None
            if self.legacy_box:
                self.emit(
                    TYPE_AI_TARGET_LOCATION_LEGACY,
                    {"b": box})
            else:
                entry = self._obs_entry(None, cl["cls"], box, t_s)
            row.update({"u": round(u, 4), "v": round(v, 4),
                        "conf": conf})
            if entry is None and not self.legacy_box:
                # MP-0215: perceive filter dropped the clutter label
                row["sent"] = False
                row["why"] = "filtered"
            self.truth.append(row, ms=self.clock_ms())
            if entry is not None:
                self._frame_obs.append(entry)


def main(argv):
    s = StubAIBox(argv)
    if s.selftest:
        s._vt_ms = 0
        s.log.log("SELFTEST", {"poses": s.selftest}, ms=0)
        s.run_selftest()
        return 0
    s.connect(adverts=s.adverts(),
              msg_filter=[TYPE_AndruavMessage_AI_Recognition_ACTION,
                          TYPE_AndruavMessage_Sync_EventFire])
    s.status()
    s.run()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
