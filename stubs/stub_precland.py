#!/usr/bin/env python3
"""stub_precland.py - a world-driven de_precland (TASK-PL-08).

The real module cannot see the SITL world - there is no renderer for an
AprilTag pad - so this stub plays it: it reads the unit's pose (SITL
serial5 on FC units, the 9102 feed on statics), decides whether the
world's `cls: pad` entity is in the downward camera's frame, and
publishes the same wire surface:

    PRECLAND_TARGET (6537)  body-FRD x/y/z + angles + tag count + RMSE,
                           capture stamp t = monotonic us, valid, tn
    PRECLAND_STATUS (6538)  state/fps/rmse/range/tags/reason, ~2 Hz +
                           on every state change
    capability ns           "precision_landing" - the advert is the
                           module's own --print-advert output
                           (stubs/precland_advert.json)
    events                  precision_landing.* facts/commands through
                           stubs/precland_events.py, a Python port of
                           CPreclandEventModel pinned by the module's
                           golden vectors

The per-frame model mirrors CPreclandMain::processingLoop: enabled is
flipped by capability invoke / legacy PRECLAND_ACTION (6536) /
precision_landing.enable|disable|set_target command events (1061,
nonce-deduped like PL-03). Per 10 Hz frame the pad centre is projected
into the tag camera (sensors.precland); the frame is a detection when
the centre is inside the FOV, the largest tag spans >= min_px_tag, the
pad is not in an occlude window and a seeded draw clears p_drop. The
tag count is how many edges of the pad's geometric tags_min_m..tags_max_m
ladder still resolve >= min_px_tag fully in frame; rmse is the
sensor's max_rmse_px scaled by how far the largest tag sits above the
resolution limit. The module-side gate (min_tags / max_reproj_rmse_px /
stale_timeout_ms) then classifies LOCKED / DEGRADED / SEARCHING exactly
like the real processing loop.

--selftest <pose.jsonl> replays poses on the virtual clock; a row may
also carry "cmd": "enable"|"disable"|"set_target" (+ "target") to
drive the command paths, and "src" picks the cause (legacy|invoke|
event, default event). Same-seed selftests are byte-identical.
"""

import json
import math
import os
import sys
import time

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
    TYPE_AndruavMessage_PRECLAND_ACTION,
    TYPE_AndruavMessage_PRECLAND_TARGET,
    TYPE_AndruavMessage_PRECLAND_STATUS,
    TYPE_AndruavMessage_Sync_EventFire)
import geo_projection as gp  # noqa: E402
import precland_events as pe  # noqa: E402

ADVERT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "precland_advert.json")
NS = "precision_landing"

# the world's tag camera is the downward nadir mount - same -90 deg
# pitch convention sensors.camera uses for the rgb camera
PRECLAND_MOUNT = gp.GeoMountParams(pitch_deg=-90.0, yaw_deg=0.0,
                                   roll_deg=0.0)
MODEL_HZ = 10.0          # the stub's processing rate (real send_hz)
STATUS_HZ = 2.0          # 6538 cadence, like the module's status_period


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


class StubPrecland(StubBase):
    MODULE_CLASS = "gen"
    MODULE_VERSION = "0.1.0"

    def __init__(self, argv):
        super().__init__(argv, "stub_precland")
        cfg = self.cfg
        det = cfg.get("detection") or {}
        stab = cfg.get("stability") or {}
        target = cfg.get("target") or {}
        # the same config keys the real module reads
        self.enabled = bool(det.get("enabled", False))
        self.send_hz = int(det.get("send_hz", 10))
        self.min_tags = int(det.get("min_tags", 1))
        self.max_reproj_rmse_px = float(
            stab.get("max_reproj_rmse_px", 8.0))
        self.ema_alpha = float(stab.get("ema_alpha", 0.4))
        self.stale_timeout_us = int(
            stab.get("stale_timeout_ms", 1000)) * 1000
        self.target_num = int(target.get("target_num", 0))
        self.allowed_targets = (target.get("targets")
                                if isinstance(target.get("targets"),
                                              list) else None)
        self.events_config = pe.events_config_from_json(cfg)
        self.event_model = pe.PreclandEventModel(self.events_config)
        self._inbound_nonces = pe.NonceRing(64)
        self._reject_logged = set()
        self.sensor = (self.world.sensors["precland"] if self.world
                       else None)
        self.camera = (gp.GeoCameraParams(
            self.sensor["hfov_deg"], self.sensor["vfov_deg"])
                       if self.sensor else None)
        self._reset_run_state()

    def _reset_run_state(self):
        """The per-run processing state (mirrors the member set of
        CPreclandMain a fresh process would have)."""
        self._state = pe.PRECLAND_STATUS_DISABLED
        self._gate_reason = pe.PRECLAND_REASON_NOMINAL
        self._last_seen_us = 0
        self._ema = None
        self._last_rmse = 0.0
        self._last_range = 0.0
        self._last_tags = []           # used tag-size list for status e
        self._last_frame10 = -1
        self._status_next_s = 0.0
        self._truth_next_s = 0.0
        self._last_vis_row = None
        # capability-state mirror (publishStatus' second half):
        # status/reason publish on change, range_m throttled to 1 Hz
        self._cap_status = None
        self._cap_reason = None
        self._cap_range = None
        self._cap_range_s = -1.0

    def reset_run(self):
        self._reset_run_state()
        self.event_model = pe.PreclandEventModel(self.events_config)
        self._inbound_nonces = pe.NonceRing(64)
        self._reject_logged = set()

    # -------------------------------------------------- time helpers

    def _now_us(self):
        """CLOCK_MONOTONIC us live; the virtual clock under selftest -
        the same basis de_mavlink ages PRECLAND_TARGET.t on."""
        if self._vt_ms is not None:
            return self._vt_ms * 1000
        return time.monotonic_ns() // 1000

    def _capture_us(self):
        """Frame-grab stamp; a touch in the past like stub_ai_box's tm
        so de_mavlink's ring already holds the pose."""
        us = self._now_us()
        return us - 50000 if self._vt_ms is None else us

    # ----------------------------------------------------- databus out

    def emit(self, mid, ms, internal=True):
        self.log.log("SEND", {"mt": mid, "ms": ms}, ms=self.clock_ms())
        if self.module is not None:
            self.module.sendJMSG("", ms, mid, internal)

    def _emit_fact(self, verb, payload):
        """One precision_landing.<verb> fact: the same EVENT_OUT stdout
        line the real module prints plus the deterministic stub-log row
        and the Sync_EventFire on the bus when connected."""
        line = f"EVENT_OUT precision_landing.{verb} " \
            f"{json.dumps(payload, separators=(',', ':'))}"
        print(line, flush=True)
        self.log.log("EVENT_OUT", {"d": f"precision_landing.{verb}",
                                   "ed": payload}, ms=self.clock_ms())
        if self.facade is not None:
            self.facade.fire_event(NS, verb, payload)

    def _emit_all(self, facts):
        for verb, payload in facts:
            self._emit_fact(verb, payload)

    def _cap_state(self, changed):
        if self.facade is not None:
            self.facade.publish_state(NS, changed)

    # --------------------------------------------------- state choke

    def _set_state(self, state, reason=pe.PRECLAND_REASON_NOMINAL,
                   cause=pe.CAUSE_LOOP, fault=""):
        """setState(): update the state, run the event model on the
        edge, print the module's state line, publish 6538 + cap state."""
        if self._state == state:
            if reason != pe.PRECLAND_REASON_NOMINAL:
                self._gate_reason = reason
            return
        prev = self._state
        self._state = state
        self._gate_reason = reason
        line = f"PRECLAND state -> {state}"
        if reason != pe.PRECLAND_REASON_NOMINAL:
            line += f" ({pe.precland_reason_name(reason)})"
        print(line, flush=True)
        self._emit_all(self.event_model.on_transition(
            prev, state, reason, self._now_us(),
            range_m=self._last_range, rmse_px=self._last_rmse,
            tags=len(self._last_tags), target=self.target_num,
            fault=fault))
        self.publish_status()

    def publish_status(self):
        """sendPreclandStatus(): 6538 {a,b,c,d,e,f} plus the capability
        state mirror (6545) - status/reason on change, range_m throttled
        to 1 Hz like the module's publishStatus()."""
        ms = {"a": self._state, "b": float(self.send_hz),
              "c": self._last_rmse, "d": self._last_range,
              "e": list(self._last_tags), "f": self._gate_reason}
        self.emit(TYPE_AndruavMessage_PRECLAND_STATUS, ms,
                  internal=False)
        status_name = pe.precland_status_name(self._state)
        reason_name = pe.precland_reason_name(self._gate_reason)
        changed = {}
        if self._cap_status != status_name:
            changed["status"] = status_name
            self._cap_status = status_name
        if self._cap_reason != reason_name:
            changed["reason"] = reason_name
            self._cap_reason = reason_name
        t_s = self.clock_ms() / 1000.0
        if self._last_range != self._cap_range and \
                t_s - self._cap_range_s >= 1.0:
            changed["range_m"] = self._last_range
            self._cap_range = self._last_range
            self._cap_range_s = t_s
        if changed:
            self._cap_state(changed)

    # ------------------------------------------------- command paths

    def enable(self, cause):
        was = self.enabled
        self.enabled = True
        self._ema = None
        self._last_seen_us = 0
        # enabled edge goes out BEFORE the state facts its transition
        # causes (PL-02 ordering)
        if was != self.enabled:
            self._emit_all(self.event_model.on_enabled_changed(
                was, True, cause, self._now_us()))
        self._set_state(pe.PRECLAND_STATUS_SEARCHING,
                        pe.PRECLAND_REASON_NOMINAL, cause)

    def disable(self, cause):
        was = self.enabled
        self.enabled = False
        self._ema = None
        if was != self.enabled:
            self._emit_all(self.event_model.on_enabled_changed(
                was, False, cause, self._now_us()))
        self._set_state(pe.PRECLAND_STATUS_DISABLED,
                        pe.PRECLAND_REASON_NOMINAL, cause)

    def set_target_num(self, target_num, cause):
        prev = self.target_num
        self.target_num = int(target_num)
        if prev != self.target_num:
            self._emit_all(self.event_model.on_target_changed(
                prev, self.target_num, cause))
        self.publish_status()

    def on_invoke(self, ns, act, params):
        """CAPABILITY_INVOKE handler - params arrive validated against
        the advert (defaults filled), like the module's worker."""
        if ns != NS:
            return None, f"unknown action {ns}.{act}"
        if act == "enable":
            self.enable(pe.CAUSE_INVOKE)
            return {"status": pe.precland_status_name(self._state)}, None
        if act == "disable":
            self.disable(pe.CAUSE_INVOKE)
            return {"status": pe.precland_status_name(self._state)}, None
        if act == "set_target":
            self.set_target_num((params or {}).get("target",
                                                   self.target_num),
                                pe.CAUSE_INVOKE)
            return {"target": self.target_num}, None
        return None, f"unknown action {ns}.{act}"

    def on_message(self, message, length, jMsg):
        super().on_message(message, length, jMsg)
        try:
            mt = jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_TYPE)
            ms = jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_CMD) or {}
            if mt == TYPE_AndruavMessage_PRECLAND_ACTION:
                self._on_precland_action(ms)
            elif mt == TYPE_AndruavMessage_Sync_EventFire:
                self._on_event_command(ms)
        except Exception:
            pass

    def _on_precland_action(self, ms):
        """Legacy PRECLAND_ACTION (6536): a = action code, b = target
        for SET_TARGET. Calibration actions have no world meaning - the
        stub has no camera to calibrate."""
        a = ms.get("a")
        if not _is_int(a):
            return
        if a == pe.PRECLAND_ACTION_DISABLE:
            self.disable(pe.CAUSE_LEGACY)
        elif a == pe.PRECLAND_ACTION_ENABLE:
            self.enable(pe.CAUSE_LEGACY)
        elif a == pe.PRECLAND_ACTION_SET_TARGET:
            if _is_int(ms.get("b")):
                self.set_target_num(ms["b"], pe.CAUSE_LEGACY)
        elif a == pe.PRECLAND_ACTION_SELFTEST:
            self.log.log("SELFTEST", {"ok": True}, ms=self.clock_ms())
            self.publish_status()
        else:
            self.log.log("ACTION", {"unsupported": a},
                         ms=self.clock_ms())

    def _on_event_command(self, cmd):
        """Consume a Sync_EventFire {d:"precision_landing.<verb>", ed?,
        m?, n?} (PL-03). Name parse -> nonce dedupe -> events.commands
        gate -> execute through the same choke points, cause EVENT.
        Rejections log once per (verb, reason)."""
        if not isinstance(cmd.get("d"), str):
            return
        verb = pe.pl_parse_command(cmd["d"])
        if verb is None:
            return
        if isinstance(cmd.get("n"), str):
            if self._inbound_nonces.seen(cmd["n"]):
                return
            self._inbound_nonces.add(cmd["n"])
        if not (self.events_config.enabled and
                self.events_config.commands):
            return
        payload = pe.pl_event_payload(cmd)
        if verb == "enable":
            self.enable(pe.CAUSE_EVENT)
        elif verb == "disable":
            self.disable(pe.CAUSE_EVENT)
        else:  # set_target
            target_num, why = pe.pl_validate_set_target(
                payload, self.allowed_targets)
            if target_num < 0:
                self._reject_once(verb, why)
                return
            self.set_target_num(target_num, pe.CAUSE_EVENT)

    def _reject_once(self, verb, reason):
        tag = f"reject {verb}: {reason}"
        if tag in self._reject_logged:
            return
        self._reject_logged.add(tag)
        line = f"precland event {tag}"
        print(line, flush=True)
        self.log.log("REJECT", {"tag": tag}, ms=self.clock_ms())

    # --------------------------------------------------------- model

    def _pad(self):
        """The active pad: world.pads()[target_num], like the module's
        layout index."""
        if self.world is None:
            return None
        pads = self.world.pads()
        if 0 <= self.target_num < len(pads):
            return pads[self.target_num]
        return None

    def _frame_geometry(self, pad, pose, t_s):
        """Project the pad into the tag camera this frame ->
        (u, v, slant_m, tag_px[], tags_in_frame). tag_px[k] is the
        projected edge in pixels of the k-th ladder size (largest
        first); a size counts when its edge resolves >= min_px_tag AND
        the tag square centred on the pad lies fully inside the
        frame."""
        lat, lng, alt = pad.pos_at(t_s, self.world)
        att = gp.GeoAttitude(roll_rad=pose["roll"],
                             pitch_rad=pose["pitch"],
                             yaw_rad=pose["yaw"])
        gp_pose = gp.GeoPose(lat_deg=pose["lat"], lng_deg=pose["lng"],
                             rel_alt_m=pose["rel_alt_m"])
        uv = gp.geo_ground_to_pixel(lat, lng, alt, self.camera,
                                    PRECLAND_MOUNT, att, gp_pose)
        if uv is None:
            return None
        u, v, slant = uv
        # frame extent in metres at this slant range (rectilinear)
        fw = 2.0 * slant * math.tan(
            math.radians(self.sensor["hfov_deg"]) / 2.0)
        fh = 2.0 * slant * math.tan(
            math.radians(self.sensor["vfov_deg"]) / 2.0)
        tag_px = []
        tags_in = 0
        for size in self.world.pad_tag_sizes(pad):
            px = min(size * self.sensor["img_w"] / fw,
                     size * self.sensor["img_h"] / fh)
            tag_px.append(px)
            if px >= self.sensor["min_px_tag"] and \
                    abs(u) + size / (2.0 * fw) <= 0.5 and \
                    abs(v) + size / (2.0 * fh) <= 0.5:
                tags_in += 1
        return (u, v, slant, tag_px, tags_in, (lat, lng, alt))

    def tick(self, t_s, _frame20):
        if self.world is None or self.sensor is None:
            return
        frame10 = int(t_s * MODEL_HZ)
        if frame10 == self._last_frame10:
            return
        self._last_frame10 = frame10
        pad = self._pad()

        # pad ground-truth row at 1 Hz (the distance_to evidence)
        if pad is not None and t_s >= self._truth_next_s:
            self._truth_next_s = t_s + 1.0
            lat, lng, alt = pad.pos_at(t_s, self.world)
            self.truth.append({"type": "pad", "tid": pad.id,
                               "lat": lat, "lng": lng, "alt": alt},
                              ms=self.clock_ms())

        if not self.enabled:
            # the real loop forwards the frame, publishes nothing but
            # the ~2 Hz status - no model ticks while disabled
            if t_s >= self._status_next_s:
                self._status_next_s = t_s + 1.0 / STATUS_HZ
                self.publish_status()
            return

        now_us = self._now_us()
        pose = self.pose_feed.pose()
        detected = False
        r = None
        u = v = slant = rmse = 0.0
        tags_in = 0
        pad_lla = None
        if pad is not None and pose is not None:
            vis = self._frame_geometry(pad, pose, t_s)
            if vis is not None:
                u, v, slant, tag_px, tags_in, pad_lla = vis
                occluded = any(a <= t_s <= b for a, b in pad.occlude)
                r = self.rng("precland", self.unit, pad.id, frame10)
                dropped = r.random() < float(self.sensor["p_drop"])
                centre_in = abs(u) <= 0.5 and abs(v) <= 0.5
                largest_ok = bool(tag_px) and \
                    tag_px[0] >= self.sensor["min_px_tag"]
                if centre_in and largest_ok and not occluded \
                        and not dropped:
                    detected = True
                    # RMSE at the resolution limit: the further the
                    # largest tag sits above min_px_tag the better
                    rmse = min(float(self.sensor["max_rmse_px"]),
                               float(self.sensor["max_rmse_px"])
                               * self.sensor["min_px_tag"] / tag_px[0])
        self._truth_change(detected, tags_in, slant)

        # the C++ loop's bookkeeping verbatim: a fresh detection stamps
        # last_seen; past stale_timeout a locked/degraded hold drops to
        # searching and the EMA resets
        last_seen_age_us = now_us - self._last_seen_us
        if detected:
            self._last_seen_us = now_us
        elif self._last_seen_us != 0 and \
                last_seen_age_us > self.stale_timeout_us:
            self._ema = None
            if self._state in (pe.PRECLAND_STATUS_LOCKED,
                               pe.PRECLAND_STATUS_DEGRADED):
                self._set_state(pe.PRECLAND_STATUS_SEARCHING,
                                pe.PRECLAND_REASON_STALE)

        # preclandGateCheck(): the reason is stored even on same-state
        # ticks (the GCS widget shows why a search finds nothing)
        if not detected:
            self._gate_reason = (pe.PRECLAND_REASON_STALE
                                 if last_seen_age_us >
                                 self.stale_timeout_us
                                 else pe.PRECLAND_REASON_NO_TARGET)
        accepted = detected and self._gate_accepted(tags_in, rmse)
        if accepted:
            self._accepted_frame(pad_lla, tags_in, rmse, r)
            self._set_state(pe.PRECLAND_STATUS_LOCKED)
        elif detected:
            self._set_state(pe.PRECLAND_STATUS_DEGRADED,
                            self._gate_reason)
        elif self._state in (pe.PRECLAND_STATUS_DISABLED,
                             pe.PRECLAND_STATUS_ERROR):
            pass
        elif self._state != pe.PRECLAND_STATUS_SEARCHING:
            self._set_state(pe.PRECLAND_STATUS_SEARCHING,
                            self._gate_reason)

        # once per processed frame: the stabilized timer
        self._emit_all(self.event_model.on_tick(
            self._state, now_us, self._last_range, self._last_rmse,
            self.target_num))

        if t_s >= self._status_next_s:
            self._status_next_s = t_s + 1.0 / STATUS_HZ
            self.publish_status()

    def _gate_accepted(self, n_tags, rmse):
        """preclandGateCheck() on a detected frame: min_tags -> RMSE ->
        accepted. Updates _gate_reason; True when the pose publishes."""
        if n_tags < self.min_tags:
            self._gate_reason = pe.PRECLAND_REASON_MIN_TAGS
            return False
        if rmse > self.max_reproj_rmse_px:
            self._gate_reason = pe.PRECLAND_REASON_RMSE
            return False
        self._gate_reason = pe.PRECLAND_REASON_NOMINAL
        return True

    def _accepted_frame(self, pad_lla, tags_in, rmse, r):
        """The gate-accepted branch of the C++ loop: EMA on the body
        pose, the status snapshot (rmse/range/used ids) and the 6537
        publish paced by send_hz inside the 10 Hz model rate."""
        lat, lng, alt = pad_lla
        # body-FRD of the pad: the NED offset through the transpose
        # attitude rotation (geo_rot_t is the NED->body map)
        pose = self.pose_feed.pose()
        north_m, east_m = gp.geo_latlng_to_ne(pose["lat"], pose["lng"],
                                              lat, lng)
        down_m = pose["rel_alt_m"] - alt
        bx, by, bz = gp.geo_rot_t(pose["roll"], pose["pitch"],
                                  pose["yaw"], (north_m, east_m, down_m))
        noise = float(self.sensor["noise_m"])
        bx += r.gauss(0.0, noise)
        by += r.gauss(0.0, noise)
        bz += r.gauss(0.0, noise)
        if self._ema is None:
            self._ema = [bx, by, bz]
        else:
            a = self.ema_alpha
            self._ema = [a * n + (1.0 - a) * o
                         for n, o in zip((bx, by, bz), self._ema)]
        x, y, z = self._ema
        self._last_rmse = rmse
        self._last_range = math.sqrt(x * x + y * y + z * z)
        self._last_tags = list(range(tags_in))
        # ax/ay use the module's own convention (bodyAngles: atan2 of the
        # forward / right component over down) - de_mavlink discards them
        # and derives ArduPilot's own angle pair from x/y/z
        ms = {"x": x, "y": y, "z": z,
              "ax": math.atan2(x, z) if z > 1e-9 else 0.0,
              "ay": math.atan2(y, z) if z > 1e-9 else 0.0,
              "n": tags_in, "e": rmse,
              "t": self._capture_us(), "v": True,
              "tn": self.target_num}
        step = max(1, int(round(MODEL_HZ / max(1, self.send_hz))))
        if self._last_frame10 % step == 0:
            self.emit(TYPE_AndruavMessage_PRECLAND_TARGET, ms)

    def _truth_change(self, visible, tags, range_m):
        """{type:"precland"} row per visibility/tag/range change."""
        row = (bool(visible), int(tags), round(range_m, 2))
        if row == self._last_vis_row:
            return
        self._last_vis_row = row
        self.truth.append({"type": "precland", "visible": visible,
                           "tags": int(tags),
                           "range_m": round(range_m, 3)},
                          ms=self.clock_ms())


def _load_advert():
    """The stored advert verbatim - the module's --print-advert string,
    so the registered bytes (and the ch hash) equal the real module's."""
    try:
        with open(ADVERT_PATH) as f:
            text = f.read().strip()
        json.loads(text)
        return text
    except (OSError, ValueError) as e:
        sys.exit(f"stub_precland: cannot load {ADVERT_PATH}: {e}")


def main(argv):
    s = StubPrecland(argv)
    if s.selftest:
        # replayed poses plus optional command rows:
        # {"t_ms","cmd":"enable"|"disable"|"set_target","target":n,
        #  "src":"legacy"|"invoke"|"event"}
        s._vt_ms = 0
        rows = []
        for ln in open(s.selftest):
            try:
                rows.append(json.loads(ln))
            except ValueError:
                continue
        rows.sort(key=lambda r: r.get("t_ms", 0))
        t_end = max(rows[-1].get("t_ms", 0) / 1000.0 if rows else 0.0,
                    s.selftest_dur)
        cause = {"legacy": pe.CAUSE_LEGACY, "invoke": pe.CAUSE_INVOKE,
                 "event": pe.CAUSE_EVENT, "config": pe.CAUSE_CONFIG}
        frame = 0
        t_s = 0.0
        pose_i = 0
        while t_s <= t_end and not s.exit_me:
            s._vt_ms = int(round(t_s * 1000))
            while pose_i < len(rows) and \
                    rows[pose_i].get("t_ms", 0) <= t_s * 1000:
                row = rows[pose_i]
                if "lat" in row:
                    s._selftest_pose(row)
                cmd = row.get("cmd")
                src = cause.get(row.get("src", "event"),
                                pe.CAUSE_EVENT)
                if cmd == "enable":
                    s.enable(src)
                elif cmd == "disable":
                    s.disable(src)
                elif cmd == "set_target":
                    s.set_target_num(row.get("target", 0), src)
                pose_i += 1
            s.tick(t_s, frame)
            frame += 1
            t_s = frame * 0.05        # the same 20 Hz quantum
        s.log.log("END", {"frames": frame}, ms=s.clock_ms())
        return 0
    s.connect(adverts=[_load_advert()],
              msg_filter=[TYPE_AndruavMessage_PRECLAND_ACTION,
                          TYPE_AndruavMessage_Sync_EventFire])
    s._set_state(pe.PRECLAND_STATUS_SEARCHING if s.enabled
                 else pe.PRECLAND_STATUS_DISABLED)
    s.run()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
