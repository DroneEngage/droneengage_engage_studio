#!/usr/bin/env python3
"""stub_payload.py - a world-driven payload release provider (TASK-P5-08,
multi-bay TASK-EV-PAY Part B).

Same de.cap/1 contract as the real providers (de_mavlink's FC gripper /
de_rpi_gpio's payload_release - src/payload/gripper_advert.hpp):

    ns "payload", action release {bay?} -> {"released": true, "t", "bay"}
    per-bay action payload.<bay> {}     -> {"released": true, "t", "bay"}
    event released {"t", "bay"}     event <bay>.released {"t", "src"}
    state armed_pin {"type": string}    (first bay's pin)

Bays come from the module config (same shape as the real providers):

    "payload": {"bays": [{"key": "left", "pin": "drop_left"},
                         {"key": "right"}]}
    "payload": {"pin": "drop_relay"}        # legacy single-bay form

Absent "bays" keeps the legacy single bay "main" on "pin" (default
"stub_pin"). A bay key is a capability name and never "release"; bad,
reserved or duplicate keys are skipped with a warning and nothing usable
falls back to {"key": "main"}. Unknown bays are invoke errors.

The stub drops at the unit's pose (9102/static fallback) and logs the
release like the gripper's COMMAND_ACK path - the event shape and the
invoke result are identical, so consumers (mission translator, drop
planner) see no difference. Where the real module blocks up to 3 s on
the FC ack, the stub releases at once; a second invoke on the same bay
while a release is in progress gets "release already in progress" like
the real one.

The payload namespace has NO command events: an inbound Sync_EventFire
naming payload.* is logged IGNORED and never releases (TASK-EV-PAY).

The released position/time lands a {"type":"detection",
"cls":"payload_drop", tid:"payload"} row in truth.jsonl so a scenario
can distance_to the drop point against world truth.
"""

import json
import os
import re
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stub_base import StubBase  # noqa: E402
from messages import (         # noqa: E402
    ANDRUAV_PROTOCOL_MESSAGE_CMD, ANDRUAV_PROTOCOL_MESSAGE_TYPE,
    TYPE_AndruavMessage_Sync_EventFire)

CAP_NAME_RE = re.compile(r"[a-z][a-z0-9_]{0,31}$")
RESERVED_KEYS = {"release"}


def bay_key_ok(key):
    return bool(CAP_NAME_RE.match(key)) and key not in RESERVED_KEYS


def parse_bays(payload_cfg):
    """config "payload" -> (bays, warnings). Each bay {"key", "pin"}.
    Absent/legacy config keeps the single bay {"main", pin}."""
    if not isinstance(payload_cfg, dict):
        return [{"key": "main", "pin": "stub_pin"}], []
    bays_cfg = payload_cfg.get("bays")
    if bays_cfg is None:
        return [{"key": "main",
                 "pin": str(payload_cfg.get("pin", "stub_pin"))}], []
    if not isinstance(bays_cfg, list):
        return [{"key": "main",
                 "pin": str(payload_cfg.get("pin", "stub_pin"))}], \
            ["payload.bays is not a list; using the single bay 'main'"]
    bays, warnings = [], []
    for i, entry in enumerate(bays_cfg):
        label = f"payload.bays[{i}]"
        if not isinstance(entry, dict) \
                or not isinstance(entry.get("key"), str):
            warnings.append(f"{label} needs a string 'key'; skipped")
            continue
        key = entry["key"]
        if not bay_key_ok(key):
            warnings.append(f"{label} key {key!r} is not a usable bay "
                            "name; skipped")
            continue
        if any(b["key"] == key for b in bays):
            warnings.append(f"{label} key {key!r} collides with an "
                            "earlier bay; skipped")
            continue
        pin = entry.get("pin", f"stub_{key}")
        bays.append({"key": key, "pin": str(pin)})
    if not bays:
        warnings.append("payload.bays produced no usable bay; using the "
                        "single bay 'main'")
        return [{"key": "main", "pin": "stub_pin"}], warnings
    return bays, warnings


def build_advert(bays):
    """ns "payload" advert, same shape as the real providers'."""
    keys = [b["key"] for b in bays]
    actions = {b["key"]: {"desc": f"release bay '{b['key']}' "
                                 f"(pin '{b['pin']}')",
                          "timeout_s": 5}
               for b in bays}
    actions["release"] = {
        "desc": "Release the payload (simulated servo pin)",
        "timeout_s": 5,
        "params": {"bay": {"type": "enum", "values": keys,
                           "optional": True, "default": keys[0],
                           "desc": f"bay to release; default "
                                   f"'{keys[0]}'"}},
    }
    return {
        "schema": "de.cap/1",
        "ns": "payload",
        "module": "stub_payload",
        "ver": "0.1.0",
        "desc": "facts: payload.released {t,bay?} and "
                "payload.<bay>.released {t,src} once per release; "
                "the payload namespace has no command events",
        "actions": actions,
        "events": {
            "released": {
                "desc": "payload released",
                "payload": {"t": {"type": "number",
                                  "desc": "unix time, seconds"},
                            "bay": {"type": "string", "optional": True,
                                    "desc": "released bay key"}},
            },
        },
        "state": {
            "armed_pin": {"type": "string",
                          "desc": "first bay's release pin name"},
        },
    }


class StubPayload(StubBase):
    MODULE_CLASS = "gen"
    MODULE_VERSION = "0.1.0"

    def __init__(self, argv):
        super().__init__(argv, "stub_payload")
        bays, warnings = parse_bays(self.cfg.get("payload"))
        self.bays = {b["key"]: dict(b) for b in bays}
        for w in warnings:
            self.log.log("CONFIG", {"payload_bays": w})
        self.armed_pin = bays[0]["pin"]
        self._busy = {b["key"]: threading.Lock() for b in bays}
        # the facts this provider emits - inbound echoes of them are not
        # command events
        self._fact_names = {"payload.released"} | \
            {f"payload.{b['key']}.released" for b in bays}

    def state(self, changed, full=False):
        self.log.log("STATE", changed, ms=self.clock_ms())
        if self.facade is not None:
            self.facade.publish_state("payload", changed, full)

    def event(self, ev, payload):
        self.log.log("EVENT", {"d": f"payload.{ev}", "ed": payload},
                     ms=self.clock_ms())
        if self.facade is not None:
            self.facade.fire_event("payload", ev, payload)

    def on_message(self, message, length, jMsg):
        super().on_message(message, length, jMsg)
        try:
            if jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_TYPE) != \
                    TYPE_AndruavMessage_Sync_EventFire:
                return
            cmd = jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_CMD) or {}
            d = cmd.get("d")
            # the payload namespace has no command events: anything
            # inbound that is not one of our own facts is ignored
            if isinstance(d, str) and d.startswith("payload.") \
                    and d not in self._fact_names:
                self.log.log("IGNORED", {"d": d,
                                         "why": "payload has no command "
                                                "events"},
                             ms=self.clock_ms())
        except Exception:
            pass

    def _release_bay(self, key):
        bay = self.bays[key]
        if not self._busy[key].acquire(blocking=False):
            return None, "release already in progress"
        try:
            pose = self.pose_feed.pose() or {}
            t = self.clock_ms() / 1000.0
            self.event("released", {"t": t, "bay": key})
            self.event(f"{key}.released", {"t": t, "src": "invoke"})
            self.log.log("RELEASE", {"t": t, "bay": key,
                                     "lat": pose.get("lat"),
                                     "lng": pose.get("lng")},
                         ms=self.clock_ms())
            if pose:
                self.truth.append({"src": "payload", "tid": "payload",
                                   "cls": "payload_drop", "bay": key,
                                   "lat": pose.get("lat"),
                                   "lng": pose.get("lng"),
                                   "alt_m": pose.get("rel_alt_m", 0.0),
                                   "sent": True}, ms=self.clock_ms())
            return {"released": True, "t": t, "bay": key}, None
        finally:
            self._busy[key].release()

    def on_invoke(self, ns, act, params):
        if act == "release":
            key = (params or {}).get("bay")
            if key is None:
                key = next(iter(self.bays))
            elif not isinstance(key, str):
                return None, "payload.release 'bay' must be a string"
            if key not in self.bays:
                return None, f"unknown bay '{key}'"
            return self._release_bay(key)
        if act in self.bays:
            return self._release_bay(act)
        return None, f"unknown action payload.{act}"

    def tick(self, t_s, frame):
        pass                       # nothing to model between invokes


def main(argv):
    s = StubPayload(argv)
    if s.selftest:
        # replayed invokes ride the pose file rows: {"t_ms","invoke":
        # "release"|"<bay>"} plus an optional "bay" for release
        s._vt_ms = 0
        for ln in open(s.selftest):
            try:
                row = json.loads(ln)
            except ValueError:
                continue
            s._vt_ms = int(row.get("t_ms", 0))
            if "lat" in row:
                s._selftest_pose(row)
            act = row.get("invoke")
            if act:
                params = {"bay": row["bay"]} if "bay" in row else {}
                data, err = s.on_invoke("payload", act, params)
                s.log.log("INVOKE", {"act": act, "params": params,
                                     "data": data, "err": err},
                          ms=s._vt_ms)
        s.log.log("END", {"frames": 0}, ms=s._vt_ms)
        return 0
    s.connect(adverts=[json.dumps(build_advert(
        [{"key": k, "pin": b["pin"]} for k, b in s.bays.items()]))],
        msg_filter=[TYPE_AndruavMessage_Sync_EventFire])
    s.state({"armed_pin": s.armed_pin}, full=True)
    s.run()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
