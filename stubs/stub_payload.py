#!/usr/bin/env python3
"""stub_payload.py - a world-driven payload release provider (TASK-P5-08).

Same de.cap/1 contract as the real providers (de_mavlink's FC gripper /
de_rpi_gpio's payload_release - src/payload/gripper_advert.hpp):

    ns "payload", action release {} -> {"released": true, "t": <unix s>}
    event released {"t": <unix s>}      state armed_pin {"type": string}

The stub drops at the unit's pose (9102/static fallback) and logs the
release like the gripper's COMMAND_ACK path - the event shape and the
invoke result are identical, so consumers (mission translator, drop
planner) see no difference. Where the real module blocks up to 3 s on
the FC ack, the stub releases at once; a second invoke while a release
is in progress gets "release already in progress" like the real one.

The released position/time lands a {"type":"detection",
"cls":"payload_drop", tid:"payload"} row in truth.jsonl so a scenario
can distance_to the drop point against world truth.
"""

import json
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stub_base import StubBase  # noqa: E402

ADVERT = {
    "schema": "de.cap/1",
    "ns": "payload",
    "module": "stub_payload",
    "ver": "0.1.0",
    "actions": {
        "release": {
            "desc": "Release the payload (simulated servo pin)",
            "timeout_s": 5,
        },
    },
    "events": {
        "released": {
            "desc": "payload released",
            "payload": {"t": {"type": "number",
                              "desc": "unix time, seconds"}},
        },
    },
    "state": {
        "armed_pin": {"type": "string", "desc": "release pin name"},
    },
}


class StubPayload(StubBase):
    MODULE_CLASS = "gen"
    MODULE_VERSION = "0.1.0"

    def __init__(self, argv):
        super().__init__(argv, "stub_payload")
        pin = (self.cfg.get("payload") or {}).get("pin", "stub_pin")
        self.armed_pin = str(pin)
        self._busy = threading.Lock()

    def state(self, changed, full=False):
        self.log.log("STATE", changed, ms=self.clock_ms())
        if self.facade is not None:
            self.facade.publish_state("payload", changed, full)

    def event(self, payload):
        self.log.log("EVENT", {"d": "payload.released", "ed": payload},
                     ms=self.clock_ms())
        if self.facade is not None:
            self.facade.fire_event("payload", "released", payload)

    def on_invoke(self, ns, act, params):
        if act != "release":
            return None, f"unknown action payload.{act}"
        if not self._busy.acquire(blocking=False):
            return None, "release already in progress"
        try:
            pose = self.pose_feed.pose() or {}
            t = self.clock_ms() / 1000.0
            self.event({"t": t})
            self.log.log("RELEASE", {"t": t,
                                     "lat": pose.get("lat"),
                                     "lng": pose.get("lng")},
                         ms=self.clock_ms())
            if pose:
                self.truth.append({"src": "payload", "tid": "payload",
                                   "cls": "payload_drop",
                                   "lat": pose.get("lat"),
                                   "lng": pose.get("lng"),
                                   "alt_m": pose.get("rel_alt_m", 0.0),
                                   "sent": True}, ms=self.clock_ms())
            return {"released": True, "t": t}, None
        finally:
            self._busy.release()

    def tick(self, t_s, frame):
        pass                       # nothing to model between invokes


def main(argv):
    s = StubPayload(argv)
    if s.selftest:
        # replayed invokes ride the pose file rows: {"t_ms","invoke":
        # "release"}
        s._vt_ms = 0
        for ln in open(s.selftest):
            try:
                row = json.loads(ln)
            except ValueError:
                continue
            s._vt_ms = int(row.get("t_ms", 0))
            if "lat" in row:
                s._selftest_pose(row)
            if row.get("invoke") == "release":
                data, err = s.on_invoke("payload", "release", {})
                s.log.log("INVOKE", {"act": "release", "data": data,
                                     "err": err}, ms=s._vt_ms)
        s.log.log("END", {"frames": 0}, ms=s._vt_ms)
        return 0
    s.connect(adverts=[json.dumps(ADVERT)])
    s.state({"armed_pin": s.armed_pin}, full=True)
    s.run()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
