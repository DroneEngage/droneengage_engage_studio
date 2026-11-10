#!/usr/bin/env python3
"""stub_rtcm.py - an RTCM3 correction source on the bus (SL-0233).

Sends a fixture RTCM3 stream as the RTCM data type (6548,
TYPE_AndruavMessage_RTCM_DATA) to de_mavlink, which injects it into the
FC as GPS_RTCM_DATA. Message layout as 6502: JSON header, one 0x00 byte,
then the raw bytes. Header: {"seq": n}.

Config (module config block "rtcm", all optional):

    "rtcm": {"file": "<fixture.bin>",   # default: the generated stream
                                        # of tests/gnss/fixtures
             "chunk_bytes": 360,        # message size limit
             "split": false,            # false: whole RTCM3 frames per
                                        #  message (a frame larger than
                                        #  chunk_bytes goes alone);
                                        #  true: raw slices of chunk_bytes
             "rate_hz": 1.0,            # messages per second
             "start_s": 0.0,            # delay after the run epoch
             "loop": false}             # false: send the stream once

Every message is logged "<ms> SEND {"mt": 6548, "seq", "n", "off"}".
When the last byte is out, the log gets "DONE {"sent", "fixture"}":
sent equals the fixture size. Control: de.cap/1 ns "rtcm", actions
"stop" and "start" pause and resume (a dead caster).
"""

import json
import os
import sys

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "tests", "gnss",
                                "fixtures"))
from stub_base import StubBase  # noqa: E402
from messages import TYPE_AndruavMessage_RTCM_DATA  # noqa: E402
import rtcm_fixture  # noqa: E402

TICKS_PER_S = 20

ADVERT = {
    "schema": "de.cap/1",
    "ns": "rtcm",
    "module": "stub_rtcm",
    "ver": "0.1.0",
    "desc": "simulated RTCM3 source: sends 6548 from a fixture stream",
    "actions": {
        "start": {"desc": "Send corrections again", "timeout_s": 5},
        "stop": {"desc": "Stop sending corrections", "timeout_s": 5},
    },
    "state": {"enabled": {"type": "bool"}},
}


def make_chunks(data, chunk_bytes, split):
    """(offset, bytes) pieces of the stream, in order."""
    out = []
    if split:
        for off in range(0, len(data), chunk_bytes):
            out.append((off, data[off:off + chunk_bytes]))
        return out
    cur, cur_off, off = b"", 0, 0
    for frame in rtcm_fixture.split_frames(data):
        if cur and len(cur) + len(frame) > chunk_bytes:
            out.append((cur_off, cur))
            cur, cur_off = b"", off
        cur += frame
        off += len(frame)
    if cur:
        out.append((cur_off, cur))
    return out


class StubRtcm(StubBase):
    MODULE_CLASS = "gen"
    MODULE_VERSION = "0.1.0"

    def __init__(self, argv):
        super().__init__(argv, "stub_rtcm")
        cfg = self.cfg.get("rtcm") or {}
        path = cfg.get("file")
        if path and not os.path.isabs(path):
            path = os.path.join(os.path.dirname(_HERE), path)
        self.data = rtcm_fixture.load_or_build(path)
        self.chunks = make_chunks(self.data,
                                  max(1, int(cfg.get("chunk_bytes", 360))),
                                  bool(cfg.get("split", False)))
        self.every = max(1, int(round(
            TICKS_PER_S / max(0.1, float(cfg.get("rate_hz", 1.0))))))
        self.start_s = float(cfg.get("start_s", 0.0))
        self.loop = bool(cfg.get("loop", False))
        self.enabled = True
        self.reset_run()

    def reset_run(self):
        self.enabled = True
        self.idx = 0
        self.seq = 0
        self.sent = 0
        self.done = False

    def emit(self, off, chunk):
        hdr = {"seq": self.seq}
        self.log.log("SEND", {"mt": TYPE_AndruavMessage_RTCM_DATA,
                              "seq": self.seq, "n": len(chunk),
                              "off": off}, ms=self.clock_ms())
        if self.module is not None:
            self.module.sendBMSG("", chunk, len(chunk),
                                 TYPE_AndruavMessage_RTCM_DATA, True, hdr)
        self.seq += 1
        self.sent += len(chunk)

    def state(self, changed, full=False):
        self.log.log("STATE", changed, ms=self.clock_ms())
        if self.facade is not None:
            self.facade.publish_state("rtcm", changed, full)

    def on_invoke(self, ns, act, params):
        if act in ("start", "stop"):
            self.enabled = act == "start"
            self.log.log("CONTROL", {"act": act}, ms=self.clock_ms())
            self.state({"enabled": self.enabled})
            return {"enabled": self.enabled}, None
        return None, f"unknown action rtcm.{act}"

    def tick(self, t_s, frame):
        if not self.enabled or self.done or t_s < self.start_s:
            return
        if frame % self.every:
            return
        off, chunk = self.chunks[self.idx]
        self.emit(off, chunk)
        self.idx += 1
        if self.idx >= len(self.chunks):
            if self.loop:
                self.idx = 0
            else:
                self.done = True
                self.log.log("DONE", {"sent": self.sent,
                                      "fixture": len(self.data)},
                             ms=self.clock_ms())


def main(argv):
    s = StubRtcm(argv)
    if s.selftest:
        s.run_selftest()
        return 0
    s.connect(adverts=[json.dumps(ADVERT)])
    s.state({"enabled": True}, full=True)
    s.run()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
