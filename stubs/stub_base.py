#!/usr/bin/env python3
"""stub_base.py - shared machinery for the P5-08 stub sensor modules.

A stub is a databus Python module, launched like a real module through
the catalog:  '<module>' -c '<generated config>'. The config carries
the s2s ports plus a "stub" block:

    "stub": {"world": "<world.yaml>", "seed": 1, "unit": "<unit>",
             "pose_port": 5765+10k}        # SITL serial5 (0 on statics)

From the config path the stub derives var/run/<sim>/ and reads:
  var/run/<sim>/world_epoch.json   {"epoch_unix_ms"} - scenario t0 written
                               by the runner; absent -> the stub idles
  var/run/<sim>/truth.jsonl        per-detection rows are appended here
  var/run/<sim>/world_overrides.jsonl  react knots the truth tick writes
                               (SL-0013); the stub reads, never writes
  var/run/<sim>/stubs/<unit>.log   one line per sent message:
                               "<unix_ms> <kind> <json>"

Pose: units with a flight controller are read on the SITL pose port
(serial5 MAVLink, ATTITUDE + GLOBAL_POSITION_INT - the same stream
de_mavlink projects from, never touching the pilot/fcb/telemetry ports);
9102 intermodule location reports fill position and yaw everywhere, and
on a static unit they are the only source (de_comm broadcasts the
configured "location" at 1 Hz).

--selftest <pose.jsonl> replaces the live feeds: the stub runs the same
model loop against scripted poses on a virtual clock (t = the row's
t_ms). Two same-seed selftest runs are byte-identical - the determinism
acceptance.
"""

import json
import os
import signal
import sys
import threading
import time

_STUB_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(_STUB_DIR)
sys.path.insert(0, os.path.join(SIM_ROOT, "engine"))

import simlib                      # noqa: E402
import world as world_mod          # noqa: E402
import geo_projection as gp        # noqa: E402

_DATABUS = os.path.join(
    simlib.repo_dir("DE_DATABUS_DIR", "de_databus"), "python")
if _DATABUS not in sys.path:
    sys.path.insert(0, _DATABUS)

from messages import (             # noqa: E402
    ANDRUAV_PROTOCOL_MESSAGE_CMD, ANDRUAV_PROTOCOL_MESSAGE_TYPE,
    TYPE_AndruavMessage_ID, TYPE_AndruavModule_Location_Info)
from de_module import CModule      # noqa: E402
from de_facade_base import CFacade_Base  # noqa: E402

DEFAULT_UDP_DATABUS_PACKET_SIZE = 8192

EARTH = 111320.0


def now_ms():
    return int(time.time() * 1000)


def load_module_cfg(argv):
    """'-c <config.json>' plus the selftest switches. The generated
    launch cmd only ever passes -c."""
    cfg_path = None
    selftest = None
    dur_s = 60.0
    i = 0
    while i < len(argv):
        if argv[i] == "-c" and i + 1 < len(argv):
            cfg_path = argv[i + 1]
            i += 2
        elif argv[i] == "--selftest" and i + 1 < len(argv):
            selftest = argv[i + 1]
            i += 2
        elif argv[i] == "--dur" and i + 1 < len(argv):
            dur_s = float(argv[i + 1])
            i += 2
        else:
            i += 1
    if not cfg_path:
        sys.exit("usage: <stub> -c <module.config.json> "
                 "[--selftest pose.jsonl] [--dur s]")
    return simlib.load_config(cfg_path), cfg_path, selftest, dur_s


class PoseFeed:
    """The unit's pose: SITL serial5 (att + position) plus the 9102
    intermodule feed (position + yaw cdeg; the only source on statics).

    pose() -> {"lat","lng","alt_m","rel_alt_m","roll","pitch","yaw",
               "t_ms","src"} or None."""

    def __init__(self, pose_port, sysid=1, yaw0_deg=0.0):
        self._lock = threading.Lock()
        self._att = {}            # roll/pitch/yaw rad from serial5
        self._pos = {}            # lat/lng/alt from serial5 GLOBAL_POSITION_INT
        self._loc9102 = {}        # the intermodule 9102 fields
        self._yaw0 = yaw0_deg
        self._sysid = sysid
        self._port = pose_port
        self._stop = threading.Event()
        self._thread = None
        # live runs set this: a sample older than this is not the pose now
        # (a dead SITL stream must not keep "seeing" from where it froze)
        self.max_age_ms = None

    def start(self):
        if self._port:
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(3)

    def on_9102(self, ms):
        """de_mavlink's or de_comm's location report {la,ln,a,r,ha,y}."""
        try:
            loc = {"lat": ms["la"] / 1e7, "lng": ms["ln"] / 1e7,
                   "alt_m": ms.get("a", 0) / 1000.0,
                   "rel_alt_m": ms.get("r", 0) / 1000.0,
                   "yaw": (ms.get("y", 0) / 100.0) * 3.14159265 / 180.0,
                   "t_ms": now_ms()}
            with self._lock:
                self._loc9102 = loc
        except (KeyError, TypeError):
            pass

    def _run(self):
        from pymavlink import mavutil
        while not self._stop.is_set():
            try:
                m = mavutil.mavlink_connection(
                    f"tcp:127.0.0.1:{self._port}", autoreconnect=False,
                    retries=2, source_system=252)
                # 10 Hz streams like the de_mavlink pose sampler
                for mid in (30, 33):          # ATTITUDE, GLOBAL_POSITION_INT
                    m.mav.command_long_send(
                        self._sysid, 1, 511, 0, mid, 100000, 0, 0, 0, 0, 0)
                while not self._stop.is_set():
                    msg = m.recv_match(type=["ATTITUDE",
                                             "GLOBAL_POSITION_INT"],
                                       blocking=True, timeout=3.0)
                    if msg is None:
                        raise ConnectionError("pose stream stalled")
                    with self._lock:
                        if msg.get_type() == "ATTITUDE":
                            self._att = {"roll": msg.roll,
                                         "pitch": msg.pitch,
                                         "yaw": msg.yaw,
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
        with self._lock:
            mav_pos = dict(self._pos)
            att = dict(self._att)
            loc = dict(self._loc9102)
        if self.max_age_ms is not None:
            now = now_ms()

            def fresh(d):
                return d if d and now - d.get("t_ms", 0) <= self.max_age_ms \
                    else {}
            mav_pos, att, loc = fresh(mav_pos), fresh(att), fresh(loc)
        pos = mav_pos or loc
        if not pos:
            return None
        out = {"lat": pos["lat"], "lng": pos["lng"],
               "alt_m": pos.get("alt_m", 0.0),
               "rel_alt_m": pos.get("rel_alt_m", 0.0),
               "roll": att.get("roll", 0.0),
               "pitch": att.get("pitch", 0.0),
               "yaw": att.get("yaw",
                              loc.get("yaw",
                                      self._yaw0 * 3.14159265 / 180.0)),
               "t_ms": now_ms(),
               "src": "mav" if mav_pos else "9102"}
        return out


class EpochSource:
    """var/run/<sim>/world_epoch.json, mtime-cached per tick. None while the
    runner has not started a world run (the stub idles - the world is
    scenario-scoped)."""

    def __init__(self, path):
        self.path = path
        self._mtime = None
        self._epoch = None

    def epoch_ms(self):
        try:
            mt = os.path.getmtime(self.path)
        except OSError:
            self._epoch = None
            self._mtime = None
            return None
        if mt != self._mtime:
            try:
                self._epoch = json.load(open(self.path))["epoch_unix_ms"]
            except (OSError, ValueError, KeyError):
                self._epoch = None
            self._mtime = mt
        return self._epoch


class StubLog:
    """stubs/<unit>.log - one line per thing the stub did, fleet time
    first (the fleet clock is unix ms on this fleet). Open-append per
    line so the runner can truncate between runs."""

    def __init__(self, path):
        self.path = path

    def log(self, kind, obj, ms=None):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        try:
            with open(self.path, "a") as f:
                f.write(f"{ms if ms is not None else now_ms()} {kind} "
                        f"{json.dumps(obj)}\n")
        except OSError:
            pass


class TruthAppender:
    """per-detection rows into var/run/<sim>/truth.jsonl - {"type":
    "detection", src, unit, tid|"clutter", cls, t_ms, ...model fields}.
    One open-append per row; reads alongside the runner's 1 Hz entity
    rows. tid "clutter" marks a false positive for detection_precision."""

    def __init__(self, path, unit):
        self.path = path
        self.unit = unit

    def append(self, row, ms=None):
        row = dict(row)
        row.setdefault("type", "detection")
        row.setdefault("unit", self.unit)
        row.setdefault("t_ms", ms if ms is not None else now_ms())
        try:
            with open(self.path, "a") as f:
                f.write(json.dumps(row) + "\n")
        except OSError:
            pass


class StubBase:
    """Config + world + pose + epoch + logs, wired for one stub process.
    Subclasses implement tick(t_s, frame) and optionally register
    callbacks (on_invoke, on_message)."""

    MODULE_CLASS = "gen"
    MODULE_VERSION = "0.1.0"

    def __init__(self, argv, name):
        self.name = name
        self.cfg, self.cfg_path, self.selftest, self.selftest_dur = \
            load_module_cfg(argv)
        # var/run/<sim>/<unit>/<mod>/<mod>.config.json -> var/run/<sim>
        self.run_dir = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(self.cfg_path))))
        spec = self.cfg.get("stub") or {}
        self.unit = spec.get("unit") or ""
        self.seed = int(spec.get("seed", 1))
        self.pose_port = int(spec.get("pose_port") or 0)
        self.log = StubLog(os.path.join(self.run_dir, "stubs",
                                        f"{self.unit or name}.log"))
        self.truth = TruthAppender(os.path.join(self.run_dir,
                                                "truth.jsonl"),
                                   self.unit)
        self.epoch = EpochSource(os.path.join(self.run_dir,
                                              "world_epoch.json"))
        self.world = None
        if spec.get("world") and os.path.isfile(spec["world"]):
            self.world = world_mod.load(spec["world"], self.seed)
            # SL-0013: the truth tick's reaction knots - the same file
            # the recorder writes, so detections track the truth's
            # reacted positions (absent file = nothing reacts)
            self.world.use_overrides(os.path.join(
                self.run_dir, "world_overrides.jsonl"))
        self.pose_feed = PoseFeed(self.pose_port,
                                  sysid=int(spec.get("sysid", 1)))
        # SL-0030 sensor contract: <stub_dir>/<stem>.sensor.yaml (or the
        # injected stub.contract path) declares the truth inputs, the
        # model defaults (world.sensors.<key> overrides them) and the
        # output seam - see docs/SENSORS.md
        self.contract = self._load_contract(spec)
        self.module = None
        self.facade = None
        self._vt_ms = None          # virtual clock under --selftest
        self.exit_me = False
        signal.signal(signal.SIGINT, self._quit)
        signal.signal(signal.SIGTERM, self._quit)

    def _quit(self, *_a):
        self.exit_me = True

    # ---------------------------------------------------- sensor contract

    def _load_contract(self, spec):
        """{} or the contract dict from <stub>.sensor.yaml beside the
        stub file (spec["contract"] overrides the path - the world
        injector does not set it; a non-stub launch can)."""
        path = spec.get("contract")
        if not path:
            # the subclass's own source file, not sys.argv[0]: a stub
            # imported in-process (test_golden, model_import) must find
            # its contract too - without it truth_entities() returns
            # every entity and the sensor sees non-emitters
            src = getattr(sys.modules.get(type(self).__module__),
                          "__file__", None) or sys.argv[0]
            stub_dir = os.path.dirname(os.path.abspath(src))
            stem = os.path.splitext(os.path.basename(src))[0]
            for cand in (os.path.join(stub_dir, stem + ".sensor.yaml"),
                         os.path.join(stub_dir, "sensor.yaml")):
                if os.path.isfile(cand):
                    path = cand
                    break
        if not path or not os.path.isfile(path):
            return {}
        return simlib.load_yaml(path) or {}

    def sensor_key(self):
        """The world.sensors key this contract reads."""
        return (self.contract.get("sensor") or {}).get("key")

    def sensor_params(self):
        """contract.model merged under world.sensors[<key>] - the world
        file always wins over the shipped defaults."""
        s = self.contract.get("sensor") or {}
        model = dict(s.get("model") or {})
        key = s.get("key")
        if key and self.world is not None:
            model.update(self.world.sensors.get(key) or {})
        return model

    def truth_entities(self):
        """The world entities this sensor reads, per contract.truth:
        `cls` (one name or a list) and/or `field` (an entity attribute
        that must be set - e.g. tx_dbm). No contract = every entity."""
        if self.world is None:
            return []
        t = (self.contract.get("sensor") or {}).get("truth") or {}
        ents = list(self.world.entities)
        cls = t.get("cls")
        if cls:
            want = {cls} if isinstance(cls, str) else set(cls)
            ents = [e for e in ents if e.cls in want]
        field = t.get("field")
        if field:
            ents = [e for e in ents if getattr(e, field, None) is not None]
        return ents

    # ----------------------------------------------------------- databus

    def connect(self, adverts=None, msg_filter=(), module_id=None):
        """Register on the unit's de_comm exactly like a real module."""
        self.module = CModule()
        self.module.m_OnReceive = self.on_message
        mid = module_id or f"{self.unit}-{self.name}"
        mkey = self.module_key()
        self.module.defineModule(self.MODULE_CLASS, mid, mkey,
                                 self.MODULE_VERSION,
                                 [TYPE_AndruavMessage_ID,
                                  TYPE_AndruavModule_Location_Info]
                                 + list(msg_filter))
        self.facade = CFacade_Base(self.module)
        if adverts:
            self.facade.set_capabilities(adverts)
            self.facade.on_invoke(self.on_invoke)
        self.module.init(
            self.cfg.get("s2s_udp_target_ip", "127.0.0.1"),
            int(self.cfg["s2s_udp_target_port"]),
            self.cfg.get("s2s_udp_listening_ip", "0.0.0.0"),
            int(self.cfg["s2s_udp_listening_port"]),
            int(self.cfg.get("s2s_udp_packet_size",
                             DEFAULT_UDP_DATABUS_PACKET_SIZE)))
        self.log.log("MODULE", {"online": True, "id": mid, "key": mkey})

    def module_key(self):
        """the generated local file's module_key, else a stub default."""
        local = os.path.join(os.path.dirname(self.cfg_path),
                             f"{self.name}.local")
        try:
            return json.load(open(local))["module_key"]
        except (OSError, ValueError, KeyError):
            return f"{self.unit}-{self.name}"[:12]

    def on_message(self, message, length, jMsg):
        """m_OnReceive - collect the 9102 pose feed; subclasses extend."""
        try:
            if jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_TYPE) == \
                    TYPE_AndruavModule_Location_Info:
                self.pose_feed.on_9102(
                    jMsg.get(ANDRUAV_PROTOCOL_MESSAGE_CMD) or {})
        except Exception:
            pass

    def on_invoke(self, ns, act, params):
        return None, f"unhandled action {ns}.{act}"

    # ------------------------------------------------------------ clock

    def now_s(self):
        """Scenario seconds (or the virtual clock under --selftest)."""
        ep = self.epoch.epoch_ms()
        if ep is None:
            return None
        return (now_ms() - ep) / 1000.0

    # ------------------------------------------------------- model hooks

    def rng(self, *parts):
        return world_mod.rng(self.seed, *parts)

    def clock_ms(self):
        """Log/truth timestamps: unix ms live, virtual ms selftest."""
        return self._vt_ms if self._vt_ms is not None else now_ms()

    def tick(self, t_s, frame):
        raise NotImplementedError

    def reset_run(self):
        """A new scenario epoch began (a reused sim keeps the stub
        process): drop every per-run model state so the run starts as on a
        fresh fleet. Subclasses extend."""

    POSE_MAX_AGE_MS = 3000

    def run(self):
        """The live loop: wait for the epoch, then tick at ~20 Hz (the
        models quantize to their own frame/sweep indices so pacing jitter
        does not change the emitted sequence)."""
        self.pose_feed.max_age_ms = self.POSE_MAX_AGE_MS
        self.pose_feed.start()
        last_frame = None
        run_epoch = None
        while not self.exit_me:
            t_s = self.now_s()
            if t_s is None:
                self.exit_me_wait(0.5)
                continue
            ep = self.epoch.epoch_ms()
            if ep != run_epoch:
                if run_epoch is not None:
                    self.log.log("RUN", {"epoch_unix_ms": ep, "reset": True})
                    self.reset_run()
                run_epoch = ep
                last_frame = None
            frame = int(t_s * 20.0)
            if frame != last_frame:
                last_frame = frame
                try:
                    self.tick(t_s, frame)
                except Exception as e:
                    self.log.log("ERR", {"tick": repr(e)})
            self.exit_me_wait(0.02)
        self.pose_feed.stop()

    def run_selftest(self, t_field="t_ms"):
        """--selftest <pose.jsonl>: replay poses on the virtual clock.
        The epoch is t=0 of the pose file; tick indices are exact."""
        rows = []
        for ln in open(self.selftest):
            try:
                rows.append(json.loads(ln))
            except ValueError:
                continue
        rows.sort(key=lambda r: r.get(t_field, 0))
        self._vt_ms = 0               # deterministic clock from here on
        t_end = rows[-1][t_field] / 1000.0 if rows else self.selftest_dur
        t_end = max(t_end, self.selftest_dur)
        frame = 0
        t_s = 0.0
        pose_i = 0
        while t_s <= t_end and not self.exit_me:
            self._vt_ms = int(round(t_s * 1000))
            while pose_i < len(rows) and rows[pose_i][t_field] <= t_s * 1000:
                self._selftest_pose(rows[pose_i])
                pose_i += 1
            self.tick(t_s, frame)
            frame += 1
            t_s = frame * 0.05            # the same 20 Hz quantum
        self.log.log("END", {"frames": frame}, ms=self.clock_ms())

    def _selftest_pose(self, row):
        f = self.pose_feed
        with f._lock:
            if "lat" in row:
                f._pos = {"lat": row["lat"], "lng": row["lng"],
                          "alt_m": row.get("alt_m", 0.0),
                          "rel_alt_m": row.get("rel_alt_m", 0.0),
                          "t_ms": row.get("t_ms", 0)}
            f._att = {"roll": row.get("roll", 0.0),
                      "pitch": row.get("pitch", 0.0),
                      "yaw": row.get("yaw", 0.0),
                      "t_ms": row.get("t_ms", 0)}
            if "yaw" in row and "lat" in row:
                f._loc9102 = {"lat": row["lat"], "lng": row["lng"],
                              "yaw": row["yaw"], "rel_alt_m":
                              row.get("rel_alt_m", 0.0)}

    def exit_me_wait(self, s):
        end = time.time() + s
        while not self.exit_me and time.time() < end:
            time.sleep(min(0.02, end - time.time()))
