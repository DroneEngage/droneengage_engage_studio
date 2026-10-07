#!/usr/bin/env python3
"""telemetry.py - the scenario tlog recorder (TASK-P5-07).

One background thread per FC unit, on the SITL serial2 TCP port
(fc["aux_port"] - serial0 stays free for ctl pilot commands). Records
one JSONL row per sample:

    {t_ms, unit, lat, lng, alt_m, rel_alt_m, hdg, roll, pitch, yaw,
     armed, mode,
     bat_v, bat_pct, fix_type, ekf_ok, rc1..rc4, t_unix_usec, t_boot_ms,
     speedup}

roll/pitch/yaw (rad, ATTITUDE) were added for P5-12: the coverage
footprint is wrong by up to alt * tan(bank) in turns without them.

t_ms is wall unix ms (the fleet clock, unsynced); t_unix_usec/t_boot_ms
come from the FC's own SYSTEM_TIME so truth matching can use sim time;
speedup estimates SITL's actual rate (t_unix_usec advancing vs wall) -
the realtime SKIP check reads the last samples' mean.
"""

import json
import threading
import time


def _mav(u):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(
        f"tcp:127.0.0.1:{u['fc']['aux_port']}", autoreconnect=False,
        retries=2, source_system=253)
    return m


def _hb_mode(kind_modes, hb):
    return kind_modes.get(hb.custom_mode, f"MODE{hb.custom_mode}")


class Telemetry:
    """Telemetry(manifest_units, path, hz=2). start()/stop()."""

    def __init__(self, units, path, hz=2.0):
        self.units = [u for u in units if u.get("fc")]
        self.path = path
        self.hz = hz
        self._stop = threading.Event()
        self._threads = []
        self._latest = {}              # unit -> last row (speedup checks)
        self._speeds = {}              # unit -> recent speed-up samples
        self.last_error = {}           # unit -> last connect/collect error
        # --hold boundary (unix ms): the recorder keeps running for the
        # parked viewer, but rows stamped after it are hold traffic -
        # rows()/latest()/age_ms() never return them to the checks
        self.until_ms = None

    def start(self):
        # a stale file from an earlier run would mix its rows into this
        # run's pairwise / position assertions
        try:
            open(self.path, "w").close()
        except OSError:
            pass
        for u in self.units:
            t = threading.Thread(target=self._run, args=(u,), daemon=True)
            t.start()
            self._threads.append(t)

    def stop(self):
        self._stop.set()
        for t in self._threads:
            t.join(5)

    def latest(self, unit_name=None):
        if unit_name is None:
            return {n: dict(r) for n, r in self._latest.items()}
        if self.until_ms is not None:
            # --hold: "latest" is the newest row before the boundary
            rows = self.rows(unit_name)
            return dict(rows[-1]) if rows else {}
        return dict(self._latest.get(unit_name) or {})

    def speedup(self, unit_name):
        """The unit's median SITL speed-up over the last 60 samples, or
        None when none was measured. The median, not the last sample: one
        glitch right before the gate must not flip the realtime verdict."""
        v = sorted(self._speeds.get(unit_name) or [])
        return v[len(v) // 2] if v else None

    def age_ms(self, unit_name, now_ms=None):
        """Age of the unit's newest row, or None when it has none - a
        row from a dead stream must not pass for the current state.
        Under --hold the newest row is the last one before the
        boundary, aged against the boundary: how stale it was when the
        evidence window closed."""
        row = self._latest.get(unit_name) if self.until_ms is None \
            else (self.rows(unit_name) or [None])[-1]
        if not row:
            return None
        now = int(time.time() * 1000) if now_ms is None else now_ms
        if self.until_ms is not None:
            now = min(now, self.until_ms)
        return now - row["t_ms"]

    def rows(self, unit_name=None):
        """Read the written file back - [(row)] in order. Rows stamped
        after until_ms (the --hold boundary) are dropped."""
        out = []
        try:
            with open(self.path, errors="replace") as f:
                for ln in f:
                    try:
                        r = json.loads(ln)
                    except ValueError:
                        continue
                    if self.until_ms is not None and \
                            r.get("t_ms", 0) > self.until_ms:
                        continue
                    if unit_name is None or r.get("unit") == unit_name:
                        out.append(r)
        except OSError:
            pass
        return out

    # --------------------------------------------------------- internals

    # MAVLink ids we want streamed and their period in us
    _WANT = [(2, 500000),      # SYSTEM_TIME     (speedup, sim clock)
             (1, 500000),      # SYS_STATUS      (battery)
             (24, 500000),     # GPS_RAW_INT     (fix_type)
             (30, 500000),     # ATTITUDE        (roll/pitch/yaw - P5-12)
             (33, 500000),     # GLOBAL_POSITION_INT
             (65, 500000),     # RC_CHANNELS     (rc_override)
             (193, 1000000)]   # EKF_STATUS_REPORT

    def _run(self, u):
        import simlib
        name = u["name"]
        sysid = u["fc"]["sysid"]
        modes = getattr(__import__("pymavlink.mavutil",
                                   fromlist=["x"]),
                        simlib.FC_KINDS[u["fc"]["vehicle"]]["modes"])
        while not self._stop.is_set():
            try:
                m = _mav(u)
                # SET_MESSAGE_INTERVAL per msg - REQUEST_DATA_STREAM
                # leaves several types silent on this SITL build
                for mid, us in self._WANT:
                    m.mav.command_long_send(
                        sysid, 1, 511, 0, mid, us, 0, 0, 0, 0, 0)
                self._collect(u, m, modes)
            except Exception as e:
                self.last_error[name] = f"{type(e).__name__}: {e}"
            self._stop.wait(3.0)

    def _collect(self, u, m, modes):
        row = {"unit": u["name"], "lat": None, "lng": None}
        speed_hist = []
        period = 1.0 / self.hz
        next_write = 0.0
        while not self._stop.is_set():
            msg = m.recv_match(timeout=2.0, blocking=True)
            wall = time.time()
            if msg is None:
                raise ConnectionError("telemetry stream stalled")
            if msg.get_srcSystem() != u["fc"]["sysid"]:
                continue
            t = msg.get_type()
            if t == "GLOBAL_POSITION_INT":
                row.update(lat=msg.lat / 1e7, lng=msg.lon / 1e7,
                           alt_m=msg.alt / 1000.0,
                           rel_alt_m=msg.relative_alt / 1000.0,
                           hdg=msg.hdg / 100.0,
                           vx=msg.vx / 100.0, vy=msg.vy / 100.0,
                           vz=msg.vz / 100.0)
            elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                row.update(armed=bool(msg.base_mode & 128),
                           mode=_hb_mode(modes, msg))
            elif t == "SYS_STATUS":
                row.update(bat_v=round(msg.voltage_battery / 1000.0, 2),
                           bat_pct=msg.battery_remaining)
            elif t == "GPS_RAW_INT":
                row.update(fix_type=msg.fix_type)
            elif t == "ATTITUDE":
                row.update(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw)
            elif t == "EKF_STATUS_REPORT":
                row.update(ekf_ok=bool(msg.flags & 0x10))
            elif t == "RC_CHANNELS":
                for ch in (1, 2, 3, 4):
                    v = getattr(msg, f"chan{ch}_raw", None)
                    if v is not None:
                        row[f"rc{ch}"] = v
            elif t == "SYSTEM_TIME":
                row["t_unix_usec"] = msg.time_unix_usec
                row["t_boot_ms"] = msg.time_boot_ms
                # speedup = FC boot clock vs wall (t_unix_usec stays 0
                # until GPS, boot_ms always advances at sim rate)
                speed_hist.append((wall, msg.time_boot_ms))
                del speed_hist[:-8]
                if len(speed_hist) >= 2:
                    du = (speed_hist[-1][1] - speed_hist[0][1]) / 1000.0
                    dw = speed_hist[-1][0] - speed_hist[0][0]
                    if dw > 0:
                        row["speedup"] = round(du / dw, 3)
            # sample on any frame, throttled to hz - waiting on one
            # message type would leave short runs with an empty log
            if wall >= next_write:
                next_write = wall + period
                out = dict(row, t_ms=int(wall * 1000),
                           speedup=row.get("speedup"))
                self._latest[u["name"]] = out
                if out.get("speedup") is not None:
                    sp = self._speeds.setdefault(u["name"], [])
                    sp.append(out["speedup"])
                    del sp[:-60]
                try:
                    with open(self.path, "a") as f:
                        f.write(json.dumps(out) + "\n")
                except OSError:
                    pass
