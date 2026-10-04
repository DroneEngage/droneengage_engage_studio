#!/usr/bin/env python3
"""telemetry.py - the scenario tlog recorder (TASK-P5-07).

One background thread per FC unit, on the SITL serial2 TCP port
(fc["aux_port"] - serial0 stays free for ctl pilot commands). Records
one JSONL row per sample:

    {t_ms, unit, lat, lng, alt_m, rel_alt_m, hdg, armed, mode,
     bat_v, bat_pct, fix_type, ekf_ok, rc1..rc4, t_unix_usec, t_boot_ms,
     speedup}

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

    def start(self):
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
        return dict(self._latest.get(unit_name) or {})

    def speedup(self, unit_name):
        """The unit's mean SITL speed-up over the recent samples, or
        None when no SYSTEM_TIME has been seen."""
        row = self._latest.get(unit_name) or {}
        return row.get("speedup")

    def rows(self, unit_name=None):
        """Read the written file back - [(row)] in order."""
        out = []
        try:
            with open(self.path, errors="replace") as f:
                for ln in f:
                    try:
                        r = json.loads(ln)
                    except ValueError:
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
            except Exception:
                pass
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
                try:
                    with open(self.path, "a") as f:
                        f.write(json.dumps(out) + "\n")
                except OSError:
                    pass
