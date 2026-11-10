#!/usr/bin/env python3
"""stub_nmea_serial.py - a GNSS receiver on a serial port, faked (MV-0027).

Creates a pty pair and symlinks the slave side at
<unit_dir>/gnss_serial.tty, then writes NMEA 0183 sentences at 4 Hz -
GGA, RMC, GSA, VTG, checksum-correct - built from the SITL truth pose,
the way a wired receiver streams them. de_mavlink's gnss_source type
"serial" opens the link path and reads them.

The link path is deterministic so the unit's de_mavlink config overlay
can point at it: var/run/<sim>/<unit>/gnss_serial.tty. de_mavlink may
start before this stub - it retries the open, which is also how the
missing-port error path is exercised.

Control: none - the receiver is dumb hardware. Kill the stub to make a
dead source (the pty master goes away, de_mavlink sees read errors and
retries the open, the 2 s stale gate stops GPS_INPUT).
"""

import os
import pty
import sys
import termios

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

_STUBS_DIR = os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), "stubs")
sys.path.insert(0, _STUBS_DIR)
from stub_base import StubBase  # noqa: E402
from stub_gnss import TruthPoseFeed  # noqa: E402

TICKS_PER_S = 20          # StubBase.run() quantum
EMIT_EVERY = 5            # one NMEA burst every 5 ticks -> 4 Hz


def _checksum(body):
    """'$'..'*' XOR checksum -> sentence with the '$%02X' tail."""
    sum_ = 0
    for c in body:
        sum_ ^= ord(c)
    return f"${body}*{sum_:02X}"


def _coord(deg, is_lat):
    """decimal degrees -> NMEA ddmm.mmmm + hemisphere letter."""
    hemi = ("N" if deg >= 0 else "S") if is_lat else \
        ("E" if deg >= 0 else "W")
    deg = abs(deg)
    d = int(deg)
    m = (deg - d) * 60.0
    return f"{d:02d}{m:07.4f}" if is_lat else f"{d:03d}{m:07.4f}", hemi


def _sentences(pose):
    """GGA + RMC + GSA + VTG for one fix epoch, from the truth pose."""
    lat_f, lat_h = _coord(pose["lat"], True)
    lng_f, lng_h = _coord(pose["lng"], False)
    alt = pose.get("alt_m", 0.0) or 584.0 + pose.get("rel_alt_m", 0.0)
    speed_mps = (pose.get("vx", 0.0) ** 2 + pose.get("vy", 0.0) ** 2) ** 0.5
    speed_kn = speed_mps / 0.514444
    speed_kmh = speed_mps * 3.6
    course = pose.get("hdg", 0.0) % 360.0
    return (
        _checksum(f"GPGGA,000000,{lat_f},{lat_h},{lng_f},{lng_h},1,12,"
                  f"0.9,{alt:.1f},M,46.9,M,,"),
        _checksum(f"GPRMC,000000,A,{lat_f},{lat_h},{lng_f},{lng_h},"
                  f"{speed_kn:.1f},{course:.1f},111126,,,A"),
        _checksum("GPGSA,A,3,04,05,09,12,24,25,29,02,,,,,1.8,0.9,1.5"),
        _checksum(f"GPVTG,{course:.1f},T,,M,{speed_kn:.1f},N,"
                  f"{speed_kmh:.1f},K,A"),
    )


class StubNmeaSerial(StubBase):
    """pty pair + NMEA writer; the slave's symlink is de_mavlink's port."""

    def __init__(self, argv):
        super().__init__(argv, "stub_nmea_serial")
        self.unit_dir = os.path.dirname(
            os.path.dirname(os.path.abspath(self.cfg_path)))
        self.link_path = os.path.join(self.unit_dir, "gnss_serial.tty")
        self._master_fd = None
        self.pose_feed = TruthPoseFeed(self.pose_port,
                                       sysid=self.pose_feed._sysid)

    def _open_link(self):
        master_fd, slave_fd = pty.openpty()
        # raw mode on the slave: no line editing, no echo of what
        # de_mavlink might write back
        attrs = termios.tcgetattr(slave_fd)
        attrs[3] &= ~(termios.ECHO | termios.ICANON | termios.ISIG)
        termios.tcsetattr(slave_fd, termios.TCSANOW, attrs)
        slave_name = os.ttyname(slave_fd)
        if os.path.lexists(self.link_path):
            os.unlink(self.link_path)
        os.symlink(slave_name, self.link_path)
        self._master_fd = master_fd
        self.log.log("PTY", {"slave": slave_name, "link": self.link_path},
                     ms=self.clock_ms())

    def run(self):
        self._open_link()
        super().run()

    def tick(self, t_s, frame):
        if self._master_fd is None or frame % EMIT_EVERY:
            return
        pose = self.pose_feed.pose()
        if pose is None:
            return
        for sentence in _sentences(pose):
            os.write(self._master_fd, (sentence + "\r\n").encode())


def main(argv):
    s = StubNmeaSerial(argv)
    if s.selftest:
        for s_ in _sentences({"lat": -35.363262, "lng": 149.165237,
                              "alt_m": 584.0, "vx": 0, "vy": 0,
                              "hdg": 90}):
            print(s_)
        return 0
    s.connect()
    s.run()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
