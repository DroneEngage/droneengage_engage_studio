#!/usr/bin/env python3
"""fake_ntrip_caster.py - a local NTRIP v1 caster for tests (SL-0233).

    python3 tools/fake_ntrip_caster.py --port 2101 --mount STUDIO \\
        --user u --password p [--fixture f.bin] [--log caster.log]

A client sends  GET /<mount> HTTP/1.0  with  Authorization: Basic ...
The caster answers:

    unknown mountpoint   "HTTP/1.0 404 Not Found"
    bad user or password "HTTP/1.0 401 Unauthorized"   (then closes)
    good request         "ICY 200 OK"  then the fixture bytes

The fixture is the RTCM3 stream of tests/gnss/fixtures (generated when
no --fixture file is given), sent in chunks of --chunk bytes every
--interval seconds; --loop repeats it. After the fixture the connection
stays open, like a real caster.

NTRIP clients send their position as NMEA GGA lines. The caster logs
each GGA line it gets:

    <unix_ms> GGA <line>
    <unix_ms> AUTH_FAIL <peer>     (401)   <unix_ms> NO_MOUNT <path>

Only 127.0.0.1 is bound by default. No real station data.
"""

import argparse
import base64
import os
import socket
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "tests", "gnss",
                                "fixtures"))
import rtcm_fixture  # noqa: E402

OK_REPLY = b"ICY 200 OK\r\n\r\n"
UNAUTH_REPLY = (b"HTTP/1.0 401 Unauthorized\r\n"
                b"WWW-Authenticate: Basic realm=\"NTRIP\"\r\n"
                b"Content-Length: 0\r\n\r\n")
NOT_FOUND_REPLY = b"HTTP/1.0 404 Not Found\r\nContent-Length: 0\r\n\r\n"


def now_ms():
    return int(time.time() * 1000)


class FakeCaster:
    def __init__(self, port=2101, mount="STUDIO", user="user",
                 password="pass", fixture=None, chunk=360, interval=1.0,
                 loop=False, log_path=None, bind="127.0.0.1"):
        self.port = port
        self.bind = bind
        self.mount = mount.lstrip("/")
        self.user = user
        self.password = password
        self.data = rtcm_fixture.load_or_build(fixture)
        self.chunk = max(1, int(chunk))
        self.interval = interval
        self.loop = loop
        self.log_path = log_path
        self.gga = []                 # the GGA lines, in order
        self.stopping = threading.Event()
        self.sock = None
        self._lock = threading.Lock()

    # --------------------------------------------------------------- log

    def log(self, kind, text):
        with self._lock:
            if kind == "GGA":
                self.gga.append(text)
            if not self.log_path:
                return
            try:
                with open(self.log_path, "a") as f:
                    f.write(f"{now_ms()} {kind} {text}\n")
            except OSError:
                pass

    # ------------------------------------------------------------ server

    def start(self):
        """Bind and serve in a thread; returns the bound port (port 0
        picks a free one)."""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.bind, self.port))
        s.listen(8)
        s.settimeout(0.5)
        self.sock = s
        self.port = s.getsockname()[1]
        threading.Thread(target=self._accept_loop, daemon=True).start()
        return self.port

    def stop(self):
        self.stopping.set()
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass

    def serve_forever(self):
        self.start()
        try:
            while not self.stopping.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        self.stop()

    def _accept_loop(self):
        while not self.stopping.is_set():
            try:
                conn, addr = self.sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            threading.Thread(target=self._client, args=(conn, addr),
                             daemon=True).start()

    # ------------------------------------------------------------ client

    @staticmethod
    def parse_request(head):
        """(path, user, password) from the request head; user and
        password are None when the Basic header is missing or bad."""
        lines = head.decode("latin-1").split("\r\n")
        parts = lines[0].split()
        path = parts[1] if len(parts) > 1 and parts[0] == "GET" else None
        user = pw = None
        for ln in lines[1:]:
            if ln.lower().startswith("authorization:"):
                val = ln.split(":", 1)[1].strip()
                if val.lower().startswith("basic "):
                    try:
                        raw = base64.b64decode(val[6:]).decode("latin-1")
                        user, _, pw = raw.partition(":")
                    except ValueError:
                        pass
        return path, user, pw

    def _client(self, conn, addr):
        peer = f"{addr[0]}:{addr[1]}"
        conn.settimeout(5)
        try:
            head = b""
            while b"\r\n\r\n" not in head and len(head) < 8192:
                chunk = conn.recv(1024)
                if not chunk:
                    return
                head += chunk
            head, _, rest = head.partition(b"\r\n\r\n")
            path, user, pw = self.parse_request(head)
            if path is None or path.lstrip("/") != self.mount:
                self.log("NO_MOUNT", str(path))
                conn.sendall(NOT_FOUND_REPLY)
                return
            if user != self.user or pw != self.password:
                self.log("AUTH_FAIL", peer)
                conn.sendall(UNAUTH_REPLY)
                return
            conn.sendall(OK_REPLY)
            self.log("CONNECT", peer)
            reader = threading.Thread(target=self._read_gga,
                                      args=(conn, rest), daemon=True)
            reader.start()
            self._send_fixture(conn)
            while not self.stopping.is_set() and reader.is_alive():
                time.sleep(0.2)
        except OSError:
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _send_fixture(self, conn):
        while not self.stopping.is_set():
            for off in range(0, len(self.data), self.chunk):
                conn.sendall(self.data[off:off + self.chunk])
                if self.stopping.wait(self.interval):
                    return
            if not self.loop:
                return

    def _read_gga(self, conn, buf):
        """Log each NMEA GGA line the client sends."""
        conn.settimeout(1.0)
        buf = bytearray(buf)
        while not self.stopping.is_set():
            while b"\n" in buf:
                i = buf.index(b"\n")
                line = bytes(buf[:i]).decode("latin-1").strip()
                del buf[:i + 1]
                if len(line) > 6 and line[3:6] == "GGA" and \
                        line[0] == "$":
                    self.log("GGA", line)
            try:
                chunk = conn.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            buf += chunk


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", type=int, default=2101)
    p.add_argument("--bind", default="127.0.0.1")
    p.add_argument("--mount", default="STUDIO")
    p.add_argument("--user", default="user")
    p.add_argument("--password", default="pass")
    p.add_argument("--fixture", help="RTCM3 file (default: generated)")
    p.add_argument("--chunk", type=int, default=360)
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--loop", action="store_true")
    p.add_argument("--log", help="log file (GGA lines, auth failures)")
    a = p.parse_args(argv)
    c = FakeCaster(a.port, a.mount, a.user, a.password, a.fixture,
                   a.chunk, a.interval, a.loop, a.log, a.bind)
    print(f"fake NTRIP caster on {a.bind}:{a.port} mount /{c.mount}",
          flush=True)
    c.serve_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
