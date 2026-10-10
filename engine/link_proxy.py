#!/usr/bin/env python3
"""link_proxy — per-unit byte-level link-fault proxy for `server: real` (SL-0009).

Sits between one unit's de_comm and the real comm server's wss port:

    de_comm --wss--> [listen <link_proxy>] --raw TCP--> droneengage_server:wss

TLS/ws/JSON all pass through untouched: the proxy relays bytes and drives
the same `link_model.Link` as the fake server, but at byte granularity
instead of per ws message. That reuses one seeded fault model for both
transports — offer() decides per direction, ordered queues hold stalled /
GE-stalled / delayed chunks, and stats work the same way.

Two semantic differences from the fake server (a byte stream, not
discrete messages):

  - A "drop" verdict (loss / GE bad-phase in `drop` mode) cannot skip a
    chunk inside a stream without desyncing it, so it kills the session —
    the byte link broke, de_comm re-dials exactly like a real hop losing
    the socket. `loss`/`loss_tx`/`loss_rx` (and profiles carrying loss
    knobs) are refused outright: message-level loss needs
    `server: {type: fake}` - the fake drops a whole ws frame before it is
    serialised, which is a different, still-useful fault.
  - `clear` purges held queues; if any bytes were mid-flight the stream
    has desynced, so the session is dropped along with them.

`cut` poisons the link for N seconds: the live session dies at once and
fresh dials are accepted then immediately dropped (de_comm sees a
connect + close, never a refused handshake — same convention as
engine/fcb_proxy.py). `stall`/`ge ... stall` hold bytes in order; a hold
longer than de_comm's ~10 s watchdog makes it drop the ws, and its
re-dial stays un-registered at the server until the hold ends and the
queue flushes — that is what makes the unit go offline/online at the
real server.

Control protocol mirrors the fake fleet server so ./sim ctl and scenario
`link` lines work verbatim:

    list                                   "d:<party>" when the session is
                                           live AND established (first
                                           server byte delivered) — a held
                                           re-dial reports offline, like an
                                           un-authed ws on the fake
    stats                                  per-party link stats
    kick <party>                           close the session (de_comm re-dials)
    link <party> delay <ms>                fixed latency each direction
    link <party> jitter <ms>               seeded +/- jitter on the delay
    link <party> bw <bytes_per_s>          bandwidth serialisation cap
    link <party> ge <p_gb> <p_bg> <loss_g> <loss_b> [drop|stall]
    link <party> stall <seconds>           hold everything, flush in order
    link <party> cut <seconds>             poison the link (see above)
    link <party> profile <name>            link_model.PROFILES preset
    link <party> clear                     remove all faults
    quit

Run directly (--once) for the unit tests; every test path is driven via
`./sim ctl` once a proxied sim is up.
"""

from __future__ import print_function

import argparse
import json
import os
import socket
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import link_model

# `link <party> <sub>` the byte proxy refuses — message-level faults that
# only the fake server can express (it drops a whole ws frame pre-TLS).
FAKE_ONLY_SUBS = ("loss", "loss_tx", "loss_rx")

FAKE_ONLY_MSG = ("needs `server: {type: fake}` - the byte proxy relays "
                 "TLS and cannot drop inside the stream")


class LinkProxy(object):
    """One de_comm session at a time; chunks relayed through the Link."""

    def __init__(self, unit, listen, ctrl, target, statedir, seed=0):
        self.unit = unit
        self.party = unit             # party_id, like the fake's links{}
        self.listen_port = listen
        self.ctrl_port = ctrl
        self.target = target          # (ip, wss port) of the real server
        self.statedir = statedir
        self.link = link_model.Link(self.party, seed)
        self.link.on_change = self._link_changed
        self.lock = threading.RLock()   # _deliver -> _close_session nests
        self.stopping = threading.Event()
        self.flush_ev = {"tx": threading.Event(), "rx": threading.Event()}
        # the one live session: (client_sock, upstream_sock, generation)
        self.client = None
        self.generation = 0
        self.established = False      # first rx chunk reached de_comm
        self.since = time.time()
        self.stats = {"conns": 0, "refused": 0, "killed": 0,
                      "tx_b": 0, "rx_b": 0}

    # ------------------------------------------------------------- pump

    def _take_client(self, client, addr):
        """New de_comm dial: kill any older session, dial upstream, pump."""
        self._close_session("new client")
        try:
            up = self._dial_upstream()
        except OSError as e:
            self.log("upstream %s:%d unreachable (%s) - client dropped"
                     % (self.target[0], self.target[1], e))
            client.close()
            return
        client.settimeout(1.0)
        up.settimeout(1.0)
        with self.lock:
            self.client = (client, up, self.generation)
            self.established = False
        self.stats["conns"] += 1
        self.log("client %s:%d -> %s:%d (gen %d)"
                 % (addr[0], addr[1], self.target[0], self.target[1],
                    self.generation))
        for direction, src in (("tx", client), ("rx", up)):
            t = threading.Thread(target=self._pump,
                                 args=(self.generation, direction, src))
            t.daemon = True
            t.start()

    def _dial_upstream(self, tries=20):
        """Dial the real comm server; de_comm may retry quickly."""
        last = None
        for _ in range(tries):
            try:
                return socket.create_connection(self.target, timeout=10)
            except OSError as e:
                last = e
                time.sleep(0.5)
        raise last

    def _pump(self, gen, direction, src):
        """Read one direction; Link.offer() decides each chunk's fate."""
        ln = self.link
        while not self.stopping.is_set():
            with self.lock:
                sess = self.client
            if sess is None or sess[2] != gen:
                return                      # session superseded/closed
            if time.time() < ln.cut_until:
                return                      # cut: closer owns the teardown
            try:
                data = src.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                self._close_session("%s read error" % direction)
                return
            if not data:
                self._close_session("%s peer closed" % direction)
                return
            now = time.time()
            if now < ln.cut_until:
                self._close_session("link cut")
                return
            verdict = ln.offer(direction, len(data), now, (gen, data))
            if verdict == "drop":
                # A byte stream cannot skip a chunk like a ws message can:
                # the link broke and de_comm re-dials, same as a real hop
                # losing the socket.
                self.stats["killed"] += 1
                self._close_session(
                    "%s: %dB dropped - byte link broke"
                    % (direction, len(data)))
                return

    def _deliver(self, direction, tag):
        """Write a released chunk to its socket; stale gen = dead session."""
        gen, chunk = tag
        with self.lock:
            sess = self.client
            if sess is None or sess[2] != gen:
                self.link.dir[direction].purged += 1
                return
            dst = sess[1] if direction == "tx" else sess[0]
            try:
                dst.sendall(chunk)
                self.stats["tx_b" if direction == "tx" else "rx_b"] += \
                    len(chunk)
                if direction == "rx" and not self.established:
                    self.established = True
                    self.log("session established (server answered)")
            except OSError:
                self._close_session("%s write error" % direction)

    def _flusher(self, direction):
        """Release due chunks in order; sleeps on release_time/on_change."""
        ln = self.link
        ev = self.flush_ev[direction]
        while not self.stopping.is_set():
            now = time.time()
            for tag in ln.pop_ready(direction, now):
                self._deliver(direction, tag)
            rel = ln.next_release(direction)
            ev.clear()
            if rel is None:
                ev.wait(1.0)                # empty or held head; offer wakes
            else:
                ev.wait(max(0.05, min(1.0, rel - now)))

    def _close_session(self, why):
        with self.lock:
            c = self.client
            self.client = None
            self.established = False
            if c is not None:
                self.generation += 1
        if c is not None:
            for s in c[:2]:
                try:
                    s.close()
                except OSError:
                    pass
            self.log("session closed (%s)" % why)

    def _link_changed(self, _party, _direction):
        self.flush_ev[_direction].set()

    # ------------------------------------------------------------ server

    def _accept_loop(self):
        ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ls.bind(("127.0.0.1", self.listen_port))
        ls.listen(4)
        while not self.stopping.is_set():
            client, addr = ls.accept()
            if time.time() < self.link.cut_until:
                # refuse by accept+drop: de_comm sees a closed conn and
                # re-dials (a refused handshake can make it exit)
                self.stats["refused"] += 1
                client.close()
                continue
            self._take_client(client, addr)

    def _session_online(self):
        with self.lock:
            return self.client is not None and self.established

    def _cmd(self, line):
        """One control line -> reply text (fake-server syntax)."""
        parts = line.split()
        if not parts:
            return "OK"
        c = parts[0]
        ln = self.link
        now = time.time()
        if c == "list":
            return "d:" + self.party if self._session_online() else "OK"
        if c == "stats":
            with self.lock:
                sess = "established" if self.established else \
                    ("held" if self.client else "none")
            return "%s: session=%s conns=%d refused=%d killed=%d %s" % (
                self.party, sess, self.stats["conns"],
                self.stats["refused"], self.stats["killed"],
                ln.stats_line(now, now - self.since))
        if c == "kick":
            target = parts[1] if len(parts) > 1 else ""
            if target not in ("all", self.party):
                return "ERR %s is not this proxy's party (%s)" % (
                    target, self.party)
            self._close_session("kick")
            return "kick %s: connection closed" % self.party
        if c == "link":
            if len(parts) < 3:
                return "ERR link needs <party> <fault> [args]"
            target, sub, rest = parts[1], parts[2], parts[3:]
            if target not in ("all", self.party):
                return "ERR %s is not this proxy's party (%s)" % (
                    target, self.party)
            if sub in FAKE_ONLY_SUBS:
                return "ERR link %s %s" % (sub, FAKE_ONLY_MSG)
            if sub == "delay":
                ln.delay_ms = int(rest[0]) if rest else 0
            elif sub == "jitter":
                ln.jitter_ms = int(rest[0]) if rest else 0
            elif sub == "bw":
                ln.bw_bps = int(rest[0]) if rest else 0
            elif sub == "ge":
                # ge <p_gb> <p_bg> <loss_g> <loss_b> [drop|stall]
                if len(rest) < 4:
                    return ("ERR ge needs <p_gb> <p_bg> <loss_g> <loss_b>"
                            " [drop|stall]")
                ln.set_ge(float(rest[0]), float(rest[1]),
                          float(rest[2]), float(rest[3]),
                          rest[4] if len(rest) > 4 else "drop")
            elif sub == "stall":
                ln.stall(float(rest[0]), now)
            elif sub == "cut":
                ln.cut(float(rest[0]), now)
                self._close_session("link cut")
            elif sub == "profile":
                name = rest[0] if rest else ""
                prof = link_model.PROFILES.get(name)
                if prof is None:
                    return "ERR unknown link profile %s" % name
                if any(k in prof for k in
                       ("loss", "loss_tx", "loss_rx")):
                    return "ERR profile %s %s" % (name, FAKE_ONLY_MSG)
                ln.apply_profile(name, now)
            elif sub == "clear":
                held = any(ln.dir[d].queue for d in ("tx", "rx"))
                ln.clear()
                if held:
                    # purged held bytes desynced the stream - drop the
                    # session rather than relay a hole
                    self.stats["killed"] += 1
                    self._close_session("clear purged held bytes")
            else:
                return "ERR unknown link fault %s" % sub
            return "link %s: %s" % (self.party,
                                    ln.stats_line(now, now - self.since))
        if c == "quit":
            return "OK"
        return "ERR unknown command '%s' - want list|stats|kick|link|quit" % c

    def _ctrl_loop(self):
        cs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        cs.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        cs.bind(("127.0.0.1", self.ctrl_port))
        cs.listen(4)
        while not self.stopping.is_set():
            conn, _ = cs.accept()
            conn.settimeout(10)
            f = conn.makefile("r")
            # reply per line like the fake server's say(): ctl reads right
            # after sending - it does not half-close
            for line in f:
                line = line.rstrip("\r\n")
                if not line:
                    continue
                reply = self._cmd(line)
                try:
                    if reply and reply != "OK":
                        conn.sendall((reply + "\nOK\n").encode())
                    else:
                        conn.sendall(b"OK\n")
                except OSError:
                    break
                if line == "quit":
                    break
            try:
                conn.close()
            except OSError:
                pass

    # -------------------------------------------------------------------

    def log(self, msg):
        print("%s linkpx[%s] %s" %
              (time.strftime("%H:%M:%S"), self.unit, msg), flush=True)

    def run(self):
        for direction in ("tx", "rx"):
            t = threading.Thread(target=self._flusher, args=(direction,))
            t.daemon = True
            t.start()
        t = threading.Thread(target=self._ctrl_loop)
        t.daemon = True
        t.start()
        self.log("listen :%d ctrl :%d -> %s:%d"
                 % (self.listen_port, self.ctrl_port,
                    self.target[0], self.target[1]))
        self._accept_loop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--unit", required=True)
    ap.add_argument("--listen", type=int, required=True,
                    help="TCP port de_comm dials")
    ap.add_argument("--ctrl", type=int, required=True,
                    help="control port (fake-server line protocol)")
    ap.add_argument("--target", required=True,
                    help="real comm server endpoint ip:port")
    ap.add_argument("--statedir", required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--once", action="store_true",
                    help="self-test: relay a byte pattern, print stats, exit")
    args = ap.parse_args()
    host, _, port = args.target.rpartition(":")
    px = LinkProxy(args.unit, args.listen, args.ctrl,
                   (host, int(port)), args.statedir, seed=args.seed)
    if args.once:
        # self-test used by engine/test_checks.py: relay through a stall
        # and a jitter profile, then dump stats - no comm server needed.
        import socketserver

        class _Echo(socketserver.BaseRequestHandler):
            def handle(self):
                while True:
                    b = self.request.recv(4096)
                    if not b:
                        return
                    self.request.sendall(b)

        echo = socketserver.TCPServer(("127.0.0.1", 0), _Echo)
        threading.Thread(target=echo.serve_forever, daemon=True).start()
        px.target = ("127.0.0.1", echo.server_address[1])
        t = threading.Thread(target=px.run, daemon=True)
        t.start()
        time.sleep(0.3)
        c = socket.create_connection(("127.0.0.1", args.listen), timeout=5)
        payload = b"linkpx-once " * 500
        px.link.stall(0.4, time.time())
        c.sendall(payload)
        got = b""
        c.settimeout(3)
        while len(got) < len(payload):
            got += c.recv(65536)
        assert got == payload, "stall relay corrupted the byte stream"
        px.link.bw_bps = 1_000_000
        px.link.jitter_ms = 5
        c.sendall(payload[:1000])
        c.recv(1000)
        c.close()
        echo.shutdown()
        print(px._cmd("stats"))
        assert px.stats["tx_b"] >= len(payload)
        assert px.stats["rx_b"] >= len(payload)
        print("link_proxy once: OK")
        return
    px.run()


if __name__ == "__main__":
    main()
