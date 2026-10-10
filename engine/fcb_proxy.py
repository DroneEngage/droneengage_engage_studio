#!/usr/bin/env python3
"""fcb_proxy.py - MAVLink-aware TCP proxy between de_mavlink and SITL's
fcb (serial1) port (TASK-06 / SL-0008).

Spawned per FC unit by simlib.gen_unit when the sim or the unit sets
`fcb_proxy: true`: de_mavlink's fcb_connection_uri points at --listen
and the proxy forwards to --target (the real fcb port). One proxy per
unit, so a fault can only ever touch this unit's own link. The process
is pidfile-tracked like every other sim proc and `sim down` kills it.

A second TCP port (--ctrl) takes one-line commands (the sim's server
control-port convention; ctl.py `fcb_*` commands talk here):

    cut <s>                     close both sides; new clients are accepted
                                then dropped until <s> expires (de_mavlink
                                sees a connect + a remote close, never a
                                refused handshake - 5 refused connects
                                would make it exit)
    stall <ms>                  hold the link: nothing is pumped until the
                                pause elapses, then buffered traffic flows
    corrupt <byte_drop%> <bit_flip%> <seed>
                                frame-aware noise model: a lost UART byte
                                kills its frame, so a parsed MAVLink
                                frame of length L is dropped whole with
                                1-(1-p)^L probability (the exact rate a
                                p% byte-loss channel produces), and one
                                random bit inside a frame's payload/CRC
                                is flipped with flip%/100 probability -
                                the header is never touched so the frame
                                still parses and dies on the receiver's
                                CRC (one flip = one doomed frame, no
                                stream desync). `seed` makes a run
                                deterministic.
    drop <msgid|all> <pct>      drop only frames of <msgid> with pct/100
                                probability; 'all' clears the msgid rules
    inject <hex>                write raw bytes towards de_mavlink (the
                                TASK-14 world ADS-B hook)
    clear                       remove corrupt/drop rules and end any
                                running cut/stall
    stats                       counters + the active rules
    quit                        close the control connection

Only 127.0.0.1 is bound. One client at a time: a new client replaces the
old one (de_mavlink reconnects on its own retry loop). With no rules
active the pumps pass bytes through untouched - zero protocol risk for
the nominal path.
"""

import argparse
import random
import socket
import sys
import threading
import time

STX_V1, STX_V2 = 0xFE, 0xFD


def _frame_len(buf, i):
    """Total length of the MAVLink frame starting at buf[i].

    0  = buf[i] is not a MAVLink STX
    -1 = a frame starts here but its bytes are not all in buf yet
    """
    n = len(buf) - i
    stx = buf[i]
    if stx == STX_V1:
        # stx len seq sysid compid msgid payload[len] crc(2)
        if n < 2:
            return -1
        need = 8 + buf[i + 1]
        return need if n >= need else -1
    if stx == STX_V2:
        # stx len incompat compat seq sysid compid msgid(3) payload[len]
        # crc(2) + signature(13) when incompat bit 0 is set
        if n < 3:
            return -1
        need = 12 + buf[i + 1] + (13 if buf[i + 2] & 0x01 else 0)
        return need if n >= need else -1
    return 0


def _frame_msgid(buf, i):
    """msgid of the frame starting at buf[i] (v1: 1 byte, v2: 3)."""
    if buf[i] == STX_V1:
        return buf[i + 5]
    return buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)


class FcbProxy:
    def __init__(self, listen_port, target_host, target_port, ctrl_port,
                 unit, seed):
        self.listen_port = listen_port
        self.target = (target_host, target_port)
        self.ctrl_port = ctrl_port
        self.unit = unit
        self.seed = seed
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        # the one client session: (client_sock, upstream_sock, generation)
        self.client = None
        self.generation = 0
        self.cut_until = 0.0         # drop every connection before this time
        self.stall_until = 0.0       # pump nothing before this time
        self.rules = {"drop_pct": 0.0, "flip_pct": 0.0, "msgids": {}}
        # a seeded RNG per direction: c2s = client->SITL, s2c = SITL->client
        self.rngs = {"c2s": random.Random(seed),
                     "s2c": random.Random(seed + 7919)}
        self.stats = {"connects": 0, "refused": 0,
                      "c2s": {"frames": 0, "bytes": 0, "dropped": 0,
                              "flipped": 0},
                      "s2c": {"frames": 0, "bytes": 0, "dropped": 0,
                              "flipped": 0}}

    def log(self, msg):
        print(f"[{self.unit}] {msg}", flush=True)

    # ------------------------------------------------------------ client side

    def _close_client(self, why):
        """Tear down the current session (both sockets); the pumps exit
        when their generation no longer matches."""
        with self.lock:
            c = self.client
            self.client = None
            self.generation += 1
        if c is not None:
            client_sock, upstream_sock, _ = c
            for s in (client_sock, upstream_sock):
                try:
                    s.close()
                except OSError:
                    pass
            self.log(f"session closed ({why})")

    def _take_client(self, client_sock, addr):
        """New de_mavlink connection: replace any old session and open
        the upstream to SITL's fcb port."""
        self._close_client("replaced by new client")
        upstream = None
        t0 = time.time()
        # SITL may still be booting (unit reset) - retry briefly, then let
        # de_mavlink's own reconnect loop drive further attempts
        while time.time() - t0 < 10 and not self.stopping.is_set():
            try:
                upstream = socket.create_connection(self.target, 2)
                break
            except OSError:
                time.sleep(0.5)
        if upstream is None or time.time() < self.cut_until:
            # no SITL yet, or a cut started while the upstream connect
            # retried - the client is dropped either way
            self.log(f"upstream {self.target[0]}:{self.target[1]} "
                     "unreachable - dropping client"
                     if upstream is None else "cut running - dropping client")
            for s in (client_sock, upstream):
                if s is None:
                    continue
                try:
                    s.close()
                except OSError:
                    pass
            return
        for s in (client_sock, upstream):
            s.settimeout(1.0)
        with self.lock:
            self.generation += 1
            self.client = (client_sock, upstream, self.generation)
            gen = self.generation
            self.stats["connects"] += 1
        self.log(f"client {addr[0]}:{addr[1]} -> "
                 f"{self.target[0]}:{self.target[1]}")
        for direction, src, dst in (
                ("c2s", client_sock, upstream),
                ("s2c", upstream, client_sock)):
            t = threading.Thread(target=self._pump,
                                 args=(gen, direction, src, dst),
                                 daemon=True)
            t.start()

    # ------------------------------------------------------------- pumping

    def _pump(self, gen, direction, src, dst):
        """Move bytes src -> dst until the session ends. A cut/stall is
        checked per loop; corruption is applied frame-aware in _shape."""
        buf = bytearray()
        stalled = False
        while not self.stopping.is_set():
            with self.lock:
                alive = self.client is not None and \
                    self.client[2] == gen
                stall = self.stall_until
                cut = self.cut_until
            if not alive:
                return
            now = time.time()
            if now < stall:
                # hold: do not read either side - the kernel buffers fill
                # like a frozen UART; what arrived flows on resume
                if not stalled:
                    stalled = True
                    self.log(f"{direction}: link held")
                time.sleep(min(0.1, stall - now))
                continue
            if stalled:
                stalled = False
                self.log(f"{direction}: resumed")
            if now < cut:
                return          # the cut's closer owns the teardown
            try:
                data = src.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                self._close_client(f"{direction} read error")
                return
            if not data:
                self._close_client(f"{direction} peer closed")
                return
            buf += data
            out = self._shape(buf, direction)
            if out:
                try:
                    dst.sendall(out)
                except OSError:
                    self._close_client(f"{direction} write error")
                    return

    def _shape(self, buf, direction):
        """Apply the corruption rules to buf's complete frames; consume
        what was forwarded. Rules empty -> pass everything through."""
        rules = self.rules
        if not rules["drop_pct"] and not rules["flip_pct"] \
                and not rules["msgids"]:
            out = bytes(buf)
            buf.clear()
            return out
        rng = self.rngs[direction]
        st = self.stats[direction]
        out = bytearray()
        i = 0
        while i < len(buf):
            fl = _frame_len(buf, i)
            if fl < 0:
                break               # incomplete frame - wait for more
            if fl == 0:
                # not a STX byte: resync by forwarding it verbatim - the
                # stream stays honest to whatever the peer emitted
                out.append(buf[i])
                i += 1
                continue
            msgid = _frame_msgid(buf, i)
            st["frames"] += 1
            st["bytes"] += fl
            # byte_drop_pct is a per-BYTE loss rate: a lost byte kills
            # its frame, so this frame dies with 1-(1-p)^fl - the exact
            # frame-loss rate the same channel would show on a UART
            bp = rules["drop_pct"] / 100.0
            drop = bp > 0 and rng.random() < 1.0 - (1.0 - bp) ** fl
            pct = rules["msgids"].get(msgid, rules["msgids"].get("*"))
            if pct is not None:
                drop = drop or rng.random() * 100.0 < pct
            if drop:
                st["dropped"] += 1
            else:
                frame = bytes(buf[i:i + fl])
                if rng.random() * 100.0 < rules["flip_pct"]:
                    # flip one bit in payload/CRC/signature - never the
                    # header (stx..msgid), so the frame keeps its length,
                    # we never desync, and the receiver drops it on CRC
                    hdr = 6 if buf[i] == STX_V1 else 10
                    b = bytearray(frame)
                    k = rng.randrange(hdr, len(b))
                    b[k] ^= 1 << rng.randrange(8)
                    frame = bytes(b)
                    st["flipped"] += 1
                out += frame
            i += fl
        del buf[:i]
        return bytes(out)

    # ------------------------------------------------------- control port

    def _cmd(self, line):
        """One control line -> reply text (without the trailing OK/ERR)."""
        parts = line.split()
        if not parts:
            return "OK"
        c = parts[0]
        now = time.time()
        if c == "cut":
            secs = float(parts[1])
            self.cut_until = now + secs
            self._close_client(f"cut {secs:g}s")
            self.log(f"cut {secs:g}s (clients refused until "
                     f"{time.strftime('%H:%M:%S', time.localtime(self.cut_until))})")
            return f"cut {secs:g}s"
        if c == "stall":
            ms = float(parts[1])
            self.stall_until = now + ms / 1000.0
            self.log(f"stall {ms:g}ms")
            return f"stall {ms:g}ms"
        if c == "corrupt":
            drop = float(parts[1])
            flip = float(parts[2]) if len(parts) > 2 else 0.0
            seed = int(parts[3]) if len(parts) > 3 else self.seed
            self.rules["drop_pct"] = drop
            self.rules["flip_pct"] = flip
            self.rngs = {"c2s": random.Random(seed),
                         "s2c": random.Random(seed + 7919)}
            self.log(f"corrupt drop={drop:g}% flip={flip:g}% seed={seed}")
            return f"corrupt drop={drop:g}% flip={flip:g}% seed={seed}"
        if c == "drop":
            if parts[1] == "all":
                self.rules["msgids"] = {}
                self.log("msgid drop rules cleared")
                return "msgid drop rules cleared"
            msgid = int(parts[1], 0)
            pct = float(parts[2]) if len(parts) > 2 else 100.0
            if pct <= 0:
                self.rules["msgids"].pop(msgid, None)
            else:
                self.rules["msgids"][msgid] = pct
            self.log(f"drop msgid {msgid} pct={pct:g}")
            return f"drop msgid {msgid} pct={pct:g}"
        if c == "inject":
            raw = bytes.fromhex(parts[1])
            with self.lock:
                c_sock = self.client[0] if self.client else None
            if c_sock is None:
                return "ERR no client connected"
            try:
                c_sock.sendall(raw)
            except OSError as e:
                return f"ERR inject failed: {e}"
            self.log(f"inject {len(raw)} bytes -> client")
            return f"inject {len(raw)} bytes"
        if c == "clear":
            self.rules = {"drop_pct": 0.0, "flip_pct": 0.0, "msgids": {}}
            self.cut_until = self.stall_until = 0.0
            self.log("clear")
            return "clear"
        if c == "stats":
            up, dn = self.stats["c2s"], self.stats["s2c"]
            state = []
            if now < self.cut_until:
                state.append(f"cut {self.cut_until - now:.1f}s left")
            if now < self.stall_until:
                state.append(f"stall {self.stall_until - now:.2f}s left")
            return (
                f"connects={self.stats['connects']} "
                f"refused={self.stats['refused']}\n"
                f"c2s frames={up['frames']} bytes={up['bytes']} "
                f"dropped={up['dropped']} flipped={up['flipped']}\n"
                f"s2c frames={dn['frames']} bytes={dn['bytes']} "
                f"dropped={dn['dropped']} flipped={dn['flipped']}\n"
                f"rules drop={self.rules['drop_pct']}% "
                f"flip={self.rules['flip_pct']}% "
                f"msgids={self.rules['msgids']}\n"
                f"{' '.join(state) or 'clean'}")
        if c == "quit":
            return "OK"
        return f"ERR unknown command {c!r}"

    def _ctrl_loop(self, ctrl_sock):
        while not self.stopping.is_set():
            try:
                conn, _addr = ctrl_sock.accept()
            except OSError:
                return
            threading.Thread(target=self._ctrl_conn, args=(conn,),
                             daemon=True).start()

    def _ctrl_conn(self, conn):
        """One control connection: line commands until the client
        closes. Replies follow the sim's control-port convention - text
        lines then a bare OK; failures as `ERR ...`."""
        conn.settimeout(30)
        buf = b""
        try:
            while True:
                try:
                    chunk = conn.recv(4096)
                except (socket.timeout, OSError):
                    return
                if not chunk:
                    return
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    reply = self._cmd(line.decode(errors="replace")
                                      .strip())
                    out = reply.encode() + b"\n"
                    if reply != "OK" and not reply.startswith("ERR"):
                        out += b"OK\n"
                    try:
                        conn.sendall(out)
                    except OSError:
                        return
        finally:
            try:
                conn.close()
            except OSError:
                pass

    # ----------------------------------------------------------------- main

    def run(self):
        listen = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listen.bind(("127.0.0.1", self.listen_port))
        listen.listen(4)
        ctrl = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ctrl.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ctrl.bind(("127.0.0.1", self.ctrl_port))
        ctrl.listen(4)
        threading.Thread(target=self._ctrl_loop, args=(ctrl,),
                         daemon=True).start()
        self.log(f"listen :{self.listen_port} -> "
                 f"{self.target[0]}:{self.target[1]}  ctrl :{self.ctrl_port}")
        try:
            while not self.stopping.is_set():
                client_sock, addr = listen.accept()
                if time.time() < self.cut_until:
                    # cut running: accept then drop - de_mavlink retries a
                    # successful-looking connect that closes, never a
                    # refused handshake (5 refused connects exit it)
                    self.stats["refused"] += 1
                    try:
                        client_sock.close()
                    except OSError:
                        pass
                    continue
                self._take_client(client_sock, addr)
        finally:
            self._close_client("proxy exiting")
            listen.close()
            ctrl.close()


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--unit", required=True)
    p.add_argument("--listen", type=int, required=True,
                   help="port de_mavlink connects to")
    p.add_argument("--target", required=True,
                   help="host:port of SITL's real fcb port")
    p.add_argument("--ctrl", type=int, required=True,
                   help="control port (cut/stall/corrupt/drop/inject)")
    p.add_argument("--statedir",
                   help="per-unit run dir (keeps the sim's var/run path "
                        "on the cmdline so stray-pid sweeps see us)")
    p.add_argument("--seed", type=int, default=1)
    a = p.parse_args()
    host, _, port = a.target.rpartition(":")
    FcbProxy(a.listen, host or "127.0.0.1", int(port), a.ctrl,
             a.unit, a.seed).run()


if __name__ == "__main__":
    sys.exit(main())
