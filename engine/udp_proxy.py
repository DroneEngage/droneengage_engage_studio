#!/usr/bin/env python3
"""udp_proxy.py - one unit's module<->de_comm s2s chunk proxy (SL-0010).

simlib's gen_unit spawns one of these per unit module that opted in
(unit `udp_proxy: [<module>]`): the module's generated
s2s_udp_target_port points at this proxy's listen port instead of
de_comm's, the proxy relays datagrams upstream to de_comm's real port,
and de_comm's replies come back to this socket (it answers the
sender's address, de_broker/de_modules_manager.cpp) and are relayed on
to the module. One socket, both directions.

de_common chunks every databus message into datagrams of
[u16le chunk_no][payload] with 0xffff marking the last chunk
(de_common/de_databus/udpClient.cpp sendMSG); the receiver concatenates
in ARRIVAL order and clears on chunk 0 (InternalReceiverEntry) - it has
no index placement and no dedup, so dropping any chunk kills a
multi-chunk message, and a duplicated or reordered chunk inside one
corrupts it. The proxy reads that same header to keep the faults
honest:

    drop_pct  % of DATAGRAMS dropped - chunk-level loss; a lost chunk
              of a multi-chunk train loses the message (the receiver
              resyncs at the next chunk 0)
    dup_pct   % of complete MESSAGES emitted twice - replaying a chunk
              inside a train is corruption, not duplication, so only a
              message seen whole (a lone 0xffff datagram) is replayed
    reorder   window of complete messages: up to N are held, each new
              arrival releases a seeded pick; multi-chunk trains pass
              through in order and flush the window first so buffered
              datagrams can never interleave one

Datagrams that are not from the upstream and parse as one JSON object
are control messages: {"set": {...knobs}}, {"clear": true},
{"stats": true} - answered with a JSON state line (`./sim ctl
udp_chunks` talks that protocol). Module datagrams always open with
the 2-byte binary chunk header, so they can never parse as JSON; any
non-upstream datagram that is not control is module traffic and the
peer address is (re)learned from it.
"""

import argparse
import json
import random
import select
import socket
import sys
import time

CHUNK_END = 0xFFFF      # de_common's last-chunk marker
FLUSH_S = 0.06          # reorder hold deadline (chunks arrive ~10ms apart)
CTL_S = 0.05            # select tick when no flush is pending


def _addr(s):
    h, _, p = s.rpartition(":")
    return (h, int(p))


class Proxy:
    def __init__(self, listen, upstream, peer, label):
        self.upstream = upstream
        self.peer = peer
        self.label = label
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(listen)
        self.knobs = {"drop_pct": 0.0, "dup_pct": 0.0,
                      "reorder": 0, "seed": 0}
        self.rng = random.Random(0)
        # direction state: counters, the reorder buffer (complete
        # messages, [(data, copies)]), whether a multi-chunk train is
        # open, a log seq
        self.dir = {d: {"rx": 0, "fwd": 0, "drop": 0, "dup": 0,
                        "reord": 0, "buf": [], "train": False, "seq": 0}
                    for d in ("up", "down")}
        self.log(f"listen {listen[0]}:{listen[1]} "
                 f"upstream {upstream[0]}:{upstream[1]} "
                 f"peer {peer[0]}:{peer[1]}")

    def log(self, what):
        print(f"UDPXY {self.label} {what}", flush=True)

    @staticmethod
    def _chunk_no(data):
        """de_common's 2-byte little-endian chunk header, or None."""
        if len(data) < 2:
            return None
        return data[0] | (data[1] << 8)

    def _send(self, st, d, data, tag):
        self.sock.sendto(data, self.upstream if d == "up" else self.peer)
        st["fwd"] += 1
        st["seq"] += 1
        n = self._chunk_no(data)
        ch = "end" if n == CHUNK_END else n
        self.log(f"{d} seq={st['seq']} ch={ch} len={len(data)} {tag}")

    def _flush(self, st, d):
        """Emit every buffered message (arrival order on drain)."""
        buf, st["buf"] = st["buf"], []
        for data, copies in buf:
            self._send(st, d, data, "emit")
            for _ in range(copies - 1):
                self._send(st, d, data, "dup")

    def _release(self, st, d):
        """The window filled: emit a seeded pick from it - that is the
        reorder, a later message can leave before an earlier one. A
        nonzero pick is the reorder itself, tagged `emitr` so a log
        grep can prove out-of-order emission."""
        i = self.rng.randrange(len(st["buf"]))
        data, copies = st["buf"].pop(i)
        if i:
            st["reord"] += 1
        self._send(st, d, data, "emitr" if i else "emit")
        for _ in range(copies - 1):
            self._send(st, d, data, "dup")

    def on_data(self, d, data):
        st = self.dir[d]
        st["rx"] += 1
        if st["rx"] % 2000 == 0:
            self.log(f"{d} rx={st['rx']} fwd={st['fwd']} "
                     f"drop={st['drop']} dup={st['dup']} "
                     f"reord={st['reord']}")
        n = self._chunk_no(data)
        # a lone 0xffff datagram is a whole message; a multi-chunk
        # train stays open from its first non-end chunk to its 0xffff
        solo = n == CHUNK_END and not st["train"]
        if n is not None:
            st["train"] = n != CHUNK_END
        # drop: every datagram is fair game - per-chunk loss is the
        # fault; a mid-train drop loses the whole message, an end drop
        # merges nothing because the receiver resyncs on chunk 0
        if self.knobs["drop_pct"] and \
                self.rng.random() * 100 < self.knobs["drop_pct"]:
            st["drop"] += 1
            st["seq"] += 1
            ch = "end" if n == CHUNK_END else n
            self.log(f"{d} seq={st['seq']} ch={ch} len={len(data)} drop")
            return
        if solo:
            copies = 1
            if self.knobs["dup_pct"] and \
                    self.rng.random() * 100 < self.knobs["dup_pct"]:
                copies = 2
                st["dup"] += 1
            win = int(self.knobs["reorder"])
            if win > 1:
                st["buf"].append((data, copies))
                if len(st["buf"]) >= win:
                    self._release(st, d)
                return
            self._send(st, d, data, "fwd")
            for _ in range(copies - 1):
                self._send(st, d, data, "dup")
            return
        # train chunk or unparseable: ordered pass-through. A train
        # start drains the window first so a held message can never
        # interleave the train's chunks on the wire.
        if n != CHUNK_END and st["buf"]:
            self._flush(st, d)
        self._send(st, d, data, "fwd")

    @staticmethod
    def _is_control(data):
        """A ctl datagram is one JSON object holding a known verb."""
        try:
            req = json.loads(data.decode())
        except (ValueError, UnicodeDecodeError):
            return None
        if isinstance(req, dict) and \
                ("set" in req or req.get("clear") or req.get("stats")):
            return req
        return None

    def on_control(self, src, req):
        if "set" in req:
            for k in ("drop_pct", "dup_pct", "reorder", "seed"):
                if k in req["set"]:
                    self.knobs[k] = req["set"][k]
            self.rng = random.Random(self.knobs["seed"])
            for d in ("up", "down"):
                if self.dir[d]["buf"]:
                    self._flush(self.dir[d], d)
            self.log(f"set {json.dumps(self.knobs, sort_keys=True)}")
        elif req.get("clear"):
            self.knobs.update({"drop_pct": 0.0, "dup_pct": 0.0,
                               "reorder": 0})
            for d in ("up", "down"):
                st = self.dir[d]
                if st["buf"]:
                    self._flush(st, d)
            self.log("clear")
        stats = {d: {k: st[k] for k in ("rx", "fwd", "drop", "dup",
                                        "reord")}
                 for d, st in self.dir.items()}
        ans = {"ok": True, "knobs": self.knobs, "stats": stats,
               "listen": self.sock.getsockname()[1]}
        self.sock.sendto(json.dumps(ans).encode(), src)

    def run(self):
        while True:
            # the reorder flush deadline doubles as the select timeout
            delay = CTL_S
            for st in self.dir.values():
                if st["buf"]:
                    delay = min(delay, FLUSH_S)
            r, _, _ = select.select([self.sock], [], [], delay)
            now = time.time()
            if r:
                data, src = self.sock.recvfrom(65536)
                if src == self.upstream:
                    self.on_data("down", data)
                elif self._is_control(data) is not None:
                    self.on_control(src, self._is_control(data))
                else:
                    if src != self.peer:
                        self.log(f"peer -> {src[0]}:{src[1]}")
                        self.peer = src
                    self.on_data("up", data)
            for d in ("up", "down"):
                st = self.dir[d]
                # a partial window flushes on the deadline: the hold is
                # a reorder, not a stall. While a train is open the
                # window waits - emitting now would interleave it.
                if st["buf"] and not st["train"]:
                    st["deadline"] = st.get("deadline", now + FLUSH_S)
                    if now >= st["deadline"]:
                        st.pop("deadline")
                        self._flush(st, d)
                elif not st["buf"]:
                    st.pop("deadline", None)


def main(argv):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--listen", required=True, help="ip:port to bind")
    p.add_argument("--upstream", required=True, help="de_comm ip:port")
    p.add_argument("--peer", required=True, help="the module's ip:port")
    p.add_argument("--label", default="", help="log tag (unit.module)")
    a = p.parse_args(argv)
    Proxy(_addr(a.listen), _addr(a.upstream), _addr(a.peer),
          a.label).run()


if __name__ == "__main__":
    main(sys.argv[1:])
