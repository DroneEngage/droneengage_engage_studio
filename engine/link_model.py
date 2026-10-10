#!/usr/bin/env python3
"""link_model.py - per-party link impairment model for the fake fleet
server (TASK-05/SL-0007).

The unit<->server hop is a WebSocket on TCP: on a bad 4G link frames are
not cleanly lost - they stall (head-of-line blocking), arrive in bursts,
and the two directions degrade differently. Link holds the fault state
of one party's link and decides, per offered frame, what happens to it:

  drop    - gone (cut / loss / GE bad state in drop mode)
  queue   - released at a computed time: fixed delay + seeded jitter,
            a directed `stall <s>` window (held, not dropped), or a
            bandwidth cap that serializes bytes through an ordered queue
  hold    - GE bad state in stall mode: parked until the chain flips
            back to good, then released in order (the link stays
            "connected" - no frames pass - so only the peer's own
            watchdog notices; the old flat `cut` drops the pipe instead)

Directions are unit-centric: "tx" is what the unit sends (unit->server,
the uplink), "rx" what it receives (server->unit, the command downlink).
A flat `loss` applies to both; `loss_tx`/`loss_rx` layer on top.

Every frame that is not dropped lands in the direction's ordered queue;
pop_ready() hands out the due head(s) in offer order. The queue and the
release clock are pure python - the server passes time.time(), the unit
test passes a virtual clock. Seeded per-direction RNG streams make the
drop/stall decisions reproducible: same seed, same counters.
"""

import random
from collections import deque

DIRECTIONS = ("tx", "rx")

# named profiles for `link <party> profile <name>` and the scenario's
# `fault: link_profile`. uplink/downlink are read from the UNIT: a dead
# uplink starves the unit's tx (it can still be commanded), a dead
# downlink starves its rx (it keeps talking, hears nothing back).
PROFILES = {
    # bad 4G at the cell edge: GE burst loss both ways + jitter
    "lte_edge": {"delay_ms": 40, "jitter_ms": 120,
                 "ge": {"p_gb": 0.08, "p_bg": 0.35, "loss_g": 1.0,
                        "loss_b": 85.0, "mode": "drop"}},
    # the same edge with head-of-line stalls instead of drops
    "lte_stall": {"jitter_ms": 60,
                  "ge": {"p_gb": 0.05, "p_bg": 0.20, "loss_g": 0.0,
                         "loss_b": 0.0, "mode": "stall"}},
    # the unit receives nothing; its telemetry still flows
    "uplink_dead": {"loss_tx": 0.0, "loss_rx": 100.0},
    # a thin pipe: bytes serialize through the ordered queue
    "dial_up": {"bw_bps": 2400, "jitter_ms": 40},
}


class GilbertElliot:
    """2-state good/bad Markov chain, one tick per offered frame.
    Long-run bad occupancy -> p_gb / (p_gb + p_bg)."""
    __slots__ = ("p_gb", "p_bg", "loss_g", "loss_b", "mode", "rng",
                 "bad", "ticks", "bad_ticks")

    def __init__(self, rng, p_gb, p_bg, loss_g, loss_b, mode):
        self.p_gb = float(p_gb)
        self.p_bg = float(p_bg)
        self.loss_g = float(loss_g)
        self.loss_b = float(loss_b)
        self.mode = mode                      # "drop" | "stall"
        self.rng = rng
        self.bad = False
        self.ticks = 0
        self.bad_ticks = 0

    def tick(self):
        self.ticks += 1
        if self.bad:
            if self.rng.random() < self.p_bg:
                self.bad = False
        elif self.rng.random() < self.p_gb:
            self.bad = True
        if self.bad:
            self.bad_ticks += 1
        return self.bad


class _Dir:
    """One direction's impairment state + counters."""
    __slots__ = ("loss", "ge", "rng", "queue", "next_free", "last_rel",
                 "dropped", "delayed", "held", "delivered", "purged")

    def __init__(self, rng):
        self.loss = 0.0              # per-direction loss %
        self.ge = None               # GilbertElliot or None
        self.rng = rng
        self.queue = deque()         # {"rel": float|None, "size", "tag"}
        self.next_free = 0.0         # bw serialization watermark
        self.last_rel = 0.0          # monotone release clamp (ordering)
        self.dropped = 0
        self.delayed = 0             # frames scheduled with rel > now
        self.held = 0                # frames parked by a stall
        self.delivered = 0           # frames handed out by pop_ready
        self.purged = 0              # queue entries deleted on clear


class Link:
    """Fault state of one party's server link (both directions)."""
    __slots__ = ("party", "loss", "delay_ms", "jitter_ms", "bw_bps",
                 "cut_until", "stall_until", "dir", "on_change",
                 "roster_bytes", "roster_msgs")

    def __init__(self, party, seed):
        self.party = party
        self.loss = 0.0              # flat loss on both directions, %
        self.delay_ms = 0
        self.jitter_ms = 0
        self.bw_bps = 0              # bytes/s cap per direction; 0 = off
        self.cut_until = 0.0
        self.stall_until = 0.0
        # one stream per direction so uplink volume can't shift the
        # downlink drop pattern
        self.dir = {d: _Dir(random.Random(f"{seed}:{party}:{d}"))
                    for d in DIRECTIONS}
        self.on_change = None        # server hook: fn(party, direction)
        # P4A-07 bandwidth: 1092 bytes/ms offered to this link
        self.roster_bytes = 0
        self.roster_msgs = 0

    # ------------------------------------------------------------ knobs

    def is_cut(self, now):
        return now < self.cut_until

    def cut(self, seconds, now):
        self.cut_until = now + float(seconds)
        self._notify("tx")
        self._notify("rx")

    def stall(self, seconds, now):
        """Directed in-order hold of ALL traffic for `seconds` - like
        `cut` but bytes are buffered, not dropped (a hard stall)."""
        self.stall_until = max(self.stall_until, now + float(seconds))
        self._notify("tx")
        self._notify("rx")

    def set_ge(self, p_gb, p_bg, loss_g, loss_b, mode="drop"):
        """Attach a Gilbert-Elliot chain to BOTH directions (same
        parameters, independent seeded streams - the bursts do not
        line up, like a real asymmetric link)."""
        for D in self.dir.values():
            D.ge = GilbertElliot(D.rng, p_gb, p_bg, loss_g, loss_b, mode)
        self._notify("tx")
        self._notify("rx")

    def apply_profile(self, name, now):
        prof = PROFILES.get(name)
        if prof is None:
            raise KeyError(f"unknown link profile {name!r} "
                           f"(have {', '.join(sorted(PROFILES))})")
        if "loss" in prof:
            self.loss = float(prof["loss"])
        if "loss_tx" in prof:
            self.dir["tx"].loss = float(prof["loss_tx"])
        if "loss_rx" in prof:
            self.dir["rx"].loss = float(prof["loss_rx"])
        if "delay_ms" in prof:
            self.delay_ms = int(prof["delay_ms"])
        if "jitter_ms" in prof:
            self.jitter_ms = int(prof["jitter_ms"])
        if "bw_bps" in prof:
            self.bw_bps = int(prof["bw_bps"])
        if "stall_s" in prof:
            self.stall(float(prof["stall_s"]), now)
        if "ge" in prof:
            self.set_ge(**prof["ge"])
        self._notify("tx")
        self._notify("rx")

    def clear(self):
        """`link ... clear`: remove every knob and purge the queues -
        held frames are stale by then, they count as purged."""
        self.loss, self.delay_ms, self.jitter_ms = 0.0, 0, 0
        self.bw_bps, self.cut_until, self.stall_until = 0, 0.0, 0.0
        for D in self.dir.values():
            D.loss, D.ge = 0.0, None
            D.next_free, D.last_rel = 0.0, 0.0
            D.purged += len(D.queue)
            D.queue.clear()
        self._notify("tx")
        self._notify("rx")

    # ----------------------------------------------------------- offer

    def _notify(self, direction):
        if self.on_change is not None:
            self.on_change(self.party, direction)

    def offer(self, direction, size, now, tag):
        """A frame of `size` bytes wants to cross `direction` at `now`.
        -> "drop" | "queued". Queued frames release through pop_ready()
        in offer order; tag is carried through opaque."""
        D = self.dir[direction]
        if now < self.cut_until:
            D.dropped += 1
            return "drop"
        ge = D.ge
        bad = ge.tick() if ge else False
        if bad and ge.mode == "stall":
            # held in order until the chain flips good - rel stays None
            D.queue.append({"rel": None, "size": size, "tag": tag})
            D.held += 1
            self._notify(direction)
            return "queued"
        if not bad:
            self._unstall(D, now)
        if now < self.stall_until:
            rel = max(self.stall_until, D.last_rel)
            D.last_rel = rel
            D.queue.append({"rel": rel, "size": size, "tag": tag})
            D.held += 1
            self._notify(direction)
            return "queued"
        # every loss source rolls: the GE state's, the flat `loss`
        # stress knob and this direction's own - one Bernoulli on the
        # combined survival probability keeps a single seeded stream
        p = 100.0 * (1.0
                     - (1.0 - (ge.loss_b if bad else ge.loss_g) / 100.0
                        if ge else 1.0)
                     * (1.0 - self.loss / 100.0)
                     * (1.0 - D.loss / 100.0))
        if p > 0 and D.rng.random() * 100.0 < p:
            D.dropped += 1
            return "drop"
        rel = now + (self.delay_ms
                     + D.rng.uniform(0.0, self.jitter_ms)) / 1000.0
        if self.bw_bps > 0:
            rel = max(rel, D.next_free)
            D.next_free = rel + size / self.bw_bps
        if rel > now + 0.0005:
            D.delayed += 1
        # TCP cannot reorder: never release before an earlier frame
        rel = max(rel, D.last_rel)
        D.last_rel = rel
        D.queue.append({"rel": rel, "size": size, "tag": tag})
        self._notify(direction)
        return "queued"

    def _unstall(self, D, now):
        """A GE stall burst just ended (this tick landed on good):
        schedule the held entries in queue order, bw-serialized like
        fresh traffic, so nothing overtakes them."""
        for e in D.queue:
            if e["rel"] is not None:
                continue
            rel = max(now, D.last_rel)
            if self.bw_bps > 0:
                rel = max(rel, D.next_free)
                D.next_free = rel + e["size"] / self.bw_bps
            e["rel"] = D.last_rel = rel

    # ----------------------------------------------------------- drain

    def pop_ready(self, direction, now):
        """The due head-of-queue tags in offer order. A held head
        (rel None - a GE stall still bad) blocks everything behind it:
        head-of-line, exactly like the real pipe."""
        D = self.dir[direction]
        out = []
        while D.queue:
            rel = D.queue[0]["rel"]
            if rel is None or rel > now:
                break
            D.delivered += 1
            out.append(D.queue.popleft()["tag"])
        return out

    def next_release(self, direction):
        """The head entry's release time; None when the queue is empty
        or the head is still held - the caller then waits on_change."""
        D = self.dir[direction]
        return D.queue[0]["rel"] if D.queue else None

    # ----------------------------------------------------------- stats

    def describe(self, now):
        """The control-port `stats`/`link` reply fields for one link."""
        out = {"loss": self.loss, "delay_ms": self.delay_ms,
               "jitter_ms": self.jitter_ms, "bw_bps": self.bw_bps,
               "cut_s": max(0.0, self.cut_until - now),
               "stall_s": max(0.0, self.stall_until - now)}
        for d, D in self.dir.items():
            ge = D.ge
            out[d] = {
                "loss": D.loss,
                "ge": ("%s p_gb=%g p_bg=%g lg=%g lb=%g %s"
                       % (ge.mode, ge.p_gb, ge.p_bg, ge.loss_g,
                          ge.loss_b, "BAD" if ge.bad else "good"))
                      if ge else "-",
                "q": len(D.queue), "dropped": D.dropped,
                "delayed": D.delayed, "held": D.held,
                "sent": D.delivered, "purged": D.purged}
        return out

    def stats_line(self, now, uptime_s):
        """One `stats` reply line - the old counters plus the model's."""
        d = self.describe(now)
        return (f"loss={d['loss']}% delay={d['delay_ms']}ms "
                f"jitter={d['jitter_ms']}ms bw={d['bw_bps']}B/s "
                f"cut={d['cut_s']:.0f}s stall={d['stall_s']:.0f}s "
                f"tx[{d['tx']['loss']}% {d['tx']['ge']} "
                f"q={d['tx']['q']} drop={d['tx']['dropped']} "
                f"held={d['tx']['held']} purged={d['tx']['purged']}] "
                f"rx[{d['rx']['loss']}% {d['rx']['ge']} "
                f"q={d['rx']['q']} drop={d['rx']['dropped']} "
                f"held={d['rx']['held']} purged={d['rx']['purged']}] "
                f"roster_msgs={self.roster_msgs} "
                f"roster_bytes={self.roster_bytes} "
                f"({self.roster_bytes / max(1.0, uptime_s):.1f} B/s)")
