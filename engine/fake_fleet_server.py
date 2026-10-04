#!/usr/bin/env python3
"""
fake_fleet_server.py - DroneEngage fleet comm server with link faults.

Copied from Phase-4A-Tasks/tools (P4A-03) and pointed at simlib. Used when a
sim sets `server: {type: fake}`; the default is the real droneengage_server.

Grows Phase-2 tools/fake_comm_server.py from one client to a fleet: N de_comm
connect over wss, get registered by their de_auth frame (s=party_id,
at='d'|'g'), and every message is routed by `tg` with the same rules as the
real server (droneengage_server js_chat_routing.js):

    tg "_GCS_" -> all GCS          tg "_GD_"  -> everyone
    tg "_AGN_" -> all drones       tg <party> -> that unit only
    no tg      -> drone senders reach GCS only, GCS senders reach everyone
    ty "g"     -> GCS senders reach everyone, drone senders reach GCS only

The server stamps `sd` with the authenticated party id (F6: the wire sender
is the connection's identity, not a self-declared field) and `p` with full
permissions, like the real server stamps the account permission bitmask.

Per-link fault injection (the replacement for tc/netem, no root needed).
Each party id owns a link state {loss %, delay ms, cut until} and one seeded
RNG per direction. Faults apply in BOTH directions and to EVERY frame,
including the 9005 ping echo: de_comm's watchdog treats any received frame
as proof of life, so a cut that let pings through would never be noticed.
A cut link therefore behaves like an unreachable server:
  * established ws: silent - no frames either way; de_comm's ping watchdog
    (max_allowed_ping_delay_in_ms, floor 10 s) drops it -> "comm line offline"
  * re-dial while cut: the de_auth is refused (ws closed) - like a refused
    TCP connect; de_comm retries every 10 s and registers again only after
    the cut expires -> "comm line online"
Seeding makes the drop decision for the k-th frame of a direction
reproducible; which frame is k-th still depends on traffic timing, so two
runs with the same seed are statistically alike, not frame-identical.

Control port (TCP, port+1), newline-delimited:

    list                              connected parties
    stats                             per-link drop/delay counters
    send <json>                       inject a wire message as GCS_FLEET
    to <tg> <json>                    same, with tg forced
    upload <pkgfile> <tg|all>         1085 DE_Package_Upload {pkg,hash}
    action <tg|all> <json>            1086 DE_Supervisor_Action
    event <name>                      1061 Sync_EventFire from unit OTHER_UNIT_1
    commline <party> <seconds>        in-band cut: 6509 {ws:false, wsd:sec}
    kick <party>                      close the unit's ws (real transport cut;
                                      combine with `link ... cut` to keep it
                                      offline for a while)
    link <party|all> loss <0-100>     seeded packet loss on that link
    link <party|all> delay <ms>       fixed outbound+inbound delay
    link <party|all> cut <seconds>    drop everything for N seconds
    link <party|all> clear            remove all faults
    clock offset <ms>                 skew the stamped server clock (P4A-07:
                                      the fleet clock is the SERVER's clock -
                                      a wrong system clock must not matter)

  Usage: python3 fake_fleet_server.py --port 60008 [--seed 1337] [--pretty]
         [--clock-offset MS]
         echo 'link fleet2 loss 30' | nc 127.0.0.1 60009
"""

import argparse
import asyncio
import hashlib
import json
import random
import ssl
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import simlib as fleet_lib

GCS_SENDER = fleet_lib.GCS_PARTY
FULL_PERM = fleet_lib.FULL_PERM

PRETTY = False
SEED = 1337
PORT = fleet_lib.SERVER_PORT
CLOCK_OFFSET_MS = 0                 # P4A-07: deliberate server-clock skew
NOCLOCK = set()                     # parties whose frames carry no "ts"
SIM_NAME = None                     # run/<name>/ this server belongs to


def _single():
    """The only generated sim's name (default --cert/--key location)."""
    names = fleet_lib.sim_names()
    return names[0] if names else None
T0 = time.time()                    # stats uptime

conns = {}          # ws -> Conn
by_party = {}       # party_id -> Conn (latest connection wins)
links = {}          # party_id -> Link (survives reconnects)


def fleet_ms():
    """The server's clock in unix ms, as stamped into 9007/9005 (CLOCK.md
    T2 - the fleet clock IS this clock, skew included)."""
    return int(time.time() * 1000) + CLOCK_OFFSET_MS


class Link:
    """Fault state of one party's server link (both directions)."""
    __slots__ = ("loss", "delay_ms", "cut_until", "rng", "dropped",
                 "delayed", "roster_bytes", "roster_msgs")

    def __init__(self, party):
        self.loss = 0.0
        self.delay_ms = 0
        self.cut_until = 0.0
        # one stream per direction so uplink volume can't shift the
        # downlink drop pattern
        self.rng = {"rx": random.Random(f"{SEED}:{party}:rx"),
                    "tx": random.Random(f"{SEED}:{party}:tx")}
        self.dropped = 0
        self.delayed = 0
        # P4A-07 bandwidth: 1092 bytes/ms offered to this link
        self.roster_bytes = 0
        self.roster_msgs = 0

    def is_cut(self):
        return time.time() < self.cut_until

    def admit(self, direction, label, loss=True):
        """True if a frame may pass now. direction 'rx' (unit -> server)
        or 'tx' (server -> unit). loss=False checks the cut only."""
        if self.is_cut():
            self.dropped += 1
            if PRETTY:
                print(f"[link  ] CUT drop {label}", flush=True)
            return False
        if loss and self.loss and self.rng[direction].random() * 100.0 < self.loss:
            self.dropped += 1
            if PRETTY:
                print(f"[link  ] LOSS drop {label}", flush=True)
            return False
        return True


def fleet_parties():
    """Every unit the harness knows - the manifest plus anything that has
    connected - so `link all ...` also covers units not connected yet."""
    parties = set(links)
    try:
        m = fleet_lib.load_manifest(SIM_NAME, required=False) \
            or {"units": []}
        parties |= {u["party_id"] for u in m["units"]}
    except SystemExit:
        pass
    return sorted(parties)


def link_for(party):
    if party not in links:
        links[party] = Link(party)
    return links[party]


class Conn:
    __slots__ = ("ws", "party", "actor")

    def __init__(self, ws, party, actor):
        self.ws = ws
        self.party = party
        self.actor = actor          # 'd' drone (de_comm) | 'g' GCS


async def deliver(conn, raw):
    """Send raw to conn honouring its link delay/drop/cut."""
    ln = link_for(conn.party)
    if not ln.admit("tx", f"-> {conn.party}"):
        return
    if ln.delay_ms > 0:
        ln.delayed += 1
        await asyncio.sleep(ln.delay_ms / 1000.0)
        if conns.get(conn.ws) is not conn:
            return
        # loss was already rolled once; only a cut that started while
        # the frame was in flight can still drop it
        if not ln.admit("tx", f"-> {conn.party} (delayed)", loss=False):
            return
    try:
        await conn.ws.send(raw)
    except Exception:
        pass


def targets_for(msg, src):
    """Mirror js_chat_routing.js: resolve recipient connections."""
    tg = msg.get("tg") or ""
    actor = src.actor if src else "g"          # control-port injects as GCS
    if msg.get("ty") == "g":
        return [c for c in conns.values()
                if c is not src and (actor == "g" or c.actor == "g")]
    if tg == "_GCS_":
        return [c for c in conns.values() if c.actor == "g" and c is not src]
    if tg == "_GD_":
        return [c for c in conns.values() if c is not src]
    if tg == "_AGN_":
        return [c for c in conns.values() if c.actor == "d" and c is not src]
    if tg:
        c = by_party.get(tg)
        return [c] if c is not None else []
    # no target: drones reach GCS only, GCS reaches everyone
    if actor == "d":
        return [c for c in conns.values() if c.actor == "g"]
    return [c for c in conns.values() if c is not src]


async def route(msg, src):
    # sd is server-stamped from the connection identity (F6); control-port
    # injections may carry an explicit sd to simulate any sender (e.g.
    # `event` fires as OTHER_UNIT_1).
    msg["sd"] = src.party if src else msg.get("sd") or GCS_SENDER
    msg["p"] = FULL_PERM
    raw = json.dumps(msg)
    tgts = targets_for(msg, src)
    if PRETTY:
        mt, tg = msg.get("mt"), msg.get("tg", "")
        print(f"[route ] sd={msg['sd']} mt={mt} tg={tg or '-'} -> "
              f"{[c.party for c in tgts] or 'nobody'}", flush=True)
    if msg.get("mt") == 1092:               # P4A-07 bandwidth counters
        for c in tgts:
            ln = link_for(c.party)
            ln.roster_bytes += len(raw)
            ln.roster_msgs += 1
    for c in tgts:
        asyncio.ensure_future(deliver(c, raw))


# ------------------------------------------------------------- pretty log

_last_status = {}


def pretty_rx(mt, ms, sd, tg):
    global _last_status
    if mt == 1087 and "plan" in ms:
        pl = ms["plan"]
        seqs = [it.get("s") for it in pl.get("items", [])]
        print(f"[{sd} PLAN] {pl.get('mission')} seqs {seqs[:1]}..{seqs[-1:]} "
              f"({len(seqs)} items)", flush=True)
        return
    if mt == 1087:
        st = ms.get("st", "?")
        parts = [f"eng={ms.get('eng')}", f"st={st}"]
        if ms.get("mis"):
            parts.append(f"mis={ms.get('mis')}/{ms.get('mst')}")
        if ms.get("seq"):
            parts.append(f"seq={ms.get('seq')}")
        if ms.get("wait"):
            parts.append(f"WAIT {ms['wait'].get('event')}")
        if ms.get("err"):
            parts.append(f"ERR={ms.get('err')}")
        line = " ".join(parts)
        if line != _last_status.get(sd):
            _last_status[sd] = line
            print(f"[{sd} STATUS] {line}", flush=True)
    elif mt == 1008:
        print(f"[{sd} NOTIFY] {ms.get('DS', ms)}", flush=True)
    elif mt == 1061:
        print(f"[{sd} EVENT] d={ms.get('d')} ed={json.dumps(ms.get('ed', {}))}",
              flush=True)
    elif mt == 1088:
        for ln in ms.get("lines", []):
            print(f"[{sd} JRNL] {ln}", flush=True)
    else:
        label = {1085: "PKG_UP", 1086: "SV_ACTION", 1027: "WPS",
                 1005: "REMOTE_EXEC", 1013: "ID"}.get(mt)
        if label:
            print(f"[{sd} {label}] tg={tg} {json.dumps(ms)[:300]}", flush=True)


# ------------------------------------------------------------ ws handler

async def handler(ws):
    conn = None
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except Exception:
                continue
            if msg.get("ty") == "s" and msg.get("mt") == "de_auth":
                party = str(msg.get("s", ""))
                actor = msg.get("at", "d")
                if actor not in ("d", "g"):
                    actor = "d"
                if link_for(party).is_cut():
                    # unreachable server: refuse the session (de_comm
                    # sees a socket error and retries on its own timer)
                    print(f"[server] {actor}:{party} refused (link cut)",
                          flush=True)
                    await ws.close()
                    return
                if not link_for(party).admit("rx", f"auth <- {party}"):
                    # the login frame itself rode the lossy link and was
                    # dropped - the client times out and retries, so a
                    # 100% loss keeps the unit offline like a real link
                    continue
                conn = Conn(ws, party, actor)
                conns[ws] = conn
                by_party[party] = conn
                link_for(party)
                await ws.send(json.dumps(
                    {"ty": "s", "mt": "de_auth_ack", "r": "ok"}))
                # P4A-07 (CLOCK.md T2.1): "ts" is the fleet clock push -
                # like the real server's "ts": Date.now(). A NOCLOCK
                # party gets no stamp - it never syncs (tq=0 from boot,
                # the P4A-10 closed-environment case).
                c9007 = {"s": f"OK:connected:tcp:127.0.0.1:{PORT}"}
                if party not in NOCLOCK:
                    c9007["ts"] = fleet_ms()
                await ws.send(json.dumps(
                    {"ty": "s", "mt": 9007, "ms": c9007}))
                print(f"[server] {actor}:{party} connected "
                      f"({sum(1 for c in conns.values() if c.actor == 'd')} units)",
                      flush=True)
                continue
            if conn is None:
                continue                     # no traffic before de_auth
            # sender-side link: a cut/lossy link cannot send either
            if not link_for(conn.party).admit("rx", f"<- {conn.party}"):
                continue
            if msg.get("mt") == 9005:        # ping - pong back through the link
                # CLOCK.md T2.2: echo tm/b and stamp ts (like the real
                # server's "s:OK:pong" + "ts:Date.now()")
                ms = msg.get("ms")
                if isinstance(ms, dict):
                    ms["s"] = "OK:pong"
                    # no "ts" for NOCLOCK parties: the pong still proves
                    # liveness (the link stays up) but carries no clock
                    # sample - the unit stays at tq=0 forever.
                    if conn.party not in NOCLOCK:
                        ms["ts"] = fleet_ms()
                    asyncio.ensure_future(deliver(conn, json.dumps(msg)))
                else:
                    asyncio.ensure_future(deliver(
                        conn, raw if isinstance(raw, str)
                        else raw.decode(errors="ignore")))
                continue
            if PRETTY:
                pretty_rx(msg.get("mt"), msg.get("ms", {}), conn.party,
                          msg.get("tg", ""))
            else:
                print(f"[server] rx sd={conn.party} mt={msg.get('mt')} "
                      f"tg={msg.get('tg', '')} ms={json.dumps(msg.get('ms', {}))[:600]}",
                      flush=True)
            await route(msg, conn)
    finally:
        if conn is not None:
            conns.pop(conn.ws, None)
            if by_party.get(conn.party) is conn:
                del by_party[conn.party]
            print(f"[server] {conn.party} disconnected", flush=True)


# ------------------------------------------------------------- control port

def build_message(line):
    """Control-line -> (target spec, wire message) or None."""
    if line.startswith("send "):
        return None, json.loads(line[5:])
    if line.startswith("to "):
        tg, _, rest = line[3:].partition(" ")
        msg = json.loads(rest)
        msg["tg"] = tg
        return None, msg
    if line.startswith("upload "):
        f, _, tg = line[7:].partition(" ")
        text = open(f.strip()).read()
        h = "sha256:" + hashlib.sha256(text.encode()).hexdigest()
        return None, {"ty": "i", "mt": 1085,
                      "tg": tg.strip() or "_AGN_",
                      "ms": {"pkg": text, "hash": h}}
    if line.startswith("action "):
        # action <json>            -> all drones (_AGN_)
        # action <tg|all> <json>   -> one unit / an explicit broadcast word
        rest = line[7:]
        tg, _, body = rest.partition(" ")
        if tg.startswith("{"):
            tg, body = "_AGN_", rest
        elif tg == "all":
            tg = "_AGN_"
        msg = {"ty": "i", "mt": 1086, "ms": json.loads(body), "tg": tg}
        return None, msg
    if line.startswith("event "):
        return None, {"ty": "i", "mt": 1061, "tg": "_AGN_",
                      "sd": "OTHER_UNIT_1",
                      "ms": {"d": line[6:].strip()}}
    if line.startswith("commline "):
        # wsd must be an unsigned int - de_comm's validateField rejects
        # floats, which would make the cut permanent (duration defaults 0)
        party, _, sec = line[9:].partition(" ")
        return None, {"ty": "i", "mt": 6509, "tg": party,
                      "ms": {"ws": False, "wsd": int(float(sec or 30))}}
    return None, None


async def control_reader(reader, writer):
    global CLOCK_OFFSET_MS
    # NOCLOCK is a set - mutated in place so --noclock and `noclock`
    # control lines share it
    print(f"[ctrl] connection from {writer.get_extra_info('peername')}",
          flush=True)

    def say(text):
        print(f"[ctrl] {text}", flush=True)
        try:
            writer.write((text + "\n").encode())
        except Exception:
            pass

    try:
        while True:
            raw = await reader.readline()
            if not raw:
                return
            line = raw.decode().strip()
            if not line:
                continue
            print(f"[ctrl] {time.strftime('%H:%M:%S')} {line[:120]}",
                  flush=True)
            try:
                if line == "list":
                    for c in conns.values():
                        say(f"{c.actor}:{c.party}")
                    continue
                if line == "stats":
                    el = max(1.0, time.time() - T0)
                    for p, ln in links.items():
                        say(f"{p}: loss={ln.loss}% delay={ln.delay_ms}ms "
                            f"cut={max(0.0, ln.cut_until - time.time()):.0f}s "
                            f"dropped={ln.dropped} delayed={ln.delayed} "
                            f"roster_msgs={ln.roster_msgs} "
                            f"roster_bytes={ln.roster_bytes} "
                            f"({ln.roster_bytes / el:.1f} B/s)")
                    continue
                if line.startswith("clock "):
                    # clock offset <ms> - skew every future ts stamp
                    _, sub, *rest = line.split()
                    if sub == "offset" and rest:
                        CLOCK_OFFSET_MS = int(rest[0])
                        say(f"clock offset now {CLOCK_OFFSET_MS} ms")
                    else:
                        say(f"clock offset is {CLOCK_OFFSET_MS} ms")
                    continue
                if line.startswith("noclock "):
                    # noclock <party> <0|1> - P5-07 no_time_sync fault:
                    # stop/restart stamping "ts" into the unit's 9005
                    # pongs + future 9007s (it stays tq=0 meanwhile)
                    _, party, flag = line.split()[:3]
                    if flag == "1":
                        NOCLOCK.add(party)
                    else:
                        NOCLOCK.discard(party)
                    say(f"noclock {party} = {flag} "
                        f"(noclock set: {sorted(NOCLOCK)})")
                    continue
                if line.startswith("kick "):
                    # close the unit's ws: a real transport cut - the unit
                    # sees a socket drop, journals "comm line offline",
                    # reconnects on its own -> "comm line online"
                    party = line[5:].strip()
                    c = by_party.get(party)
                    if c is None:
                        say(f"kick {party}: not connected")
                    else:
                        await c.ws.close()
                        say(f"kick {party}: connection closed")
                    continue
                if line.startswith("link "):
                    _, party, cmd, *rest = line.split()
                    for p in (fleet_parties() if party == "all" else [party]):
                        ln = link_for(p)
                        if cmd == "loss":
                            ln.loss = float(rest[0])
                        elif cmd == "delay":
                            ln.delay_ms = int(rest[0])
                        elif cmd == "cut":
                            ln.cut_until = time.time() + float(rest[0])
                        elif cmd == "clear":
                            ln.loss, ln.delay_ms, ln.cut_until = 0.0, 0, 0.0
                        say(f"link {p}: loss={ln.loss}% "
                            f"delay={ln.delay_ms}ms "
                            f"cut={max(0.0, ln.cut_until - time.time()):.0f}s")
                    continue
                _tg, msg = build_message(line)
                if msg is None:
                    print(f"[ctrl] ignored: {line[:80]}", flush=True)
                    continue
                await route(msg, None)
            except Exception as e:
                print(f"[ctrl] bad input {line!r}: {e}", flush=True)
    except Exception as e:
        print(f"[ctrl] handler error: {e!r}", flush=True)


async def serve(port, cert, key):
    import websockets
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert, key)
    async with websockets.serve(handler, "127.0.0.1", port, ssl=ctx,
                                compression=None, ping_interval=None):
        ctl = await asyncio.start_server(control_reader, "127.0.0.1", port + 1)
        print(f"[server] wss on 127.0.0.1:{port}, control on {port + 1}, "
              f"seed={SEED}", flush=True)
        async with ctl:
            await asyncio.Future()          # run forever


def main():
    global PRETTY, SEED, PORT, CLOCK_OFFSET_MS, NOCLOCK, SIM_NAME
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=fleet_lib.SERVER_PORT)
    ap.add_argument("--sim", default=None,
                    help="sim name - its run/<name>/sim.json is the unit list")
    ap.add_argument("--cert", default=None)
    ap.add_argument("--key", default=None)
    ap.add_argument("--seed", type=int, default=SEED)
    ap.add_argument("--clock-offset", type=int, default=0,
                    help="ms added to every stamped server clock (P4A-07)")
    ap.add_argument("--noclock", default="",
                    help="csv of party ids that never get a clock stamp "
                         "(9007 ts / 9005 pong ts) - they stay tq=0")
    ap.add_argument("--pretty", action="store_true")
    args = ap.parse_args()
    PRETTY, SEED, PORT = args.pretty, args.seed, args.port
    SIM_NAME = args.sim
    CLOCK_OFFSET_MS = args.clock_offset
    NOCLOCK = {p for p in args.noclock.split(",") if p}
    cert = args.cert or fleet_lib.cert_path(args.sim or _single())
    key = args.key or fleet_lib.key_path(args.sim or _single())
    asyncio.run(serve(args.port, cert, key))


if __name__ == "__main__":
    main()
