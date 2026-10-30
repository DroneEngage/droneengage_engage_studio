#!/usr/bin/env python3
"""
fake_fleet_server.py - DroneEngage fleet comm server with link faults.

Copied from Phase-4A-Tasks/tools (P4A-03) and pointed at simlib. Used when a
sim sets `server: {type: fake}`; the default is the real droneengage_server.

Grows Phase-2 tools/fake_comm_server.py from one client to a fleet: N de_comm
connect over wss, get registered by their de_auth frame (s=party_id,
at='d'|'g'), and every message is routed by `tg` with the same rules as the
real server (droneengage_server js_chat_routing.js - the Node original,
kept on the repo's pr_nodejs_last_version branch):

    tg "_GCS_" -> all GCS          tg "_GD_"  -> everyone
    tg "_AGN_" -> all drones       tg <party> -> that unit only
    no tg      -> drone senders reach GCS only, GCS senders reach everyone
    ty "g"     -> GCS senders reach everyone, drone senders reach GCS only

The server stamps `sd` with the authenticated party id (F6: the wire sender
is the connection's identity, not a self-declared field) and `p` with full
permissions, like the real server stamps the account permission bitmask.

Binary ws frames ("json header" \0 payload - packed MAVLink telemetry:
GPS/NavInfo/IMU) are routed on the header's tg/ty with sd+p stamped and
the payload carried verbatim - same as js_chat_routing.js does.

Per-link fault injection (the replacement for tc/netem, no root needed).
Each party id owns a link_model.Link: the link state plus one seeded RNG
per direction (tx = unit->server, rx = server->unit, unit-centric). Faults
apply in BOTH directions and to EVERY frame, including the 9005 ping echo:
de_comm's watchdog treats any received frame as proof of life, so a cut
that let pings through would never be noticed. The model (engine/
link_model.py) adds what a real bad link does on top of the flat knobs:
per-direction loss, Gilbert-Elliot bursts (bad state = drop, or = stall:
frames held in order while the ws stays connected - only the peer's own
watchdog notices), a bandwidth cap with an ordered byte queue, seeded
jitter, and a directed `stall <s>` (buffered blackout). Every admitted
frame lands in the direction's ordered queue; a per-direction flusher
task releases it at its computed time, so order is never violated.
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
    stats                             per-link model counters
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
    link <party|all> loss_tx <0-100>  loss on the unit's tx (unit->server)
    link <party|all> loss_rx <0-100>  loss on the unit's rx (server->unit)
    link <party|all> delay <ms>       fixed outbound+inbound delay
    link <party|all> jitter <ms>      seeded jitter added to the delay
    link <party|all> bw <bytes/s>     bandwidth cap - bytes serialize
                                      through the ordered queue
    link <party|all> ge <p_gb> <p_bg> <loss_g> <loss_b> [drop|stall]
                                      Gilbert-Elliot burst loss per
                                      direction; bad state drops or
                                      holds frames in order
    link <party|all> stall <seconds>  hold ALL traffic in order for N s
                                      (a buffered blackout - the ws
                                      stays up, the watchdog notices)
    link <party|all> cut <seconds>    drop everything for N seconds
    link <party|all> profile <name>   named preset (link_model.PROFILES:
                                      lte_edge, lte_stall, uplink_dead,
                                      dial_up)
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
import os
import ssl
import sys
import time

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import simlib as fleet_lib
import link_model

GCS_SENDER = fleet_lib.GCS_PARTY
FULL_PERM = fleet_lib.FULL_PERM

PRETTY = False
SEED = 1337
PORT = fleet_lib.SERVER_PORT
CLOCK_OFFSET_MS = 0                 # P4A-07: deliberate server-clock skew
NOCLOCK = set()                     # parties whose frames carry no "ts"
SIM_NAME = None                     # var/run/<name>/ this server belongs to


def _single():
    """The only generated sim's name (default --cert/--key location)."""
    names = fleet_lib.sim_names()
    return names[0] if names else None
T0 = time.time()                    # stats uptime

conns = {}          # ws -> Conn
by_party = {}       # party_id -> Conn (latest connection wins)
links = {}          # party_id -> link_model.Link (survives reconnects)


def fleet_ms():
    """The server's clock in unix ms, as stamped into 9007/9005 (CLOCK.md
    T2 - the fleet clock IS this clock, skew included)."""
    return int(time.time() * 1000) + CLOCK_OFFSET_MS


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


flushers = {}       # (party_id, direction) -> {"ev", "task"}


def link_for(party):
    if party not in links:
        ln = link_model.Link(party, SEED)
        ln.on_change = _link_changed
        links[party] = ln
    return links[party]


def _link_changed(party, direction):
    """link_model.on_change: the direction's queue gained a head or a
    stall released - wake its flusher (created lazily, on the loop)."""
    fl = flushers.get((party, direction))
    if fl is None:
        fl = flushers[(party, direction)] = {"ev": asyncio.Event(),
                                             "task": None}
    fl["ev"].set()
    if fl["task"] is None:
        fl["task"] = asyncio.ensure_future(_flusher(party, direction))


async def _flusher(party, direction):
    """Drain one link direction forever: release due queue heads to
    _emit in order, then sleep until the next release time or the next
    on_change (a GE-stall head has no release - it waits on on_change)."""
    ln = link_for(party)
    ev = flushers[(party, direction)]["ev"]
    while True:
        now = time.time()
        for tag in ln.pop_ready(direction, now):
            try:
                await _emit(party, direction, tag)
            except Exception as e:
                print(f"[link  ] emit {party}:{direction}: {e!r}",
                      flush=True)
        rel = ln.next_release(direction)
        ev.clear()
        try:
            if rel is None:
                await ev.wait()
            else:
                await asyncio.wait_for(ev.wait(), max(0.05, rel - now))
        except asyncio.TimeoutError:
            pass


async def _emit(party, direction, tag):
    """Hand one released frame to its destination. rx (server->unit)
    goes to the party's CURRENT session - a frame held over a reconnect
    still lands, in order. tx (unit->server) replays the ws receive
    path; a frame whose ws already died is stale (TCP would RST it)."""
    ln = link_for(party)
    if direction == "rx":
        conn = by_party.get(party)
        if conn is None or ln.is_cut(time.time()):
            ln.dir["rx"].purged += 1
            return
        try:
            await conn.ws.send(tag)
        except Exception:
            pass
        return
    state, ws, raw = tag
    if ws.state.name == "CLOSED" or ln.is_cut(time.time()):
        # ws died while the frame rode the link (TCP would RST it), or
        # a cut started in flight - same re-check rx does above
        ln.dir["tx"].purged += 1
        return
    await process_frame(state, ws, raw)


def egress(party, raw):
    """Queue one server->unit frame on the party's rx link (was
    deliver()): loss/delay/stall/bw all apply. Delivery targets the
    party's CURRENT session at release time."""
    ln = link_for(party)
    ln.offer("rx", len(raw), time.time(), raw)


def _auth_party(raw):
    """The party id of a de_auth frame, else None - the rx loop uses it
    to pick the tx link before the session's conn exists (pre-auth
    traffic has no link: it is skipped, same as before)."""
    try:
        if isinstance(raw, (bytes, bytearray)):
            sep = raw.find(b"\0")
            msg = json.loads(raw[:sep] if sep >= 0 else raw)
        else:
            msg = json.loads(raw)
    except Exception:
        return None
    if msg.get("ty") == "s" and msg.get("mt") == "de_auth":
        return str(msg.get("s", ""))
    return None


class Conn:
    __slots__ = ("ws", "party", "actor")

    def __init__(self, ws, party, actor):
        self.ws = ws
        self.party = party
        self.actor = actor          # 'd' drone (de_comm) | 'g' GCS


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


async def route(msg, src, wire=None):
    # sd is server-stamped from the connection identity (F6); control-port
    # injections may carry an explicit sd to simulate any sender (e.g.
    # `event` fires as OTHER_UNIT_1).
    msg["sd"] = src.party if src else msg.get("sd") or GCS_SENDER
    msg["p"] = FULL_PERM
    if wire is None:
        raw = json.dumps(msg)
    else:
        # binary frame ("json header" \0 payload): rebuild the stamped
        # header, keep the payload verbatim (same as the real server,
        # js_chat_routing.js fn_parseMessage)
        sep = wire.find(b"\0")
        raw = json.dumps(msg).encode() + (wire[sep:] if sep >= 0 else b"")
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
        egress(c.party, raw)


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
    state = {"conn": None, "party": None}
    try:
        async for raw in ws:
            # every inbound frame rides the party's tx link (unit ->
            # server): loss/delay/stall/bw apply, then the model's
            # flusher replays it through process_frame in order. The
            # party comes from the session, or from a de_auth in flight:
            # remember it so frames pipelined behind the auth queue
            # behind it instead of falling off (they still die in
            # process_frame if the auth itself was dropped).
            party = state["conn"].party if state["conn"] is not None \
                else (_auth_party(raw) or state["party"])
            if party is None:
                continue                     # no traffic before de_auth
            if state["conn"] is None and state["party"] is None:
                state["party"] = party
            ln = link_for(party)
            if state["conn"] is None and ln.is_cut(time.time()):
                # unreachable server: refuse the session (de_comm
                # sees a socket error and retries on its own timer)
                print(f"[server] d:{party} refused (link cut)",
                      flush=True)
                await ws.close()
                return
            ln.offer("tx", len(raw), time.time(), (state, ws, raw))
    finally:
        conn = state["conn"]
        if conn is not None:
            conns.pop(conn.ws, None)
            if by_party.get(conn.party) is conn:
                del by_party[conn.party]
            print(f"[server] {conn.party} disconnected", flush=True)


async def process_frame(state, ws, raw):
    """One unit->server frame after its tx link released it - the old
    receive loop body: de_auth registers the session, the 9005 ping
    pongs back through the rx link, everything else routes.
    Binary frames (sendMavlinkData_Packed & co: packed MAVLink
    telemetry - GPS/NavInfo/IMU) arrive as bytes: "json header" \0
    payload. Route on the header like the real server does."""
    conn = state["conn"]
    binary = isinstance(raw, (bytes, bytearray))
    if binary:
        raw = bytes(raw)
    try:
        if binary:
            sep = raw.find(b"\0")
            msg = json.loads(raw[:sep] if sep >= 0 else raw)
        else:
            msg = json.loads(raw)
    except Exception:
        return
    if msg.get("ty") == "s" and msg.get("mt") == "de_auth":
        party = str(msg.get("s", ""))
        actor = msg.get("at", "d")
        if actor not in ("d", "g"):
            actor = "d"
        conn = Conn(ws, party, actor)
        conns[ws] = conn
        by_party[party] = conn
        link_for(party)
        egress(party, json.dumps({"ty": "s", "mt": "de_auth_ack",
                                  "r": "ok"}))
        # P4A-07 (CLOCK.md T2.1): "ts" is the fleet clock push -
        # like the real server's "ts": Date.now(). A NOCLOCK
        # party gets no stamp - it never syncs (tq=0 from boot,
        # the P4A-10 closed-environment case).
        c9007 = {"s": f"OK:connected:tcp:127.0.0.1:{PORT}"}
        if party not in NOCLOCK:
            c9007["ts"] = fleet_ms()
        egress(party, json.dumps({"ty": "s", "mt": 9007, "ms": c9007}))
        state["conn"] = conn
        print(f"[server] {actor}:{party} connected "
              f"({sum(1 for c in conns.values() if c.actor == 'd')} units)",
              flush=True)
        return
    if conn is None:
        return                            # no traffic before de_auth
    if msg.get("mt") == 9005 and not binary:
        # ping - pong back through the link
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
            egress(conn.party, json.dumps(msg))
        else:
            egress(conn.party, raw if isinstance(raw, str)
                   else raw.decode(errors="ignore"))
        return
    if PRETTY:
        pretty_rx(msg.get("mt"), msg.get("ms")
                  if isinstance(msg.get("ms"), dict) else {},
                  conn.party, msg.get("tg", ""))
    else:
        print(f"[server] rx sd={conn.party} mt={msg.get('mt')} "
              f"tg={msg.get('tg', '')} ms={json.dumps(msg.get('ms', {}))[:600]}",
              flush=True)
    await route(msg, conn, raw if binary else None)


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
                        say(f"{p}: {ln.stats_line(time.time(), el)}")
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
                        now = time.time()
                        if cmd == "loss":
                            ln.loss = float(rest[0])
                        elif cmd == "loss_tx":
                            ln.dir["tx"].loss = float(rest[0])
                        elif cmd == "loss_rx":
                            ln.dir["rx"].loss = float(rest[0])
                        elif cmd == "delay":
                            ln.delay_ms = int(rest[0])
                        elif cmd == "jitter":
                            ln.jitter_ms = int(rest[0])
                        elif cmd == "bw":
                            ln.bw_bps = int(rest[0])
                        elif cmd == "ge":
                            # ge <p_gb> <p_bg> <loss_g> <loss_b> [drop|stall]
                            ln.set_ge(float(rest[0]), float(rest[1]),
                                      float(rest[2]), float(rest[3]),
                                      rest[4] if len(rest) > 4
                                      else "drop")
                        elif cmd == "stall":
                            ln.stall(float(rest[0]), now)
                        elif cmd == "cut":
                            ln.cut(float(rest[0]), now)
                        elif cmd == "profile":
                            ln.apply_profile(rest[0], now)
                        elif cmd == "clear":
                            ln.clear()
                        say(f"link {p}: {ln.stats_line(now, time.time() - T0)}")
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
    async with websockets.serve(handler, BIND, port, ssl=ctx,
                                compression=None, ping_interval=None):
        # same 64 KiB readline cap as gcs_bridge: `send` lines carry
        # whole packages, give them real headroom
        ctl = await asyncio.start_server(control_reader, BIND,
                                         port + 1, limit=16 << 20)
        print(f"[server] wss on {BIND}:{port}, control on {port + 1}, "
              f"seed={SEED}", flush=True)
        async with ctl:
            await asyncio.Future()          # run forever


def main():
    global PRETTY, SEED, PORT, CLOCK_OFFSET_MS, NOCLOCK, SIM_NAME, BIND
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=fleet_lib.SERVER_PORT)
    ap.add_argument("--bind", default="127.0.0.1",
                    help="wss+control bind address - 0.0.0.0 when a sim "
                         "unit runs off-host (SL-0095 android)")
    ap.add_argument("--sim", default=None,
                    help="sim name - its var/run/<name>/sim.json is the unit list")
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
    BIND = args.bind
    SIM_NAME = args.sim
    CLOCK_OFFSET_MS = args.clock_offset
    NOCLOCK = {p for p in args.noclock.split(",") if p}
    cert = args.cert or fleet_lib.cert_path(args.sim or _single())
    key = args.key or fleet_lib.key_path(args.sim or _single())
    asyncio.run(serve(args.port, cert, key))


if __name__ == "__main__":
    main()
