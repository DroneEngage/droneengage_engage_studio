"""prelaunch_checks/relay.py - stage 3: MAVLink through the real relay and
the real firewall, seen from a host that is not the server.

This is the fault that started the gate: a firewall blocked the UDP proxy
range and every sim test stayed green, because the sim runs on one host.

mavlink_both_ways: a real GCS session asks the unit for its relay endpoint
(mt 1071). Then a plain UDP socket does what QGroundControl does: it sends a
HEARTBEAT to that port, counts FC HEARTBEATs, and sends one harmless command
(REQUEST_AUTOPILOT_CAPABILITIES) that must return a COMMAND_ACK. It never
arms or moves a vehicle. The relay locks to the first GCS, so a real GCS on
the same unit makes this fail with a message that says so.

range_edges: a firewall rule can cover only part of udp_port_range. A UDP
send to a free port gives an ICMP "port unreachable" when the packet arrives,
and silence when a firewall drops it. So each end of the range is tried. Note
that this also reads silence if a network path filters ICMP, and the message
says so.
"""

import asyncio
import json
import socket
import time

from prelaunch_checks import reach

HEARTBEATS_MIN = 10
HEARTBEAT_WINDOW_S = 25.0
ENDPOINT_BUDGET_S = 30.0
EDGE_TRIES = 4         # ports tried at each end of the range
CMD_CAPABILITIES = 520
MAV_TYPE_GCS = 6


async def _send(ws, msg):
    await ws.send(json.dumps(msg))


async def _ask_endpoint(ws, party_hint, budget_s):
    """-> (party, ip, port), or (party, None, None) when no unit announces
    one. A unit is asked with RemoteExecute {C:1071}, and the telemetry of a
    paused proxy is resumed with {C:108, Act:3}."""
    me = reach.PARTY
    party = party_hint
    end = time.time() + budget_s
    next_ask = 0.0
    while time.time() < end:
        if time.time() >= next_ask:
            next_ask = time.time() + 3.0
            if party:
                await _send(ws, {"ty": "i", "mt": 1005, "tg": party, "sd": me,
                                 "ms": {"C": 1071}})
            else:
                await _send(ws, {"ty": "g", "mt": 1005, "sd": me,
                                 "ms": {"C": 1004}})
        try:
            raw = await asyncio.wait_for(ws.recv(), 1.0)
        except asyncio.TimeoutError:
            continue
        if not isinstance(raw, str):
            continue
        msg = json.loads(raw)
        sd = msg.get("sd")
        if not sd or sd == me or str(sd).startswith("GCS"):
            continue
        if not party:
            party = sd
            next_ask = 0.0
            continue
        ms = msg.get("ms") or {}
        if sd == party and msg.get("mt") == 1071 and ms.get("en") \
                and ms.get("a") and ms.get("p"):
            await _send(ws, {"ty": "i", "mt": 1005, "tg": party, "sd": me,
                             "ms": {"C": 108, "Act": 3}})
            return party, ms["a"], int(ms["p"])
    return party, None, None


def _endpoint(srv, party_hint):
    import gcs_client

    async def go():
        client = gcs_client.GCSClient(srv, reach.PARTY)
        ws = await client.connect()
        try:
            return await _ask_endpoint(ws, party_hint, ENDPOINT_BUDGET_S)
        finally:
            await ws.close()
    return asyncio.run(asyncio.wait_for(go(), ENDPOINT_BUDGET_S + 30))


def gcs_exchange(ip, port, seconds=HEARTBEAT_WINDOW_S, sysid=None):
    """Do what a real GCS does over one UDP socket.
    -> {heartbeats, ack, sysid}. Returns early once all is proved."""
    from pymavlink import mavutil
    mav = mavutil.mavlink.MAVLink(None)
    mav.srcSystem, mav.srcComponent = 255, 190
    hb = mav.heartbeat_encode(MAV_TYPE_GCS,
                              mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                              0, 0, 0).pack(mav)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 0))
    s.settimeout(0.25)
    parser = mavutil.mavlink.MAVLink(None)
    out = {"heartbeats": 0, "ack": None, "sysid": sysid}
    end, next_hb, next_cmd = time.time() + seconds, 0.0, 0.0
    try:
        while time.time() < end:
            now = time.time()
            if now >= next_hb:
                s.sendto(hb, (ip, port))
                next_hb = now + 1.0
            if out["sysid"] and out["ack"] is None and now >= next_cmd:
                cmd = mav.command_long_encode(
                    out["sysid"], 1, CMD_CAPABILITIES, 0,
                    1, 0, 0, 0, 0, 0, 0).pack(mav)
                s.sendto(cmd, (ip, port))
                next_cmd = now + 2.0
            try:
                data, _ = s.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:      # ICMP from an earlier send
                continue
            try:
                msgs = parser.parse_buffer(bytearray(data))
            except mavutil.mavlink.MAVError:
                parser = mavutil.mavlink.MAVLink(None)
                continue
            for m in msgs or []:
                if m is None:
                    continue
                name = m.get_type()
                if name == "HEARTBEAT" and m.type != MAV_TYPE_GCS:
                    if out["sysid"] is None:
                        out["sysid"] = m.get_srcSystem()
                    if m.get_srcSystem() == out["sysid"]:
                        out["heartbeats"] += 1
                elif name == "COMMAND_ACK" and m.command == CMD_CAPABILITIES:
                    out["ack"] = int(m.result)
            if out["heartbeats"] >= HEARTBEATS_MIN and out["ack"] is not None:
                break
    finally:
        s.close()
    return out


def judge_exchange(ip, port, res, probe_host):
    """(status, detail) from a gcs_exchange result."""
    where = f"probe {probe_host} to relay {ip}:{port}"
    if res["heartbeats"] == 0:
        return "FAIL", (f"{where}: no HEARTBEAT came back. The relay port is "
                        f"not reachable: a firewall may block it, or another "
                        f"GCS holds the lock on this unit")
    if res["heartbeats"] < HEARTBEATS_MIN:
        return "FAIL", (f"{where}: only {res['heartbeats']} HEARTBEATs in the "
                        f"window, and {HEARTBEATS_MIN} are needed")
    if res["ack"] is None:
        return "FAIL", (f"{where}: {res['heartbeats']} HEARTBEATs came back, "
                        f"but the GCS command got no COMMAND_ACK. The path "
                        f"from the GCS to the FC is broken")
    return "PASS", (f"{where}: {res['heartbeats']} HEARTBEATs and COMMAND_ACK "
                    f"result {res['ack']}")


def mavlink_both_ways(ctx):
    srv = reach._srv(ctx)
    relay = ctx.service("relay")
    wrong = reach._wrong_vantage(ctx, relay["host"])
    if wrong:
        return "FAIL", wrong
    hint = (ctx.target.get("test_unit") or {}).get("party")
    try:
        party, ip, port = _endpoint(srv, hint)
    except Exception as e:
        return "FAIL", f"cannot open the GCS session to ask for the endpoint: {e!r}"
    if not party:
        return "SKIP", ("no test unit is online for this account. Start the "
                        "unit before this stage")
    if not ip:
        return "FAIL", (f"unit {party} does not announce a UDP proxy endpoint "
                        f"(mt 1071). Look at the udp_relay config and the "
                        f"relay link")
    lo, hi = relay.get("udp_range") or (None, None)
    if lo is not None and not lo <= port <= hi:
        return "FAIL", (f"unit {party} announces port {port}, outside the "
                        f"relay range {lo}-{hi}")
    return judge_exchange(ip, port, gcs_exchange(ip, port),
                          socket.gethostname())


def udp_port_state(host, port, timeout=1.5, tries=2):
    """'open' when an ICMP port-unreachable shows the packet arrived with no
    listener; 'silent' when nothing returns (a firewall drop, or a socket in
    use that ignores this datagram)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.connect((host, int(port)))
        for _ in range(tries):
            s.send(b"prelaunch-edge-probe")
            try:
                s.recv(2048)
                return "open"
            except ConnectionRefusedError:
                return "open"
            except socket.timeout:
                continue
        return "silent"
    except OSError as e:
        return f"error: {e}"
    finally:
        s.close()


def edge_state(host, ports):
    """One end of the range is open when any of its ports answers: a port
    in use by a live pair is silent even with no firewall."""
    states = {}
    for p in ports:
        states[p] = udp_port_state(host, p)
        if states[p] == "open":
            return "open", states
    return "silent", states


def range_edges(ctx):
    relay = ctx.service("relay")
    rng = relay.get("udp_range")
    if not rng:
        return "SKIP", f"target {ctx.name} has no services.relay.udp_range"
    wrong = reach._wrong_vantage(ctx, relay["host"])
    if wrong:
        return "FAIL", wrong
    lo, hi = int(rng[0]), int(rng[1])
    n = min(EDGE_TRIES, hi - lo + 1)
    low_ports = list(range(lo, lo + n))
    high_ports = list(range(hi, hi - n, -1))
    low, low_s = edge_state(relay["host"], low_ports)
    high, high_s = edge_state(relay["host"], high_ports)
    faults = []
    if low != "open":
        faults.append(f"low end {low_ports[0]}-{low_ports[-1]}")
    if high != "open":
        faults.append(f"high end {high_ports[-1]}-{high_ports[0]}")
    if faults:
        return "FAIL", (f"relay {relay['host']}: no answer from the "
                        f"{' and '.join(faults)} of udp_port_range {lo}-{hi}. "
                        f"A firewall rule may cover only part of the range, "
                        f"or a network on the path may filter ICMP")
    return "PASS", f"relay {relay['host']}: both ends of {lo}-{hi} are open"
