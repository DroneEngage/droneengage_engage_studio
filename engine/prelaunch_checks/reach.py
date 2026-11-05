"""prelaunch_checks/reach.py - stage 2: what the deployed servers look like
from a host that is not the server.

This is the view a real unit has, so it sees a firewall. A "connection
refused" means the host answers and no service listens; a timeout usually
means a firewall drops the packets. The text says which one, because the
next step differs.

The probes run from the unit Pi or the outside host. On any target but
`local` they refuse to run on the server host itself, because loopback
passes through no firewall.
"""

import asyncio
import datetime
import json
import os
import socket
import ssl
import tempfile
import time

CONNECT_TIMEOUT_S = 5.0
CERT_MIN_DAYS = 14
PARTY = "GCS_PRELAUNCH"
_LOCAL_IPS_CACHE = None


def tcp_state(host, port, timeout=CONNECT_TIMEOUT_S):
    """-> 'open' | 'refused' | 'timeout' | 'unresolved' | 'error: ...'"""
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            return "open"
    except ConnectionRefusedError:
        return "refused"
    except socket.timeout:
        return "timeout"
    except socket.gaierror:
        return "unresolved"
    except OSError as e:
        return f"error: {e}"


def _why(state):
    return {"refused": "connection refused: the host answers and no service "
                       "listens",
            "timeout": "no answer: a firewall probably drops the packets",
            "unresolved": "the hostname does not resolve"}.get(state, state)


def _local_ips():
    global _LOCAL_IPS_CACHE
    if _LOCAL_IPS_CACHE is None:
        ips = {"127.0.0.1"}
        try:
            _, _, addrs = socket.gethostbyname_ex(socket.gethostname())
            ips.update(addrs)
        except OSError:
            pass
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("192.0.2.1", 9))
            ips.add(s.getsockname()[0])
            s.close()
        except OSError:
            pass
        _LOCAL_IPS_CACHE = ips
    return _LOCAL_IPS_CACHE


def is_this_host(host):
    try:
        return socket.gethostbyname(host) in _local_ips()
    except OSError:
        return False


def _wrong_vantage(ctx, host):
    """A fault text when this probe runs on the server it tests."""
    if ctx.role != "local" and is_this_host(host):
        return (f"this probe runs on {host}, the server itself. Loopback "
                f"passes through no firewall. Run it from another host")
    return None


def _services(ctx, names):
    all_svc = ctx.target.get("services") or {}
    return [(n, all_svc[n]) for n in names if all_svc.get(n)]


def ports(ctx):
    svcs = _services(ctx, ("auth", "comm", "storage"))
    if not svcs:
        return "SKIP", f"target {ctx.name} has no auth, comm or storage service"
    faults, oks = [], []
    for name, s in svcs:
        if s.get("public") is False:
            oks.append(f"{name} not public")
            continue
        wrong = _wrong_vantage(ctx, s["host"])
        if wrong:
            return "FAIL", wrong
        st = tcp_state(s["host"], s["port"])
        if st == "open":
            oks.append(f"{name} {s['host']}:{s['port']}")
        else:
            faults.append(f"{name} {s['host']}:{s['port']}: {_why(st)}")
    if faults:
        return "FAIL", "; ".join(faults)
    return "PASS", "open: " + ", ".join(oks)


def _decode_cert(der):
    """Parse a DER certificate with no extra package. -> dict or None."""
    pem = ssl.DER_cert_to_PEM_cert(der)
    with tempfile.NamedTemporaryFile("w", suffix=".pem") as fh:
        fh.write(pem)
        fh.flush()
        try:
            return ssl._ssl._test_decode_cert(fh.name)
        except (AttributeError, ssl.SSLError, OSError):
            return None


def cert_info(host, port, ca_file=None, verify=True, timeout=CONNECT_TIMEOUT_S):
    """-> (days_left or None, fault or None). verify=False reads the expiry
    of a self-signed certificate that no CA file covers."""
    ctx = ssl.create_default_context(cafile=ca_file)
    if not verify:
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
    try:
        with socket.create_connection((host, int(port)),
                                      timeout=timeout) as raw:
            with ctx.wrap_socket(raw, server_hostname=host) as tls:
                info = _decode_cert(tls.getpeercert(True))
    except ssl.SSLCertVerificationError as e:
        return None, f"certificate not valid: {e.verify_message}"
    except (ssl.SSLError, OSError) as e:
        return None, f"TLS failed: {e}"
    if not info or not info.get("notAfter"):
        return None, "cannot read the certificate"
    end = ssl.cert_time_to_seconds(info["notAfter"])
    return int((end - time.time()) // 86400), None


def tls(ctx):
    svcs = [(n, s) for n, s in _services(ctx, ("auth", "comm", "storage"))
            if s.get("tls", True) and s.get("public") is not False]
    if not svcs:
        return "SKIP", f"target {ctx.name} has no TLS service"
    faults, oks = [], []
    for name, s in svcs:
        days, err = cert_info(s["host"], s["port"], s.get("ca_file"),
                              s.get("verify_tls", True))
        where = f"{name} {s['host']}:{s['port']}"
        if err:
            faults.append(f"{where}: {err}")
        elif days < CERT_MIN_DAYS:
            faults.append(f"{where}: the certificate has {days} days left")
        else:
            oks.append(f"{where} {days} days")
    if faults:
        return "FAIL", "; ".join(faults)
    return "PASS", ", ".join(oks)


def _srv(ctx):
    a = ctx.service("auth")
    user, code = ctx.account()
    return {"auth": True, "auth_ip": a["host"], "auth_port": a["port"],
            "account": {"user": user, "code": code}}


def _gcs_login(srv):
    """-> (comm host, comm port, key) from the real web login."""
    import gcs_client
    return gcs_client._login_key(srv)


async def _open_session(srv, listen_s):
    """A real GCS session: login, wss to comm, de_auth. -> parties heard."""
    import gcs_client
    client = gcs_client.GCSClient(srv, PARTY)
    ws = await client.connect()
    heard, end = set(), time.time() + listen_s
    try:
        while time.time() < end:
            try:
                raw = await asyncio.wait_for(ws.recv(),
                                             max(0.1, end - time.time()))
            except asyncio.TimeoutError:
                break
            if isinstance(raw, str):
                sd = (json.loads(raw) or {}).get("sd")
                if sd and sd != PARTY:
                    heard.add(sd)
    finally:
        await ws.close()
    return heard


def login(ctx):
    srv = _srv(ctx)
    wrong = _wrong_vantage(ctx, srv["auth_ip"])
    if wrong:
        return "FAIL", wrong
    st = tcp_state(srv["auth_ip"], srv["auth_port"])
    if st != "open":
        return "FAIL", (f"auth {srv['auth_ip']}:{srv['auth_port']}: {_why(st)}")
    try:
        _gcs_login(srv)
    except Exception as e:
        return "FAIL", (f"the test account cannot log in, so check that it "
                        f"exists on this server with GCS permission: {e}")
    try:
        heard = asyncio.run(asyncio.wait_for(_open_session(srv, 3.0), 30))
    except Exception as e:
        return "FAIL", (f"login worked, but the GCS session to the comm server "
                        f"failed: {e!r}")
    return "PASS", f"logged in and opened a GCS session; {len(heard)} parties heard"


def public_host(ctx):
    srv = _srv(ctx)
    try:
        host, port, _ = _gcs_login(srv)
    except Exception as e:
        return "FAIL", f"the test account cannot log in: {e}"
    st = tcp_state(host, port)
    if st != "open":
        return "FAIL", (f"de_auth announces public_host {host}:{port} for the "
                        f"comm server, and this probe cannot reach it "
                        f"({_why(st)}). public_host may be stale")
    return "PASS", f"announced comm {host}:{port} is reachable"


def relay_control_closed(ctx):
    r = ctx.service("relay")
    if r.get("host") and ctx.role == "local":
        return "SKIP", "on the local target the probe is on the server"
    wrong = _wrong_vantage(ctx, r["host"])
    if wrong:
        return "FAIL", wrong
    st = tcp_state(r["host"], r["control_port"])
    if st == "open":
        return "FAIL", (f"relay control port {r['host']}:{r['control_port']} "
                        f"is open to this host. Open it only to comm servers")
    return "PASS", f"relay control port is closed here ({st})"
