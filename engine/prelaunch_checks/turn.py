"""prelaunch_checks/turn.py - stage 2: prove each STUN and TURN server in
the shipped ICE lists really works.

An open port proves little. For a TURN entry this runs the real flow:
allocate with the shipped credentials, then send UDP from a peer socket to
the allocated relay port and read it back as a Data indication, then send
back the other way. That finds wrong credentials, a closed relay port range
(with 3478 still open), and a bad TLS certificate. The allocation is
released at the end.

The STUN codec follows RFC 5389 and RFC 5766. It uses only the standard
library, so it runs on a Pi with no extra package.
"""

import hashlib
import hmac
import os
import select
import socket
import ssl
import struct
import time

from prelaunch_checks import config

MAGIC = 0x2112A442
A_USERNAME, A_INTEGRITY, A_ERROR = 0x0006, 0x0008, 0x0009
A_LIFETIME, A_PEER, A_DATA = 0x000D, 0x0012, 0x0013
A_REALM, A_NONCE, A_RELAYED = 0x0014, 0x0015, 0x0016
A_TRANSPORT, A_MAPPED = 0x0019, 0x0020
BINDING, ALLOCATE, REFRESH, PERMISSION = 0x0001, 0x0003, 0x0004, 0x0008
SEND_IND, DATA_IND = 0x0016, 0x0017
TIMEOUT_S = 5.0


class TurnFault(Exception):
    """A plain-words reason, shown in the report."""


# ---- codec ----------------------------------------------------------

def _attr(atype, value):
    pad = b"\0" * (-len(value) % 4)
    return struct.pack("!HH", atype, len(value)) + value + pad


def encode(mtype, txid, attrs=(), key=None):
    body = b"".join(_attr(t, v) for t, v in attrs)
    if key is None:
        return struct.pack("!HHI", mtype, len(body), MAGIC) + txid + body
    # MESSAGE-INTEGRITY covers the message with the length that counts it
    head = struct.pack("!HHI", mtype, len(body) + 24, MAGIC) + txid
    mac = hmac.new(key, head + body, hashlib.sha1).digest()
    return head + body + _attr(A_INTEGRITY, mac)


def decode(data):
    """-> (type, txid, {attr: value}); first value wins for a repeated attr."""
    if len(data) < 20:
        raise TurnFault("the server sent a short STUN message")
    mtype, length, magic = struct.unpack("!HHI", data[:8])
    if magic != MAGIC:
        raise TurnFault("the server did not answer with STUN")
    attrs, i, end = {}, 20, min(20 + length, len(data))
    while i + 4 <= end:
        t, n = struct.unpack("!HH", data[i:i + 4])
        attrs.setdefault(t, data[i + 4:i + 4 + n])
        i += 4 + n + (-n % 4)
    return mtype, data[8:20], attrs


def xor_addr(value):
    """XOR-*-ADDRESS -> (ip, port), IPv4 only."""
    family, xport = struct.unpack("!xBH", value[:4])
    if family != 1:
        raise TurnFault("the server gave an IPv6 address; only IPv4 is handled")
    port = xport ^ (MAGIC >> 16)
    raw = struct.unpack("!I", value[4:8])[0] ^ MAGIC
    return socket.inet_ntoa(struct.pack("!I", raw)), port


def encode_xor_addr(ip, port):
    raw = struct.unpack("!I", socket.inet_aton(ip))[0] ^ MAGIC
    return struct.pack("!xBHI", 1, port ^ (MAGIC >> 16), raw)


def error_code(attrs):
    v = attrs.get(A_ERROR)
    if not v or len(v) < 4:
        return None
    return v[2] * 100 + v[3]


# ---- transports -----------------------------------------------------

class _Link:
    """One client connection: UDP, TCP or TLS. recv returns one message."""

    def __init__(self, host, port, proto, ca_file=None):
        self.proto = proto
        self.buf = b""
        kind = socket.SOCK_DGRAM if proto == "udp" else socket.SOCK_STREAM
        self.sock = socket.socket(socket.AF_INET, kind)
        self.sock.settimeout(TIMEOUT_S)
        try:
            self.sock.connect((host, port))
        except OSError as e:
            self.sock.close()
            raise TurnFault(f"cannot connect to {host}:{port}/{proto}: {e}")
        if proto == "tls":
            ctx = ssl.create_default_context(cafile=ca_file)
            try:
                self.sock = ctx.wrap_socket(self.sock, server_hostname=host)
            except (ssl.SSLError, OSError) as e:
                self.sock.close()
                raise TurnFault(f"TLS to {host}:{port} failed: {e}")

    def send(self, data):
        self.sock.sendall(data)

    def recv(self, timeout):
        self.sock.settimeout(timeout)
        if self.proto == "udp":
            return self.sock.recv(65535)
        while True:
            if len(self.buf) >= 20:
                n = 20 + struct.unpack("!H", self.buf[2:4])[0]
                n += -n % 4
                if len(self.buf) >= n:
                    msg, self.buf = self.buf[:n], self.buf[n:]
                    return msg
            chunk = self.sock.recv(65535)
            if not chunk:
                raise OSError("the server closed the connection")
            self.buf += chunk

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


def _transact(link, msg, txid, what):
    """Send a request, retry on UDP, and return the answer with this txid."""
    tries = 3 if link.proto == "udp" else 1
    deadline_each = TIMEOUT_S / tries
    last = None
    for _ in range(tries):
        try:
            link.send(msg)
            end = time.time() + deadline_each
            while time.time() < end:
                try:
                    m = decode(link.recv(max(0.05, end - time.time())))
                except socket.timeout:
                    break
                if m[1] == txid:
                    return m
        except OSError as e:
            last = e
    raise TurnFault(f"no answer to {what}"
                    + (f" ({last})" if last else "")
                    + ". The port is closed or blocked")


def _txid():
    return os.urandom(12)


# ---- the flows ------------------------------------------------------

def stun_binding(link):
    """-> the (ip, port) the server sees."""
    tx = _txid()
    mtype, _, attrs = _transact(link, encode(BINDING, tx), tx, "STUN binding")
    if mtype != BINDING | 0x100 or A_MAPPED not in attrs:
        raise TurnFault("STUN binding got no mapped address")
    return xor_addr(attrs[A_MAPPED])


def _request(link, method, attrs, auth, what):
    """An authenticated request. Returns the attrs of a success answer."""
    tx = _txid()
    if auth is None:            # an open server: see allocate()
        msg = encode(method, tx, list(attrs))
    else:
        msg = encode(method, tx, list(attrs) + [
            (A_USERNAME, auth["user"].encode()), (A_REALM, auth["realm"]),
            (A_NONCE, auth["nonce"])], auth["key"])
    mtype, _, got = _transact(link, msg, tx, what)
    if mtype != method | 0x100:
        raise TurnFault(f"{what} was refused with error {error_code(got)}")
    return got


def allocate(link, user, password):
    """-> (auth, relayed (ip, port), mapped (ip, port)). auth is None when
    the server gave the allocation with no credentials: an open relay."""
    transport = (A_TRANSPORT, struct.pack("!B3x", 17))
    tx = _txid()
    mtype, _, got = _transact(link, encode(ALLOCATE, tx, [transport]), tx,
                              "TURN allocate")
    if mtype == ALLOCATE | 0x100:
        if A_RELAYED not in got or A_MAPPED not in got:
            raise TurnFault("the TURN allocate answer has no relay address")
        return None, xor_addr(got[A_RELAYED]), xor_addr(got[A_MAPPED])
    if error_code(got) != 401 or A_REALM not in got or A_NONCE not in got:
        raise TurnFault(f"TURN allocate got error {error_code(got)}")
    realm, nonce = got[A_REALM], got[A_NONCE]
    key = hashlib.md5(f"{user}:{realm.decode()}:{password}".encode()).digest()
    auth = {"user": user, "realm": realm, "nonce": nonce, "key": key}
    tx = _txid()
    msg = encode(ALLOCATE, tx, [transport, (A_USERNAME, user.encode()),
                                (A_REALM, realm), (A_NONCE, nonce)], key)
    mtype, _, got = _transact(link, msg, tx, "TURN allocate with credentials")
    if mtype != ALLOCATE | 0x100:
        code = error_code(got)
        if code in (401, 431, 441):
            raise TurnFault(f"the server refused the TURN credentials "
                            f"(error {code})")
        raise TurnFault(f"TURN allocate was refused with error {code}")
    if A_RELAYED not in got or A_MAPPED not in got:
        raise TurnFault("the TURN allocate answer has no relay address")
    return auth, xor_addr(got[A_RELAYED]), xor_addr(got[A_MAPPED])


def _wait_data(link, want, seconds):
    """Read until a Data indication holds `want`. -> the peer address."""
    end = time.time() + seconds
    while time.time() < end:
        try:
            mtype, _, attrs = decode(link.recv(max(0.05, end - time.time())))
        except socket.timeout:
            return None
        if mtype == DATA_IND and attrs.get(A_DATA) == want:
            return xor_addr(attrs[A_PEER])
    return None


def relay_round_trip(link, auth, relayed, mapped):
    """Peer -> relay port -> client, then client -> relay -> peer. The
    first leg is what a firewall on the relay range blocks."""
    _request(link, PERMISSION, [(A_PEER, encode_xor_addr(mapped[0], 0))], auth,
             "CreatePermission")
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("0.0.0.0", 0))
    try:
        token = b"prelaunch-" + os.urandom(6).hex().encode()
        seen = None
        for _ in range(4):
            peer.sendto(token, relayed)
            seen = _wait_data(link, token, 1.0)
            if seen:
                break
        if not seen:
            raise TurnFault(f"allocate worked, but data to relay port "
                            f"{relayed[0]}:{relayed[1]} did not arrive. The "
                            f"TURN relay port range is blocked")
        back = b"back-" + token
        link.send(encode(SEND_IND, _txid(), [(A_PEER, encode_xor_addr(*seen)),
                                             (A_DATA, back)]))
        ready, _, _ = select.select([peer], [], [], 3.0)
        if not ready or peer.recv(2048) != back:
            raise TurnFault("data from the client through the relay did not "
                            "reach the peer")
    finally:
        peer.close()


def release(link, auth):
    """Give the allocation back. A failure here does not matter."""
    try:
        tx = _txid()
        life = (A_LIFETIME, struct.pack("!I", 0))
        if auth is None:
            msg = encode(REFRESH, tx, [life])
        else:
            msg = encode(REFRESH, tx, [life,
                                       (A_USERNAME, auth["user"].encode()),
                                       (A_REALM, auth["realm"]),
                                       (A_NONCE, auth["nonce"])], auth["key"])
        link.send(msg)
    except OSError:
        pass


# ---- urls -----------------------------------------------------------

def parse_ice_url(url):
    """stun:h:p | turn:h:p?transport=tcp | turns:h:p ->
    (scheme, host, port, proto)."""
    scheme, _, rest = url.partition(":")
    rest, _, query = rest.partition("?")
    host, _, port = rest.partition(":")
    secure = scheme in ("turns", "stuns")
    proto = "tls" if secure else "udp"
    if "transport=tcp" in query:
        proto = "tls" if secure else "tcp"
    return scheme, host, int(port) if port else (5349 if secure else 3478), \
        proto


def check_entry(url, user, password, ca_file=None):
    """Prove one url. Raises TurnFault. -> a short success text."""
    scheme, host, port, proto = parse_ice_url(url)
    try:
        host_ip = socket.gethostbyname(host)
    except OSError as e:
        raise TurnFault(f"cannot resolve {host}: {e}")
    link = _Link(host if proto == "tls" else host_ip, port, proto, ca_file)
    try:
        if scheme.startswith("stun"):
            ip, _ = stun_binding(link)
            return f"binding sees {ip}"
        if not user or not password:
            raise TurnFault("the TURN entry has no username or password")
        auth, relayed, mapped = allocate(link, user, password)
        try:
            relay_round_trip(link, auth, relayed, mapped)
        finally:
            release(link, auth)
        if auth is None:
            raise TurnFault("data flows, but the server gives an allocation "
                            "with NO credentials. It is an open relay that "
                            "anyone can use, and the shipped credentials "
                            "were not tested. Check lt-cred-mech in coturn")
        return "allocate and data both ways"
    except socket.timeout:
        raise TurnFault("the server stopped answering")
    finally:
        link.close()


def allocate_check(ctx, entries):
    """Run check_entry on each distinct (url, user). -> (status, detail)."""
    turn = (ctx.target.get("services") or {}).get("turn") or {}
    ca = turn.get("ca_file")
    done, faults, oks = set(), [], []
    for label, field, items, err in entries:
        if err:
            faults.append(f"{label}: {err}")
            continue
        for e in items or []:
            pwd = e.get("password") or e.get("credential")
            if pwd:
                ctx.secrets.add(pwd)
            for url in config._entry_urls(e):
                key = (url, e.get("username"), pwd)
                if key in done:
                    continue
                done.add(key)
                try:
                    oks.append(f"{url}: {check_entry(url, e.get('username'), pwd, ca)}")
                except TurnFault as f:
                    faults.append(f"{url} ({label}): {f}")
    if not done and not faults:
        return "FAIL", "no STUN or TURN entry to test: the ICE lists are empty"
    if faults:
        return "FAIL", "; ".join(faults)
    return "PASS", "; ".join(oks)


def allocate_probe(ctx):
    lists = config._ice_lists(ctx)
    if not lists:
        return "SKIP", f"target {ctx.name} has no ICE config files"
    return allocate_check(ctx, lists)
