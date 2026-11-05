"""turn_test.py - the TURN probe against a small fake TURN server. The fake
checks the same MESSAGE-INTEGRITY a real coturn checks, so the codec is
tested. It can refuse credentials and it can block its relay port."""

import hashlib
import hmac
import os
import socket
import struct
import sys
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import prelaunch  # noqa: E402
from prelaunch_checks import turn as T  # noqa: E402

REALM, NONCE = b"test.realm", b"abcd1234"


class FakeTurn(threading.Thread):
    def __init__(self, user="u1", password="pw1", block_relay=False,
                 stream=False, open_relay=False):
        super().__init__(daemon=True)
        self.open_relay = open_relay
        self.user, self.password = user, password
        self.block_relay, self.stream = block_relay, stream
        kind = socket.SOCK_STREAM if stream else socket.SOCK_DGRAM
        self.sock = socket.socket(socket.AF_INET, kind)
        self.sock.bind(("127.0.0.1", 0))
        if stream:
            self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.relay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.relay.bind(("127.0.0.1", 0))
        self.relay.settimeout(0.05)
        self.client = None
        self.perms = set()
        self.released = False
        self.stop_flag = False
        self.key = hashlib.md5(
            f"{user}:{REALM.decode()}:{password}".encode()).digest()

    def _send(self, data):
        if self.stream:
            self.conn.sendall(data)
        else:
            self.sock.sendto(data, self.client)

    def _integrity_ok(self, data):
        i = 20
        while i < len(data):
            t, n = struct.unpack("!HH", data[i:i + 4])
            if t == T.A_INTEGRITY:
                head = data[:2] + struct.pack("!H", i - 20 + 24) + data[4:i]
                mac = hmac.new(self.key, head, hashlib.sha1).digest()
                return mac == data[i + 4:i + 24]
            i += 4 + n + (-n % 4)
        return False

    def handle(self, data):
        mtype, tx, attrs = T.decode(data)
        method = mtype & 0x000F
        if mtype == T.BINDING:
            self._send(T.encode(0x0101, tx, [(T.A_MAPPED, T.encode_xor_addr(
                "127.0.0.1", 5555))]))
        elif mtype == T.SEND_IND:
            ip, port = T.xor_addr(attrs[T.A_PEER])
            if ip in self.perms:
                self.relay.sendto(attrs[T.A_DATA], (ip, port))
        elif not self.open_relay and (
                T.A_INTEGRITY not in attrs or not self._integrity_ok(data) or
                attrs.get(T.A_USERNAME, b"").decode() != self.user):
            self._send(T.encode(method | 0x110, tx, [
                (T.A_ERROR, struct.pack("!HBB", 0, 4, 1) + b"Unauthorized"),
                (T.A_REALM, REALM), (T.A_NONCE, NONCE)]))
        elif method == T.ALLOCATE:
            self._send(T.encode(0x0103, tx, [
                (T.A_RELAYED, T.encode_xor_addr(
                    "127.0.0.1", self.relay.getsockname()[1])),
                (T.A_MAPPED, T.encode_xor_addr("127.0.0.1", 5555))]))
        elif method == T.PERMISSION:
            self.perms.add(T.xor_addr(attrs[T.A_PEER])[0])
            self._send(T.encode(0x0108, tx))
        elif method == T.REFRESH:
            self.released = True
            self._send(T.encode(0x0104, tx))

    def run(self):
        self.sock.settimeout(0.05)
        if self.stream:
            while not self.stop_flag:
                try:
                    self.conn, _ = self.sock.accept()
                    break
                except socket.timeout:
                    pass
            if self.stop_flag:
                return
            self.conn.settimeout(0.05)
        buf = b""
        while not self.stop_flag:
            try:
                if self.stream:
                    chunk = self.conn.recv(65535)
                    if not chunk:
                        return
                    buf += chunk
                    while len(buf) >= 20:
                        n = 20 + struct.unpack("!H", buf[2:4])[0]
                        if len(buf) < n:
                            break
                        self.handle(buf[:n])
                        buf = buf[n:]
                else:
                    data, self.client = self.sock.recvfrom(65535)
                    self.handle(data)
            except socket.timeout:
                pass
            except OSError:
                return
            try:
                data, frm = self.relay.recvfrom(65535)
                if not self.block_relay and frm[0] in self.perms:
                    self._send(T.encode(T.DATA_IND, os.urandom(12), [
                        (T.A_PEER, T.encode_xor_addr(*frm)),
                        (T.A_DATA, data)]))
            except socket.timeout:
                pass

    def stop(self):
        self.stop_flag = True


class Flow(unittest.TestCase):
    def serve(self, **kw):
        s = FakeTurn(**kw)
        s.start()
        self.addCleanup(s.stop)
        return s

    def test_stun_binding(self):
        s = self.serve()
        self.assertIn("127.0.0.1",
                      T.check_entry(f"stun:127.0.0.1:{s.port}", None, None))

    def test_turn_udp_data_both_ways_and_release(self):
        s = self.serve()
        out = T.check_entry(f"turn:127.0.0.1:{s.port}", "u1", "pw1")
        self.assertIn("both ways", out)

    def test_turn_tcp(self):
        s = self.serve(stream=True)
        out = T.check_entry(f"turn:127.0.0.1:{s.port}?transport=tcp",
                            "u1", "pw1")
        self.assertIn("both ways", out)

    def test_wrong_credentials(self):
        s = self.serve()
        with self.assertRaises(T.TurnFault) as e:
            T.check_entry(f"turn:127.0.0.1:{s.port}", "u1", "wrong")
        self.assertIn("credentials", str(e.exception))

    def test_blocked_relay_range_with_open_control_port(self):
        s = self.serve(block_relay=True)
        with self.assertRaises(T.TurnFault) as e:
            T.check_entry(f"turn:127.0.0.1:{s.port}", "u1", "pw1")
        self.assertIn("relay port range is blocked", str(e.exception))

    def test_open_relay_is_a_fault_even_when_data_flows(self):
        s = self.serve(open_relay=True)
        with self.assertRaises(T.TurnFault) as e:
            T.check_entry(f"turn:127.0.0.1:{s.port}", "u1", "pw1")
        self.assertIn("open relay", str(e.exception))
        self.assertIn("data flows", str(e.exception))

    def test_closed_port(self):
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
        probe.close()
        with self.assertRaises(T.TurnFault):
            T.check_entry(f"stun:127.0.0.1:{port}", None, None)

    def test_url_parse(self):
        self.assertEqual(T.parse_ice_url("turn:h:3478?transport=tcp"),
                         ("turn", "h", 3478, "tcp"))
        self.assertEqual(T.parse_ice_url("turns:h"), ("turns", "h", 5349, "tls"))
        self.assertEqual(T.parse_ice_url("stun:h:19302"),
                         ("stun", "h", 19302, "udp"))


class Report(unittest.TestCase):
    def test_credentials_stay_out_of_the_detail(self):
        s = FakeTurn()
        s.start()
        self.addCleanup(s.stop)
        ctx = prelaunch.Ctx({"name": "t", "services": {}}, "/tmp", "v")
        entry = {"urls": f"turn:127.0.0.1:{s.port}", "username": "u1",
                 "password": "wrong-secret"}
        st, detail = T.allocate_check(ctx, [("web", "F", [entry], None)])
        self.assertEqual(st, "FAIL")
        self.assertNotIn("wrong-secret", ctx.scrub(detail))
        self.assertIn("wrong-secret", ctx.secrets)

    def test_empty_lists_fail(self):
        ctx = prelaunch.Ctx({"name": "t", "services": {}}, "/tmp", "v")
        self.assertEqual(T.allocate_check(ctx, [("web", "F", [], None)])[0],
                         "FAIL")


if __name__ == "__main__":
    unittest.main()
