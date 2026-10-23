#!/usr/bin/env python3
"""Offline unit tests for bt_spp_bridge (SL-0105) - the RFCOMM client
half of the bridge is a plain socket, so pump()/handle_client() are
exercised end to end with a socketpair as the 'RFCOMM client' and a
loopback TCP echo server as the fcb proxy:

    python3 engine/test_bt_spp.py

Covered:
  * bytes move both directions (client->proxy and proxy->client)
  * a client hang-up ends the session and half-closes the proxy side
  * handle_client re-arms `rate <baud>` on the ctrl port per client
"""
import os
import socket
import sys
import threading
import time
import unittest

sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "var", "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bt_spp_bridge as spp


def echo_server():
    """One-shot TCP echo: accept, bounce bytes until EOF, close."""
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]
    state = {"got_eof": False}

    def run():
        c, _ = srv.accept()
        try:
            while True:
                data = c.recv(65536)
                if not data:
                    state["got_eof"] = True
                    break
                c.sendall(data)
        finally:
            c.close()
            srv.close()

    threading.Thread(target=run, daemon=True).start()
    return port, state


class TestPump(unittest.TestCase):

    def test_bidirectional_bytes(self):
        port, _ = echo_server()
        cli_end, bridge_end = socket.socketpair()
        tcp = socket.create_connection(("127.0.0.1", port), 5)
        t = threading.Thread(target=spp.pump, args=(bridge_end, tcp),
                             daemon=True)
        t.start()
        cli_end.sendall(b"c2s:hello")
        self.assertEqual(cli_end.recv(64), b"c2s:hello")
        cli_end.sendall(b"x" * 4000)
        got = b""
        while len(got) < 4000:
            got += cli_end.recv(8192)
        self.assertEqual(got, b"x" * 4000)
        cli_end.close()
        t.join(5)
        self.assertFalse(t.is_alive(), "pump did not end on client EOF")
        tcp.close()

    def test_client_eof_halfcloses_proxy(self):
        port, state = echo_server()
        cli_end, bridge_end = socket.socketpair()
        tcp = socket.create_connection(("127.0.0.1", port), 5)
        t = threading.Thread(target=spp.pump, args=(bridge_end, tcp),
                             daemon=True)
        t.start()
        cli_end.sendall(b"ping")
        self.assertEqual(cli_end.recv(16), b"ping")
        cli_end.close()
        t.join(5)
        tcp.close()
        deadline = time.time() + 5
        while not state["got_eof"] and time.time() < deadline:
            time.sleep(0.05)
        self.assertTrue(state["got_eof"],
                        "proxy side never saw the client hang-up")


class TestHandleClient(unittest.TestCase):

    def test_dials_proxy_and_arms_rate(self):
        # a fake proxy ctrl port records the `rate` line the bridge
        # sends when a client connects (a baud on the link)
        ctrl = socket.socket()
        ctrl.bind(("127.0.0.1", 0))
        ctrl.listen(1)
        got = []

        def ctrl_serve():
            c, _ = ctrl.accept()
            got.append(c.recv(64))
            c.close()
            ctrl.close()

        threading.Thread(target=ctrl_serve, daemon=True).start()
        ctrl_port = ctrl.getsockname()[1]
        port, _ = echo_server()
        cli_end, bridge_end = socket.socketpair()
        t = threading.Thread(
            target=spp.handle_client,
            args=(bridge_end, port, ctrl_port, 57600), daemon=True)
        t.start()
        cli_end.sendall(b"mav")
        self.assertEqual(cli_end.recv(16), b"mav")
        deadline = time.time() + 5
        while not got and time.time() < deadline:
            time.sleep(0.05)
        self.assertEqual(got, [b"rate 57600\n"])
        cli_end.close()
        t.join(5)

    def test_proxy_refused_drops_client_cleanly(self):
        # nothing listens on this port - handle_client must just log
        # and return (the accept loop keeps serving)
        dead = socket.socket()
        dead.bind(("127.0.0.1", 0))
        port = dead.getsockname()[1]
        dead.close()
        _, bridge_end = socket.socketpair()
        t0 = time.time()
        spp.handle_client(bridge_end, port, 0, 0)
        self.assertLess(time.time() - t0, 5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
