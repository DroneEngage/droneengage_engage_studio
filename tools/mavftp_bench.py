#!/usr/bin/env python3
"""mavftp_bench.py - time MAVFTP over a UDP MAVLink endpoint, as a GCS sees it.

Point it at any GCS-side endpoint (a relay pair's GCS port, a SITL udp
port) and it measures the two things a param load is made of:

    rtt    N single ReadFile request/reply pairs (one block each)
    burst  full downloads of the file with BurstReadFile; gaps are refilled
           with ReadFile, as Mission Planner does

Only one GCS can hold a relay pair - close Mission Planner first.

    tools/mavftp_bench.py --gcs 10.33.232.200:40802 --sysid 40 --label fixed

The last line is a JSON summary; --out also writes it to a file.
"""

import argparse
import json
import os
import socket
import statistics
import struct
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

OP_TERMINATE, OP_RESET, OP_OPEN_RO, OP_READ, OP_BURST = 1, 2, 4, 5, 15
OP_ACK, OP_NAK = 128, 129
ERR_EOF = 6
BLOCK = 239


class Ftp:
    def __init__(self, p_gcs, p_sysid, p_compid):
        v_host, v_port = p_gcs.rsplit(":", 1)
        self.m_addr = (v_host, int(v_port))
        self.m_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.m_sock.settimeout(0.05)
        self.m_mav = mavutil.mavlink.MAVLink(None, srcSystem=250, srcComponent=190)
        self.m_mav.robust_parsing = True
        self.m_sysid, self.m_compid = p_sysid, p_compid
        self.m_seq = 0
        self.m_session = 0
        self.m_stop = False
        self.m_fc_seen = 0
        threading.Thread(target=self._beat, daemon=True).start()

    def _send(self, p_msg):
        self.m_sock.sendto(p_msg.pack(self.m_mav), self.m_addr)

    def _beat(self):
        while not self.m_stop:
            self._send(self.m_mav.heartbeat_encode(6, 8, 0, 0, 0))
            time.sleep(1.0)

    def request(self, p_op, p_size=0, p_offset=0, p_data=b""):
        self.m_seq = (self.m_seq + 1) & 0xFFFF
        v_payload = struct.pack("<HBBBBBBI", self.m_seq, self.m_session, p_op, p_size,
                                0, 0, 0, p_offset) + p_data
        self._send(self.m_mav.file_transfer_protocol_encode(
            0, self.m_sysid, self.m_compid, list(v_payload.ljust(251, b"\0"))))

    def poll(self, p_timeout):
        """@returns FTP replies (op, size, req_op, burst_done, offset, data) for us"""
        v_end = time.monotonic() + p_timeout
        v_out = []
        while time.monotonic() < v_end and not v_out:
            try:
                v_buf = self.m_sock.recv(2048)
            except socket.timeout:
                continue
            for v_msg in self.m_mav.parse_buffer(v_buf) or []:
                v_type = v_msg.get_type()
                if v_type == "HEARTBEAT" and v_msg.get_srcSystem() == self.m_sysid:
                    self.m_fc_seen += 1
                if v_type != "FILE_TRANSFER_PROTOCOL" or v_msg.target_system != 250:
                    continue
                v_raw = bytes(v_msg.payload)
                _, _, v_op, v_size, v_req, v_done, _, v_off = struct.unpack("<HBBBBBBI", v_raw[:12])
                v_out.append((v_op, v_size, v_req, v_done, v_off, v_raw[12:12 + v_size]))
        return v_out

    def call(self, p_op, p_size=0, p_offset=0, p_data=b"", p_timeout=1.0, p_tries=8):
        """Retry until the reply to p_op arrives. @returns (reply, retries)"""
        for v_try in range(p_tries):
            self.request(p_op, p_size, p_offset, p_data)
            v_end = time.monotonic() + p_timeout
            while time.monotonic() < v_end:
                for v_r in self.poll(v_end - time.monotonic()):
                    if v_r[2] == p_op and v_r[0] in (OP_ACK, OP_NAK):
                        if p_op != OP_READ or v_r[0] == OP_NAK or v_r[4] == p_offset:
                            return v_r, v_try
        raise TimeoutError(f"no reply to op {p_op}")

    def open(self, p_path):
        self.call(OP_TERMINATE)
        v_name = p_path.encode()
        v_r, _ = self.call(OP_OPEN_RO, len(v_name), 0, v_name)
        if v_r[0] != OP_ACK:
            raise RuntimeError(f"open {p_path}: NAK {v_r[5]!r}")
        self.m_session = 0

    def wait_fc(self, p_secs):
        v_end = time.monotonic() + p_secs
        while time.monotonic() < v_end and self.m_fc_seen < 1:
            self.poll(0.2)
        return self.m_fc_seen > 0


def bench_rtt(p_ftp, p_path, p_count):
    p_ftp.open(p_path)
    v_rtts, v_retries = [], 0
    for _ in range(p_count):
        v_t0 = time.monotonic()
        _, v_retry = p_ftp.call(OP_READ, BLOCK, 0)
        v_retries += v_retry
        if v_retry == 0:
            v_rtts.append((time.monotonic() - v_t0) * 1000.0)
    p_ftp.call(OP_TERMINATE)
    v_rtts.sort()
    return {"count": len(v_rtts), "retries": v_retries,
            "p50_ms": round(statistics.median(v_rtts), 1),
            "p95_ms": round(v_rtts[int(len(v_rtts) * 0.95)], 1),
            "max_ms": round(v_rtts[-1], 1)}


def bench_burst(p_ftp, p_path):
    p_ftp.open(p_path)
    v_t0 = time.monotonic()
    v_got = {}
    v_eof = None
    v_next_burst = 0
    p_ftp.request(OP_BURST, BLOCK, 0)
    v_quiet_since = time.monotonic()
    v_gaps = 0
    v_bursts = 1
    while True:
        v_replies = p_ftp.poll(0.1)
        for v_op, v_size, v_req, v_done, v_off, v_data in v_replies:
            if v_req != OP_BURST:
                continue
            v_quiet_since = time.monotonic()
            if v_op == OP_NAK:
                if v_data and v_data[0] == ERR_EOF:
                    v_eof = v_next_burst
                continue
            v_got[v_off] = v_data
            v_next_burst = max(v_next_burst, v_off + v_size)
            if v_done:
                p_ftp.request(OP_BURST, BLOCK, v_next_burst)
                v_bursts += 1
        if v_eof is not None:
            break
        if time.monotonic() - v_quiet_since > 0.5:
            p_ftp.request(OP_BURST, BLOCK, v_next_burst)
            v_quiet_since = time.monotonic()
    v_burst_s = time.monotonic() - v_t0
    # refill what the burst lost
    v_off = 0
    while v_off < v_eof:
        if v_off in v_got:
            v_off += len(v_got[v_off])
            continue
        v_gaps += 1
        v_r, _ = p_ftp.call(OP_READ, BLOCK, v_off)
        if v_r[0] != OP_ACK:
            break
        v_got[v_off] = v_r[5]
    v_total = time.monotonic() - v_t0
    p_ftp.call(OP_TERMINATE)
    return {"bytes": v_eof, "secs": round(v_total, 2), "burst_secs": round(v_burst_s, 2),
            "blocks": len(v_got), "burst_requests": v_bursts,
            "gaps_refilled": v_gaps, "kB_s": round(v_eof / 1000.0 / v_total, 1)}


def main():
    v_ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    v_ap.add_argument("--gcs", required=True, help="GCS endpoint ip:port")
    v_ap.add_argument("--sysid", type=int, required=True)
    v_ap.add_argument("--compid", type=int, default=1)
    v_ap.add_argument("--file", default="@PARAM/param.pck?withdefaults=1")
    v_ap.add_argument("--rtt", type=int, default=300)
    v_ap.add_argument("--bursts", type=int, default=5)
    v_ap.add_argument("--label", default="")
    v_ap.add_argument("--out")
    v_args = v_ap.parse_args()

    v_ftp = Ftp(v_args.gcs, v_args.sysid, v_args.compid)
    if not v_ftp.wait_fc(15):
        sys.exit(f"no HEARTBEAT from sysid {v_args.sysid} on {v_args.gcs}")
    v_rtt = bench_rtt(v_ftp, v_args.file, v_args.rtt)
    print("rtt  ", json.dumps(v_rtt), flush=True)
    v_bursts = []
    for _ in range(v_args.bursts):
        v_bursts.append(bench_burst(v_ftp, v_args.file))
        print("burst", json.dumps(v_bursts[-1]), flush=True)
    v_ftp.m_stop = True
    v_result = {"label": v_args.label, "gcs": v_args.gcs, "file": v_args.file, "rtt": v_rtt,
                "bursts": v_bursts,
                "median_secs": statistics.median(b["secs"] for b in v_bursts)}
    if v_args.out:
        with open(v_args.out, "w") as v_f:
            json.dump(v_result, v_f, indent=2)
    print(json.dumps(v_result))


if __name__ == "__main__":
    main()
