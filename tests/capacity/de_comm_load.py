#!/usr/bin/env python3
"""de_comm_load - SL-0225 lab driver: measure a target's de_comm under
rising unit count and message rate over SSH.

    SIM_TARGET=user@host python3 de_comm_load.py

Ramps (units,msg/s) steps; for each, samples the target's de_comm CPU
and RSS (/proc) and UDP bandwidth and loss (/proc/net/snmp); reports the
highest step before degradation (cpu > 90% or loss > 1%). The report
pair lands in var/reports/de_comm_load-<utc>.{json,txt} in the
SL-0223 format. The target is left clean - every process the driver
starts is killed and its files removed.
"""

import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "engine"))
import simlib  # noqa: E402

STEPS = [(1, 10), (2, 50), (4, 100), (8, 200)]
CPU_LIMIT = 90.0
LOSS_LIMIT = 1.0


STEP_S = 8
PORT = 60000
PKT = 256
TAG = "de_comm_load"

# Runs ON the target (piped over ssh stdin, saved in a temp dir that
# the shell trap removes). Sends `units` UDP senders x `rate` msg/s to
# the de_comm module port for STEP_S seconds and reads real counters:
# /proc/<pid>/stat ticks (CPU), VmRSS, and /proc/net/snmp Udp deltas
# (InDatagrams, RcvbufErrors, InErrors). Prints one JSON line.
REMOTE_GEN = r"""
import json, os, socket, sys, time
units, rate, secs, port, pkt, pid = (int(x) for x in sys.argv[1:7])
def snmp():
    L = [l.split() for l in open("/proc/net/snmp") if l.startswith("Udp:")]
    return dict(zip(L[0][1:], (int(x) for x in L[1][1:])))
def ticks():
    f = open("/proc/%d/stat" % pid).read().rsplit(")", 1)[1].split()
    return int(f[11]) + int(f[12])
def rss_kb():
    for l in open("/proc/%d/status" % pid):
        if l.startswith("VmRSS:"):
            return int(l.split()[1])
    raise RuntimeError("no VmRSS")
socks = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
         for _ in range(units)]
data = b"x" * pkt
s0, t0, w0 = snmp(), ticks(), time.time()
sent = 0
nxt = time.time()
end = nxt + secs
while time.time() < end:
    for s in socks:
        try:
            s.sendto(data, ("127.0.0.1", port)); sent += 1
        except OSError:
            pass
    nxt += 1.0 / rate
    d = nxt - time.time()
    if d > 0:
        time.sleep(d)
el = time.time() - w0
time.sleep(0.3)
s1, t1 = snmp(), ticks()
hz = os.sysconf("SC_CLK_TCK")
drops = (s1["RcvbufErrors"] - s0["RcvbufErrors"]) + \
        (s1["InErrors"] - s0["InErrors"] - (s1["RcvbufErrors"] - s0["RcvbufErrors"]))
print(json.dumps({"sent": sent, "elapsed": el, "bytes": sent * pkt,
    "in_datagrams": s1["InDatagrams"] - s0["InDatagrams"],
    "drops": max(0, drops),
    "cpu_pct": 100.0 * (t1 - t0) / hz / (time.time() - w0),
    "rss_kb": rss_kb()}))
"""

# The shell side: temp dir, trap kills the generator and removes it.
REMOTE_SH = (
    "pid=$(pgrep -x de_comm | head -1); [ -n \"$pid\" ] || exit 3; "
    "d=$(mktemp -d /tmp/%(tag)s.XXXXXX) || exit 4; "
    "trap 'kill $(cat \"$d/pid\" 2>/dev/null) 2>/dev/null; "
    "rm -rf \"$d\"' EXIT INT TERM HUP; "
    "cat > \"$d/gen.py\"; "
    "python3 \"$d/gen.py\" %(units)d %(rate)d %(secs)d %(port)d %(pkt)d "
    "$pid > \"$d/out\" & echo $! > \"$d/pid\"; wait $!; "
    "cat \"$d/out\"")


def ssh(t, cmd, timeout=15, stdin=None):
    r = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
         t, cmd], capture_output=True, text=True, timeout=timeout,
        input=stdin)
    return r.returncode, (r.stdout or "") + (r.stderr or "")


def parse_sample(out):
    """Last JSON line of `out`; raise loudly if it is not a sample."""
    for line in reversed(out.strip().splitlines()):
        if line.startswith("{"):
            d = json.loads(line)
            for k in ("sent", "elapsed", "bytes", "in_datagrams",
                      "drops", "cpu_pct", "rss_kb"):
                if k not in d:
                    raise ValueError(f"sample missing {k}: {line}")
            return d
    raise ValueError(f"cannot parse sample from target output: {out!r}")


def leftovers(t):
    """Anything of ours still on the target (processes or files)."""
    rc, out = ssh(t, "ls -d /tmp/%s.* 2>/dev/null; " % TAG +
                  " pgrep -f '[d]e_comm_load[.][A-Za-z0-9]*/gen.py'"
                  " 2>/dev/null; true")
    return out.strip()


def main():
    target = os.environ.get("SIM_TARGET")
    if not target:
        sys.exit("de_comm_load: no SIM_TARGET - run via "
                 "./sim suite --tier lab --target user@host")
    rows = []
    limit = None
    try:
        for units, rate in STEPS:
            cmd = REMOTE_SH % {"tag": TAG, "units": units, "rate": rate,
                               "secs": STEP_S, "port": PORT, "pkt": PKT}
            rc, out = ssh(target, cmd, timeout=STEP_S + 30,
                          stdin=REMOTE_GEN)
            if rc == 3:
                limit = {"units": units, "msg_s": rate,
                         "reason": "de_comm not running on target"}
                break
            if rc != 0:
                sys.exit(f"de_comm_load: step {units}u x {rate}/s "
                         f"failed on target (rc {rc}): {out.strip()}")
            smp = parse_sample(out)      # raises on a bad sample
            sent = smp["sent"]
            lost = max(0, sent - smp["in_datagrams"]) if \
                smp["in_datagrams"] <= sent else 0
            loss = 100.0 * max(smp["drops"], lost) / sent if sent else 100.0
            rows.append({"units": units, "msg_s": rate,
                         "cpu_pct": round(smp["cpu_pct"], 1),
                         "rss_mb": round(smp["rss_kb"] / 1024.0, 1),
                         "bw_kbps": round(smp["bytes"] * 8 / 1000.0
                                          / smp["elapsed"], 1),
                         "loss_pct": round(loss, 2)})
            if rows[-1]["cpu_pct"] > CPU_LIMIT:
                limit = {"units": units, "msg_s": rate,
                         "reason": f"cpu {rows[-1]['cpu_pct']}% > "
                                   f"{CPU_LIMIT}%"}
                break
            if loss > LOSS_LIMIT:
                limit = {"units": units, "msg_s": rate,
                         "reason": f"loss {rows[-1]['loss_pct']}% > "
                                   f"{LOSS_LIMIT}%"}
                break
    finally:
        # belt and braces: the remote trap already cleans; kill any
        # straggler, then verify nothing of ours is left.
        ssh(target, "pkill -f '[d]e_comm_load[.][A-Za-z0-9]*/gen.py' 2>/dev/null; "
                    "rm -rf /tmp/%s.* 2>/dev/null; true" % TAG)
        left = leftovers(target)
        if left:
            print("de_comm_load: LEFTOVER on target:", left,
                  file=sys.stderr)
            rows_left = left
        else:
            rows_left = ""
    ts = time.strftime("%Y-%m-%dT%H-%M-%SZ", time.gmtime())
    base = os.path.join(simlib.VAR_DIR, "reports",
                        f"de_comm_load-{ts}")
    doc = {"test": "de_comm_load", "utc": ts, "host": {"target": target},
           "versions": {}, "metrics": {"steps": rows},
           "clean": not rows_left, "limits": limit or
           {"reason": f"survived {STEPS[-1][0]} units x "
                      f"{STEPS[-1][1]} msg/s"}}
    simlib.write_json(base + ".json", doc, indent=2)
    lines = [f"lab report: de_comm_load ({ts})", f"target: {target}",
             "steps:"]
    lines += [f"  {r['units']}u x {r['msg_s']}/s: cpu {r['cpu_pct']}% "
              f"rss {r['rss_mb']}MB bw {r['bw_kbps']}kbps "
              f"loss {r['loss_pct']}%"
              for r in rows]
    lines.append("limit: " + str(doc["limits"]))
    simlib.write_text(base + ".txt", "\n".join(lines) + "\n")
    print("report:", base + ".json")
    return 1 if rows_left else 0


if __name__ == "__main__":
    sys.exit(main())
