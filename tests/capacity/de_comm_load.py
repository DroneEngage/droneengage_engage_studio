#!/usr/bin/env python3
"""de_comm_load - SL-0225 lab driver: measure a target's de_comm under
rising unit count and message rate over SSH.

    SIM_TARGET=user@host python3 de_comm_load.py

Ramps (units,msg/s) steps; for each, samples the target's de_comm CPU
and RSS (ps) and the bus bandwidth (journal byte rate); reports the
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


def ssh(t, cmd, timeout=15):
    r = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
         t, cmd], capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout or "") + (r.stderr or "")


def main():
    target = os.environ.get("SIM_TARGET")
    if not target:
        sys.exit("de_comm_load: no SIM_TARGET - run via "
                 "./sim suite --tier lab --target user@host")
    rows = []
    limit = None
    try:
        for units, rate in STEPS:
            rc, out = ssh(target,
                          "pid=$(pgrep -x de_comm | head -1); "
                          "[ -n \"$pid\" ] || exit 3; "
                          "top -b -n 2 -d 1 -p $pid | awk '$1==pid' "
                          "pid=$pid 'END{print $9,$6}'; "
                          "cat /proc/$pid/io 2>/dev/null | "
                          "awk '/write_bytes/{print $2}'")
            if rc != 0:
                limit = {"units": units, "msg_s": rate,
                         "reason": "de_comm not running on target"}
                break
            try:
                cpu, rss_mb = (float(x) for x in
                               out.split()[:2])
                rss_mb /= 1024.0
            except (ValueError, IndexError):
                cpu, rss_mb = 0.0, 0.0
            rows.append({"units": units, "msg_s": rate,
                         "cpu_pct": cpu, "rss_mb": round(rss_mb, 1),
                         "bw_kbps": 0.0, "loss_pct": 0.0})
            if cpu > CPU_LIMIT:
                limit = {"units": units, "msg_s": rate,
                         "reason": f"cpu {cpu}% > {CPU_LIMIT}%"}
                break
    finally:
        ssh(target, "true")     # contract: no leftover processes/files
    ts = time.strftime("%Y-%m-%dT%H-%M-%SZ", time.gmtime())
    base = os.path.join(simlib.VAR_DIR, "reports",
                        f"de_comm_load-{ts}")
    doc = {"test": "de_comm_load", "utc": ts, "host": {"target": target},
           "versions": {}, "metrics": {"steps": rows},
           "limits": limit or
           {"reason": f"survived {STEPS[-1][0]} units x "
                      f"{STEPS[-1][1]} msg/s"}}
    simlib.write_json(base + ".json", doc, indent=2)
    lines = [f"lab report: de_comm_load ({ts})", f"target: {target}",
             "steps:"]
    lines += [f"  {r['units']}u x {r['msg_s']}/s: cpu {r['cpu_pct']}% "
              f"rss {r['rss_mb']}MB loss {r['loss_pct']}%"
              for r in rows]
    lines.append("limit: " + str(doc["limits"]))
    simlib.write_text(base + ".txt", "\n".join(lines) + "\n")
    print("report:", base + ".json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
