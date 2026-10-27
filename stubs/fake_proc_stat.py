#!/usr/bin/env python3
"""fake_proc_stat.py - SL-0067 live fake /proc/stat writer.

Launched per unit module whose `env:` carries DE_PROC_STAT_PATH (proc
fake_proc_<module>, before its reader - see engine/simlib.py). Every
tick it rewrites --out with an advancing aggregate `cpu ` line, so the
module's 1 Hz CPU sampler sees a steady synthetic load and reports its
board state over 6547.

The load percent comes from the --ctl JSON file ({"load": 0-100}),
polled each tick - a scenario `write_file` step into
<unit>/<module>/cpu_load.json flips busy<->idle. A missing or invalid
file defaults to idle (5%). Writes are atomic (tmp + replace) so the
reader never parses a half-written file.
"""

import argparse
import json
import os
import sys
import time

# ticks added to the aggregate line per writer tick - with a 10 Hz tick
# each column advances ~100 jiffies/sec like a real busy/idle kernel
TICK_TOTAL = 100


def read_load(path, default=5):
    try:
        with open(path) as fh:
            v = float((json.load(fh) or {}).get("load", default))
        return min(100.0, max(0.0, v))
    except (OSError, ValueError):
        return default


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True,
                    help="fake /proc/stat file (the DE_PROC_STAT_PATH)")
    ap.add_argument("--ctl", required=True,
                    help="JSON file read each tick: {\"load\": 0-100}")
    ap.add_argument("--hz", type=float, default=10.0,
                    help="writer ticks per second (default 10)")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)),
                exist_ok=True)
    tmp = args.out + ".tmp"

    user = nice = system = idle = iowait = irq = softirq = steal = 0
    last_load = None
    while True:
        load = read_load(args.ctl)
        if load != last_load:
            print(f"fake_proc_stat {args.out}: load={load:g}%",
                  flush=True)
            last_load = load
        busy = int(round(TICK_TOTAL * load / 100.0))
        system += busy
        idle += TICK_TOTAL - busy
        # aggregate `cpu ` line (de_cpu_monitor reads this one only);
        # one mirrored cpu0 row keeps the file realistic
        text = (f"cpu  {user} {nice} {system} {idle} {iowait} "
                f"{irq} {softirq} {steal} 0 0\n"
                f"cpu0 {user} {nice} {system} {idle} {iowait} "
                f"{irq} {softirq} {steal} 0 0\n")
        with open(tmp, "w") as fh:
            fh.write(text)
        os.replace(tmp, args.out)
        time.sleep(1.0 / args.hz)


if __name__ == "__main__":
    sys.exit(main())
