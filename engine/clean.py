"""sim clean - prune generated artifacts under var/run/ and var/results/.

  clean [--days N] [--all] [--dry-run]

var/run/<sim>:   deleted only when no pidfile under it is alive and ours -
                 a downed sim keeps its manifest for post-mortem, so age
                 decides, never liveness alone.
var/results/*:   deleted by age (legacy <ts>-<test> and <utc>/<name> alike).
--days N     keep anything touched in the last N days (default 7)
--all        ignore age; live sims are still never touched
--dry-run    list what would go, delete nothing
"""
import os
import shutil
import sys
import time

import simlib

MIN_KEEP_S = 60          # never delete something written seconds ago


def _tree_bytes(path):
    total = 0
    for base, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.lstat(os.path.join(base, f)).st_size
            except OSError:
                pass
    return total


def _mb(n):
    return f"{n / 1048576.0:.1f}M"


def _sweep(root, days, dry, alive_check=None):
    """Delete entries of root older than `days` days (None = all)."""
    if not os.path.isdir(root):
        return 0
    cutoff = time.time() - days * 86400 if days is not None else 0
    freed = 0
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name)
        if not os.path.isdir(path):
            continue
        if alive_check and alive_check(name):
            print(f"  keep  {name:32s} live")
            continue
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        if (days is not None and mtime > cutoff) \
                or time.time() - mtime < MIN_KEEP_S:
            continue
        size = _tree_bytes(path)
        print(f"  {'would rm' if dry else 'rm'}     {name:32s} {_mb(size)}")
        if not dry:
            shutil.rmtree(path, ignore_errors=True)
        freed += size
    return freed


def main(argv):
    days = 7.0
    dry = False
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--days":
            i += 1
            days = float(argv[i])
        elif a == "--all":
            days = None
        elif a == "--dry-run":
            dry = True
        else:
            sys.exit(__doc__)
        i += 1
    print(f"var/run/ ({simlib.RUN_DIR})")
    freed = _sweep(simlib.RUN_DIR, days, dry,
                   alive_check=lambda n: bool(simlib.read_pids(n)))
    print(f"var/results/ ({simlib.RESULTS_DIR})")
    freed += _sweep(simlib.RESULTS_DIR, days, dry)
    print(f"{'would free' if dry else 'freed'} {_mb(freed)}")
    return 0
