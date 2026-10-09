#!/usr/bin/env python3
"""gen_bigplan.py - write a de.package/2 carrying one very long mission:
a takeoff, N-2 waypoints on a single straight line (spacing_m apart -
the survey is a ~100 m corridor, not an area) and an RTL. The point is
package/protocol/FC/GCS capacity, not coverage; the tests in
tests/capacity/ push the result through a real upload.

    python3 tools/gen_bigplan.py
        [--count 1000] [--spacing-m 0.1] [--alt 40]
        [--lat -35.363200] [--lng 149.166100]
        [--vehicle plane] [--out packages/plan_survey_1000.json]

The generated file is committed - tests run it like every other
packages/*.json fixture; regenerate it instead of editing by hand.
"""

import argparse
import json
import math
import os
import sys

SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--count", type=int, default=1000,
                    help="total fc_items incl. takeoff + RTL")
    ap.add_argument("--spacing-m", type=float, default=0.1,
                    help="metres between consecutive waypoints")
    ap.add_argument("--alt", type=float, default=40.0)
    ap.add_argument("--lat", type=float, default=-35.363200)
    ap.add_argument("--lng", type=float, default=149.166100)
    ap.add_argument("--vehicle", default="plane")
    ap.add_argument("--out", default=os.path.join(
        SIM_ROOT, "packages", "plan_survey_1000.json"))
    args = ap.parse_args(argv)
    if args.count < 3:
        sys.exit("--count must cover takeoff + one wp + RTL")

    n_wp = args.count - 2
    # east line: constant latitude, tiny monotonic longitude steps (no
    # exact duplicates - some readers treat identical points specially)
    deg_per_m = 1.0 / (111320.0 * math.cos(math.radians(args.lat)))
    alt = round(args.alt, 2)

    fc_items = [{"c": 22, "ft": 3, "mv": [0, 0, 0, 0, 0, 0, alt]}]
    manifest = [{"seq": 1, "mission": "main", "step": "t",
                 "kind": "nav.takeoff"}]
    for i in range(1, n_wp + 1):
        lat = args.lat
        lng = args.lng + i * args.spacing_m * deg_per_m
        fc_items.append({"c": 16, "ft": 3,
                         "mv": [0, 5, 0, 0, round(lat, 6),
                                round(lng, 6), alt]})
        manifest.append({"seq": i + 1, "mission": "main",
                         "step": f"w{i}", "kind": "nav.wp"})
    fc_items.append({"c": 20, "ft": 2, "mv": [0, 0, 0, 0, 0, 0, 0]})
    manifest.append({"seq": args.count, "mission": "main",
                     "step": "end", "kind": "nav.rtl"})

    pkg = {
        "schema": "de.package/2",
        "op": f"capacity_survey_{args.count}",
        "role": "solo",
        "vehicle": {"type": args.vehicle},
        "requires": [],
        "fc_items": fc_items,
        "missions": {
            "main": {
                "trigger": {"type": "op.start"},
                "priority": 0,
                "segment": {"start": 1, "end": args.count},
                "end_state": "rtl",
                "end_backstop": {"timeout": 300, "then": "rtl"},
            }
        },
        "manifest": manifest,
        "events": {},
        "handlers": {},
        "hash": "sha256:0000",
    }
    out = args.out if os.path.isabs(args.out) else \
        os.path.join(SIM_ROOT, args.out)
    with open(out, "w") as f:
        f.write(json.dumps(pkg, indent=1) + "\n")
    span_m = n_wp * args.spacing_m
    size = os.path.getsize(out)
    print(f"{out}: {args.count} fc_items ({n_wp} wp on a "
          f"{span_m:.0f} m line), {size} bytes")


if __name__ == "__main__":
    main(sys.argv[1:])
