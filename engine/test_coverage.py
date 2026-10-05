#!/usr/bin/env python3
"""test_coverage.py - closed-form tests for the P5-12 live coverage +
POD map (no SITL, no de_comm):

    python3 engine/test_coverage.py

A synthetic tlog (telemetry row dicts with roll/pitch/yaw) replays
through the same camera/LOS model the stubs use, so the checks are
closed-form to 1e-6: a nadir hover over a cell gives POD =
p_detect_nadir after one look; a canopy cell gives p * gap_fraction;
a building cell gives 0 with looks counted; a lawnmower covers its
rectangle; a missing leg leaves the expected strip uncovered. The
checks-level cases exercise SKIP on a tlog without attitude and the
coverage_complete/pod_at verdicts.
"""
import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)

import simlib                      # noqa: E402
import world as world_mod          # noqa: E402
import geo_projection as gp        # noqa: E402
import coverage                    # noqa: E402
import checks                      # noqa: E402

ORIGIN = (-35.363262, 149.165237)          # the SITL home
M_PER_DEG_LAT = 111320.0
M_PER_DEG_LNG = 111320.0 * math.cos(math.radians(ORIGIN[0]))
P_NADIR = 0.95                             # CAMERA_DEFAULTS
EDGE_M = 40.0 * math.tan(math.radians(62.2 / 2))   # nadir 40 m footprint


def latlng(north_m, east_m):
    return [ORIGIN[0] + north_m / M_PER_DEG_LAT,
            ORIGIN[1] + east_m / M_PER_DEG_LNG]


def make_world(**extra):
    spec = {"version": 1, "origin": [ORIGIN[0], ORIGIN[1]],
            "entities": [{"id": "p1", "cls": "person",
                          "pos": latlng(11.0, 11.0)}]}
    spec.update(extra)
    d = tempfile.mkdtemp()
    p = os.path.join(d, "w.yaml")
    with open(p, "w") as f:
        json.dump(spec, f)
    return world_mod.load(p, 7)


def rect(x0, y0, x1, y1):
    """A lat/lng ring for the ENU rectangle (x east, y north)."""
    return [latlng(y0, x0), latlng(y1, x0),
            latlng(y1, x1), latlng(y0, x1)]


def look(north_m, east_m, alt_m=40.0, roll=0.0, pitch=0.0, yaw=0.0):
    """(att, pose) of one thinned look."""
    lat, lng = latlng(north_m, east_m)
    return (gp.GeoAttitude(roll, pitch, yaw),
            gp.GeoPose(lat, lng, alt_m))


def tele_row(north_m, east_m, alt_m=40.0, t_ms=1000, att=True):
    r = {"unit": "cam1", "t_ms": t_ms,
         "lat": latlng(north_m, east_m)[0],
         "lng": latlng(north_m, east_m)[1],
         "rel_alt_m": alt_m}
    if att:
        r.update(roll=0.0, pitch=0.0, yaw=0.0)
    return r


class FakeTele:
    def __init__(self, rows):
        self._rows = rows

    def rows(self, unit_name=None):
        return list(self._rows.get(unit_name or "", []))


def ctx_for(wld, tele, **kw):
    units = [{"name": "cam1", "fc": True,
              "modules": ["stub_ai_box"], "config": {}}]
    c = {"manifest": {"name": "t", "units": units, "server_procs": []},
         "world": wld, "telemetry": tele, "marks": {},
         "file": "/x/s.yaml", "results_dir": None}
    c.update(kw)
    return c


class T(unittest.TestCase):
    def test_nadir_hover_pod_is_p_nadir(self):
        wld = make_world()
        poly = rect(0, 0, 20, 20)
        env = wld.sensor_factors("rgb")
        # the cell holding (10,10) is centred (12.5,12.5) - hover over
        # the CENTRE so the ray is truly nadir (u = v = 0)
        grid = coverage.compute(poly, ORIGIN, 5.0, wld,
                                wld.sensors["camera"],
                                [(*look(12.5, 12.5), env, "rgb")])
        cell = coverage.cell_at(grid, *latlng(11.0, 11.0))
        self.assertIsNotNone(cell)
        self.assertAlmostEqual(cell["pod"], P_NADIR, places=9)
        self.assertEqual(cell["looks"], 1)
        self.assertTrue(cell["covered"])
        # a second same look compounds: 1 - (1-p)^2
        grid2 = coverage.compute(poly, ORIGIN, 5.0, wld,
                                 wld.sensors["camera"],
                                 [(*look(12.5, 12.5), env, "rgb")] * 2)
        cell2 = coverage.cell_at(grid2, *latlng(11.0, 11.0))
        self.assertAlmostEqual(cell2["pod"], 1 - (1 - P_NADIR) ** 2,
                               places=9)

    def test_lawnmower_covers_rectangle(self):
        # rect 0..75 x (east) by 0..20 y (north); two legs 35 m apart
        # at 40 m nadir overlap ~28 % of the 48 m frame width
        wld = make_world()
        env = wld.sensor_factors("rgb")
        looks = [(*look(10.0, 20.0), env, "rgb"),
                 (*look(10.0, 55.0), env, "rgb")]
        grid = coverage.compute(rect(0, 0, 75, 20), ORIGIN, 5.0,
                                wld, wld.sensors["camera"], looks)
        s = coverage.summary(grid)
        self.assertEqual(s["coverage"], 1.0)

    def test_missing_leg_leaves_the_strip(self):
        wld = make_world()
        env = wld.sensor_factors("rgb")
        grid = coverage.compute(rect(0, 0, 75, 20), ORIGIN, 5.0,
                                wld, wld.sensors["camera"],
                                [(*look(10.0, 20.0), env, "rgb")])
        cells = grid["cells"]
        bare = [c for c in cells.values() if c["looks"] == 0]
        self.assertTrue(bare)
        # every uncovered cell sits east of the leg's footprint edge
        for c in bare:
            self.assertGreater(c["x"], 20.0 + EDGE_M - 5.0)
        covered = [c for c in cells.values() if c["covered"]]
        for c in covered:
            self.assertLess(c["x"], 20.0 + EDGE_M + 5.0)

    def test_building_cell_pod_zero(self):
        hut = {"id": "hut", "kind": "building", "height_m": 4,
               "poly": [latlng(45, 45), latlng(55, 45),
                        latlng(55, 55), latlng(45, 55)]}
        wld = make_world(obstacles=[hut])
        env = wld.sensor_factors("rgb")
        grid = coverage.compute(rect(40, 40, 60, 60), ORIGIN, 5.0,
                                wld, wld.sensors["camera"],
                                [(*look(50.0, 50.0), env, "rgb")])
        cell = coverage.cell_at(grid, *latlng(51.0, 51.0))
        self.assertIsNotNone(cell)
        self.assertEqual(cell["pod"], 0.0)
        self.assertGreater(cell["looks"], 0)
        self.assertFalse(cell["covered"])

    def test_canopy_cell_pod_is_p_times_gap(self):
        crown = {"id": "wood", "kind": "canopy", "height_m": 15,
                 "base_m": 3, "gap_fraction": 0.3,
                 "poly": [latlng(45, 45), latlng(55, 45),
                          latlng(55, 55), latlng(45, 55)]}
        wld = make_world(obstacles=[crown])
        env = wld.sensor_factors("rgb")
        # cell holding (50,50) is centred (52.5,52.5) - nadir look over
        # the centre: vis = gap_fraction, p_cam = p_nadir
        grid = coverage.compute(rect(40, 40, 60, 60), ORIGIN, 5.0,
                                wld, wld.sensors["camera"],
                                [(*look(52.5, 52.5), env, "rgb")])
        cell = coverage.cell_at(grid, *latlng(51.0, 51.0))
        self.assertIsNotNone(cell)
        self.assertAlmostEqual(cell["pod"], P_NADIR * 0.3, places=6)

    def test_thin_rows_respects_fps(self):
        rows = [tele_row(0, 0, t_ms=t) for t in
                (0, 200, 400, 550, 1100, 1500)]
        thinned = coverage.thin_rows(rows, 2.0)
        self.assertEqual([r["t_ms"] for r in thinned],
                         [0, 550, 1100])

    def test_checks_skip_without_attitude(self):
        wld = make_world(areas={"sector_a": {"poly": rect(0, 0, 20, 20)}})
        tele = FakeTele({"cam1": [tele_row(10.0, 10.0, att=False)]})
        r = checks.check_assertion(
            {"coverage_complete": {"area": "sector_a", "min": 0.5}},
            ctx_for(wld, tele))
        self.assertEqual(r["status"], "SKIP")
        self.assertIn("attitude", r["detail"])

    def test_checks_coverage_and_pod_at(self):
        poly = rect(0, 0, 75, 20)
        wld = make_world(areas={"sector_a": {"poly": poly}})
        rows = [tele_row(10.0, 20.0, t_ms=1000),
                tele_row(10.0, 55.0, t_ms=1500)]
        c = ctx_for(wld, FakeTele({"cam1": rows}))
        r = checks.check_assertion(
            {"coverage_complete": {"area": "sector_a", "min": 0.95}}, c)
        self.assertEqual(r["status"], "PASS", r)
        self.assertAlmostEqual(r["value"], 1.0)
        # pod_at over the same cached grid: p1's cell (centre
        # 12.5,12.5) sits inside the first leg's frame, off-axis ->
        # pod a little under p_nadir
        r2 = checks.check_assertion(
            {"pod_at": {"truth": "p1", "t_s": 0, "min": 0.8}}, c)
        self.assertEqual(r2["status"], "PASS", r2)
        self.assertGreater(r2["value"], 0.8)
        # pod_mean needs min
        r3 = checks.check_assertion({"pod_mean": {"area": "sector_a"}}, c)
        self.assertEqual(r3["status"], "FAIL")

    def test_world_areas_parse(self):
        wld = make_world(areas={"a1": {"poly": rect(0, 0, 10, 10)}})
        self.assertEqual(len(wld.areas["a1"]["poly"]), 4)
        with self.assertRaises(simlib.SimError):
            make_world(areas={"bad": {"poly": [[1, 2]]}})

    def test_telemetry_requests_attitude(self):
        import telemetry
        self.assertIn((30, 500000), telemetry.Telemetry._WANT)

    def test_runtime_budget(self):
        # the spec budget: 30 min * 3 units * 2 Hz over 1 km^2 at 5 m
        # cells in < 20 s - here: 1 km^2 grid + 200 looks must be fast
        import time
        wld = make_world()
        env = wld.sensor_factors("rgb")
        grid = coverage.build_grid(rect(0, 0, 1000, 1000), ORIGIN, 5.0)
        t0 = time.time()
        for i in range(200):
            coverage.replay_look(
                grid, wld,
                gp.GeoCameraParams(62.2, 48.8),
                gp.GeoMountParams(-90.0, 0.0, 0.0),
                wld.sensors["camera"],
                *look(500.0, float(i * 5)),
                env_f=env, cam_kind="rgb")
        dt = time.time() - t0
        self.assertLess(dt, 20.0)
        print(f"  budget: 200 looks over {len(grid['cells'])} cells "
              f"in {dt:.1f} s")


if __name__ == "__main__":
    unittest.main(verbosity=2)
