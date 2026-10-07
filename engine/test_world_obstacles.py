#!/usr/bin/env python3
"""test_world_obstacles.py - closed-form tests for the P5-10 occluding
world (no SITL, no de_comm):

    python3 engine/test_world_obstacles.py

Canopy transmittance checks the Beer-Lambert band math against its two
defining cases (nadir -> gap_fraction, oblique -> gap_fraction to the
1/cos theta), buildings the opaque case, and rf_loss_db the D6 model
(per-metre crown loss capped, per-building loss). The clutter case
replays stub_ai_box frames in-process (stubs/model_import.py shims the
databus ids the import needs) and checks a ground-anchored false
positive stays on its ground point while the pose moves. The lens
round-trip checks geo_projection's angle_linear and rectilinear image
<-> ray maps to 1e-9. Obstacle load errors must name the obstacle id.
"""
import json
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)

import simlib                      # noqa: E402
import world as world_mod          # noqa: E402
import geo_projection as gp        # noqa: E402

ORIGIN = (-35.363262, 149.165237)          # the SITL home
M_PER_DEG_LAT = 111320.0
M_PER_DEG_LNG = 111320.0 * math.cos(math.radians(ORIGIN[0]))

WOOD_N = [[-35.36290, 149.16580], [-35.36260, 149.16580],
          [-35.36260, 149.16630], [-35.36290, 149.16630]]
# hut_1: a 6 m square centred 95 m north / 75 m east of the origin
HUT_1 = [[-35.362437, 149.166030], [-35.362383, 149.166030],
        [-35.362383, 149.166096], [-35.362437, 149.166096]]
# hut_2: the same square 10 m east of hut_1
HUT_2 = [[-35.362437, 149.166126], [-35.362383, 149.166126],
        [-35.362383, 149.166192], [-35.362437, 149.166192]]

# local metres of the WOOD_N ring corners (north, east)
WOOD_N_N = [(lat - ORIGIN[0]) * M_PER_DEG_LAT for lat in
            (-35.36290, -35.36260)]
WOOD_N_E = [(lng - ORIGIN[1]) * M_PER_DEG_LNG for lng in
            (149.16580, 149.16630)]
# hut_1 centre: 95 m north / 75 m east of the origin
HUT_N = (-35.362410 - ORIGIN[0]) * M_PER_DEG_LAT
HUT_E = (149.166060 - ORIGIN[1]) * M_PER_DEG_LNG


def at(north_m, east_m, alt_m=0.0):
    """(lat, lng, alt) of a local ENU point about the origin."""
    return (ORIGIN[0] + north_m / M_PER_DEG_LAT,
            ORIGIN[1] + east_m / M_PER_DEG_LNG, alt_m)


def latlng(north_m, east_m):
    return [ORIGIN[0] + north_m / M_PER_DEG_LAT,
            ORIGIN[1] + east_m / M_PER_DEG_LNG]


def make_world(obstacles=None, **extra):
    """Load a world.yaml built from dicts in a scratch dir -> World."""
    spec = {"version": 1, "origin": [ORIGIN[0], ORIGIN[1]],
            "entities": [{"id": "p1", "cls": "person",
                          "pos": [ORIGIN[0], ORIGIN[1]]}]}
    if obstacles is not None:
        spec["obstacles"] = obstacles
    spec.update(extra)
    d = tempfile.mkdtemp()
    p = os.path.join(d, "w.yaml")
    with open(p, "w") as f:
        json.dump(spec, f)
    return world_mod.load(p, 7)


def canopy(**over):
    out = {"id": "wood_n", "kind": "canopy", "poly": WOOD_N,
           "height_m": 15.0, "base_m": 3.0, "gap_fraction": 0.3}
    out.update(over)
    return out


def hut(oid="hut_1", poly=None, **over):
    out = {"id": oid, "kind": "building",
           "poly": poly or (HUT_1 if oid == "hut_1" else HUT_2),
           "height_m": 4.0}
    out.update(over)
    return out


class ObstacleGeometry(unittest.TestCase):
    def setUp(self):
        self.w = make_world([canopy(), hut()])

    def test_nadir_through_the_band_sees_the_gap(self):
        # straight down through the full 12 m crown: T = gap_fraction
        mid_n = sum(WOOD_N_N) / 2.0
        mid_e = sum(WOOD_N_E) / 2.0
        t, blockers = self.w.transmittance(at(mid_n, mid_e, 40.0),
                                           at(mid_n, mid_e, 0.0))
        self.assertAlmostEqual(t, 0.3, delta=1e-6)
        self.assertEqual(blockers, ["wood_n"])

    def test_oblique_ray_sees_gap_pow_one_over_cos(self):
        # 45 deg off nadir through the same crown: the band chord is
        # (height-base)/cos(theta), so T = gap^(1/cos theta)
        mid_n = sum(WOOD_N_N) / 2.0
        mid_e = sum(WOOD_N_E) / 2.0
        t, _ = self.w.transmittance(at(mid_n - 40.0, mid_e, 40.0),
                                    at(mid_n, mid_e, 0.0))
        self.assertAlmostEqual(t, 0.3 ** (1.0 / math.cos(math.radians(45))),
                                delta=1e-6)

    def test_ray_beside_the_prism_is_clear(self):
        # a parallel ray 100 m east of the wood: no obstacle at all
        mid_n = sum(WOOD_N_N) / 2.0
        far_e = max(WOOD_N_E) + 100.0
        t, blockers = self.w.transmittance(at(mid_n, far_e, 40.0),
                                           at(mid_n, far_e, 0.0))
        self.assertEqual((t, blockers), (1.0, []))

    def test_target_inside_a_building_is_never_seen(self):
        # D4: the ray from any camera enters the prism - T = 0
        t, blockers = self.w.transmittance(at(HUT_N + 40.0, HUT_E, 40.0),
                                           at(HUT_N, HUT_E, 0.25))
        self.assertEqual(t, 0.0)
        self.assertEqual(blockers, ["hut_1"])

    def test_high_camera_sees_over_a_building(self):
        # 40 m camera directly above the hut, target 30 m behind it on
        # the ground: the ray only dips into the hut's band well past
        # its footprint - T = 1
        t, blockers = self.w.transmittance(at(HUT_N, HUT_E, 40.0),
                                           at(HUT_N + 30.0, HUT_E, 0.25))
        self.assertEqual((t, blockers), (1.0, []))

    def test_rf_canopy_loss_per_metre_capped(self):
        # a vertical ray through 10 m of the crown band: 0.3 dB/m -> 3
        # dB; a tight rf_max_db caps it (D6)
        mid_n = sum(WOOD_N_N) / 2.0
        mid_e = sum(WOOD_N_E) / 2.0
        loss, blockers = self.w.rf_loss_db(at(mid_n, mid_e, 15.0),
                                          at(mid_n, mid_e, 5.0))
        self.assertAlmostEqual(loss, min(3.0, 20.0), delta=1e-6)
        self.assertEqual(blockers, ["wood_n"])
        w2 = make_world([canopy(rf_max_db=2.0)])
        loss2, _ = w2.rf_loss_db(at(mid_n, mid_e, 15.0),
                                 at(mid_n, mid_e, 5.0))
        self.assertAlmostEqual(loss2, 2.0, delta=1e-6)

    def test_rf_two_buildings_double_the_loss(self):
        # a ground-level ray crossing both huts: 2 x rf_loss_db
        w2 = make_world([hut(), hut("hut_2")])
        loss, blockers = w2.rf_loss_db(at(HUT_N, HUT_E - 20.0, 1.5),
                                       at(HUT_N, HUT_E + 20.0, 1.5))
        self.assertAlmostEqual(loss, 24.0, delta=1e-6)
        self.assertEqual(blockers, ["hut_1", "hut_2"])

    def test_obstacles_json_is_renderer_ready(self):
        rows = self.w.obstacles_json()
        self.assertEqual([r["id"] for r in rows], ["wood_n", "hut_1"])
        self.assertEqual(rows[0]["poly"], WOOD_N)
        self.assertEqual((rows[0]["base_m"], rows[0]["height_m"]),
                         (3.0, 15.0))
        self.assertIn("gap_fraction", rows[0])
        self.assertIn("rf_loss_db", rows[1])

    def test_no_obstacles_answer_without_geometry(self):
        w = make_world()
        self.assertEqual(w.transmittance(at(0, 0, 40), at(0, 0, 0)),
                         (1.0, []))
        self.assertEqual(w.rf_loss_db(at(0, 0, 40), at(0, 0, 0)),
                         (0.0, []))
        self.assertEqual(w.obstacles_json(), [])


class ObstacleValidation(unittest.TestCase):
    def _bad(self, obstacles, needle):
        with self.assertRaises(simlib.SimError) as cm:
            make_world(obstacles)
        self.assertIn(needle, str(cm.exception))
        return str(cm.exception)

    def test_errors_name_the_obstacle_id(self):
        msg = self._bad([canopy(poly=WOOD_N[:2])],
                        "obstacle 'wood_n': poly needs at least 3")
        self.assertIn("wood_n", msg)
        self._bad([canopy(base_m=15.0)], "0 <= base_m < height_m")
        self._bad([canopy(gap_fraction=0.0)], "0 < gap_fraction")
        self._bad([{"id": "x", "kind": "hedge", "poly": WOOD_N,
                    "height_m": 5}], "kind must be canopy|building")
        self._bad([{"id": "x"}], "obstacle 'x'")

    def test_unknown_keys_are_scoped_to_the_kind(self):
        self._bad([canopy(rf_loss_db=5)], "unknown keys")
        self._bad([hut(gap_fraction=0.5)], "unknown keys")
        self._bad([{"id": "x", "kind": "canopy", "poly": WOOD_N,
                   "height_m": 5, "trees": 12}], "unknown keys")

    def test_self_intersecting_footprint_fails(self):
        # a bow-tie ring crosses itself
        bow = [latlng(0, 0), latlng(20, 20), latlng(20, 0), latlng(0, 20)]
        self._bad([canopy(poly=bow)], "not simple")

    def test_duplicate_id_fails(self):
        with self.assertRaises(simlib.SimError) as cm:
            make_world([canopy(), canopy()])
        self.assertIn("duplicate obstacle id", str(cm.exception))

    def test_bad_lens_and_anchor_fail_at_load(self):
        with self.assertRaises(simlib.SimError) as cm:
            make_world(sensors={"camera": {"lens": "fisheye"}})
        self.assertIn("unknown lens", str(cm.exception))
        with self.assertRaises(simlib.SimError) as cm:
            make_world(clutter={"anchor": "tree"})
        self.assertIn("unknown anchor", str(cm.exception))


class LensRoundTrip(unittest.TestCase):
    """geo_ground_to_pixel (inverse) then geo_ray_camera (forward) must
    return the exact ray to the point, for both lenses, to 1e-9."""

    MOUNT = gp.GeoMountParams(pitch_deg=-90.0)
    ATT = gp.GeoAttitude()
    POSE = gp.GeoPose(lat_deg=ORIGIN[0], lng_deg=ORIGIN[1], rel_alt_m=40.0)

    def _round_trip(self, cam):
        worst = 0.0
        for north_m in (-40.0, -20.0, -5.0, 0.0, 5.0, 20.0, 40.0):
            for east_m in (-60.0, -30.0, 0.0, 30.0, 60.0):
                lat, lng = gp.geo_offset_to_latlng(
                    self.POSE.lat_deg, self.POSE.lng_deg, north_m, east_m)
                uv = gp.geo_ground_to_pixel(lat, lng, 0.0, cam,
                                            self.MOUNT, self.ATT,
                                            self.POSE)
                self.assertIsNotNone(uv)
                u, v, _slant = uv
                ray_ned = gp.geo_ray_ned(
                    gp.geo_ray_body(gp.geo_ray_camera(u, v, cam),
                                    self.MOUNT), self.ATT)
                want_n, want_e = gp.geo_latlng_to_ne(
                    self.POSE.lat_deg, self.POSE.lng_deg, lat, lng)
                want = (want_n, want_e, 40.0)
                n_want = math.sqrt(sum(x * x for x in want))
                n_ray = math.sqrt(sum(x * x for x in ray_ned))
                worst = max(
                    worst,
                    max(abs(a / n_want - b / n_ray)
                        for a, b in zip(want, ray_ned)))
        self.assertLess(worst, 1e-9)

    def test_angle_linear_round_trip(self):
        self._round_trip(gp.GeoCameraParams(62.2, 48.8, "angle_linear"))

    def test_rectilinear_round_trip(self):
        self._round_trip(gp.GeoCameraParams(62.2, 48.8, "rectilinear"))


class GroundClutter(unittest.TestCase):
    """A ground-anchored false positive is a fixed GROUND point: while
    the pose moves across 10 frames, the point keeps its lat/lng (the
    image position moves, the rock does not)."""

    def test_clutter_point_stays_on_the_ground(self):
        sys.path.insert(0, os.path.join(ROOT, "stubs"))
        import model_import
        stub_ai_box = model_import.import_stubs()["stub_ai_box"]
        with tempfile.TemporaryDirectory() as d:
            wpath = os.path.join(d, "w.yaml")
            with open(wpath, "w") as f:
                json.dump({"version": 1,
                           "origin": [ORIGIN[0], ORIGIN[1]],
                           "entities": [{"id": "p1", "cls": "person",
                                         "pos": [ORIGIN[0],
                                                 ORIGIN[1]]}],
                           "clutter": {"cls": "person",
                                       "rate_per_km2_min": 5e5,
                                       "lifetime_s": 20.0,
                                       "anchor": "ground"},
                           "sensors": {"camera": {
                               "fps_boxes": 2,
                               "p_detect_nadir": 1.0,
                               "p_detect_edge": 1.0}}},
                          f)
            mdir = os.path.join(d, "var", "run", "sim", "u1", "stub_ai_box")
            os.makedirs(mdir)
            cfg = os.path.join(mdir, "stub_ai_box.config.json")
            with open(cfg, "w") as f:
                json.dump({"stub": {"world": wpath, "seed": 7,
                                    "unit": "u1"},
                           "s2s_udp_target_port": "1"}, f)
            s = stub_ai_box.StubAIBox(["-c", cfg])
            # a moving camera at 40 m: 13 ticks, the pose 1 m north per
            # tick - slow enough that a spawned point stays in frame
            # across the whole run (the clutter lifetime spans it all)
            s._vt_ms = 0
            for k in range(13):
                set_pose(s, ORIGIN[0] + (1.0 * k) / M_PER_DEG_LAT,
                         ORIGIN[1], 40.0)
                s.tick(k * 0.5, k)
            with open(os.path.join(d, "var", "run", "sim", "truth.jsonl")) as f:
                rows = [json.loads(ln) for ln in f]
            clutter = [r for r in rows if r.get("tid") == "clutter"
                       and r.get("lat") is not None]
            self.assertTrue(clutter, "no ground clutter spawned")
            by_point = {}
            for r in clutter:
                by_point.setdefault((r["lat"], r["lng"]), []).append(r)
            # the frames the pose moved across: u varies per frame while
            # the ground point never moves
            steady = [pts for pts in by_point.values()
                      if len({p["frame"] for p in pts}) >= 10]
            self.assertTrue(
                steady, "no clutter point held its ground spot across "
                "10 frames (saw "
                f"{sorted(len({p['frame'] for p in pts}) for pts in by_point.values())})")
            for pts in steady:
                # the pose moves north, so the image v of the fixed
                # ground point must slide frame to frame; its lat/lng
                # above already proved the point itself never moves
                self.assertGreater(
                    len({round(p["v"], 4) for p in pts}), 1,
                    "the image position moved while the ground point "
                    "stayed - a ground-anchored clutter must re-project")


def set_pose(stub, lat, lng, rel_alt, t_ms=0):
    f = stub.pose_feed
    f._pos = {"lat": lat, "lng": lng, "alt_m": 584 + rel_alt,
              "rel_alt_m": rel_alt, "t_ms": t_ms}
    f._att = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "t_ms": t_ms}


if __name__ == "__main__":
    unittest.main()
