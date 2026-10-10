#!/usr/bin/env python3
"""test_world_behaviour.py - closed-form tests for the P5-11 behaviour
and environment models (no SITL, no de_comm):

    python3 engine/test_world_behaviour.py

The seeded behaviour track must be identical in every process (D1): two
World objects over the same spec and seed reproduce the same positions
over 3600 s, and another seed does not. wander/seek never step inside
a building prism (D3). stationary.after_s freezes the anchor motion,
trail stays on its polyline until the seeded leave draw fires, and the
environment's D5 factors answer the documented values for rgb and
thermal cameras at sun_elev 30 / 0 / -10. World wind lands in a FC
unit's sim.parm params as SIM_WIND_* (D6).

The TestReact cases cover SL-0013: the react: spec validates,
world.react_knots fires a trigger on a synthetic pose feed, the knot
lands in world_overrides.jsonl through the TruthRecorder (the single
authority, fleet poses stubbed by a callable), and pos_at given the
file is deterministic - two Worlds reading the same overrides file
reproduce identical reacted positions while an unread file changes
nothing for a non-reactive entity.
"""
import contextlib
import json
import math
import os
import sys
import tempfile
import time
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


def at(north_m, east_m):
    """[lat, lng] of a local ENU point about the origin."""
    return [ORIGIN[0] + north_m / M_PER_DEG_LAT,
            ORIGIN[1] + east_m / M_PER_DEG_LNG]


def spec_world(entities, seed=7, extra=None):
    """A World built straight from a spec dict - world.load's file
    layer is already covered by the scenario fixtures."""
    spec = {"version": 1, "origin": list(ORIGIN), "entities": entities}
    if extra:
        spec.update(extra)
    return world_mod.World(spec, seed, "test")


WANDERER = {"id": "w1", "cls": "person", "pos": at(0, 0),
            "behaviour": {"kind": "wander", "speed_mps": 0.6,
                          "turn_sigma_deg": 25, "pause_p": 0.02,
                          "pause_s": 60, "bound_m": 400}}

# a 10 m square building centred at N 60, E 0
HUT_POLY = [at(55, -5), at(65, -5), at(65, 5), at(55, 5)]


def positions(world, eid, t_end_s, step=1.0):
    e = world.entity(eid)
    return [e.pos_at(t, world) for t in
            (i * step for i in range(int(t_end_s / step)))]


class TestSeededTrack(unittest.TestCase):

    def test_same_seed_identical_3600s(self):
        w1 = spec_world([dict(WANDERER)])
        w2 = spec_world([dict(WANDERER)])
        self.assertEqual(positions(w1, "w1", 3600),
                         positions(w2, "w1", 3600))

    def test_other_seed_differs(self):
        w1 = spec_world([dict(WANDERER)], seed=7)
        w2 = spec_world([dict(WANDERER)], seed=8)
        self.assertNotEqual(positions(w1, "w1", 600),
                            positions(w2, "w1", 600))

    def test_wander_moves_and_pauses(self):
        w = spec_world([dict(WANDERER)])
        e = w.entity("w1")
        seen_moving = seen_still = 0
        for t in range(3600):
            st = e.state_at(t, w)
            self.assertEqual(st["behaviour"], "wander")
            seen_moving += 1 if st["moving"] else 0
            seen_still += 0 if st["moving"] else 1
        self.assertGreater(seen_moving, 0)
        self.assertGreater(seen_still, 0)   # pause_p fires over an hour

    def test_wander_bound_holds(self):
        e = dict(WANDERER)
        e["behaviour"] = dict(e["behaviour"], bound_m=50)
        w = spec_world([e])
        for lat, lng, _a in positions(w, "w1", 3600, 5.0):
            self.assertLessEqual(
                gp.geo_distance_m(ORIGIN[0], ORIGIN[1], lat, lng),
                50.0 + 1.0)                  # bound + one step


class TestBuildingRejection(unittest.TestCase):

    def test_wander_never_enters_building(self):
        # the wanderer starts next to the hut, bound 60 encloses it
        e = {"id": "w1", "cls": "person", "pos": at(0, 0),
             "behaviour": {"kind": "wander", "speed_mps": 1.0,
                           "turn_sigma_deg": 20, "pause_p": 0.0,
                           "pause_s": 0, "bound_m": 60}}
        for seed in range(10):
            w = spec_world([e], seed=seed,
                           extra={"obstacles": [
                               {"id": "hut", "kind": "building",
                                "poly": HUT_POLY, "height_m": 4}]})
            ent = w.entity("w1")
            for t in range(0, 3600, 2):
                lat, lng, _a = ent.pos_at(t, w)
                self.assertFalse(
                    w.point_in_building(lat, lng, 0.0),
                    f"seed {seed} t {t}: {lat},{lng} inside hut")

    def test_canopy_is_walkable(self):
        # same hut shape as a canopy: positions may land inside it
        e = {"id": "w1", "cls": "person", "pos": at(60, 0),
             "behaviour": {"kind": "wander", "speed_mps": 0.6,
                           "turn_sigma_deg": 25, "pause_p": 0.0,
                           "pause_s": 0, "bound_m": 30}}
        w = spec_world([e], extra={"obstacles": [
            {"id": "wood", "kind": "canopy", "poly": HUT_POLY,
             "height_m": 15}]})
        ent = w.entity("w1")
        self.assertTrue(all(
            not w.point_in_building(*ent.pos_at(t, w))
            for t in range(0, 600, 5)))      # never raises, never errs


class TestStationary(unittest.TestCase):

    def test_after_s_holds_position(self):
        # walks a 500 m path at 0.8 m/s, freezes at t = 300 (240 m in)
        e = {"id": "p1", "cls": "person",
             "path": [at(0, 0), at(500, 0)], "speed_mps": 0.8,
             "behaviour": {"kind": "stationary", "after_s": 300}}
        w = spec_world([e])
        ent = w.entity("p1")
        frozen = ent.pos_at(300, w)
        later = ent.pos_at(400, w)
        self.assertAlmostEqual(frozen[0], later[0], places=9)
        self.assertAlmostEqual(frozen[1], later[1], places=9)
        self.assertFalse(ent.state_at(400, w)["moving"])
        # before after_s it walks: t=150 -> ~120 m north of origin
        n, _e2 = gp.geo_latlng_to_ne(ORIGIN[0], ORIGIN[1],
                                     *ent.pos_at(150, w)[:2])
        self.assertAlmostEqual(n, 120.0, delta=1.0)
        self.assertTrue(ent.state_at(150, w)["moving"])

    def test_stationary_default_is_still(self):
        e = {"id": "p1", "cls": "person", "pos": at(10, 10),
             "behaviour": {"kind": "stationary"}}
        w = spec_world([e])
        ent = w.entity("p1")
        self.assertEqual(ent.pos_at(0, w), ent.pos_at(600, w))


class TestTrailSeek(unittest.TestCase):

    TRAILS = {"trail_a": [at(0, 0), at(200, 0), at(200, 200)]}

    def _trail_world(self, seed=7):
        e = {"id": "h1", "cls": "person",
             "behaviour": {"kind": "trail", "trail": "trail_a",
                           "speed_mps": 1.1, "leave_p": 0.05}}
        return spec_world([e], seed=seed, extra={"trails": self.TRAILS})

    @staticmethod
    def _dist_to_poly(pts, lat, lng):
        n_m, e_m = gp.geo_latlng_to_ne(pts[0][0], pts[0][1], lat, lng)
        best = 1e9
        for a, b in zip(pts, pts[1:]):
            an, ae = gp.geo_latlng_to_ne(pts[0][0], pts[0][1], *a)
            bn, be = gp.geo_latlng_to_ne(pts[0][0], pts[0][1], *b)
            q = world_mod._nearest_on_seg(n_m, e_m, an, ae, bn, be)
            best = min(best, math.hypot(n_m - q[0], e_m - q[1]))
        return best

    def test_trail_stays_on_polyline_until_leave(self):
        w = self._trail_world()
        e = w.entity("h1")
        left_at = None
        for t in range(0, 3600, 2):
            e.pos_at(t, w)                     # extend the track
            mode = e._behav["mode"]
            lat, lng, _a = e.pos_at(t, w)
            if mode == "trail":
                self.assertLessEqual(
                    self._dist_to_poly(self.TRAILS["trail_a"], lat, lng),
                    1.0, f"t {t}: off trail at {lat},{lng}")
            elif left_at is None:
                left_at = t
        # a 400 m trail at 1.1 m/s ends by t ~= 364 s whether or not
        # the leave draw fires first - the mode must have flipped
        self.assertIsNotNone(left_at)
        self.assertEqual(e.state_at(0, w)["behaviour"], "trail")

    def test_seek_reaches_feature(self):
        e = {"id": "s1", "cls": "person", "pos": at(0, 0),
             "behaviour": {"kind": "seek", "target": "water",
                           "speed_mps": 1.0, "turn_sigma_deg": 5,
                           "bound_m": 100}}
        w = spec_world([e], extra={"features": {
            "water": {"poly": [at(90, -10), at(110, -10),
                               at(110, 10), at(90, 10)]}}})
        ent = w.entity("s1")
        ent.pos_at(0, w)                       # builds the seek track
        # the nearest poly edge to the anchor at(0,0) is N 90: the
        # resolved target must sit at ~(90, 0)
        tgt = ent._behav["target"]
        self.assertLessEqual(
            gp.geo_distance_m(tgt[0], tgt[1], at(90, 0)[0], at(90, 0)[1]),
            1.0)
        ent.pos_at(600, w)                     # ~100 m at ~1 m/s
        self.assertEqual(ent._behav["mode"], "wander")
        lat, lng, _a = ent.pos_at(600, w)
        self.assertLessEqual(
            gp.geo_distance_m(lat, lng, tgt[0], tgt[1]),
            102.0)            # wander stays bound_m around the target


class TestEnvironment(unittest.TestCase):

    def _env_world(self, env):
        e = {"id": "p1", "cls": "person", "pos": at(0, 0)}
        return spec_world([e], extra={"environment": env})

    def test_d5_factors_rgb_thermal(self):
        for elev, want_rgb, want_th in ((30.0, 1.0, 1.0),
                                        (0.0, 0.5, 1.0),
                                        (-10.0, 0.0, 1.0)):
            w = self._env_world({"sun_elev_deg": elev})
            self.assertAlmostEqual(
                w.sensor_factors("rgb")["light"], want_rgb)
            self.assertAlmostEqual(
                w.sensor_factors("thermal")["light"], want_th)
            self.assertAlmostEqual(
                w.sensor_factors("rgb")["p_scale"], want_rgb)
        # precip multiplies both kinds; the default is off
        w = self._env_world({"precip": "rain", "sun_elev_deg": 30})
        self.assertAlmostEqual(w.sensor_factors("rgb")["p_scale"], 0.8)
        self.assertAlmostEqual(w.sensor_factors("thermal")["p_scale"],
                               0.8)
        w = spec_world([{"id": "p1", "cls": "person", "pos": at(0, 0)}])
        self.assertEqual(w.sensor_factors("rgb")["p_scale"], 1.0)

    def test_visibility_caps_range(self):
        w = self._env_world({"visibility_m": 600})
        self.assertEqual(w.sensor_factors("rgb")["max_range_m"],
                         w.sensors["camera"]["max_range_m"])  # 120<600
        w = self._env_world({"visibility_m": 80})
        self.assertEqual(w.sensor_factors("rgb")["max_range_m"], 80.0)

    def test_time_of_day_sun_elev(self):
        self.assertGreater(
            self._env_world({"time_of_day": "12:00"})
            .environment["sun_elev_deg"], 50.0)
        self.assertLessEqual(
            self._env_world({"time_of_day": "18:40"})
            .environment["sun_elev_deg"], world_mod.SUN_DUSK_DEG)

    def test_environment_validation(self):
        for env in ({"time_of_day": "12:00", "sun_elev_deg": 4},
                    {"precip": "hail"},
                    {"time_of_day": "noon"},
                    {"wind": {"speed_mps": -1}},
                    {"wind": {"dir_deg": 400}},
                    {"bogus": 1}):
            with self.assertRaises(simlib.SimError, msg=str(env)):
                self._env_world(env)

    def test_wind_exported_and_pushed(self):
        w = self._env_world({"wind": {"speed_mps": 6, "dir_deg": 270,
                                      "gust_mps": 3, "turb": 0.2}})
        self.assertEqual(w.wind()["speed_mps"], 6.0)
        self.assertIsNone(self._env_world({}).wind())
        # D6: prepare_sim lands SIM_WIND_* on FC units only
        scn = {"world": "w.yaml", "seed": 7,
               "sim": {"slot_base": 10, "units": [
                   {"name": "c1", "block": "copter"},
                   {"name": "m1", "block": "comm_only"}]}}
        world_mod.prepare_sim(scn, w)
        params = scn["sim"]["units"][0]["fc"]["params"]
        self.assertEqual(params["SIM_WIND_SPD"], 6.0)
        self.assertEqual(params["SIM_WIND_DIR"], 270.0)
        self.assertEqual(params["SIM_WIND_TURB"], 0.2)
        self.assertNotIn("fc", scn["sim"]["units"][1])


class TestBehaviourValidation(unittest.TestCase):

    def test_bad_kinds_and_refs(self):
        for b in ({"kind": "teleport"},
                  {"kind": "wander", "bogus": 1},
                  {"kind": "wander", "speed_mps": 0},
                  {"kind": "wander", "pause_p": 2},
                  {"kind": "trail"},                      # needs trail
                  {"kind": "seek"},                       # needs target
                  {"kind": "seek", "target": "nowhere"},
                  {"kind": "stationary", "after_s": -1}):
            with self.assertRaises(simlib.SimError, msg=str(b)):
                spec_world([{"id": "p1", "cls": "person",
                             "pos": at(0, 0), "behaviour": b}])

    def test_attach_behaviour_rejected(self):
        spec = {"version": 1, "origin": list(ORIGIN), "entities": [
            {"id": "p1", "cls": "person", "pos": at(0, 0)},
            {"id": "p2", "cls": "person", "attach": "p1",
             "behaviour": {"kind": "wander"}}]}
        with self.assertRaises(simlib.SimError):
            world_mod.World(spec, 7, "test")

    def test_trail_entity_may_omit_anchor(self):
        spec = {"version": 1, "origin": list(ORIGIN), "entities": [
            {"id": "h1", "cls": "person",
             "behaviour": {"kind": "trail", "trail": "trail_a"}}],
            "trails": {"trail_a": [at(0, 0), at(100, 0)]}}
        w = world_mod.World(spec, 7, "test")
        lat, lng, _a = w.entity("h1").pos_at(0, w)
        self.assertAlmostEqual(lat, at(0, 0)[0])
        self.assertAlmostEqual(lng, at(0, 0)[1])


class TestReact(unittest.TestCase):
    """SL-0013: react: {near:{unit|unit_cls, dist_m, then:{kind,
    speed_mps}}} - the truth tick is the single authority; pos_at
    applies the recorded knot from the overrides file."""

    REACTER = {"id": "p1", "cls": "person", "pos": at(0, 0),
               "react": {"near": {"unit": "cam1", "dist_m": 30,
                                   "then": {"kind": "flee",
                                            "speed_mps": 1.5}}}}

    @staticmethod
    def _pose(lat, lng, cls="copter"):
        return {"lat": lat, "lng": lng, "cls": cls}

    def test_react_validation(self):
        bad = ({"near": {"unit": "c1", "unit_cls": "copter",
                         "dist_m": 10, "then": {"kind": "flee"}}},
               {"near": {"unit": "c1", "then": {"kind": "flee"}}},
               {"near": {"unit": "c1", "dist_m": -5,
                         "then": {"kind": "flee"}}},
               {"near": {"unit": "c1", "dist_m": 10}},
               {"near": {"unit": "c1", "dist_m": 10,
                         "then": {"kind": "teleport"}}},
               {"near": {"unit": "c1", "dist_m": 10,
                         "then": {"kind": "flee", "speed_mps": 0}}},
               {"near": {"unit": "c1", "dist_m": 10,
                         "then": {"kind": "freeze", "speed_mps": 2}}},
               {"near": {"unit": "c1", "dist_m": 10,
                         "then": {"kind": "flee", "bogus": 1}}},
               {"near": {"unit": "c1", "dist_m": 10,
                         "then": {"kind": "flee"}, "bogus": 1}},
               {"far": {"unit": "c1", "dist_m": 10,
                        "then": {"kind": "flee"}}})
        for r in bad:
            with self.assertRaises(simlib.SimError, msg=str(r)):
                spec_world([{"id": "p1", "cls": "person",
                             "pos": at(0, 0), "react": r}])

    def test_trigger_fires_knot(self):
        """A synthetic pose feed inside dist_m produces one knot with
        everything pos_at needs; outside it produces none, and the
        fired latch suppresses a second trigger."""
        w = spec_world([dict(self.REACTER)])
        far = {"cam1": self._pose(*at(0, 200))}
        self.assertEqual(world_mod.react_knots(w, far, 10.0, 10000,
                                               set()), [])
        near = {"cam1": self._pose(*at(0, 20))}
        fired = set()
        knots = world_mod.react_knots(w, near, 10.0, 10000, fired)
        self.assertEqual(len(knots), 1)
        k = knots[0]
        self.assertEqual((k["tid"], k["kind"], k["unit"]),
                         ("p1", "flee", "cam1"))
        self.assertEqual(k["from"], at(0, 0))
        self.assertEqual(k["threat"], at(0, 20))
        self.assertAlmostEqual(k["dist_m"], 20.0, delta=0.5)
        self.assertEqual(k["speed_mps"], 1.5)
        # threat due east of the entity -> flee walks west (~270 deg)
        self.assertAlmostEqual(k["heading_deg"], 270.0, delta=1.0)
        fired.add(k["tid"])
        self.assertEqual(world_mod.react_knots(w, near, 11.0, 11000,
                                               fired), [])

    def test_unit_cls_and_unit_matching(self):
        e = dict(self.REACTER)
        e["react"] = {"near": {"unit_cls": "copter", "dist_m": 30,
                               "then": {"kind": "freeze"}}}
        w = spec_world([e])
        # nearest matching unit decides; a non-matching cls is ignored
        poses = {"s1": self._pose(*at(0, 5), cls="static"),
                 "c1": self._pose(*at(0, 25), cls="copter")}
        knots = world_mod.react_knots(w, poses, 5.0, 5000, set())
        self.assertEqual(len(knots), 1)
        self.assertEqual(knots[0]["unit"], "c1")
        self.assertEqual(knots[0]["kind"], "freeze")

    def _write_overrides(self, d, knots):
        path = os.path.join(d, "world_overrides.jsonl")
        with open(path, "w") as f:
            for k in knots:
                f.write(json.dumps(k) + "\n")
        return path

    def test_pos_at_given_file_is_deterministic(self):
        """The knot file alone decides the override: two Worlds over
        the same spec + file reproduce identical reacted positions
        (the single-authority contract - the stub and the truth
        recorder never diverge)."""
        w = spec_world([dict(self.REACTER)])
        k = world_mod.react_knots(
            w, {"cam1": self._pose(*at(0, 20))}, 10.0, 10000,
            set())[0]
        with self._tmp() as d:
            path = self._write_overrides(d, [k])
            w1 = spec_world([dict(self.REACTER)])
            w2 = spec_world([dict(self.REACTER)])
            w1.use_overrides(path)
            w2.use_overrides(path)
            self.assertEqual(positions(w1, "p1", 600),
                             positions(w2, "p1", 600))
            # before the knot the scripted anchor still answers
            self.assertEqual(positions(w1, "p1", 9),
                             positions(spec_world([dict(self.REACTER)]),
                                       "p1", 9))
            # after it: flees west at 1.5 m/s - t=100 is 90 s * 1.5 m
            lat, lng, _a = w1.entity("p1").pos_at(100.0, w1)
            n, e2 = gp.geo_latlng_to_ne(ORIGIN[0], ORIGIN[1], lat, lng)
            self.assertAlmostEqual(e2, -135.0, delta=2.0)
            st = w1.entity("p1").state_at(100.0, w1)
            self.assertEqual(st["behaviour"], "react:flee")
            self.assertTrue(st["moving"])

        # an unread file changes nothing (non-reactive consumers
        # and pre-attach worlds are untouched)
        w3 = spec_world([dict(self.REACTER)])
        lat3, lng3, _a3 = w3.entity("p1").pos_at(100.0, w3)
        self.assertAlmostEqual(lat3, at(0, 0)[0])
        self.assertAlmostEqual(lng3, at(0, 0)[1])

    def test_freeze_and_seek_knots(self):
        with self._tmp() as d:
            e = dict(self.REACTER)
            e["react"] = {"near": {"unit": "cam1", "dist_m": 30,
                                   "then": {"kind": "freeze"}}}
            w = spec_world([e])
            k = world_mod.react_knots(
                w, {"cam1": self._pose(*at(0, 10))}, 4.0, 4000,
                set())[0]
            w.use_overrides(self._write_overrides(d, [k]))
            self.assertEqual(w.entity("p1").pos_at(4.0, w),
                             w.entity("p1").pos_at(400.0, w))
            self.assertFalse(w.entity("p1").state_at(400.0, w)["moving"])

            e = dict(self.REACTER)
            e["react"] = {"near": {"unit": "cam1", "dist_m": 30,
                                   "then": {"kind": "seek",
                                            "speed_mps": 5}}}
            w = spec_world([e])
            k = world_mod.react_knots(
                w, {"cam1": self._pose(*at(0, 20))}, 4.0, 4000,
                set())[0]
            w.use_overrides(self._write_overrides(d, [k]))
            # 20 m at 5 m/s: arrived by t=8 and holds the threat pos
            lat, lng, _a = w.entity("p1").pos_at(60.0, w)
            self.assertAlmostEqual(lat, at(0, 20)[0])
            self.assertAlmostEqual(lng, at(0, 20)[1])
            self.assertFalse(w.entity("p1").state_at(60.0, w)["moving"])

    def test_attach_follows_reacted_parent(self):
        """An attach child has no react of its own; the parent's knot
        moves it anyway (absolute from/threat, offset reapplied)."""
        spec = [dict(self.REACTER),
                {"id": "ph1", "cls": "phone", "attach": "p1",
                 "offset_m": [0, 5]}]
        w = spec_world(spec)
        k = world_mod.react_knots(
            w, {"cam1": self._pose(*at(0, 20))}, 10.0, 10000, set())[0]
        with self._tmp() as d:
            w.use_overrides(self._write_overrides(d, [k]))
            plat, plng, _a = w.entity("p1").pos_at(100.0, w)
            lat, lng, _a = w.entity("ph1").pos_at(100.0, w)
            d = gp.geo_distance_m(plat, plng, lat, lng)
            self.assertAlmostEqual(d, 5.0, delta=0.5)

    def test_truth_recorder_is_the_authority(self):
        """The 1 Hz tick writes the overrides file + the world_event
        row and latches once - the synthetic fleet pose is a callable
        so this exercises the whole authority path."""
        with self._tmp() as d:
            w = spec_world([dict(self.REACTER)])
            ov = os.path.join(d, "world_overrides.jsonl")
            tp = os.path.join(d, "truth.jsonl")
            epoch_ms = int(time.time() * 1000)
            rec = world_mod.TruthRecorder(
                w, epoch_ms, tp, hz=4.0, overrides_path=ov,
                fleet_poses=lambda: {"cam1": self._pose(*at(0, 10))})
            rec.start()
            time.sleep(1.4)
            rec.stop()
            with open(ov) as f:
                knots = [json.loads(ln) for ln in f]
            self.assertEqual(len(knots), 1)         # latched once
            self.assertEqual(knots[0]["kind"], "flee")
            rows = [json.loads(ln) for ln in open(tp)]
            evs = [r for r in rows if r.get("type") == "world_event"]
            self.assertEqual(len(evs), 1)
            self.assertEqual(evs[0]["event"], "react")
            self.assertIn("react p1 flee", evs[0]["msg"])
            # and the truth rows after the knot already show reacted
            # positions (the recorder's own world reads the file)
            moved = [r for r in rows
                     if r.get("state", {}).get("behaviour")
                     == "react:flee"]
            self.assertTrue(moved)

    # a tiny temp-dir helper keeps the tests' file artifacts out of the
    # tree (var/ is for live runs only)
    @contextlib.contextmanager
    def _tmp(self):
        with tempfile.TemporaryDirectory() as d:
            yield d


if __name__ == "__main__":
    unittest.main(verbosity=2)
