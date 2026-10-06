#!/usr/bin/env python3
"""test_world_pad.py - the PL-08 cls:pad entity surface (no SITL, no
de_comm):

    python3 engine/test_world_pad.py

A pad is an ordinary entity plus yaw_deg / tags_min_m / tags_max_m /
occlude: world.pads() lists it, pad_tag_sizes() returns the geometric
tag ladder the stub counts, and occlude is validated to [t0,t1] world
seconds. Malformed pads fail at load with the entity id in the message.
"""
import json
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

ORIGIN = (-35.363262, 149.165237)


def make_world(entities=None, **extra):
    spec = {"version": 1, "origin": [ORIGIN[0], ORIGIN[1]],
            "entities": entities or []}
    spec.update(extra)
    d = tempfile.mkdtemp()
    p = os.path.join(d, "w.yaml")
    with open(p, "w") as f:
        json.dump(spec, f)
    return world_mod.load(p, 7)


def pad_ent(**kw):
    e = {"id": "pad_1", "cls": "pad", "pos": [ORIGIN[0], ORIGIN[1]],
         "size_m": 0.8, "tags_min_m": 0.02, "tags_max_m": 0.68}
    e.update(kw)
    return e


class T(unittest.TestCase):
    def test_pad_fields_parse(self):
        w = make_world([pad_ent(yaw_deg=45, occlude=[[40, 44], [60, 60]])])
        pads = w.pads()
        self.assertEqual([p.id for p in pads], ["pad_1"])
        pad = pads[0]
        self.assertEqual(pad.yaw_deg, 45.0)
        self.assertEqual(pad.tags_min_m, 0.02)
        self.assertEqual(pad.tags_max_m, 0.68)
        self.assertEqual(pad.occlude, [[40.0, 44.0], [60.0, 60.0]])
        # still an entity: position and truth work like every other cls
        self.assertIs(w.entity("pad_1"), pad)
        lat, lng, alt = pad.pos_at(100.0, w)
        self.assertAlmostEqual(lat, ORIGIN[0])
        self.assertEqual(alt, 0.0)

    def test_pad_defaults(self):
        w = make_world([pad_ent()])
        pad = w.pads()[0]
        self.assertEqual(pad.yaw_deg, 0.0)
        self.assertEqual(pad.occlude, [])

    def test_non_pad_entities_are_not_pads(self):
        w = make_world([pad_ent(),
                        {"id": "p1", "cls": "person",
                         "pos": [ORIGIN[0], ORIGIN[1]]}])
        self.assertEqual([p.id for p in w.pads()], ["pad_1"])
        self.assertEqual(w.classes(), ["pad", "person"])

    def test_no_pad_world(self):
        w = make_world()
        self.assertEqual(w.pads(), [])

    def test_tag_ladder(self):
        w = make_world([pad_ent()])
        sizes = w.pad_tag_sizes(w.pads()[0])
        # geometric x4 ladder largest-first, floor at tags_min_m
        self.assertEqual(sizes, [0.68, 0.17, 0.0425])
        w2 = make_world([pad_ent(tags_min_m=0.02, tags_max_m=0.02)])
        self.assertEqual(w2.pad_tag_sizes(w2.pads()[0]), [0.02])

    def test_pad_missing_tag_bounds_fails(self):
        with self.assertRaises(simlib.SimError) as cm:
            make_world([{"id": "pad_1", "cls": "pad",
                         "pos": [ORIGIN[0], ORIGIN[1]],
                         "size_m": 0.8, "tags_max_m": 0.68}])
        self.assertIn("pad_1", str(cm.exception))

    def test_pad_inverted_ladder_fails(self):
        with self.assertRaises(simlib.SimError):
            make_world([pad_ent(tags_min_m=0.7, tags_max_m=0.02)])

    def test_bad_occlude_fails(self):
        with self.assertRaises(simlib.SimError) as cm:
            make_world([pad_ent(occlude=[[44, 40]])])
        self.assertIn("pad_1", str(cm.exception))
        with self.assertRaises(simlib.SimError):
            make_world([pad_ent(occlude=[[40]])])
        with self.assertRaises(simlib.SimError):
            make_world([pad_ent(occlude="soon")])

    def test_pad_field_world_loads(self):
        w = world_mod.load(os.path.join(ROOT, "worlds",
                                        "pad_field.yaml"), 1)
        pads = w.pads()
        self.assertEqual([p.id for p in pads], ["pad_1"])
        pad = pads[0]
        # 4 m north-east of the origin (the PL-09 nominal land point)
        lat, lng, _alt = pad.pos_at(0.0, w)
        d = world_mod._dist(lat, lng, *ORIGIN)
        self.assertAlmostEqual(d, 4.0, delta=0.01)
        n, e = world_mod._latlng_to_ne(ORIGIN[0], ORIGIN[1], lat, lng)
        self.assertGreater(n, 0.0)
        self.assertGreater(e, 0.0)
        # 7-decimal stored coords leave ~mm asymmetry on the 45-deg leg
        self.assertAlmostEqual(n, e, delta=0.01)
        pl = w.sensors["precland"]
        self.assertEqual((pl["hfov_deg"], pl["vfov_deg"],
                          pl["min_px_tag"], pl["max_rmse_px"],
                          pl["noise_m"], pl["p_drop"]),
                         (60.0, 47.0, 12.0, 2.0, 0.03, 0.02))

    def test_pad_none_world_loads(self):
        w = world_mod.load(os.path.join(ROOT, "worlds",
                                        "pad_none.yaml"), 1)
        self.assertEqual(w.pads(), [])
        self.assertIn("precland", w.sensors)


if __name__ == "__main__":
    unittest.main()
