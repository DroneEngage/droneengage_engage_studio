#!/usr/bin/env python3
"""world.py - the de.world/1 ground-truth model (TASK-P5-08).

A world.yaml describes what is really out there - the people, phones and
clutter the stub modules turn into the messages the real modules would
send. The runner loads it once, injects the stub + geo_projection config
into the fleet, writes run/<sim>/world_epoch.json (scenario t0) and
records 1 Hz truth rows into run/<sim>/truth.jsonl. Each stub process
loads the same file and evaluates entity positions against the shared
epoch, so truth and detections always agree.

    version: 1
    origin: [lat, lng]
    entities:
      - {id: person_1, cls: person,
         path: [[lat,lng],[lat,lng]], speed_mps: 0.4, start_s: 0, loop: true}
      - {id: phone_1, cls: phone, attach: person_1, offset_m: [30,0],
         tx_dbm: 20}
      - {id: mast, cls: phone, pos: [lat,lng], alt_m: 0, tx_dbm: 20}
    obstacles:                       # optional (TASK-P5-10)
      - {id: wood_n, kind: canopy, poly: [[lat,lng],...], height_m: 15,
         base_m: 3, gap_fraction: 0.3, rf_db_per_m: 0.3, rf_max_db: 20}
      - {id: hut_1, kind: building, poly: [[lat,lng],...], height_m: 4,
         rf_loss_db: 12}
    clutter: {cls: person, rate_per_km2_min: 0.2, lifetime_s: 20,
              anchor: ground}
    sensors:
      camera: {lens: angle_linear, hfov_deg, vfov_deg, mount_pitch_deg,
               mount_yaw_deg, mount_roll_deg, mount_height_m, img_w,
               img_h, p_detect_nadir, p_detect_edge, max_range_m,
               pos_noise_px, fps_boxes, size_m: {person: 0.5, "*": 0.5}}
      sdr:    {freq_hz, path_loss_n, ref_dbm_1m, noise_db, threshold_dbm,
               bin_hz, sweep_s}

Entity motion (t = seconds since the scenario epoch):
  pos:    static.
  path:   piecewise-linear walk at speed_mps starting at start_s; with
          loop: true the entity patrols back and forth, otherwise it
          holds the last vertex.
  attach: parent position + offset_m [north, east] (constant offset).

Obstacles (P5-10) are vertical prisms: a footprint polygon (>= 3
vertices, simple, closed implicitly) over a [base_m, height_m] band
above the flat home plane. A canopy attenuates light and RF
Beer-Lambert style - a nadir ray through a full crown sees with
gap_fraction, an oblique ray at angle theta off nadir with
gap_fraction**(1/cos theta) - and a building is opaque. Geometry runs
in local ENU metres about origin (flat earth) with a bbox prefilter;
transmittance()/rf_loss_db() are pure and use no rng.

Obstacle defaults are ESTIMATES (plausible UHF/foliage magnitudes, not
measurements - each world overrides them): canopy base_m 3,
gap_fraction 0.3, rf_db_per_m 0.3, rf_max_db 20; building base_m 0,
rf_loss_db 12. height_m is always required.

All randomness derives streams from (scenario seed, purpose, unit, entity,
tick) so two same-seed runs produce identical model output. A world
with no obstacles and the default sensor keys replays byte-identically
through the stubs (stubs/golden, test_golden.py).
"""

import hashlib
import json
import math
import os
import random
import threading
import time

import simlib

VERSION = 1

CAMERA_DEFAULTS = {
    "lens": "angle_linear",              # | rectilinear (P5-10 D7)
    "hfov_deg": 62.2, "vfov_deg": 48.8,
    "mount_pitch_deg": -90.0, "mount_yaw_deg": 0.0, "mount_roll_deg": 0.0,
    "mount_height_m": 0.0,               # static units only (SU-04)
    "img_w": 1280, "img_h": 720,
    "p_detect_nadir": 0.95, "p_detect_edge": 0.6,
    "max_range_m": 120.0, "pos_noise_px": 8.0, "fps_boxes": 2.0,
    "size_m": {},                        # per-cls box size override
}

SDR_DEFAULTS = {
    "freq_hz": 869000000.0, "path_loss_n": 2.7, "ref_dbm_1m": -30.0,
    "noise_db": 3.0, "threshold_dbm": -95.0,
    "bin_hz": 25000.0, "sweep_s": 2.0,
}

CLUTTER_DEFAULTS = {"cls": "person", "rate_per_km2_min": 0.0,
                    "lifetime_s": 20.0,
                    "anchor": "image"}    # | ground (P5-10 D8)

DEFAULT_SIZE_M = {"person": 0.5, "car": 4.0, "phone": 0.2, "*": 0.5}

# obstacle defaults (ESTIMATES - see the module docstring)
CANOPY_DEFAULTS = {"base_m": 3.0, "gap_fraction": 0.3,
                   "rf_db_per_m": 0.3, "rf_max_db": 20.0}
BUILDING_DEFAULTS = {"base_m": 0.0, "rf_loss_db": 12.0}
OBSTACLE_KEYS = {"id", "kind", "poly", "height_m", "base_m",
                 "gap_fraction", "rf_db_per_m", "rf_max_db", "rf_loss_db"}


# ---------------------------------------------------------------- model

class Entity:
    """One world entity: id, cls, motion (pos | path | attach) and the
    optional RF emitter fields used by the sdr model."""

    def __init__(self, spec, path):
        self.id = str(spec.get("id") or "")
        if not self.id:
            raise simlib.SimError(f"{path}: entity missing 'id'")
        self.cls = str(spec.get("cls") or "object")
        self.pos = spec.get("pos")
        self.path = [list(p) for p in (spec.get("path") or [])]
        self.speed_mps = float(spec.get("speed_mps", 0.0))
        self.start_s = float(spec.get("start_s", 0.0))
        self.loop = bool(spec.get("loop", False))
        self.attach = spec.get("attach")
        self.offset_m = list(spec.get("offset_m") or [0.0, 0.0])
        self.tx_dbm = float(spec["tx_dbm"]) if "tx_dbm" in spec else None
        self.freq_hz = float(spec["freq_hz"]) if "freq_hz" in spec else None
        self.alt_m = float(spec.get("alt_m", 0.0))
        self.size_m = (float(spec["size_m"]) if "size_m" in spec else None)
        kinds = sum(1 for k in ("pos", "path", "attach") if spec.get(k))
        if kinds != 1:
            raise simlib.SimError(
                f"{path}: entity {self.id}: exactly one of pos/path/attach")

    def _path_pos(self, t_s):
        """Position along the path at speed_mps (patrol or hold-at-end)."""
        if len(self.path) == 1 or self.speed_mps <= 0.0:
            return self.path[0]
        seg_lens = []
        total = 0.0
        for a, b in zip(self.path, self.path[1:]):
            d = _dist(a[0], a[1], b[0], b[1])
            seg_lens.append(d)
            total += d
        if total <= 0.0:
            return self.path[0]
        d = max(0.0, t_s - self.start_s) * self.speed_mps
        if self.loop:
            d = d % (2.0 * total)
            if d > total:
                d = 2.0 * total - d
        else:
            d = min(d, total)
        for i, seg in enumerate(seg_lens):
            if d <= seg or i == len(seg_lens) - 1:
                f = 0.0 if seg <= 0.0 else d / seg
                a, b = self.path[i], self.path[i + 1]
                return [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]
            d -= seg
        return self.path[-1]

    def pos_at(self, t_s, world, _depth=0):
        """(lat, lng, alt_above_home_m) at scenario time t_s."""
        if _depth > 8:
            raise simlib.SimError(f"entity {self.id}: attach cycle")
        if self.pos is not None:
            lat, lng = self.pos[0], self.pos[1]
        elif self.path:
            lat, lng = self._path_pos(t_s)
        else:
            parent = world.entity(self.attach)
            if parent is None:
                raise simlib.SimError(
                    f"entity {self.id}: attach {self.attach!r} unknown")
            plat, plng, _palt = parent.pos_at(t_s, world, _depth + 1)
            lat, lng = _ne_to_latlng(plat, plng,
                                     self.offset_m[0], self.offset_m[1])
        return (lat, lng, self.alt_m)


class World:
    def __init__(self, spec, seed, path):
        self.path = path
        self.seed = seed
        self.origin = list(spec.get("origin") or [])
        self.entities = [Entity(e, path) for e in spec.get("entities") or []]
        self._by_id = {e.id: e for e in self.entities}
        self.obstacles = [Obstacle(o, path, i, self.origin)
                          for i, o in enumerate(spec.get("obstacles") or [])]
        clutter = dict(CLUTTER_DEFAULTS)
        clutter.update(spec.get("clutter") or {})
        self.clutter = clutter
        sensors = spec.get("sensors") or {}
        cam = dict(CAMERA_DEFAULTS)
        cam.update(sensors.get("camera") or {})
        if cam.get("lens") not in ("angle_linear", "rectilinear"):
            raise simlib.SimError(
                f"{path}: sensors.camera: unknown lens {cam.get('lens')!r} "
                "(angle_linear | rectilinear)")
        if clutter.get("anchor") not in ("image", "ground"):
            raise simlib.SimError(
                f"{path}: clutter: unknown anchor "
                f"{clutter.get('anchor')!r} (image | ground)")
        sdr = dict(SDR_DEFAULTS)
        sdr.update(sensors.get("sdr") or {})
        self.sensors = {"camera": cam, "sdr": sdr}

    def entity(self, tid):
        return self._by_id.get(tid)

    def classes(self):
        """Distinct entity classes (the 1077 class list order is stable:
        first-seen entity order)."""
        out = []
        for e in self.entities:
            if e.cls not in out:
                out.append(e.cls)
        if self.clutter["cls"] not in out:
            out.append(self.clutter["cls"])
        return out

    def rf_entities(self):
        return [e for e in self.entities if e.tx_dbm is not None]

    def size_m(self, e):
        if e.size_m is not None:
            return e.size_m
        sizes = dict(DEFAULT_SIZE_M)
        sizes.update(self.sensors["camera"].get("size_m") or {})
        return float(sizes.get(e.cls, sizes["*"]))

    # ------------------------------------------------------ obstacles

    def _enu(self, lla):
        """(x=east, y=north, z=alt) metres about world.origin - the
        flat-earth frame every obstacle query runs in (D9)."""
        import geo_projection
        north_m, east_m = geo_projection.geo_latlng_to_ne(
            self.origin[0], self.origin[1], lla[0], lla[1])
        return (east_m, north_m, lla[2])

    def transmittance(self, a_lla, b_lla):
        """(T in [0, 1], blocker_ids) for the a->b ray. Canopy
        attenuates exp(-k*L) with L the ray path inside the band (a
        nadir ray through a full crown sees with gap_fraction, an
        oblique ray with gap_fraction**(1/cos theta)); a building is
        opaque. a/b are (lat, lng, alt_above_home_m). Pure, no rng; an
        obstacle-free world answers (1.0, []) without geometry work.
        blocker_ids lists every obstacle the ray crosses, world order."""
        if not self.obstacles:
            return (1.0, [])
        a = self._enu(a_lla)
        b = self._enu(b_lla)
        seg_m = math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
                          + (b[2] - a[2]) ** 2)
        t_out, blockers, blocked = 1.0, [], False
        for obs in self.obstacles:
            dt = obs.path_fraction(a, b)
            if dt <= 0.0:
                continue
            blockers.append(obs.id)
            if obs.kind == "building":
                blocked = True          # opaque - any entry kills the ray
            else:
                t_out *= math.exp(-obs.k * dt * seg_m)
        return (0.0 if blocked else t_out, blockers)

    def rf_loss_db(self, a_lla, b_lla):
        """(excess_loss_db >= 0, blocker_ids) for the a->b ray: canopy
        rf_db_per_m per metre of path inside the band, capped at
        rf_max_db per crown, plus rf_loss_db per building crossed
        (D6). Pure, no rng; an obstacle-free world answers (0.0, [])."""
        if not self.obstacles:
            return (0.0, [])
        a = self._enu(a_lla)
        b = self._enu(b_lla)
        seg_m = math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
                          + (b[2] - a[2]) ** 2)
        loss, blockers = 0.0, []
        for obs in self.obstacles:
            dt = obs.path_fraction(a, b)
            if dt <= 0.0:
                continue
            blockers.append(obs.id)
            if obs.kind == "canopy":
                loss += min(obs.rf_db_per_m * dt * seg_m, obs.rf_max_db)
            else:
                loss += obs.rf_loss_db
        return (loss, blockers)

    def obstacles_json(self):
        """Renderer-ready obstacle list (P5-13): one dict per obstacle
        with its footprint ring, band and model parameters, world
        order."""
        return [obs.to_json() for obs in self.obstacles]


def _dist(lat1, lng1, lat2, lng2):
    import geo_projection
    return geo_projection.geo_distance_m(lat1, lng1, lat2, lng2)


def _ne_to_latlng(lat, lng, north_m, east_m):
    import geo_projection
    return geo_projection.geo_offset_to_latlng(lat, lng, north_m, east_m)


# --------------------------------------------------------- obstacle math

def _seg_orient(px, py, qx, qy, rx, ry):
    """Cross product sign of q-p x r-p: -1, 0 (collinear) or 1."""
    v = (qx - px) * (ry - py) - (qy - py) * (rx - px)
    if abs(v) < 1e-9:
        return 0
    return 1 if v > 0 else -1


def _seg_touch(p1, p2, p3, p4):
    """True when the closed segments p1-p2 and p3-p4 share any point."""
    def on(a, b, c):
        return min(a[0], b[0]) - 1e-9 <= c[0] <= max(a[0], b[0]) + 1e-9 \
            and min(a[1], b[1]) - 1e-9 <= c[1] <= max(a[1], b[1]) + 1e-9
    d1 = _seg_orient(*p1, *p2, *p3)
    d2 = _seg_orient(*p1, *p2, *p4)
    d3 = _seg_orient(*p3, *p4, *p1)
    d4 = _seg_orient(*p3, *p4, *p2)
    if d1 * d2 < 0 and d3 * d4 < 0:
        return True                  # proper crossing
    return ((d1 == 0 and on(p1, p2, p3)) or (d2 == 0 and on(p1, p2, p4))
            or (d3 == 0 and on(p3, p4, p1)) or (d4 == 0 and on(p3, p4, p2)))


def _cross_t(ax, ay, bx, by, cx, cy, dx, dy):
    """t in [0, 1] where segment a->b crosses segment c->d, else None
    (parallel/collinear edges return None - the caller's midpoint
    tests decide those degenerate boundaries)."""
    d1x, d1y = bx - ax, by - ay
    d2x, d2y = dx - cx, dy - cy
    den = d1x * d2y - d1y * d2x
    if abs(den) < 1e-12:
        return None
    t = ((cx - ax) * d2y - (cy - ay) * d2x) / den
    u = ((cx - ax) * d1y - (cy - ay) * d1x) / den
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return t
    return None


def _point_in_poly(x, y, pts):
    """Even-odd ray cast - True when (x, y) is inside the vertex ring."""
    inside = False
    n = len(pts)
    j = n - 1
    for i in range(n):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and \
                x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _poly_simple(pts):
    """A vertex ring is simple when no edge is degenerate and no two
    non-adjacent edges share a point (adjacent edges share exactly
    their common vertex)."""
    n = len(pts)
    for i in range(n):
        if pts[i] == pts[(i + 1) % n]:
            return False
    for i in range(n):
        e1 = (pts[i], pts[(i + 1) % n])
        for j in range(i + 1, n):
            if (j - i) % n in (1, n - 1):
                continue              # adjacent
            if _seg_touch(e1[0], e1[1], pts[j], pts[(j + 1) % n]):
                return False
    return True


class Obstacle:
    """One vertical prism: a footprint polygon (lat/lng ring, closed
    implicitly) extruded over the [base_m, height_m] band above the
    home plane. A canopy attenuates Beer-Lambert style with
    k = -ln(gap_fraction)/(height_m - base_m) (D3); a building is
    opaque (D4). All stored geometry is local ENU about world.origin
    (x east, y north, D9)."""

    def __init__(self, spec, path, idx, origin):
        oid = spec.get("id")
        self.id = str(oid) if oid else ""
        label = f"obstacle {self.id!r}" if self.id else f"obstacle #{idx}"
        if not isinstance(spec, dict):
            raise simlib.SimError(f"{path}: {label}: not a mapping")
        if not self.id:
            raise simlib.SimError(f"{path}: {label}: missing 'id'")
        kind = spec.get("kind")
        if kind not in ("canopy", "building"):
            raise simlib.SimError(
                f"{path}: {label}: kind must be canopy|building, "
                f"got {kind!r}")
        self.kind = kind
        allowed = {"id", "kind", "poly", "height_m", "base_m"}
        allowed |= ({"gap_fraction", "rf_db_per_m", "rf_max_db"}
                    if kind == "canopy" else {"rf_loss_db"})
        bad = set(spec) - allowed
        if bad:
            raise simlib.SimError(
                f"{path}: {label}: unknown keys {sorted(bad)} "
                f"({kind} allows {sorted(allowed)})")
        poly = spec.get("poly")
        if not isinstance(poly, list) or len(poly) < 3:
            raise simlib.SimError(
                f"{path}: {label}: poly needs at least 3 vertices")
        for p in poly:
            if not isinstance(p, (list, tuple)) or len(p) != 2 or \
                    not all(isinstance(c, (int, float))
                            and not isinstance(c, bool) for c in p):
                raise simlib.SimError(
                    f"{path}: {label}: poly vertices are [lat, lng] pairs")
        self.poly = [list(p) for p in poly]
        if self.poly[0] == self.poly[-1]:
            self.poly = self.poly[:-1]     # closed implicitly
        if len(self.poly) < 3:
            raise simlib.SimError(
                f"{path}: {label}: poly needs at least 3 vertices")
        dflt = CANOPY_DEFAULTS if kind == "canopy" else BUILDING_DEFAULTS
        self.base_m = float(spec.get("base_m", dflt["base_m"]))
        if "height_m" not in spec:
            raise simlib.SimError(f"{path}: {label}: missing 'height_m'")
        self.height_m = float(spec["height_m"])
        if not 0.0 <= self.base_m < self.height_m:
            raise simlib.SimError(
                f"{path}: {label}: needs 0 <= base_m < height_m "
                f"(got {self.base_m} >= {self.height_m})")
        if kind == "canopy":
            self.gap_fraction = float(
                spec.get("gap_fraction", dflt["gap_fraction"]))
            if not 0.0 < self.gap_fraction <= 1.0:
                raise simlib.SimError(
                    f"{path}: {label}: needs 0 < gap_fraction <= 1 "
                    f"(got {self.gap_fraction})")
            self.rf_db_per_m = float(
                spec.get("rf_db_per_m", dflt["rf_db_per_m"]))
            self.rf_max_db = float(spec.get("rf_max_db", dflt["rf_max_db"]))
            self.k = -math.log(self.gap_fraction) / \
                (self.height_m - self.base_m)
        else:
            self.rf_loss_db = float(
                spec.get("rf_loss_db", dflt["rf_loss_db"]))
        # local ENU footprint + bbox (the query prefilter)
        import geo_projection
        self.xy = [tuple(reversed(geo_projection.geo_latlng_to_ne(
            origin[0], origin[1], p[0], p[1]))) for p in self.poly]
        xs = [p[0] for p in self.xy]
        ys = [p[1] for p in self.xy]
        self.min_x, self.max_x = min(xs), max(xs)
        self.min_y, self.max_y = min(ys), max(ys)
        if not _poly_simple(self.xy):
            raise simlib.SimError(f"{path}: {label}: poly is not simple "
                                  "(self-intersecting footprint)")

    def to_json(self):
        out = {"id": self.id, "kind": self.kind, "poly": self.poly,
               "base_m": self.base_m, "height_m": self.height_m}
        if self.kind == "canopy":
            out.update({"gap_fraction": self.gap_fraction,
                        "rf_db_per_m": self.rf_db_per_m,
                        "rf_max_db": self.rf_max_db})
        else:
            out["rf_loss_db"] = self.rf_loss_db
        return out

    def path_fraction(self, a, b):
        """The fraction of the a->b segment (local ENU) inside the prism
        - footprint AND band - 0.0 when the ray misses. The footprint
        enters as parameter intervals (crossings split the segment,
        midpoints decide inside/outside, correct for concave rings),
        intersected with the z interval inside the band."""
        ax, ay, az = a
        bx, by, bz = b
        if max(ax, bx) < self.min_x or min(ax, bx) > self.max_x or \
                max(ay, by) < self.min_y or min(ay, by) > self.max_y:
            return 0.0
        if max(az, bz) < self.base_m or min(az, bz) > self.height_m:
            return 0.0
        ts = {0.0, 1.0}
        n = len(self.xy)
        for i in range(n):
            cx, cy = self.xy[i]
            dx, dy = self.xy[(i + 1) % n]
            t = _cross_t(ax, ay, bx, by, cx, cy, dx, dy)
            if t is not None:
                ts.add(t)
        inside = []
        marks = sorted(ts)
        for t0, t1 in zip(marks, marks[1:]):
            tm = 0.5 * (t0 + t1)
            if _point_in_poly(ax + (bx - ax) * tm, ay + (by - ay) * tm,
                              self.xy):
                inside.append((t0, t1))
        if not inside:
            return 0.0
        # the band along the segment: z(t) = az + t*(bz-az) in the band
        if az == bz:
            if not self.base_m <= az <= self.height_m:
                return 0.0
            band_lo, band_hi = 0.0, 1.0
        else:
            tb = (self.base_m - az) / (bz - az)
            th = (self.height_m - az) / (bz - az)
            band_lo = max(0.0, min(tb, th))
            band_hi = min(1.0, max(tb, th))
            if band_lo > band_hi:
                return 0.0
        total = 0.0
        for t0, t1 in inside:
            lo, hi = max(t0, band_lo), min(t1, band_hi)
            if hi > lo:
                total += hi - lo
        return total


def load(path, seed):
    """Parse + validate a world.yaml. Raises simlib.SimError on any
    format violation."""
    spec = simlib.load_yaml(path)
    if not isinstance(spec, dict):
        raise simlib.SimError(f"{path}: not a world file")
    if int(spec.get("version", 0)) != VERSION:
        raise simlib.SimError(f"{path}: unsupported world version "
                              f"{spec.get('version')!r}")
    bad = set(spec) - {"version", "origin", "entities", "clutter",
                       "sensors", "obstacles"}
    if bad:
        raise simlib.SimError(f"{path}: unknown keys {sorted(bad)}")
    if len(list(spec.get("origin") or [])) != 2:
        raise simlib.SimError(f"{path}: origin must be [lat, lng]")
    w = World(spec, int(seed), path)
    if len({e.id for e in w.entities}) != len(w.entities):
        raise simlib.SimError(f"{path}: duplicate entity id")
    if len({o.id for o in w.obstacles}) != len(w.obstacles):
        raise simlib.SimError(f"{path}: duplicate obstacle id")
    return w


# ------------------------------------------------------------- rng

def rng(seed, *parts):
    """A deterministic stream for (seed, *parts). Everything the stubs
    draw - detection hits, pixel noise, clutter spawns, sdr noise -
    comes from here so a same-seed run reproduces exactly."""
    h = hashlib.sha256(
        ("|".join([str(seed)] + [str(p) for p in parts])).encode()
    ).hexdigest()
    return random.Random(int(h[:16], 16))


# --------------------------------------------------------- sim wiring

STUB_MODULES = ("stub_ai_box", "stub_sdr", "stub_payload")


def unit_modules(entry):
    """Effective module list of a scenario fleet entry: block modules +
    the entry's own add_modules (mirrors simlib._expand_unit)."""
    mods = list(simlib.load_block(entry["block"])["modules"])
    mods += [m for m in entry.get("add_modules", []) if m not in mods]
    return mods


def prepare_sim(scn, world):
    """Inject the world into the scenario's sim spec, before
    launcher.ensure(): each unit running a stub gets the `stub` config
    block (world path, seed, unit name); the geo_projection model of the
    world's camera is pushed onto the unit's projector (de_mavlink on FC
    units, de_comm on statics) so the stub's boxes land on the product
    path the scenario asserts."""
    cam = world.sensors["camera"]
    sdr = world.sensors["sdr"]
    geo = {"enabled": True,
           "hfov_deg": cam["hfov_deg"], "vfov_deg": cam["vfov_deg"],
           "mount_pitch_deg": cam["mount_pitch_deg"],
           "mount_yaw_deg": cam["mount_yaw_deg"],
           "mount_roll_deg": cam["mount_roll_deg"]}
    for idx, entry in enumerate(scn["sim"]["units"]):
        mods = unit_modules(entry)
        cfg = entry.setdefault("config", {})
        slot = scn["sim"]["slot_base"] + idx
        has_fc = bool(simlib.load_block(entry["block"])["fc"])
        stub = {"world": scn["world"], "seed": scn["seed"],
                "unit": entry["name"], "sysid": slot if has_fc else 1,
                # mirrors simlib._expand_unit's `pose` handling: serial5
                # tcp:5 -> 5765+10k on FC units; statics use the 9102 feed
                "pose_port": (5765 + 10 * slot)
                             if (has_fc and "stub_ai_box" in mods) else 0}
        for m in STUB_MODULES:
            if m in mods:
                cfg[m] = simlib.deep_merge(cfg.get(m) or {}, {"stub": stub})
        if "stub_ai_box" in mods:
            if "de_mavlink" in mods:
                cfg["de_mavlink"] = simlib.deep_merge(
                    cfg.get("de_mavlink") or {}, {"geo_projection": geo})
            else:
                # static unit: de_comm's own projector (SU-04) - it needs
                # the camera height above the flat ground as well
                cfg["de_comm"] = simlib.deep_merge(
                    cfg.get("de_comm") or {},
                    {"geo_projection":
                     dict(geo, mount_height_m=cam["mount_height_m"])})
        if "stub_sdr" in mods:
            tx = [e.tx_dbm for e in world.rf_entities()
                  if e.tx_dbm is not None]
            ref = sdr["ref_dbm_1m"] + (tx[0] if tx else 0.0)
            cfg["stub_sdr"] = simlib.deep_merge(
                cfg.get("stub_sdr") or {},
                {"signal_detect": {"ref_power_dbm": ref,
                                   "path_loss_n": sdr["path_loss_n"],
                                   "debounce_sweeps": 3}})


# ------------------------------------------------------- runtime paths

def run_dir(sim):
    return simlib.run_dir(sim)


def epoch_path(sim):
    return os.path.join(simlib.run_dir(sim), "world_epoch.json")


def truth_path(sim):
    return os.path.join(simlib.run_dir(sim), "truth.jsonl")


def stubs_dir(sim):
    return os.path.join(simlib.run_dir(sim), "stubs")


def stub_log_path(sim, unit):
    return os.path.join(stubs_dir(sim), f"{unit}.log")


def write_epoch(sim, world_path, seed):
    """Scenario t0 for every truth function - returns the epoch ms."""
    epoch_ms = int(time.time() * 1000)
    simlib.write_json(epoch_path(sim),
                      {"epoch_unix_ms": epoch_ms,
                       "world": os.path.abspath(world_path),
                       "seed": seed})
    return epoch_ms


def clear_run_state(sim):
    """Reset per-run world state before a scenario starts: drop the
    epoch (stubs pause), truncate the truth file and the stub logs so a
    reused sim only records this run."""
    for p in (epoch_path(sim), truth_path(sim)):
        if os.path.isfile(p):
            os.unlink(p)
    sdir = stubs_dir(sim)
    if os.path.isdir(sdir):
        for f in os.listdir(sdir):
            if f.endswith(".log"):
                os.unlink(os.path.join(sdir, f))


# ------------------------------------------------------------ truth log

class TruthRecorder:
    """1 Hz entity rows -> truth.jsonl:
        {"tid", "cls", "t_ms", "lat", "lng", "alt_m"}
    The stubs append {"type":"detection", ...} rows for every message
    they emit; distance_to/coverage_complete/detection_precision read
    both. start()/stop(); t_ms is unix ms (the fleet clock)."""

    def __init__(self, world, epoch_ms, path, hz=1.0):
        self.world = world
        self.epoch_ms = epoch_ms
        self.path = path
        self.hz = hz
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        open(self.path, "a").close()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(5)

    def _run(self):
        next_t = time.time()
        while not self._stop.is_set():
            now = int(time.time() * 1000)
            t_s = (now - self.epoch_ms) / 1000.0
            for e in self.world.entities:
                lat, lng, alt = e.pos_at(t_s, self.world)
                row = {"tid": e.id, "cls": e.cls, "t_ms": now,
                       "lat": round(lat, 7), "lng": round(lng, 7),
                       "alt_m": round(alt, 2)}
                try:
                    with open(self.path, "a") as f:
                        f.write(json.dumps(row) + "\n")
                except OSError:
                    pass
            next_t += 1.0 / self.hz
            self._stop.wait(max(0.05, next_t - time.time()))
