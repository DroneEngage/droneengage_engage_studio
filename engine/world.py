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
  behaviour (P5-11): a seeded motion model on top of the anchor -
      stationary: {after_s} - follow the anchor motion until after_s,
          then hold (default after_s 0: never moves; the "found
          exhausted" case is a path + a stop).
      wander: {speed_mps, turn_sigma_deg, pause_p, pause_s, bound_m} -
          a correlated random walk around the anchor: heading += a
          gaussian turn each 1 s knot, a pause starts with per-second
          probability pause_p and lasts pause_s seconds, and steps
          past bound_m from the anchor turn back to it.
      trail: {trail, speed_mps, leave_p, bound_m} - walk the named
          trails: polyline (the entity may then omit pos/path/attach -
          the first vertex anchors it), leaving it for wander mode
          with probability leave_p per minute.
      seek: {target, speed_mps, turn_sigma_deg, bound_m} - head for
          target (a features: name or [lat, lng]) with wander noise;
          inside/at the target it becomes a wanderer anchored there.
      The stochastic part is a lazily-extended seeded piecewise-linear
      track in 1 s knots (D1): every process derives the same stream
      from (seed, "behaviour", entity id), so pos_at stays pure.
      wander/seek reject a step that enters a building prism (re-draw
      from the same stream, bounded tries) - canopy is walkable (D3).
      Entity.state_at(t) reports {moving, heading_deg, behaviour} for
      the truth rows.

trails:   {name: [[lat, lng], ...]}      # >= 2 vertices
features: {name: {pos: [lat,lng] | line: [[..]>=2] | poly: [[..]>=3]}}
environment (P5-11, static per run - D4):
  time_of_day:  "hh:mm" local -> a sun-elevation ESTIMATE
                (60 deg at noon, sinusoidal, -60 at midnight; use
                sun_elev_deg directly when the value matters)
  sun_elev_deg: the sun elevation alternative (exclusive with
                time_of_day)
  visibility_m: fog/haze cap on the camera's max_range_m
  precip:       none | rain | snow - the p_detect multiplier
                f_precip is an ESTIMATE (rain 0.8, snow 0.6)
  wind: {speed_mps, dir_deg (FROM, true), gust_mps, turb} - pushed to
        every FC unit's sim.parm as SIM_WIND_SPD (m/s) / SIM_WIND_DIR
        (deg FROM) / SIM_WIND_TURB (m/s) at bring-up (D6; verified on
        the pinned ardupilot SITL.cpp GROUPINFO - there is no gust
        parameter, gust_mps rides the world for P5-16). SIM_WIND_TC
        keeps the ArduPilot 5 s default.

Sensor effects (P5-11 D5, ESTIMATES): p_detect *= f_light * f_precip;
f_light ramps 0 at sun_elev -6 to 1 at +6; effective max_range_m =
min(max_range_m, visibility_m); pixel noise *= 1/f_light. A thermal
camera (sensors.camera.kind: thermal) ignores light and uses flat
p_detect_thermal instead of the nadir/edge ramp. The sdr model adds
rf_precip_db_per_km of excess loss per km of slant path when precip
is on (default 0 at UHF - documented and effectively off).

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
    "kind": "rgb",                       # | thermal (P5-11 D5)
    "p_detect_thermal": 0.85,            # flat p for kind: thermal
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
    "rf_precip_db_per_km": 0.0,          # P5-11 D5 - UHF estimate: off
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

# behaviour defaults (ESTIMATES - see the module docstring)
BEHAVIOUR_KINDS = ("stationary", "wander", "trail", "seek")
WANDER_DEFAULTS = {"speed_mps": 0.6, "turn_sigma_deg": 25.0,
                   "pause_p": 0.02, "pause_s": 60.0, "bound_m": 400.0}
TRAIL_DEFAULTS = {"speed_mps": 1.0, "leave_p": 0.05, "bound_m": 400.0}
SEEK_DEFAULTS = {"speed_mps": 0.8, "turn_sigma_deg": 25.0,
                 "bound_m": 400.0}
BEHAVIOUR_KEYS = {
    "stationary": {"kind", "after_s"},
    "wander": {"kind"} | set(WANDER_DEFAULTS),
    "trail": {"kind", "trail"} | set(TRAIL_DEFAULTS),
    "seek": {"kind", "target"} | set(SEEK_DEFAULTS),
}

# environment defaults (ESTIMATES - see the module docstring)
PRECIP_FACTOR = {"none": 1.0, "rain": 0.8, "snow": 0.6}
SUN_DUSK_DEG = -6.0                    # rgb goes blind below this
SUN_DEFAULT_DEG = 30.0                 # unspecified environment = day
ENV_KEYS = {"time_of_day", "sun_elev_deg", "visibility_m", "precip",
            "wind"}
WIND_KEYS = {"speed_mps", "dir_deg", "gust_mps", "turb"}


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
        self.behaviour = _parse_behaviour(spec.get("behaviour"),
                                          path, self.id)
        self._behav = None            # lazily-built seeded track state
        kinds = sum(1 for k in ("pos", "path", "attach") if spec.get(k))
        if self.behaviour is not None and self.attach is not None:
            raise simlib.SimError(
                f"{path}: entity {self.id}: behaviour needs pos|path "
                "(attach + behaviour is not supported)")
        if kinds != 1:
            if not (kinds == 0 and self.behaviour is not None
                    and self.behaviour["kind"] == "trail"):
                raise simlib.SimError(
                    f"{path}: entity {self.id}: exactly one of "
                    "pos/path/attach (a trail-behaviour entity may "
                    "omit all three - the trail anchors it)")

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
        if self.behaviour is not None:
            kind = self.behaviour["kind"]
            if kind == "stationary":
                # follow the anchor motion until after_s, then hold
                return self._base_pos_at(min(t_s, self.behaviour["after_s"]),
                                         world, _depth)
            tr = self._extend_track(t_s, world)
            knots = tr["knots"]
            i = max(0, int(math.floor(t_s)))
            f = max(0.0, t_s - i)
            a, b = knots[i], knots[i + 1]
            lat = a[0] + (b[0] - a[0]) * f
            lng = a[1] + (b[1] - a[1]) * f
            return (lat, lng, self.alt_m)
        return self._base_pos_at(t_s, world, _depth)

    def _base_pos_at(self, t_s, world, _depth=0):
        """The anchor motion - pos | path | attach - without any
        behaviour layer."""
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

    def state_at(self, t_s, world):
        """{moving, heading_deg, behaviour} for the truth rows. Knot
        state for tracked behaviours, a 0.5 s position delta otherwise
        (a patrolling path flips its heading on the U-turn, a frozen
        stationary reports still)."""
        kind = self.behaviour["kind"] if self.behaviour else None
        if self.behaviour is not None and kind != "stationary":
            tr = self._extend_track(t_s, world)
            k = tr["knots"][max(0, int(math.floor(t_s)))]
            return {"moving": bool(k[2]),
                    "heading_deg": round(k[3], 1),
                    "behaviour": kind}
        p0 = self.pos_at(t_s, world)
        p1 = self.pos_at(t_s + 0.5, world)
        north_m, east_m = _latlng_to_ne(p0[0], p0[1], p1[0], p1[1])
        if math.hypot(north_m, east_m) < 1e-3:
            return {"moving": False, "heading_deg": 0.0,
                    "behaviour": kind}
        return {"moving": True,
                "heading_deg": round(math.degrees(
                    math.atan2(east_m, north_m)) % 360.0, 1),
                "behaviour": kind}

    # -------------------------------------------------------- behaviour
    # The stochastic track of a behaviour entity (D1): a lazily-built
    # list of 1 s knots [lat, lng, moving, heading_deg] extended from
    # one seeded stream, so every process derives identical positions.
    # _behav = {"knots", "rng", "pause_left", "mode", "anchor",
    #           "trail_m"/"trail_pts"/"trail_cum" (trail),
    #           "target" (seek)}

    def _track(self, world):
        if self._behav is None:
            b = self.behaviour
            r = rng(world.seed, "behaviour", self.id)
            if b["kind"] == "trail":
                trail = world.trails[b["trail"]]
                pos = list(trail["pts"][0])
                cum = [0.0]
                for a, bb in zip(trail["pts"], trail["pts"][1:]):
                    cum.append(cum[-1] + _dist(a[0], a[1], bb[0], bb[1]))
                extra = {"mode": "trail", "anchor": pos,
                         "trail_m": 0.0,
                         "trail_pts": trail["pts"],
                         "trail_cum": cum}
            else:
                lat, lng, _a = self._base_pos_at(0.0, world)
                pos = [lat, lng]
                extra = {"mode": "seek" if b["kind"] == "seek"
                         else "wander",
                         "anchor": pos}
            tr = {"knots": [[pos[0], pos[1], False,
                             r.uniform(0.0, 360.0)]],
                  "rng": r, "pause_left": 0}
            tr.update(extra)
            if tr["mode"] == "seek":
                tr["target"] = _seek_target(world, b["target"], pos)
            self._behav = tr
        return self._behav

    def _extend_track(self, t_s, world):
        """Grow the knot list until it covers t_s (knots[i+1] must
        exist for pos_at's interpolation)."""
        tr = self._track(world)
        need = max(0, int(math.floor(t_s))) + 1
        while len(tr["knots"]) <= need:
            self._step_knot(tr, world)
        return tr

    def _step_knot(self, tr, world):
        """Advance one 1 s knot. All seeded draws - the initial heading,
        the per-knot pause/leave checks and every heading re-draw - come
        from the same stream in the same order in every process."""
        b = self.behaviour
        r = tr["rng"]
        lat, lng, _moving, heading = tr["knots"][-1]
        if tr["pause_left"] > 0:
            tr["pause_left"] -= 1
            tr["knots"].append([lat, lng, False, heading])
            return
        if tr["mode"] == "trail":
            if r.random() < b["leave_p"] / 60.0:
                tr["mode"], tr["anchor"] = "wander", [lat, lng]
        if tr["mode"] == "trail":
            pts, cum = tr["trail_pts"], tr["trail_cum"]
            d = min(tr["trail_m"] + b["speed_mps"], cum[-1])
            tr["trail_m"] = d
            pos, seg_heading = _polyline_at(pts, cum, d, heading)
            if d >= cum[-1] - 1e-9:
                tr["mode"], tr["anchor"] = "wander", list(pos)
            tr["knots"].append([pos[0], pos[1], True, seg_heading])
            return
        if tr["mode"] == "seek":
            tgt = tr["target"]
            d = _dist(lat, lng, tgt[0], tgt[1])
            if d <= max(b["speed_mps"], 2.0):
                tr["mode"], tr["anchor"] = "wander", list(tgt)
                tr["knots"].append([tgt[0], tgt[1], True, heading])
                return
            for _try in range(8):
                h = (_bearing_deg(lat, lng, tgt[0], tgt[1])
                     + r.gauss(0.0, b["turn_sigma_deg"])) % 360.0
                clat, clng = _walk_to(lat, lng, b["speed_mps"], h)
                if world.point_in_building(clat, clng, self.alt_m):
                    continue
                tr["knots"].append([clat, clng, True, h])
                return
            tr["knots"].append([lat, lng, False, heading])
            return
        # wander (also the post-leave/post-arrive mode of trail/seek -
        # their own speed_mps and bound_m, wander defaults for the rest)
        pause_p = float(b.get("pause_p", WANDER_DEFAULTS["pause_p"]))
        if r.random() < pause_p:
            tr["pause_left"] = max(1, int(round(
                b.get("pause_s", WANDER_DEFAULTS["pause_s"]))))
            tr["knots"].append([lat, lng, False, heading])
            return
        sigma = float(b.get("turn_sigma_deg",
                            WANDER_DEFAULTS["turn_sigma_deg"]))
        bound = float(b.get("bound_m", WANDER_DEFAULTS["bound_m"]))
        speed = float(b["speed_mps"])
        for _try in range(8):
            h = (heading + r.gauss(0.0, sigma)) % 360.0
            clat, clng = _walk_to(lat, lng, speed, h)
            if tr["anchor"] is not None and \
                    _dist(clat, clng, tr["anchor"][0],
                          tr["anchor"][1]) > bound:
                h = _bearing_deg(clat, clng, tr["anchor"][0],
                                 tr["anchor"][1])
                clat, clng = _walk_to(lat, lng, speed, h)
            if world.point_in_building(clat, clng, self.alt_m):
                continue
            tr["knots"].append([clat, clng, True, h])
            return
        tr["knots"].append([lat, lng, False, heading])  # cornered


class World:
    def __init__(self, spec, seed, path):
        self.path = path
        self.seed = seed
        self.origin = list(spec.get("origin") or [])
        self.trails = _parse_trails(spec.get("trails"), path)
        self.features = _parse_features(spec.get("features"), path,
                                        self.origin)
        self.environment = _parse_environment(spec.get("environment"),
                                              path)
        self.entities = [Entity(e, path) for e in spec.get("entities") or []]
        self._by_id = {e.id: e for e in self.entities}
        self.obstacles = [Obstacle(o, path, i, self.origin)
                          for i, o in enumerate(spec.get("obstacles") or [])]
        for e in self.entities:
            b = e.behaviour
            if b is None:
                continue
            if b["kind"] == "trail" and b["trail"] not in self.trails:
                raise simlib.SimError(
                    f"{path}: entity {e.id}: unknown trail "
                    f"{b['trail']!r} (trails: {sorted(self.trails)})")
            if b["kind"] == "seek" and isinstance(b["target"], str) \
                    and b["target"] not in self.features:
                raise simlib.SimError(
                    f"{path}: entity {e.id}: unknown feature "
                    f"{b['target']!r} (features: {sorted(self.features)})")
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
        if cam.get("kind") not in ("rgb", "thermal"):
            raise simlib.SimError(
                f"{path}: sensors.camera: unknown kind "
                f"{cam.get('kind')!r} (rgb | thermal)")
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

    def point_in_building(self, lat, lng, alt_m):
        """True when (lat, lng, alt_above_home) lies inside any
        building prism - the wander/seek step rejection test (D3).
        Canopy is walkable and answers False."""
        if not self.obstacles:
            return False
        enu = self._enu((lat, lng, alt_m))
        return any(o.kind == "building" and o.contains(enu)
                   for o in self.obstacles)

    # ------------------------------------------------------ environment

    def wind(self):
        """The environment's wind dict {speed_mps, dir_deg (FROM),
        gust_mps, turb} or None - the P5-16 payload-drift source."""
        return self.environment["wind"]

    def sensor_factors(self, kind):
        """P5-11 D5 multiplicative factors for the camera model:
        p_scale multiplies p_detect (f_light * f_precip; light is 1
        for a thermal camera), noise_scale multiplies pixel noise
        (1/f_light, clamped), max_range_m is the visibility-capped
        range. A world without an environment block answers the
        identity factors."""
        env = self.environment
        precip = PRECIP_FACTOR[env["precip"]]
        if kind == "thermal":
            light = 1.0
        else:
            light = _f_light(env["sun_elev_deg"])
        return {"light": light, "precip": precip,
                "p_scale": light * precip,
                "noise_scale": (1.0 if kind == "thermal"
                                else 1.0 / max(light, 0.05)),
                "max_range_m": min(
                    float(self.sensors["camera"]["max_range_m"]),
                    env["visibility_m"])}


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

    def contains(self, enu):
        """(x, y, z) local ENU inside the prism: footprint AND band."""
        x, y, z = enu
        if x < self.min_x or x > self.max_x or \
                y < self.min_y or y > self.max_y:
            return False
        if not self.base_m <= z <= self.height_m:
            return False
        return _point_in_poly(x, y, self.xy)

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


# ------------------------------------------------------- behaviour geo

def _latlng_to_ne(lat1, lng1, lat2, lng2):
    """(north_m, east_m) of point 2 relative to point 1."""
    import geo_projection
    return geo_projection.geo_latlng_to_ne(lat1, lng1, lat2, lng2)


def _bearing_deg(lat1, lng1, lat2, lng2):
    """True heading 0..360 from point 1 to point 2."""
    north_m, east_m = _latlng_to_ne(lat1, lng1, lat2, lng2)
    return math.degrees(math.atan2(east_m, north_m)) % 360.0


def _walk_to(lat, lng, dist_m, heading_deg):
    """Step dist_m along a true heading from (lat, lng)."""
    rad = math.radians(heading_deg)
    return _ne_to_latlng(lat, lng,
                         dist_m * math.cos(rad),
                         dist_m * math.sin(rad))


def _polyline_at(pts, cum, d_m, last_heading):
    """(pos, heading_deg) at arclength d_m along the polyline; a
    degenerate segment keeps the last heading."""
    for i in range(len(cum) - 1):
        if d_m <= cum[i + 1] or i == len(cum) - 2:
            a, b = pts[i], pts[i + 1]
            seg = cum[i + 1] - cum[i]
            f = 0.0 if seg <= 0.0 else (d_m - cum[i]) / seg
            pos = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]
            if seg > 1e-6:
                last_heading = _bearing_deg(a[0], a[1], b[0], b[1])
            return pos, last_heading
    return list(pts[-1]), last_heading


def _nearest_on_seg(px, py, ax, ay, bx, by):
    """The point on segment a->b nearest to p (local ENU)."""
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 <= 0.0 else max(
        0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return (ax + dx * t, ay + dy * t)


def _seek_target(world, target, start_ll):
    """Resolve a seek target to a [lat, lng]: a literal point, or the
    feature's nearest point to the entity's anchor (polyline vertex,
    ring boundary; already-inside stays at the anchor). Deterministic
    - no stream draws."""
    if isinstance(target, list):
        return list(target)
    f = world.features[target]
    sx, sy, _z = world._enu((start_ll[0], start_ll[1], 0.0))
    if f["kind"] == "pos":
        return list(f["pts"][0])
    if f["kind"] == "poly" and _point_in_poly(sx, sy, f["xy"]):
        return list(start_ll)          # already inside - wander here
    xy = f["xy"]
    n = len(xy)
    best, best_d = None, None
    for i in range(n) if f["kind"] == "poly" else range(n - 1):
        q = _nearest_on_seg(sx, sy, xy[i][0], xy[i][1],
                            xy[(i + 1) % n][0], xy[(i + 1) % n][1])
        d = math.hypot(sx - q[0], sy - q[1])
        if best is None or d < best_d:
            best, best_d = q, d
    lat, lng = _ne_to_latlng(world.origin[0], world.origin[1],
                             best[1], best[0])
    return [lat, lng]


def _f_light(sun_elev_deg):
    """ESTIMATE: the rgb light factor - 0 at/below civil twilight
    (-6 deg), ramping linearly to 1 at +6 deg."""
    return max(0.0, min(1.0, (sun_elev_deg - SUN_DUSK_DEG) / 12.0))


# ------------------------------------------------- behaviour parsers

def _parse_behaviour(spec, path, eid):
    """Validate + normalize an entity's behaviour block (None when
    absent)."""
    if spec is None:
        return None
    label = f"{path}: entity {eid}: behaviour"
    if not isinstance(spec, dict):
        raise simlib.SimError(f"{label}: not a mapping")
    kind = spec.get("kind")
    if kind not in BEHAVIOUR_KINDS:
        raise simlib.SimError(
            f"{label}: kind must be {'|'.join(BEHAVIOUR_KINDS)}, "
            f"got {kind!r}")
    bad = set(spec) - BEHAVIOUR_KEYS[kind]
    if bad:
        raise simlib.SimError(
            f"{label}: unknown keys {sorted(bad)} ({kind} allows "
            f"{sorted(BEHAVIOUR_KEYS[kind])})")
    out = {"kind": kind}
    if kind == "stationary":
        out["after_s"] = float(spec.get("after_s", 0.0))
        if out["after_s"] < 0.0:
            raise simlib.SimError(f"{label}: after_s must be >= 0")
    elif kind == "trail":
        if "trail" not in spec:
            raise simlib.SimError(f"{label}: a trail needs its name")
        out["trail"] = str(spec["trail"])
        out["speed_mps"] = float(spec.get(
            "speed_mps", TRAIL_DEFAULTS["speed_mps"]))
        out["leave_p"] = float(spec.get(
            "leave_p", TRAIL_DEFAULTS["leave_p"]))
        out["bound_m"] = float(spec.get(
            "bound_m", TRAIL_DEFAULTS["bound_m"]))
        if out["speed_mps"] <= 0.0 or not 0.0 <= out["leave_p"] <= 1.0 \
                or out["bound_m"] <= 0.0:
            raise simlib.SimError(
                f"{label}: needs speed_mps>0, 0<=leave_p<=1, bound_m>0")
    elif kind == "seek":
        target = spec.get("target")
        if isinstance(target, str):
            out["target"] = target
        elif isinstance(target, (list, tuple)) and len(target) == 2 \
                and all(isinstance(c, (int, float))
                        and not isinstance(c, bool) for c in target):
            out["target"] = [float(target[0]), float(target[1])]
        else:
            raise simlib.SimError(
                f"{label}: target must be a feature name or [lat, lng]")
        out["speed_mps"] = float(spec.get(
            "speed_mps", SEEK_DEFAULTS["speed_mps"]))
        out["turn_sigma_deg"] = float(spec.get(
            "turn_sigma_deg", SEEK_DEFAULTS["turn_sigma_deg"]))
        out["bound_m"] = float(spec.get(
            "bound_m", SEEK_DEFAULTS["bound_m"]))
        if out["speed_mps"] <= 0.0 or out["turn_sigma_deg"] < 0.0 \
                or out["bound_m"] <= 0.0:
            raise simlib.SimError(
                f"{label}: needs speed_mps>0, turn_sigma_deg>=0, "
                "bound_m>0")
    else:  # wander
        out.update({k: float(spec.get(k, d))
                    for k, d in WANDER_DEFAULTS.items()})
        if out["speed_mps"] <= 0.0 or out["turn_sigma_deg"] < 0.0 \
                or not 0.0 <= out["pause_p"] <= 1.0 \
                or out["pause_s"] < 0.0 or out["bound_m"] <= 0.0:
            raise simlib.SimError(
                f"{label}: needs speed_mps>0, turn_sigma_deg>=0, "
                "0<=pause_p<=1, pause_s>=0, bound_m>0")
    return out


def _parse_trails(spec, path):
    """trails: {name: [[lat, lng], ...>=2]} -> {name: {"pts"}}."""
    out = {}
    for name, poly in (spec or {}).items():
        label = f"{path}: trails.{name}"
        if not isinstance(poly, list) or len(poly) < 2 or \
                not all(isinstance(p, (list, tuple)) and len(p) == 2 and
                        all(isinstance(c, (int, float))
                            and not isinstance(c, bool) for c in p)
                        for p in poly):
            raise simlib.SimError(
                f"{label}: a trail is >= 2 [lat, lng] vertices")
        out[str(name)] = {"pts": [list(p) for p in poly]}
    return out


def _parse_features(spec, path, origin):
    """features: {name: {pos | line | poly}} -> {name: {"kind", "pts",
    "xy"}} - the seek targets. pts in lat/lng, xy in local ENU."""
    import geo_projection
    out = {}
    for name, f in (spec or {}).items():
        label = f"{path}: features.{name}"
        if not isinstance(f, dict):
            raise simlib.SimError(f"{label}: not a mapping")
        kinds = [k for k in ("pos", "line", "poly") if k in f]
        if len(kinds) != 1 or set(f) - {"pos", "line", "poly"}:
            raise simlib.SimError(
                f"{label}: exactly one of pos|line|poly")
        kind = kinds[0]
        pts = f[kind]
        need = 1 if kind == "pos" else (2 if kind == "line" else 3)
        pts = [pts] if kind == "pos" else pts
        if not isinstance(pts, list) or len(pts) < need or \
                not all(isinstance(p, (list, tuple)) and len(p) == 2 and
                        all(isinstance(c, (int, float))
                            and not isinstance(c, bool) for c in p)
                        for p in pts):
            raise simlib.SimError(
                f"{label}: {kind} needs >= {need} [lat, lng] vertices")
        if kind == "pos" and len(pts) != 1:
            raise simlib.SimError(f"{label}: pos is one [lat, lng]")
        xy = [tuple(reversed(geo_projection.geo_latlng_to_ne(
            origin[0], origin[1], p[0], p[1]))) for p in pts]
        out[str(name)] = {"kind": kind, "pts": [list(p) for p in pts],
                          "xy": xy}
    return out


def _parse_environment(spec, path):
    """environment: -> {"sun_elev_deg", "visibility_m", "precip",
    "wind"} with the documented defaults. Static per run (D4)."""
    env = {"sun_elev_deg": SUN_DEFAULT_DEG,
           "visibility_m": float("inf"),
           "precip": "none", "wind": None}
    if not spec:
        return env
    label = f"{path}: environment"
    if not isinstance(spec, dict):
        raise simlib.SimError(f"{label}: not a mapping")
    bad = set(spec) - ENV_KEYS
    if bad:
        raise simlib.SimError(f"{label}: unknown keys {sorted(bad)}")
    if "time_of_day" in spec and "sun_elev_deg" in spec:
        raise simlib.SimError(
            f"{label}: time_of_day and sun_elev_deg are exclusive")
    if "time_of_day" in spec:
        tod = spec["time_of_day"]
        if not isinstance(tod, str) or ":" not in tod:
            raise simlib.SimError(
                f"{label}: time_of_day is an 'hh:mm' string")
        try:
            hh, mm = (int(x) for x in tod.split(":", 1))
        except ValueError:
            raise simlib.SimError(
                f"{label}: time_of_day is an 'hh:mm' string")
        if not (0 <= hh <= 23 and 0 <= mm <= 59):
            raise simlib.SimError(
                f"{label}: time_of_day {tod!r} out of range")
        # ESTIMATE: 60 deg at noon, sinusoidal day/night curve
        env["sun_elev_deg"] = 60.0 * math.sin(
            math.pi * (hh + mm / 60.0 - 6.0) / 12.0)
    elif "sun_elev_deg" in spec:
        env["sun_elev_deg"] = float(spec["sun_elev_deg"])
        if not -90.0 <= env["sun_elev_deg"] <= 90.0:
            raise simlib.SimError(
                f"{label}: sun_elev_deg out of range")
    if "visibility_m" in spec:
        env["visibility_m"] = float(spec["visibility_m"])
        if env["visibility_m"] <= 0.0:
            raise simlib.SimError(f"{label}: visibility_m must be > 0")
    if "precip" in spec:
        if spec["precip"] not in PRECIP_FACTOR:
            raise simlib.SimError(
                f"{label}: precip must be "
                f"{'|'.join(sorted(PRECIP_FACTOR))}, "
                f"got {spec['precip']!r}")
        env["precip"] = spec["precip"]
    if "wind" in spec:
        w = spec["wind"]
        if not isinstance(w, dict) or set(w) - WIND_KEYS:
            raise simlib.SimError(
                f"{label}: wind keys are {sorted(WIND_KEYS)}")
        wind = {"speed_mps": float(w.get("speed_mps", 0.0)),
                "dir_deg": float(w.get("dir_deg", 0.0)),
                "gust_mps": float(w.get("gust_mps", 0.0)),
                "turb": float(w.get("turb", 0.0))}
        if wind["speed_mps"] < 0.0 or wind["gust_mps"] < 0.0 \
                or wind["turb"] < 0.0 \
                or not 0.0 <= wind["dir_deg"] < 360.0:
            raise simlib.SimError(
                f"{label}: needs speed/gust/turb >= 0, "
                "0 <= dir_deg < 360")
        env["wind"] = wind
    return env


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
                       "sensors", "obstacles", "trails", "features",
                       "environment"}
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
        wind = world.environment["wind"]
        if has_fc and wind and wind["speed_mps"] > 0.0:
            # P5-11 D6: the world wind reaches SITL as SIM_WIND_SPD /
            # SIM_WIND_DIR (deg FROM) / SIM_WIND_TURB in sim.parm -
            # entry params, if set, win over the world
            fce = entry.setdefault("fc", {})
            fce["params"] = simlib.deep_merge(
                {"SIM_WIND_SPD": wind["speed_mps"],
                 "SIM_WIND_DIR": wind["dir_deg"],
                 "SIM_WIND_TURB": wind["turb"]},
                fce.get("params") or {})
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
                       "alt_m": round(alt, 2),
                       "state": e.state_at(t_s, self.world)}
                try:
                    with open(self.path, "a") as f:
                        f.write(json.dumps(row) + "\n")
                except OSError:
                    pass
            next_t += 1.0 / self.hz
            self._stop.wait(max(0.05, next_t - time.time()))
