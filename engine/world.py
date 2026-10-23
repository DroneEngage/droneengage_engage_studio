#!/usr/bin/env python3
"""world.py - the de.world/1 ground-truth model (TASK-P5-08).

A world.yaml describes what is really out there - the people, phones and
clutter the stub modules turn into the messages the real modules would
send. The runner loads it once, injects the stub + geo_projection config
into the fleet, writes var/run/<sim>/world_epoch.json (scenario t0) and
records 1 Hz truth rows into var/run/<sim>/truth.jsonl. Each stub process
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
    an entity may carry temp_c (TASK-13): the temperature a thermal
    camera contrasts against the ambient - unset falls back to the
    cls ESTIMATE (person 34, vehicle/car 40 while it runs and ambient
    once it holds, every other cls ambient).
    obstacles:                       # optional (TASK-P5-10)
      - {id: wood_n, kind: canopy, poly: [[lat,lng],...], height_m: 15,
         base_m: 3, gap_fraction: 0.3, rf_db_per_m: 0.3, rf_max_db: 20}
      - {id: hut_1, kind: building, poly: [[lat,lng],...], height_m: 4,
         rf_loss_db: 12}
    clutter: {cls: person, rate_per_km2_min: 0.2, lifetime_s: 20,
              anchor: ground}
    sensors:
      camera: {lens: rectilinear, hfov_deg, vfov_deg, k1, k2, mount_pitch_deg,
               mount_yaw_deg, mount_roll_deg, mount_height_m, img_w,
               img_h, p_detect_nadir, p_detect_edge, max_range_m,
               pos_noise_px, fps_boxes, size_m: {person: 0.5, "*": 0.5}}
      sdr:    {freq_hz, path_loss_n, ref_dbm_1m, noise_db, threshold_dbm,
               bin_hz, sweep_s}
      precland: {hfov_deg, vfov_deg, min_px_tag, max_rmse_px, noise_m,
                 p_drop}            # stub_precland's downward camera (PL-08)

A `cls: pad` entity is a precision-landing AprilTag pad (PL-08): an
ordinary entity for position/truth plus the pad keys yaw_deg, size_m
(board edge), tags_min_m / tags_max_m (smallest/largest tag edge in the
geometric tag ladder) and occlude [[t0_s, t1_s], ...] (world seconds the
pad is invisible). world.pads() lists them; stub_ai_box never reports a
pad as a detection class.

A `cls: aircraft` entity (SL-0016) is an ADS-B intruder: an ordinary
moving entity (pos/path/attach + alt_m) plus icao (required - the 24-bit
ICAO address, an int or a hex string like "0xABCDEF"), callsign (<= 8
chars, defaults to the entity id), emitter_type (MAVLink
ADSB_EMITTER_TYPE, default 1 = light aircraft), squawk (optional) and
units (optional list of unit names - the frames go only to those units).
While a scenario's fleet is up, an AdsbFeeder thread in the runner
encodes ADSB_VEHICLE frames from the entities' truth at 1 Hz and injects
them through each fcb-proxied FC unit's `inject` control command
(SL-0008) - de_mavlink parses and forwards them to the GCS exactly like
the frames SITL's own SIM_ADSB_* traffic produces, and world.aircraft()
lists the entities it feeds.

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
areas:    {name: {poly: [[lat,lng],...]}}# named search polygons the
          coverage assertions grid over (P5-12 D2)
environment (P5-11, static per run - D4):
  time_of_day:  "hh:mm" local -> a sun-elevation ESTIMATE
                (60 deg at noon, sinusoidal, -60 at midnight; use
                sun_elev_deg directly when the value matters)
  sun_elev_deg: the sun elevation alternative (exclusive with
                time_of_day)
  visibility_m: fog/haze cap on the camera's max_range_m
  precip:       none | rain | snow - the p_detect multiplier
                f_precip is an ESTIMATE (rain 0.8, snow 0.6)
  ambient_c:    the thermal model's background temperature (TASK-13)
                - absent: ESTIMATE 15 + 0.4 * sun_elev_deg (39 at sun
                60, -9 at -60; use ambient_c directly when the value
                matters)
  wind: {speed_mps, dir_deg (FROM, true), gust_mps, turb} - pushed to
        every FC unit's sim.parm as SIM_WIND_SPD (m/s) / SIM_WIND_DIR
        (deg FROM) / SIM_WIND_TURB (m/s) at bring-up (D6; verified on
        the pinned ardupilot SITL.cpp GROUPINFO - there is no gust
        parameter, gust_mps rides the world for P5-16). SIM_WIND_TC
        keeps the ArduPilot 5 s default.

Sensor effects (P5-11 D5, ESTIMATES): p_detect *= f_light * f_precip;
f_light ramps 0 at sun_elev -6 to 1 at +6; effective max_range_m =
min(max_range_m, visibility_m); pixel noise *= 1/f_light. A thermal
camera (sensors.camera.kind: thermal) ignores light and uses
p_detect_thermal instead of the nadir/edge ramp, scaled by the
TASK-13 contrast ESTIMATE |temp_c - ambient_c| / 15 capped at 1 - a
target at ambient (the dawn/dusk crossover) reads 0. The sdr model adds
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

Reactive entities (SL-0013) - `react:` on an entity, ONE authority:

    - {id: person_1, cls: person, pos: [lat,lng],
       react: {near: {unit: cam1, dist_m: 30,
                      then: {kind: flee, speed_mps: 1.2}}}}
    - {id: guard_1, cls: person, path: [...],
       react: {near: {unit_cls: copter, dist_m: 50,
                      then: {kind: freeze}}}}

A reaction depends on fleet pose, so per-process evaluation would
diverge truth from detections (pos_at is pure; every stub evaluates it
independently). The design: the runner's 1 Hz truth tick is the single
authority. It evaluates near: triggers against the fleet poses
(horizontal ground distance; `unit` a fleet unit name, `unit_cls` the
scenario fleet cls), appends each latched knot - once per entity per
run - to var/run/<sim>/world_overrides.jsonl and logs a
{"type": "world_event", "event": "react", ...} row to truth.jsonl.
then: {kind: flee|freeze|seek, speed_mps}: flee walks straight away
from the recorded threat pose at speed_mps, freeze holds the trigger
position, seek walks to it (a moving unit is RECORDED at trigger time,
not tracked - pos_at stays pure GIVEN THE FILE). Stubs and the world
stream read the same file (world.use_overrides), so truth, detections
and the render feed see the same motion. A reactive world is excluded
from golden byte-identity (the authority is wall-clock - statistically
reproducible only); a non-reactive world never touches the file. The
scenario `on:` watcher accepts {world: <entity_id>, re: ...} against
the world_event rows.

All randomness derives streams from (scenario seed, purpose, unit, entity,
tick) so two same-seed runs produce identical model output. A world
with no obstacles and the default sensor keys replays byte-identically
through the stubs (stubs/golden, test_golden.py).
"""

import hashlib
import io
import json
import math
import os
import random
import socket
import threading
import time

import simlib

VERSION = 1

CAMERA_DEFAULTS = {
    "lens": "rectilinear",               # | angle_linear (P5-10 D7; P5-18 default)
    "kind": "rgb",                       # | thermal (P5-11 D5)
    "p_detect_thermal": 0.85,            # flat p for kind: thermal
    "hfov_deg": 62.2, "vfov_deg": 48.8,
    "k1": 0.0, "k2": 0.0,                # Brown radial distortion (P5-18 D3)
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

# stub_precland's downward camera (PL-08): FOV of the tag camera, the
# minimum projected tag edge the detector resolves, the RMSE the model
# reports at the resolution limit, pose noise and the per-frame drop
# probability. img_w/img_h give the tag edge its pixel scale.
PRECLAND_DEFAULTS = {"hfov_deg": 60.0, "vfov_deg": 47.0,
                     "min_px_tag": 12.0, "max_rmse_px": 2.0,
                     "noise_m": 0.03, "p_drop": 0.02,
                     "img_w": 1280, "img_h": 720}

# geometric ratio between consecutive tag sizes on a pad (tags_min_m ..
# tags_max_m) - an AprilTag landing target nests tags 4x apart
PAD_TAG_RATIO = 4.0

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

# react: defaults (SL-0013 - see the module docstring; the single
# authority is the truth tick, not the entity model itself)
REACT_THEN_KINDS = ("flee", "freeze", "seek")
REACT_DEFAULT_SPEED_MPS = {"flee": 1.5, "seek": 1.0}
REACT_NEAR_KEYS = {"unit", "unit_cls", "dist_m", "then"}
REACT_THEN_KEYS = {"kind", "speed_mps"}

# environment defaults (ESTIMATES - see the module docstring)
PRECIP_FACTOR = {"none": 1.0, "rain": 0.8, "snow": 0.6}
SUN_DUSK_DEG = -6.0                    # rgb goes blind below this
SUN_DEFAULT_DEG = 30.0                 # unspecified environment = day
ENV_KEYS = {"time_of_day", "sun_elev_deg", "visibility_m", "precip",
            "wind", "ambient_c"}
WIND_KEYS = {"speed_mps", "dir_deg", "gust_mps", "turb"}

# thermal model defaults (TASK-13, ESTIMATES - see the docstring)
AMBIENT_BASE_C = 15.0                  # shade temp at sun_elev 0
AMBIENT_GAIN_C_PER_DEG = 0.4           # per degree of sun elevation
TEMP_C_PERSON = 34.0
TEMP_C_VEHICLE_RUNNING = 40.0          # cold-soaks to ambient when still
VEHICLE_CLS = {"vehicle", "car"}
THERMAL_FULL_CONTRAST_C = 15.0         # |temp - ambient| reading full p


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
        # TASK-13: the thermal contrast target - None falls back to
        # the cls ESTIMATE at world.temp_c_at()
        self.temp_c = (float(spec["temp_c"]) if "temp_c" in spec
                       else None)
        # PL-08 pad metadata (meaningful on cls: pad - a landing board's
        # yaw, its tag-edge ladder and the world-second windows it is
        # occluded in); unused on every other cls
        self.yaw_deg = float(spec.get("yaw_deg", 0.0))
        self.tags_min_m = float(spec["tags_min_m"]) \
            if "tags_min_m" in spec else None
        self.tags_max_m = float(spec["tags_max_m"]) \
            if "tags_max_m" in spec else None
        self.occlude = _parse_occlude(spec.get("occlude"), path, self.id)
        # SL-0016 aircraft fields (meaningful on cls: aircraft - the
        # ADSB_VEHICLE identity the feeder encodes); unused on every
        # other cls
        self.icao = _parse_icao(spec.get("icao"), path, self.id)
        self.callsign = str(spec.get("callsign") or self.id)[:8]
        self.emitter_type = int(spec.get("emitter_type", 1))
        self.squawk = int(spec["squawk"]) if "squawk" in spec else None
        self.units = _parse_str_list(spec.get("units"), path, self.id)
        self.behaviour = _parse_behaviour(spec.get("behaviour"),
                                          path, self.id)
        self._behav = None            # lazily-built seeded track state
        # SL-0013: react: is only a TRIGGER DECLARATION - the truth tick
        # latches it into world_overrides.jsonl; pos_at applies the
        # recorded knot (see _react_pos_at). Nothing reacts here.
        self.react = _parse_react(spec.get("react"), path, self.id)
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
        if self.react is not None:
            # SL-0013: a latched knot overrides every other motion from
            # its t_s on (the file makes it identical in every process)
            pos = self._react_pos_at(t_s, world)
            if pos is not None:
                return pos
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
        if self.react is not None:
            st = self._react_state_at(t_s, world)
            if st is not None:
                return st
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

    # ------------------------------------------------------- react
    # SL-0013: the authority (the runner's truth tick) appends a knot
    # row per fired trigger to var/run/<sim>/world_overrides.jsonl;
    # world.reacts holds the parsed rows (every evaluator reads the same
    # file, so pos_at stays deterministic GIVEN THE FILE - the entity
    # itself never looks at the fleet).

    def _react_knot(self, t_s, world):
        """The newest knot for this entity at/before t_s, or None -
        the trigger has not latched yet."""
        world._sync_reacts()
        knot = None
        for k in world.reacts.get(self.id) or ():
            if k["t_s"] <= t_s + 1e-9:
                knot = k
        return knot

    def _react_pos_at(self, t_s, world):
        """Position under the active react knot (closed form, no rng):
        freeze holds the recorded spot, flee walks its recorded heading
        away from the recorded threat, seek interpolates from -> threat
        and holds on arrival."""
        knot = self._react_knot(t_s, world)
        if knot is None:
            return None
        kind = knot["kind"]
        lat0, lng0 = knot["from"]
        if kind == "freeze":
            return (lat0, lng0, self.alt_m)
        speed = float(knot.get("speed_mps") or 0.0)
        dt = max(0.0, t_s - knot["t_s"])
        if kind == "seek":
            d = _dist(lat0, lng0, knot["threat"][0], knot["threat"][1])
            f = 1.0 if d <= 0.0 else min(1.0, speed * dt / d)
            return (lat0 + (knot["threat"][0] - lat0) * f,
                    lng0 + (knot["threat"][1] - lng0) * f, self.alt_m)
        lat, lng = _walk_to(lat0, lng0, speed * dt,
                            knot.get("heading_deg", 0.0))
        return (lat, lng, self.alt_m)

    def _react_state_at(self, t_s, world):
        """{moving, heading_deg, behaviour} under an active knot;
        behaviour reports "react:<kind>" so a truth row shows the
        reaction, not the scripted anchor model."""
        knot = self._react_knot(t_s, world)
        if knot is None:
            return None
        kind = knot["kind"]
        if kind == "freeze":
            return {"moving": False, "heading_deg": 0.0,
                    "behaviour": "react:freeze"}
        speed = float(knot.get("speed_mps") or 0.0)
        if kind == "seek":
            h = _bearing_deg(knot["from"][0], knot["from"][1],
                             knot["threat"][0], knot["threat"][1])
            d = _dist(knot["from"][0], knot["from"][1],
                      knot["threat"][0], knot["threat"][1])
            arrived = speed * max(0.0, t_s - knot["t_s"]) >= d
            return {"moving": not arrived, "heading_deg": round(h, 1),
                    "behaviour": "react:seek"}
        return {"moving": speed > 0.0,
                "heading_deg": round(float(knot.get("heading_deg", 0.0)),
                                     1),
                "behaviour": "react:flee"}

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
        # SL-0013 react overrides: the parsed world_overrides.jsonl rows
        # (tid -> [knot, ...]); _react_file is set by use_overrides -
        # absent file/None means no overrides ever apply, so a
        # non-reactive or pre-run world is untouched
        self.reacts = {}
        self._react_file = None
        self._react_off = 0
        self._react_size = -1
        self.obstacles = [Obstacle(o, path, i, self.origin)
                          for i, o in enumerate(spec.get("obstacles") or [])]
        self.areas = _parse_areas(spec.get("areas"), path)
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
        # sensors.precland feeds stub_precland's downward tag camera
        # (PL-08); img_w/img_h default to the rgb camera's frame size
        precland = dict(PRECLAND_DEFAULTS)
        precland["img_w"] = cam["img_w"]
        precland["img_h"] = cam["img_h"]
        precland.update(sensors.get("precland") or {})
        # sensors.payload is a pass-through for the render feed
        # (P5-13 hello.units[].payload: mass/drag for the P5-16
        # drop model - the engine itself does not read it yet)
        self.sensors = {"camera": cam, "sdr": sdr, "precland": precland,
                        "payload": dict(sensors.get("payload") or {})}
        # SL-0030: any other sensors.<name> block passes through as the
        # sensor's model params - a contract-declared sensor reads
        # world.sensors[its key] with no engine edit
        for k, v in sensors.items():
            if k not in self.sensors:
                self.sensors[k] = dict(v) if isinstance(v, dict) else v
        for e in self.entities:
            if e.cls != "pad":
                continue
            if e.size_m is None or e.tags_min_m is None \
                    or e.tags_max_m is None:
                raise simlib.SimError(
                    f"{path}: pad {e.id}: needs size_m, tags_min_m and "
                    "tags_max_m")
            if not 0.0 < e.tags_min_m <= e.tags_max_m:
                raise simlib.SimError(
                    f"{path}: pad {e.id}: needs "
                    "0 < tags_min_m <= tags_max_m")
        for e in self.entities:
            if e.cls == "aircraft" and e.icao is None:
                raise simlib.SimError(
                    f"{path}: aircraft {e.id}: needs icao (24-bit "
                    "address, int or hex string)")

    def entity(self, tid):
        return self._by_id.get(tid)

    def pads(self):
        """The cls: pad entities - precision-landing targets (PL-08)."""
        return [e for e in self.entities if e.cls == "pad"]

    def aircraft(self):
        """The cls: aircraft entities - ADS-B intruders (SL-0016)."""
        return [e for e in self.entities if e.cls == "aircraft"]

    def pad_tag_sizes(self, pad):
        """The pad's tag-edge ladder, largest first: tags_max_m,
        tags_max_m/PAD_TAG_RATIO, ... down to >= tags_min_m."""
        sizes = []
        s = pad.tags_max_m
        while s >= pad.tags_min_m - 1e-9:
            sizes.append(s)
            s /= PAD_TAG_RATIO
        return sizes

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

    def clutter_at(self, t_s, tick_hz=10.0):
        """P5-13: the world's own ground-anchored clutter field, as a
        list of live points {id, lat, lng, cls} at t_s. The stubs'
        per-camera clutter spawns inside a frustum; the world stream
        needs world points instead, so the field spawns determinis-
        tically per tick index (rng(seed,"wsclutter",k)) over the
        entity/obstacle bbox + 50 m at the configured rate - a same-seed
        replay produces the same field. Only for anchor: ground."""
        import geo_projection
        cl = self.clutter
        if cl["anchor"] != "ground" or cl["rate_per_km2_min"] <= 0.0:
            return []
        st = getattr(self, "_ws_clutter", None)
        if st is None:
            st = self._ws_clutter = {"pts": [], "tick": -1, "n": 0}
        tick = int(t_s * tick_hz + 1e-9)
        while st["tick"] < tick:
            st["tick"] += 1
            xs, ys = [], []
            for e in self.entities:
                n, east = geo_projection.geo_latlng_to_ne(
                    self.origin[0], self.origin[1],
                    *e.pos_at(t_s, self)[:2])
                xs.append(east)
                ys.append(n)
            for o in self.obstacles:
                for px, py in o.xy:
                    xs.append(px)
                    ys.append(py)
            if not xs:
                break
            lo_x, hi_x = min(xs) - 50.0, max(xs) + 50.0
            lo_y, hi_y = min(ys) - 50.0, max(ys) + 50.0
            area_km2 = (hi_x - lo_x) * (hi_y - lo_y) / 1e6
            p = cl["rate_per_km2_min"] * area_km2 / (60.0 * tick_hz)
            r = rng(self.seed, "wsclutter", st["tick"])
            if r.random() < p:
                st["n"] += 1
                east = r.uniform(lo_x, hi_x)
                north = r.uniform(lo_y, hi_y)
                lat, lng = geo_projection.geo_offset_to_latlng(
                    self.origin[0], self.origin[1], north, east)
                st["pts"].append({"id": f"c{st['n']}", "lat": lat,
                                  "lng": lng, "cls": cl["cls"],
                                  "exp": t_s + cl["lifetime_s"]})
        st["pts"] = [c for c in st["pts"] if c["exp"] > t_s]
        return [{"id": c["id"], "lat": c["lat"], "lng": c["lng"],
                 "cls": c["cls"]} for c in st["pts"]]

    def point_in_building(self, lat, lng, alt_m):
        """True when (lat, lng, alt_above_home) lies inside any
        building prism - the wander/seek step rejection test (D3).
        Canopy is walkable and answers False."""
        if not self.obstacles:
            return False
        enu = self._enu((lat, lng, alt_m))
        return any(o.kind == "building" and o.contains(enu)
                   for o in self.obstacles)

    # --------------------------------------------------- react
    # SL-0013: var/run/<sim>/world_overrides.jsonl is the reaction
    # record the single authority (the truth tick) appends. Every
    # pos_at caller - the recorder, each stub, the world stream - reads
    # the same append-only file, so truth and detections agree.

    def use_overrides(self, path):
        """Point pos_at at the run's world_overrides.jsonl (set by the
        recorder and by every stub from its run dir)."""
        self._react_file = path
        self._react_off = 0
        self._react_size = -1
        self.reacts = {}
        self._sync_reacts()

    def _sync_reacts(self):
        """Pull newly appended knots off the overrides file into
        self.reacts. Only complete lines are consumed (a torn tail is
        re-read next call); a truncated file (a fresh run on a reused
        sim) rewinds. Pure given the file contents."""
        if not self._react_file:
            return
        try:
            size = os.path.getsize(self._react_file)
        except OSError:
            return
        if size == self._react_size:
            return
        if size < self._react_size:      # truncated - a new run began
            self._react_off = 0
            self.reacts = {}
        try:
            with open(self._react_file, "rb") as f:
                f.seek(self._react_off)
                data = f.read()
        except OSError:
            return
        end = data.rfind(b"\n") + 1
        self._react_off += end
        self._react_size = size
        for ln in data[:end].decode(errors="replace").splitlines():
            try:
                k = json.loads(ln)
            except ValueError:
                continue
            if isinstance(k, dict) and k.get("tid") is not None \
                    and k.get("t_s") is not None:
                self.reacts.setdefault(str(k["tid"]), []).append(k)

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

    # --------------------------------------------------------- thermal

    def ambient_c(self):
        """The environment's background temperature for the thermal
        model - the explicit ambient_c, else the sun-elevation
        ESTIMATE (see _ambient_c)."""
        return self.environment["ambient_c"]

    def temp_c_at(self, e, t_s):
        """The entity's temperature at t_s: its own temp_c when set,
        else the cls ESTIMATE - a person is a 34 C body, a vehicle/car
        is 40 C while it runs and cold-soaked to ambient once it
        holds, every other cls sits at ambient (invisible to the
        contrast model)."""
        if e.temp_c is not None:
            return e.temp_c
        if e.cls == "person":
            return TEMP_C_PERSON
        if e.cls in VEHICLE_CLS and e.state_at(t_s, self)["moving"]:
            return TEMP_C_VEHICLE_RUNNING
        return self.environment["ambient_c"]

    def thermal_factor(self, e, t_s):
        """TASK-13 contrast ESTIMATE for a thermal camera: 1 at/above
        THERMAL_FULL_CONTRAST_C of |temp - ambient|, ramping linearly
        to 0 as the target reaches ambient (the dawn/dusk crossover).
        An entity id that resolves to nothing keeps the old flat p
        (1.0)."""
        if e is None:
            return 1.0
        return min(1.0, abs(self.temp_c_at(e, t_s)
                            - self.environment["ambient_c"])
                   / THERMAL_FULL_CONTRAST_C)


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


def _ambient_c(sun_elev_deg):
    """ESTIMATE: the ambient (background) temperature tracks the sun
    elevation linearly - AMBIENT_BASE_C at the horizon plus
    AMBIENT_GAIN_C_PER_DEG per degree (39 C at sun_elev 60, -9 at
    -60). environment.ambient_c overrides it when the value matters."""
    return AMBIENT_BASE_C + AMBIENT_GAIN_C_PER_DEG * sun_elev_deg


# ------------------------------------------------- behaviour parsers

def _parse_icao(spec, path, eid):
    """A 24-bit ICAO address: an int, a "0x..." hex string or bare hex
    digits ("A1B2C3"). None when absent; SimError on anything else."""
    if spec is None:
        return None
    label = f"{path}: entity {eid}: icao"
    try:
        if isinstance(spec, int) and not isinstance(spec, bool):
            v = spec
        elif isinstance(spec, str):
            s = spec.strip()
            v = int(s, 0) if s.lower().startswith(("0x", "0o", "0b")) \
                or s.isdigit() else int(s, 16)
        else:
            raise ValueError
    except ValueError:
        raise simlib.SimError(
            f"{label}: not a 24-bit address: {spec!r}")
    if not 0 <= v <= 0xFFFFFF:
        raise simlib.SimError(
            f"{label}: out of range: {spec!r} (needs 0..0xffffff)")
    return v


def _parse_str_list(spec, path, eid):
    """Optional list of strings (the aircraft `units:` filter)."""
    if spec is None:
        return []
    if not isinstance(spec, list) or \
            not all(isinstance(x, str) for x in spec):
        raise simlib.SimError(
            f"{path}: entity {eid}: units must be a list of unit names")
    return list(spec)


def _parse_occlude(spec, path, eid):
    """occlude: [[t0_s, t1_s], ...] -> a list of closed world-second
    intervals the entity is not visible (PL-08 pads)."""
    out = []
    for w in (spec or []):
        label = f"{path}: entity {eid}: occlude"
        if not isinstance(w, (list, tuple)) or len(w) != 2 or \
                not all(isinstance(x, (int, float))
                        and not isinstance(x, bool) for x in w):
            raise simlib.SimError(
                f"{label}: windows are [t0_s, t1_s] pairs")
        if w[1] < w[0]:
            raise simlib.SimError(f"{label}: window end before start")
        out.append([float(w[0]), float(w[1])])
    return out


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


def _parse_react(spec, path, eid):
    """react: {near: {unit|unit_cls, dist_m, then: {kind, speed_mps}}}
    -> a normalized trigger dict or None (SL-0013). The entity only
    DECLARES the reaction; the truth tick evaluates it."""
    if spec is None:
        return None
    label = f"{path}: entity {eid}: react"
    if not isinstance(spec, dict) or set(spec) != {"near"}:
        raise simlib.SimError(f"{label}: the only trigger is 'near'")
    near = spec["near"]
    if not isinstance(near, dict):
        raise simlib.SimError(f"{label}.near: not a mapping")
    bad = set(near) - REACT_NEAR_KEYS
    if bad:
        raise simlib.SimError(
            f"{label}.near: unknown keys {sorted(bad)} "
            f"(allows {sorted(REACT_NEAR_KEYS)})")
    if ("unit" in near) == ("unit_cls" in near):
        raise simlib.SimError(
            f"{label}.near: exactly one of unit|unit_cls")
    if near.get("unit") is not None \
            and not isinstance(near["unit"], str):
        raise simlib.SimError(f"{label}.near: unit is a fleet unit name")
    if near.get("unit_cls") is not None \
            and not isinstance(near["unit_cls"], str):
        raise simlib.SimError(
            f"{label}.near: unit_cls is a fleet cls name")
    try:
        dist_m = float(near["dist_m"])
    except (KeyError, TypeError, ValueError):
        raise simlib.SimError(f"{label}.near: dist_m must be > 0")
    if dist_m <= 0.0:
        raise simlib.SimError(f"{label}.near: dist_m must be > 0")
    then = near.get("then")
    if not isinstance(then, dict):
        raise simlib.SimError(
            f"{label}.near.then: a behaviour override mapping")
    bad = set(then) - REACT_THEN_KEYS
    if bad:
        raise simlib.SimError(
            f"{label}.near.then: unknown keys {sorted(bad)} "
            f"(allows {sorted(REACT_THEN_KEYS)})")
    kind = then.get("kind")
    if kind not in REACT_THEN_KINDS:
        raise simlib.SimError(
            f"{label}.near.then: kind must be "
            f"{'|'.join(REACT_THEN_KINDS)}, got {kind!r}")
    speed = then.get("speed_mps")
    if kind == "freeze":
        if speed is not None:
            raise simlib.SimError(
                f"{label}.near.then: freeze takes no speed_mps")
        speed = 0.0
    else:
        speed = float(speed if speed is not None
                      else REACT_DEFAULT_SPEED_MPS[kind])
        if speed <= 0.0:
            raise simlib.SimError(
                f"{label}.near.then: speed_mps must be > 0")
    return {"unit": (str(near["unit"]) if near.get("unit") is not None
                     else None),
            "unit_cls": (str(near["unit_cls"])
                         if near.get("unit_cls") is not None else None),
            "dist_m": dist_m,
            "then": {"kind": kind, "speed_mps": speed}}


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


def _parse_areas(spec, path):
    """areas: {name: {poly: [[lat,lng],...]}} -> {name: {"poly"}} - the
    named search polygons coverage_complete/pod_mean grid over
    (P5-12 D2)."""
    out = {}
    for name, a in (spec or {}).items():
        label = f"{path}: areas.{name}"
        if not isinstance(a, dict):
            raise simlib.SimError(f"{label}: not a mapping")
        bad = set(a) - {"poly"}
        if bad:
            raise simlib.SimError(f"{label}: unknown keys {sorted(bad)}")
        poly = a.get("poly")
        if not isinstance(poly, list) or len(poly) < 3 or \
                not all(isinstance(p, (list, tuple)) and len(p) == 2 and
                        all(isinstance(c, (int, float))
                            and not isinstance(c, bool) for c in p)
                        for p in poly):
            raise simlib.SimError(
                f"{label}: poly needs >= 3 [lat, lng] vertices")
        out[str(name)] = {"poly": [list(p) for p in poly]}
    return out


def _parse_environment(spec, path):
    """environment: -> {"sun_elev_deg", "visibility_m", "precip",
    "wind"} with the documented defaults. Static per run (D4)."""
    env = {"sun_elev_deg": SUN_DEFAULT_DEG,
           "visibility_m": float("inf"),
           "precip": "none", "wind": None,
           "ambient_c": _ambient_c(SUN_DEFAULT_DEG)}
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
    if "ambient_c" in spec:
        env["ambient_c"] = float(spec["ambient_c"])
        if not -80.0 <= env["ambient_c"] <= 80.0:
            raise simlib.SimError(f"{label}: ambient_c out of range")
    else:
        env["ambient_c"] = _ambient_c(env["sun_elev_deg"])
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
                       "environment", "areas"}
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

# a catalog module is a world-driven stub when its spec carries
# `stub: true` (SL-0030 - a block's catalog.yaml registers its sensor
# module the same way, no engine edit)


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
    catalog = simlib.load_catalog()
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
                # tcp:5 -> the leased SITL block's base+5 on FC units
                # running a pose-flagged catalog module; statics use the
                # 9102 feed. The stamped value is a placeholder - the
                # key's presence is what _expand_unit/_apply_ports
                # restamp with the leased port at commit (0 = no feed)
                "pose_port": 0}
        for m in mods:
            if catalog.get(m, {}).get("stub"):
                cfg[m] = simlib.deep_merge(cfg.get(m) or {}, {"stub": stub})
        if "stub_ai_box" in mods or "de_yolo_ai" in mods:
            # P5-15: de_yolo_ai emits real obs frames through the same
            # de_mavlink/de_comm projector - same injection, real pixels
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


def overrides_path(sim):
    """SL-0013: the append-only reaction-knot record the truth tick
    writes and every evaluator reads."""
    return os.path.join(simlib.run_dir(sim), "world_overrides.jsonl")


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
    epoch (stubs pause), truncate the truth file, the overrides file
    (SL-0013) and the stub logs so a reused sim only records this
    run."""
    for p in (epoch_path(sim), truth_path(sim), overrides_path(sim)):
        if os.path.isfile(p):
            os.unlink(p)
    sdir = stubs_dir(sim)
    if os.path.isdir(sdir):
        for f in os.listdir(sdir):
            if f.endswith(".log"):
                os.unlink(os.path.join(sdir, f))


# ------------------------------------------------- react authority

def react_knots(world, poses, t_s, now_ms, fired):
    """SL-0013 single-authority trigger evaluation - called ONLY by
    the runner's truth tick (a per-stub evaluation would race the file
    and diverge truth from detections).

    poses: {unit_name: {"lat", "lng", "cls"}} - FC units from the
    telemetry sampler, statics from their manifest position; `cls` is
    the scenario fleet cls for near.unit_cls matching. fired: the set
    of entity ids already latched this run (a react fires once - the
    knot latches, the entity keeps the new motion).

    -> [knot dicts] to append to world_overrides.jsonl; each carries
    everything pos_at needs ({t_s, tid, kind, from, threat, unit,
    dist_m, speed_mps, heading_deg?}) so the file alone decides the
    override - deterministic given the file."""
    out = []
    for e in world.entities:
        r = e.react
        if r is None or e.id in fired:
            continue
        lat, lng, _a = e.pos_at(t_s, world)
        best = None
        for uname, p in poses.items():
            if r["unit"] is not None and uname != r["unit"]:
                continue
            if r["unit_cls"] is not None and p.get("cls") != r["unit_cls"]:
                continue
            if p.get("lat") is None or p.get("lng") is None:
                continue
            # horizontal ground distance - "approached", not altitude
            d = _dist(lat, lng, p["lat"], p["lng"])
            if best is None or d < best[0]:
                best = (d, uname, p)
        if best is None or best[0] > r["dist_m"]:
            continue
        d, uname, p = best
        then = r["then"]
        knot = {"v": 1, "t_s": round(t_s, 3), "t_ms": now_ms,
                "tid": e.id, "kind": then["kind"],
                "from": [lat, lng],
                "threat": [p["lat"], p["lng"]],
                "unit": uname, "dist_m": round(d, 2),
                "speed_mps": then["speed_mps"]}
        if then["kind"] == "flee":
            # directly away from the recorded threat; a degenerate
            # zero-distance bearing takes a seeded draw so every
            # reader still computes the same track
            h = _bearing_deg(p["lat"], p["lng"], lat, lng) \
                if d >= 1e-3 else \
                rng(world.seed, "react", e.id).uniform(0.0, 360.0)
            knot["heading_deg"] = round(h, 2)
        out.append(knot)
    return out


def world_event_row(knot, now_ms):
    """The truth.jsonl row a fired knot logs: the on:{world:..}
    watcher matches its `re` against `msg`."""
    return {"type": "world_event", "event": "react",
            "tid": knot["tid"], "kind": knot["kind"],
            "unit": knot.get("unit"), "dist_m": knot["dist_m"],
            "t_s": knot["t_s"], "t_ms": now_ms,
            "msg": (f"react {knot['tid']} {knot['kind']} near "
                    f"{knot.get('unit')} dist_m {knot['dist_m']}")}


# ------------------------------------------------------------ truth log

class TruthRecorder:
    """1 Hz entity rows -> truth.jsonl:
        {"tid", "cls", "t_ms", "lat", "lng", "alt_m"}
    The stubs append {"type":"detection", ...} rows for every message
    they emit; distance_to/coverage_complete/detection_precision read
    both. start()/stop(); t_ms is unix ms (the fleet clock)."""

    def __init__(self, world, epoch_ms, path, hz=1.0,
                 overrides_path=None, fleet_poses=None):
        self.world = world
        self.epoch_ms = epoch_ms
        self.path = path
        self.hz = hz
        self._stop = threading.Event()
        self._thread = None
        # SL-0013 react authority: when the runner hands us the
        # overrides path + a fleet-pose callable, this tick alone fires
        # react: triggers (knots -> the file, world_event rows ->
        # truth.jsonl) and the world applies them
        self._ov_path = overrides_path
        self._poses = fleet_poses
        self._fired = set()
        self._reactive = []
        if overrides_path:
            world.use_overrides(overrides_path)
            self._reactive = [e for e in world.entities
                              if e.react is not None]

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
            if self._reactive:
                self._react_tick(t_s, now)
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

    def _react_tick(self, t_s, now):
        """The ONE authority for react: triggers (SL-0013): evaluate
        the fleet poses once per truth row, append each latched knot to
        the overrides file and its world_event row to truth.jsonl. The
        rows written this same tick already see the knot (pos_at reads
        the file - from == the current position, so motion is
        continuous across the trigger)."""
        poses = {}
        try:
            poses = self._poses() if self._poses else {}
            knots = react_knots(self.world, poses, t_s, now,
                                self._fired)
        except Exception:
            return                      # a pose hiccup must not kill truth
        for k in knots:
            try:
                with open(self._ov_path, "a") as f:
                    f.write(json.dumps(k) + "\n")
            except OSError:
                continue                # no knot on disk -> no event
            self._fired.add(k["tid"])
            try:
                with open(self.path, "a") as f:
                    f.write(json.dumps(world_event_row(k, now)) + "\n")
            except OSError:
                pass


# ------------------------------------------------------------ ADS-B feed

# ADSB_FLAGS bits set on every emitted frame: valid coords, altitude,
# heading, velocity, callsign and vertical velocity, plus SIMULATED -
# the ADS-B equivalent of the truth rows (SITL's own SIM_ADSB traffic
# does not set it, a world aircraft always does)
ADSB_FLAG_BASE = 0x01 | 0x02 | 0x04 | 0x08 | 0x10 | 0x40 | 0x80
ADSB_FLAG_SQUAWK = 0x20
ADSB_ALTITUDE_GEOMETRIC = 1          # altitude is AMSL, not QNH


def adsb_targets(entities, units):
    """{entity: [unit,...]} - the fcb-proxied FC units each aircraft
    entity feeds: all of them, or the subset named by its `units:`
    filter (names that resolve to no unit are an authoring error)."""
    out = {}
    for e in entities:
        if not e.units:
            out[e] = list(units)
            continue
        by_name = {u["name"]: u for u in units}
        missing = [n for n in e.units if n not in by_name]
        if missing:
            raise simlib.SimError(
                f"aircraft {e.id}: units {missing} are not fcb-proxied "
                f"FC units (have {[u['name'] for u in units]})")
        out[e] = [by_name[n] for n in e.units]
    return out


class AdsbFeeder:
    """1 Hz ADSB_VEHICLE frames for the world's cls:aircraft entities,
    written into each target unit's fcb proxy via the `inject <hex>`
    control command (SL-0016 on the SL-0008 hook) - de_mavlink parses
    and forwards them to the GCS the same way it forwards the frames
    SITL's SIM_ADSB_* traffic emits. One persistent control socket per
    unit, re-dialed when it breaks; a unit whose proxy is briefly gone
    (reset) just misses ticks. start()/stop(); frames are MAVLink v1
    stamped with the unit's own sysid/compid - what the FC itself emits.
    """

    def __init__(self, world, epoch_ms, units, home_alt_m, hz=1.0):
        self.world = world
        self.epoch_ms = epoch_ms
        self.home_alt_m = float(home_alt_m)
        self.hz = hz
        self.targets = adsb_targets(world.aircraft(), units)
        # per-unit encode state: one BytesIO the pymavlink writer packs
        # frames into (a shared io keeps each unit's own seq counter)
        self._links = {u["name"]: {"sock": None, "io": io.BytesIO(),
                                   "mav": None} for u in units}
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(5)
        for link in self._links.values():
            if link["sock"] is not None:
                try:
                    link["sock"].close()
                except OSError:
                    pass

    # ------------------------------------------------------------ frames

    def _frame(self, u, e, t_s):
        """One complete ADSB_VEHICLE frame for entity e at t_s, stamped
        with the unit's sysid/compid (the same identity SITL's own
        traffic carries)."""
        from pymavlink import mavutil
        link = self._links[u["name"]]
        if link["mav"] is None:
            link["mav"] = mavutil.mavlink.MAVLink(
                link["io"], srcSystem=u["fc"]["sysid"], srcComponent=1)
        lat, lng, alt = e.pos_at(t_s, self.world)
        # velocity from the same 0.5 s delta state_at() uses for heading
        p1 = e.pos_at(t_s + 0.5, self.world)
        heading = _bearing_deg(lat, lng, p1[0], p1[1])
        hvel_ms = _dist(lat, lng, p1[0], p1[1]) * 2.0
        vvel_ms = (p1[2] - alt) * 2.0
        flags = ADSB_FLAG_BASE | (ADSB_FLAG_SQUAWK if e.squawk else 0)
        bio = link["io"]
        bio.seek(0)
        bio.truncate(0)
        link["mav"].adsb_vehicle_send(
            e.icao, int(round(lat * 1e7)), int(round(lng * 1e7)),
            ADSB_ALTITUDE_GEOMETRIC,
            int(round((self.home_alt_m + alt) * 1000)),
            int(round(heading * 100)) % 36000,
            int(round(hvel_ms * 100)), int(round(vvel_ms * 100)),
            e.callsign.encode("ascii", errors="replace")[:8],
            e.emitter_type, 1, flags, e.squawk or 0)
        return bio.getvalue()

    # ------------------------------------------------------------ socket

    def _inject(self, u, frame):
        """`inject <hex>` on the unit's fcb proxy control port over a
        persistent socket (re-dialed when it broke) -> True when the
        proxy acknowledged the write."""
        link = self._links[u["name"]]
        for _retry in range(2):
            if link["sock"] is None:
                try:
                    link["sock"] = socket.create_connection(
                        ("127.0.0.1", int(u["fc"]["fcb_proxy"]["ctrl"])),
                        2)
                    link["sock"].settimeout(2.0)
                except OSError:
                    return False
            try:
                link["sock"].sendall(
                    b"inject " + frame.hex().encode() + b"\n")
                rep = link["sock"].recv(4096)
            except OSError:
                try:
                    link["sock"].close()
                except OSError:
                    pass
                link["sock"] = None
                continue
            if b"ERR" in rep:
                print(f"  adsb feed {u['name']}: {rep.decode(errors='replace').strip()}",
                      flush=True)
                return False
            return True
        return False

    # -------------------------------------------------------------- loop

    def _run(self):
        next_t = time.time()
        while not self._stop.is_set():
            t_s = (int(time.time() * 1000) - self.epoch_ms) / 1000.0
            for e, units in self.targets.items():
                if t_s < e.start_s:
                    continue
                for u in units:
                    try:
                        self._inject(u, self._frame(u, e, t_s))
                    except Exception as exc:
                        print(f"  adsb feed {u['name']}/{e.id}: "
                              f"{type(exc).__name__}: {exc}", flush=True)
            next_t += 1.0 / self.hz
            self._stop.wait(max(0.05, next_t - time.time()))
