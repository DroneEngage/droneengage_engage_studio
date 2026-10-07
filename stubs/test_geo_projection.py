#!/usr/bin/env python3
"""test_geo_projection.py - P5-08 projection round-trip check.

Compiles stubs/geo_projection_test.cpp against the vendored
de_common/helpers/geo_projection.hpp (the same header de_mavlink and
de_comm's static projector are built with) and verifies:

  1. truth -> box via the Python inverse (engine/geo_projection.py)
     -> C++ forward projection lands within 1 m at 40 m nadir;
  2. a grid of altitudes / attitudes / mount pitches stays within a
     small budget;
  3. a forward projection from the C++ side matches the Python forward
     port element for element;
  4. below-horizon targets are rejected by both sides.

Run:  python3 stubs/test_geo_projection.py        (exit 0 = all pass)
"""

import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)
# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.join(SIM_ROOT, "engine"))

import simlib                      # noqa: E402
import geo_projection as gp        # noqa: E402

# the vendored header copies - first one found wins (they are kept in
# sync across the de_* repos)
HEADER_DIRS = [
    ("DE_DATABUS_DIR", "de_databus",
     "client/src/de_common/helpers"),
    ("DE_MAVLINK_DIR", "droneengage_mavlink", "src/de_common/helpers"),
    ("DE_COMM_DIR", "droneengage_communication", "src/de_common/helpers"),
]

BUILD_DIR = os.path.join(simlib.RUN_DIR, "stub_tests")
BIN = os.path.join(BUILD_DIR, "geo_projection_test")


def build():
    """Compile the C++ forward projector against the shared header."""
    for env_key, repo, sub in HEADER_DIRS:
        d = os.path.join(simlib.repo_dir(env_key, repo), sub)
        if os.path.isfile(os.path.join(d, "geo_projection.hpp")):
            os.makedirs(BUILD_DIR, exist_ok=True)
            subprocess.check_call(
                ["g++", "-std=c++17", "-O2", "-I", d,
                 os.path.join(HERE, "geo_projection_test.cpp"), "-o", BIN])
            return BIN
    sys.exit("geo_projection.hpp not found in any de_* repo "
             "(checked databus, mavlink, communication)")


def forward_cpp(u, v, bw, bh, cam, mount, att, pose,
                sigma_att_deg=0.0, sigma_alt_m=0.0, rangefinder_m=-1.0):
    req = {
        "u": u, "v": v, "bw": bw, "bh": bh,
        "hfov_deg": cam.hfov_deg, "vfov_deg": cam.vfov_deg,
        "k1": cam.k1, "k2": cam.k2,
        "mount_pitch_deg": mount.pitch_deg,
        "mount_yaw_deg": mount.yaw_deg,
        "mount_roll_deg": mount.roll_deg,
        "roll_rad": att.roll_rad, "pitch_rad": att.pitch_rad,
        "yaw_rad": att.yaw_rad,
        "lat_deg": pose.lat_deg, "lng_deg": pose.lng_deg,
        "rel_alt_m": pose.rel_alt_m,
        "sigma_att_deg": sigma_att_deg, "sigma_alt_m": sigma_alt_m,
        "rangefinder_m": rangefinder_m,
    }
    r = subprocess.run([BIN, json.dumps(req)],
                       capture_output=True, text=True)
    return json.loads(r.stdout)


def main():
    build()
    cam = gp.GeoCameraParams()
    nadir = gp.GeoMountParams()
    home = gp.GeoPose(-35.363262, 149.165237, 40.0)
    fails = []

    # ---- 1. the acceptance case: 40 m nadir, truth -> box -> pos < 1 m
    cases = []
    for dn, de in ((0, 0), (5, 0), (0, 8), (-12, 6), (18, -14), (-20, -20)):
        t_lat, t_lng = gp.geo_offset_to_latlng(home.lat_deg, home.lng_deg,
                                             dn, de)
        cases.append((f"nadir40 dN={dn} dE={de}", t_lat, t_lng, 0.0,
                      nadir, gp.GeoAttitude(), home))

    # ---- 2. attitudes / altitudes / off-nadir mounts
    for label, att, alt, mount in (
            ("roll10", gp.GeoAttitude(math.radians(10), 0, 0), 40.0, nadir),
            ("pitch-5", gp.GeoAttitude(0, math.radians(-5), 0), 40.0, nadir),
            ("yaw30", gp.GeoAttitude(0, 0, math.radians(30)), 40.0, nadir),
            ("alt60", gp.GeoAttitude(), 60.0, nadir),
            ("mount-60", gp.GeoAttitude(), 50.0,
             gp.GeoMountParams(pitch_deg=-60.0)),
            ("mount-45_yaw20", gp.GeoAttitude(0, 0, math.radians(10)), 45.0,
             gp.GeoMountParams(pitch_deg=-45.0, yaw_deg=20.0))):
        pose = gp.GeoPose(home.lat_deg, home.lng_deg, alt)
        for dn, de in ((0, 0), (8, -6), (-15, 10)):
            t_lat, t_lng = gp.geo_offset_to_latlng(home.lat_deg, home.lng_deg,
                                                 dn, de)
            cases.append((f"{label} dN={dn} dE={de}", t_lat, t_lng, 0.0,
                          mount, att, pose))

    for label, t_lat, t_lng, t_alt, mount, att, pose in cases:
        px = gp.geo_ground_to_pixel(t_lat, t_lng, t_alt, cam, mount, att, pose)
        if px is None:
            # genuinely invisible (behind the camera / too shallow): the
            # C++ forward could never produce it either - only a bug when
            # the mandatory nadir cases go invisible
            if label.startswith("nadir40"):
                fails.append(f"{label}: inverse returned None")
            else:
                print(f"  [skip] {label}: not in view")
            continue
        u, v, _slant = px
        r = forward_cpp(u, v, 0.02, 0.03, cam, mount, att, pose)
        if not r["ok"]:
            fails.append(f"{label}: C++ forward !ok")
            continue
        err = gp.geo_distance_m(t_lat, t_lng, r["lat"], r["lng"])
        budget = 1.0 if label.startswith("nadir40") else 2.0
        status = "ok" if err <= budget else "FAIL"
        print(f"  [{status}] {label}: u={u:+.4f} v={v:+.4f} err={err:.4f} m")
        if err > budget:
            fails.append(f"{label}: {err:.3f} m > {budget} m")

    # ---- 3. the Python forward port matches the C++ element for
    # element - including the P5-18 rectilinear s_box and a distorted
    # (k1/k2) camera
    att = gp.GeoAttitude(math.radians(3), math.radians(-7),
                         math.radians(40))
    cam_k = gp.GeoCameraParams(k1=0.05, k2=0.02)
    for u, v, bw, bh, ccam in (
            (0.0, 0.0, 0.1, 0.1, cam), (0.2, -0.3, 0.05, 0.2, cam),
            (-0.4, 0.1, 0.02, 0.02, cam),
            (0.3, 0.25, 0.1, 0.1, cam_k), (-0.2, 0.4, 0.1, 0.1, cam_k)):
        c = forward_cpp(u, v, bw, bh, ccam, nadir, att, home,
                        sigma_att_deg=2.0, sigma_alt_m=2.0)
        p = gp.geo_project_to_ground(u, v, bw, bh, ccam, nadir, att, home,
                                     2.0, 2.0)
        if c["ok"] != p.ok or (p.ok and (
                abs(c["lat"] - p.lat) > 1e-9 or
                abs(c["lng"] - p.lng) > 1e-9 or
                abs(c["ground_range_m"] - p.ground_range_m) > 1e-4 or
                abs(c["unc_m"] - p.unc_m) > 1e-4)):
            fails.append(f"forward mismatch u={u} v={v}: "
                         f"c={c} py={{ok:{p.ok},lat:{p.lat},lng:{p.lng}}}")
    print("  [ok] forward port matches C++ for the tested cases")

    # ---- 4. below-horizon / impossible-ray rejection on both sides:
    # mount -10 deg (camera axis 10 deg below the horizon). v=-0.4 tilts
    # the ray ~19.5 deg up from the axis -> above the horizon; the C++
    # forward and the Python inverse must both refuse it.
    tilted = gp.GeoMountParams(pitch_deg=-10.0)
    c2 = forward_cpp(0.0, -0.4, 0.1, 0.1, cam, tilted,
                     gp.GeoAttitude(), home)
    if c2["ok"]:
        fails.append("C++ accepted a ray above the horizon")
    flat = gp.GeoPose(home.lat_deg, home.lng_deg, 0.0)
    px2 = gp.geo_ground_to_pixel(home.lat_deg, home.lng_deg, 0.0,
                                 cam, tilted, gp.GeoAttitude(), flat)
    if px2 is not None:
        fails.append("inverse accepted a target at the camera plane")
    px3 = gp.geo_ground_to_pixel(home.lat_deg, home.lng_deg,
                                 home.rel_alt_m + 1.0, cam, nadir,
                                 gp.GeoAttitude(), home)
    if px3 is not None:
        fails.append("inverse accepted a target above the vehicle")
    print("  [ok] horizon/above-vehicle rejection")

    if fails:
        print("\nFAIL:")
        for f in fails:
            print("  -", f)
        return 1
    print("\ngeo_projection round-trip: all cases passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
