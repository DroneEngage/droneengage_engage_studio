#!/usr/bin/env python3
"""geo_projection.py - Python port of the shared pinhole-camera ground
projection (de_common/helpers/geo_projection.hpp, P3-07) plus the inverse
the P5-08 stub AI box needs (ground truth -> normalized image box).

Frames: camera (x right, y down, z forward) -> body FRD (x fwd, y right,
z down) -> NED. Mount pitch -90 deg = camera looking straight down; with
that mount image u+ maps to body +y (right) and image v+ maps to body -x
(behind). Normalized image coordinates are centered -0.5..0.5; the 1078
box convention (0..1 top-left) converts with geoBoxToCentered.

The forward path is a line-for-line port so the C++ round-trip test
(stubs/test_geo_projection.py) can compare bit-for-bit projections.
"""

import math
from dataclasses import dataclass, field

M_PER_DEG_LAT = 111320.0
MIN_DOWN = 0.0871557427          # sin(5 deg): the ray must point below this
NEAR_VERTICAL = 0.9659258263     # cos(15 deg): rangefinder region


def geo_deg2rad(deg):
    return deg * math.pi / 180.0


@dataclass
class GeoCameraParams:
    hfov_deg: float = 62.2
    vfov_deg: float = 48.8


@dataclass
class GeoMountParams:
    pitch_deg: float = -90.0
    yaw_deg: float = 0.0
    roll_deg: float = 0.0


@dataclass
class GeoAttitude:
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0


@dataclass
class GeoPose:
    lat_deg: float = 0.0
    lng_deg: float = 0.0
    rel_alt_m: float = 0.0       # height above the home-altitude ground plane


@dataclass
class GeoProjectionResult:
    ok: bool = False
    lat: float = 0.0
    lng: float = 0.0
    ground_range_m: float = 0.0
    unc_m: float = 0.0


def geo_box_to_centered(normalized01):
    """0..1 normalized box coordinate -> the -0.5..0.5 centered convention."""
    return normalized01 - 0.5


def geo_ray_camera(u, v, cam):
    """Pinhole ray in camera frame (x right, y down, z forward)."""
    return (math.tan(u * geo_deg2rad(cam.hfov_deg)),
            math.tan(v * geo_deg2rad(cam.vfov_deg)),
            1.0)


def _rot_elems(r, p, y):
    """ZYX euler rotation matrix elements: Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp,     cp * sr,             cp * cr))


def geo_rot(r, p, y, v):
    """R * v - same element order as the C++ geoRot."""
    m = _rot_elems(r, p, y)
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def geo_rot_t(r, p, y, v):
    """R^T * v - the inverse rotation used by the pixel projection."""
    m = _rot_elems(r, p, y)
    return (m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2],
            m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2],
            m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2])


def geo_ray_body(ray_cam, mount):
    """camera frame -> body FRD through the mount euler."""
    # base alignment: body_x = cam_z, body_y = cam_x, body_z = cam_y
    base = (ray_cam[2], ray_cam[0], ray_cam[1])
    return geo_rot(geo_deg2rad(mount.roll_deg), geo_deg2rad(mount.pitch_deg),
                   geo_deg2rad(mount.yaw_deg), base)


def geo_ray_ned(ray_body, att):
    """body FRD -> NED through the vehicle attitude."""
    return geo_rot(att.roll_rad, att.pitch_rad, att.yaw_rad, ray_body)


def geo_offset_to_latlng(lat_deg, lng_deg0, north_m, east_m):
    clat = math.cos(geo_deg2rad(lat_deg))
    if abs(clat) < 1e-6:
        clat = 1e-6
    m_per_deg_lng = M_PER_DEG_LAT * clat
    return (lat_deg + north_m / M_PER_DEG_LAT,
            lng_deg0 + east_m / m_per_deg_lng)


def geo_latlng_to_ne(lat0, lng0, lat1, lng1):
    """(north_m, east_m) of lat1,lng1 relative to lat0,lng0 - the inverse
    of geo_offset_to_latlng, evaluated at the reference latitude."""
    clat = math.cos(geo_deg2rad(lat0))
    if abs(clat) < 1e-6:
        clat = 1e-6
    return ((lat1 - lat0) * M_PER_DEG_LAT,
            (lng1 - lng0) * M_PER_DEG_LAT * clat)


def geo_distance_m(lat1, lng1, lat2, lng2):
    dlat = (lat2 - lat1) * M_PER_DEG_LAT
    dlng = (lng2 - lng1) * M_PER_DEG_LAT * \
        math.cos(geo_deg2rad((lat1 + lat2) * 0.5))
    return math.sqrt(dlat * dlat + dlng * dlng)


def geo_project_to_ground(u, v, box_w, box_h, cam, mount, att, pose,
                          sigma_att_deg, sigma_alt_m, rangefinder_m=-1.0):
    """Forward: a normalized image point -> ground position. Same math as
    geoProjectToGround()."""
    res = GeoProjectionResult()

    ray_cam = geo_ray_camera(u, v, cam)
    ray_body = geo_ray_body(ray_cam, mount)
    ray_ned = geo_ray_ned(ray_body, att)

    n = math.sqrt(sum(x * x for x in ray_ned))
    if n < 1e-9:
        return res
    rn, re, rd = (x / n for x in ray_ned)

    if rd < MIN_DOWN:
        return res

    depth_m = pose.rel_alt_m
    if rangefinder_m > 0.0 and rd >= NEAR_VERTICAL:
        depth_m = rangefinder_m
    if depth_m <= 0.0:
        return res

    slant_m = depth_m / rd
    north_m, east_m = slant_m * rn, slant_m * re
    res.lat, res.lng = geo_offset_to_latlng(pose.lat_deg, pose.lng_deg,
                                          north_m, east_m)
    res.ground_range_m = math.sqrt(north_m * north_m + east_m * east_m)

    s_alt = sigma_alt_m / rd
    s_att = geo_deg2rad(sigma_att_deg) * slant_m
    s_box = 0.5 * slant_m * max(
        abs(box_w) * geo_deg2rad(cam.hfov_deg),
        abs(box_h) * geo_deg2rad(cam.vfov_deg))
    res.unc_m = 2.0 * math.sqrt(s_alt * s_alt + s_att * s_att + s_box * s_box)

    res.ok = True
    return res


def geo_ground_to_pixel(lat, lng, alt_above_plane_m, cam, mount, att, pose):
    """Inverse: a point on the ground plane (or alt_above_plane_m above it)
    -> the normalized image coordinate (u, v) the camera would see it at.

    Returns (u, v, slant_m) or None when the point is behind the camera,
    or the resulting ray would fail the forward projection's horizon
    check (rd < sin 5 deg) - the same reachability rule the C++ uses.
    u, v are the centered -0.5..0.5 convention; callers decide how to
    handle |u| > 0.5 (off-frame)."""
    north_m, east_m = geo_latlng_to_ne(pose.lat_deg, pose.lng_deg, lat, lng)
    down_m = pose.rel_alt_m - alt_above_plane_m
    n = math.sqrt(north_m * north_m + east_m * east_m + down_m * down_m)
    if n < 1e-9:
        return None
    ray_ned = (north_m / n, east_m / n, down_m / n)
    if ray_ned[2] < MIN_DOWN:
        return None                 # the forward path could not project it

    ray_body = geo_rot_t(att.roll_rad, att.pitch_rad, att.yaw_rad, ray_ned)
    base = geo_rot_t(geo_deg2rad(mount.roll_deg),
                     geo_deg2rad(mount.pitch_deg),
                     geo_deg2rad(mount.yaw_deg), ray_body)
    # undo geo_ray_body's base alignment: base = (cam_z, cam_x, cam_y)
    cam_x, cam_y, cam_z = base[1], base[2], base[0]
    if cam_z <= 1e-9:
        return None
    u = math.atan2(cam_x, cam_z) / geo_deg2rad(cam.hfov_deg)
    v = math.atan2(cam_y, cam_z) / geo_deg2rad(cam.vfov_deg)
    return (u, v, n)
