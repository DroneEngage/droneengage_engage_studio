#!/usr/bin/env python3
"""coverage.py - live coverage + probability-of-detection map (P5-12).

Computed OFFLINE at assert time from telemetry.jsonl (D1): each camera
unit's recorded poses replayed through the world's own camera/LOS model
produce, per grid cell of a search area:

    looks    - how many looks landed on the cell (in frustum + range),
    pod      - 1 - prod(1 - p_look): the cumulative probability a
               person standing there would have been detected,
    covered  - at least one look with p_look > 0 (D4: separates
               "never looked" from "looked badly").

A look is one thinned telemetry row (the camera's fps_boxes; D3) and
p_look uses the SAME functions the stub detection does:
p_cam(u, v) * env_p_scale * world.transmittance() - coverage and
recall can never disagree about what was "visible". Looks are assumed
independent (documented simplification).

The grid is square cells of cell_m in local ENU over an area polygon
(world `areas:` name or an inline `poly:`), evaluated at cell centres
on the ground plane.

Pure python, no numpy: a per-look bbox prefilter (the projected frame
corners, or the max-range disc when a corner is above the horizon)
keeps the inner loop to the cells the frustum can actually see.
"""

import math

import geo_projection as gp

DEFAULT_CELL_M = 5.0


def _point_in_poly(x, y, pts):
    """Ray-cast point-in-polygon; pts are (x, y) pairs."""
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and \
                x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def build_grid(poly_ll, origin, cell_m):
    """Square cells over a lat/lng ring -> {"cells": {(ix,iy): cell},
    "origin", "cell_m", "min_x", "min_y"}. cell centres sit in ENU
    (x east, y north) about origin and carry their lat/lng back."""
    pts = []
    for p in poly_ll:
        north, east = gp.geo_latlng_to_ne(origin[0], origin[1],
                                          p[0], p[1])
        pts.append((east, north))
    min_x = min(p[0] for p in pts)
    max_x = max(p[0] for p in pts)
    min_y = min(p[1] for p in pts)
    max_y = max(p[1] for p in pts)
    cells = {}
    iy = 0
    y = min_y + cell_m * 0.5
    while y < max_y:
        ix = 0
        x = min_x + cell_m * 0.5
        while x < max_x:
            if _point_in_poly(x, y, pts):
                lat, lng = gp.geo_offset_to_latlng(origin[0], origin[1],
                                                 y, x)
                cells[(ix, iy)] = {"x": x, "y": y, "lat": lat,
                                   "lng": lng, "looks": 0,
                                   "miss": 1.0, "covered": False,
                                   "pod": 0.0}
            ix += 1
            x += cell_m
        iy += 1
        y += cell_m
    return {"cells": cells, "origin": list(origin), "cell_m": cell_m,
            "min_x": min_x, "min_y": min_y, "poly": pts}


def cell_at(grid, lat, lng):
    """The cell containing a lat/lng point, or None when it lies
    outside the area polygon. Direct containment (the index math
    loses points sitting exactly on a cell edge to float noise)."""
    north, east = gp.geo_latlng_to_ne(grid["origin"][0],
                                      grid["origin"][1], lat, lng)
    if not _point_in_poly(east, north, grid["poly"]):
        return None
    half = grid["cell_m"] * 0.5 + 1e-9
    for cell in grid["cells"].values():
        if abs(cell["x"] - east) < half and abs(cell["y"] - north) < half:
            return cell
    return None


def thin_rows(rows, fps):
    """One telemetry row per 1/fps second of t_ms (D3 - a look per
    camera frame, not per 2 Hz sample)."""
    if fps <= 0:
        return list(rows)
    out = []
    last = None
    step_ms = 1000.0 / fps
    for r in rows:
        t = r.get("t_ms")
        if t is None:
            continue
        if last is None or t - last >= step_ms - 1e-6:
            out.append(r)
            last = t
    return out


def p_detect_uv(u, v, cam, kind):
    """The stub's per-pixel detection probability (stub_ai_box D5):
    rgb interpolates nadir -> frame edge by the off-axis fraction of
    the half-frame; thermal is the flat p_detect_thermal."""
    if kind == "thermal":
        return float(cam.get("p_detect_thermal", 0.85))
    edge = max(abs(u), abs(v)) * 2.0
    return cam["p_detect_nadir"] + \
        (cam["p_detect_edge"] - cam["p_detect_nadir"]) * min(1.0, edge)


def _look_bbox(grid, cam, mount, att, pose, max_range_m):
    """The ENU bbox the frustum can see this look: the four frame
    corners projected to the ground (any above-horizon corner falls
    back to the max-range disc about the nadir)."""
    north0, east0 = gp.geo_latlng_to_ne(grid["origin"][0],
                                        grid["origin"][1],
                                        pose.lat_deg, pose.lng_deg)
    corners = []
    for u in (-0.5, 0.5):
        for v in (-0.5, 0.5):
            r = gp.geo_project_to_ground(u, v, 0.0, 0.0, cam, mount,
                                         att, pose, 0.0, 0.0)
            if not r.ok:
                break
            n, e = gp.geo_latlng_to_ne(grid["origin"][0],
                                       grid["origin"][1], r.lat, r.lng)
            corners.append((e, n))
        else:
            continue
        break
    if len(corners) == 4:
        xs = [c[0] for c in corners] + [east0]
        ys = [c[1] for c in corners] + [north0]
        # generous pad: cell centres near a slanted edge must not drop
        pad = grid["cell_m"] * 1.5
        return (min(xs) - pad, max(xs) + pad,
                min(ys) - pad, max(ys) + pad)
    return (east0 - max_range_m, east0 + max_range_m,
            north0 - max_range_m, north0 + max_range_m)


def replay_look(grid, world, cam, mount, cam_spec, att, pose, env_f,
                cam_kind):
    """One look: every in-frustum cell gets looks += 1, miss *=
    1 - p_look, covered |= p_look > 0. The grid index prefilter keeps
    the inner loop to the cells the frustum's bbox covers."""
    if pose.rel_alt_m <= 0.0:
        return
    max_range_m = env_f["max_range_m"]
    x0, x1, y0, y1 = _look_bbox(grid, cam, mount, att, pose,
                                max_range_m)
    cells = grid["cells"]
    if not cells:
        return
    cm = grid["cell_m"]
    # bbox -> cell index window (the +-1 margin eats float noise on
    # the window edge; membership is decided by the frustum test)
    ix0 = int((x0 - grid["min_x"]) / cm) - 1
    ix1 = int((x1 - grid["min_x"]) / cm) + 1
    iy0 = int((y0 - grid["min_y"]) / cm) - 1
    iy1 = int((y1 - grid["min_y"]) / cm) + 1
    cam_lla = (pose.lat_deg, pose.lng_deg, pose.rel_alt_m)
    for iy in range(iy0, iy1 + 1):
        for ix in range(ix0, ix1 + 1):
            cell = cells.get((ix, iy))
            if cell is None or cell["x"] < x0 or cell["x"] > x1 \
                    or cell["y"] < y0 or cell["y"] > y1:
                continue
            uv = gp.geo_ground_to_pixel(cell["lat"], cell["lng"], 0.0,
                                        cam, mount, att, pose)
            if uv is None:
                continue
            u, v, rng_m = uv
            if abs(u) > 0.5 or abs(v) > 0.5 or rng_m > max_range_m:
                continue
            cell["looks"] += 1
            vis, _ = world.transmittance(
                cam_lla, (cell["lat"], cell["lng"], 0.0))
            p = p_detect_uv(u, v, cam_spec, cam_kind) \
                * env_f["p_scale"] * vis
            if p > 0.0:
                cell["covered"] = True
            cell["miss"] *= (1.0 - p)


def compute(poly_ll, origin, cell_m, world, cam_spec, looks):
    """The whole map: build the grid over poly_ll and replay looks.

    cam_spec - the world's sensors.camera dict (its lens/k1/k2/mount
    and p_detect model). looks - [(att, pose, env_f, cam_kind)] with
    att/pose geo_projection dataclasses and env_f the per-unit
    world.sensor_factors."""
    grid = build_grid(poly_ll, origin, cell_m)
    gp_cam = gp.GeoCameraParams(cam_spec["hfov_deg"], cam_spec["vfov_deg"],
                                cam_spec.get("lens", "rectilinear"),
                                cam_spec.get("k1", 0.0),
                                cam_spec.get("k2", 0.0))
    mount = gp.GeoMountParams(cam_spec["mount_pitch_deg"],
                              cam_spec["mount_yaw_deg"],
                              cam_spec["mount_roll_deg"])
    for att, pose, env_f, kind in looks:
        replay_look(grid, world, gp_cam, mount, cam_spec, att, pose,
                    env_f, kind)
    for cell in grid["cells"].values():
        cell["pod"] = 1.0 - cell["miss"]
        del cell["miss"]
    return grid


def summary(grid):
    """The coverage.json summary: cell/covered counts, the covered
    fraction and the mean POD over the whole area."""
    cells = list(grid["cells"].values())
    n = len(cells)
    covered = sum(1 for c in cells if c["covered"])
    return {"cells": n, "covered": covered,
            "coverage": covered / n if n else 0.0,
            "pod_mean": sum(c["pod"] for c in cells) / n if n else 0.0,
            "looks": sum(c["looks"] for c in cells)}


def geojson(grid):
    """cells as square polygon features (pod/looks/covered properties)
    - opens in any GIS and the P6 replay view (D5)."""
    feats = []
    half = grid["cell_m"] * 0.5
    for c in grid["cells"].values():
        ring = []
        for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1)):
            lat, lng = gp.geo_offset_to_latlng(
                grid["origin"][0], grid["origin"][1],
                c["y"] + dy * half, c["x"] + dx * half)
            ring.append([lng, lat])
        feats.append({"type": "Feature",
                      "properties": {"pod": round(c["pod"], 4),
                                     "looks": c["looks"],
                                     "covered": c["covered"]},
                      "geometry": {"type": "Polygon",
                                   "coordinates": [ring]}})
    return {"type": "FeatureCollection", "features": feats}
