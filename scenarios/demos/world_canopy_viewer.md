# world_canopy_viewer — how to run

Forest-search demo watched live in 3D: one copter sweeps a forest; the
hiker in the open must be found, the one inside the hut must stay
hidden, and coverage / POD are graded at the end. This is
`world_canopy_sweep` (P5-14 manual acceptance) with the Mavlink3DMap
viewer live on the fake server.

## Setup

- `MAVLINK3DMAP_DIR` in `sim.local.env` — render mode `viewer` launches
  `udp2websocket.js` + a vite dev server from that checkout under
  `var/run/<sim>/render/`.
- Standard builds: de_comm, de_mavlink binaries.

## Run

```bash
./sim view scenarios/demos/world_canopy_viewer.yaml
```

`view` = `run --hold`: the timeline (deploy → arm → start → sweep) and
assertions run, then the run parks so the viewer session stays alive
until Ctrl-C. On `up` it prints the viewer URL:

```
http://127.0.0.1:<8070+slot>/?world=ws://127.0.0.1:<61000+slot>&mavlink=ws://127.0.0.1:<8801+slot>
```

## What to look at (manual checklist)

- `person_hut` renders as an occluded OUTLINE — every look row is why
  "occluded" (the negative control).
- `person_open` gets found: a yellow world-model target pin + ring
  lands on it (bot.targets() overlay, 1 Hz).
- The copter visibly flies the three lawnmower legs over canopy, open
  ground and hut.
- Everything renders offline — obstacles/entities/overlays come from
  the world stream, ground texture from `frontend/public` (no Mapbox,
  no CDN).

## Assertions graded at the end

detection_recall (open found, hut not), detection_precision ≥ 0.8,
distance-to-truth ≤ 25 m, coverage ≥ 95 % of sector_a, POD means and
per-area POD thresholds.

## Teardown

Ctrl-C the `view` run — teardown `land` brings the copter down.
