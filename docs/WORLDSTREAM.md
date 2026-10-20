# de.worldstream/1 — the sim ↔ renderer world contract

Versioned schema for the per-run world feed a scenario runner serves to
renderers (the Mavlink3DMap world scene, the render-camera farm, the
physics loop). The runner is the server; renderers are WebSocket
clients. One feed per run — the runner owns the truth, so every
renderer sees the same world the assertions grade.

Implementations: `engine/world_stream.py` (server, this repo),
`frontend/src/js/world/world_client.js` (client, Mavlink3DMap).

## Transport

- `ws://127.0.0.1:<render.stream_port>` — localhost only, one port per
  run (the slot block allocates it).
- Text frames, one complete JSON object per frame. Strict JSON only
  (see Rules).
- On connect the server sends exactly one `hello`, then `tick`s at
  10 Hz. A dropped client reconnects and gets a fresh `hello` — there
  is no resume and no backlog beyond the one snapshot.

## Frame envelope

Every frame is a JSON object discriminated by `t`:

| `t` | Direction | Purpose |
|---|---|---|
| `hello` | server → client | world snapshot on connect |
| `tick` | server → client | entity truth + clutter at 10 Hz; every 10th tick embeds `overlay` |
| `overlay` | (inside `tick`) | belief layer at 1 Hz — targets, states, detections |
| `cmd` | server → client | renderer action derived from truth rows |
| `event` | client → server | a renderer/physics observation appended to `truth.jsonl` |

## Time bases

- `t_s` — scenario seconds, float, `t = (t_unix_ms − epoch_unix_ms) / 1000`.
- `t_unix_ms` — wall-clock epoch milliseconds of the sample the frame
  was computed from. Ticks carry both; events stamp the pose sample,
  not send time.
- `epoch_unix_ms` — the run's epoch, sent once in `hello`.

## Units

| Field | Unit |
|---|---|
| `lat`, `lng`, `origin` | degrees, WGS-84 |
| `alt_m`, `size_m`, `impact_mps`, `collision_radius_m`, `unc_m` | metres (m/s) |
| `heading_deg` | degrees, 0 = north, clockwise |
| `*_ms`, `t_unix_ms`, `t_ms` | milliseconds |
| `t_s` | seconds, float |
| `px` | rendered pixel count (client resolution) |
| `box` | normalized viewport rect `{x, y, w, h}` in 0..1, y down |

## `hello` — the connect snapshot

```json
{"t":"hello","schema":"de.worldstream/1","sim":"ws_test","seed":7,
 "origin":[lat,lng],"epoch_unix_ms":0,
 "obstacles":[...],"areas":{...},"trails":{...},"environment":{...},
 "entities":[{"id":"person_open","cls":"person","size_m":1.8}],
 "units":[{"name":"cam1","sysid":1,"block":"static_cam",
           "camera":{...},"payload":{...},"collision_radius_m":0.3}]}
```

| Field | Meaning |
|---|---|
| `schema` | literal `"de.worldstream/1"` — check it first |
| `sim`, `seed` | run name and the world seed (same seed, same world) |
| `origin` | `[lat, lng]` — the renderer's flat-earth anchor (`M_PER_DEG_LAT = 111320`) |
| `epoch_unix_ms` | run epoch; `t_s` counts from here |
| `obstacles`, `areas`, `trails`, `environment` | world geometry and conditions as authored (occluders, named regions, entity paths, visibility/wind) |
| `entities[]` | every truth entity: `id`, `cls`, `size_m` |
| `units[]` | every unit: `name`, `sysid`, `block`; `camera`/`payload`/`collision_radius_m` only on units that have them |

## `tick` — 10 Hz truth

```json
{"t":"tick","t_s":12.3,"t_unix_ms":0,
 "entities":[{"id":"person_open","lat":1.5,"lng":2.5,"alt_m":0.0,
              "heading_deg":90.0,"moving":true}],
 "clutter":[{"id":"cl_7","lat":1.5,"lng":2.5,"cls":"bush"}],
 "overlay":{...}}
```

- `entities[]` — ground-truth pose of every entity this tick
  (`heading_deg`, `moving` from the entity's motion model).
- `clutter[]` — the live ground-anchored clutter field (decoys).
- `overlay` — present on every 10th tick (1 Hz):

```json
"overlay":{"targets":{"tid_42":{"cls":"person","pos":{"lat":1.5,"lng":2.5}}},
           "states":{"person_open":"seen","person_hut":"occluded"},
           "detections":[{"tid":"person_open","cls":"person",
                          "lat":1.0,"lng":2.0,"alt_m":0.0,"t_ms":0}]}
```

| Field | Meaning |
|---|---|
| `targets` | the operator bot's world model — reported `cls`/`pos` per tid (the uncertainty ring) |
| `states` | latest truth-row outcome per tid: `seen` \| `miss` \| `occluded` |
| `detections` | recent sent look rows (capped at the last 40), for detection markers |

## `cmd` — renderer action

```json
{"t":"cmd","cmd":"payload_release","unit":"cam1","t_unix_ms":0}
```

Emitted when a `cls:"payload_drop"` row lands in `truth.jsonl` — a real
release and a stubbed one look the same, so the renderer animates a
drop on either. `t_unix_ms` is the truth row's `t_ms`.

## `event` — client → server

```json
{"t":"event","kind":"visibility","src":"render_vis","unit":"cam1",
 "frame_no":12,"tid":"person_open","px":341,"box":{...},"t_unix_ms":0}
```

A client observation the run grades. The server appends it to
`truth.jsonl` as `{type:"event", src:"render" (or the given src),
t_ms: t_unix_ms || now}` — assertions read the same file.

Known `kind`s:

| `kind` | Src | Fields |
|---|---|---|
| `payload_landed` | physics | `unit`, `lat`, `lng`, `alt_m`, `hit`, `impact_mps` |
| `collision` | physics | `unit`, `with` (what was hit) |
| `frame_meta` | render | `unit`, `frame_no`, `t_unix_ms` (pose sample), `render_t_ms` — joins `frame_seen` for the latency probe |
| `visibility` | render_vis | `unit`, `frame_no`, `tid`, `px`, `box` (omitted when `px: 0`) |

New kinds are additive: the truth row is stored verbatim, so checks
can key on a kind the stream itself does not know.

## Rules

1. **Strict JSON.** No `Infinity`, no `NaN` (browsers drop the whole
   frame on `JSON.parse`). Non-finite values are serialized as `null`
   — e.g. `environment.visibility_m = null` means unbounded.
2. **Additive evolution.** Unknown `t` values and unknown fields are
   ignored on both sides. New frame types, fields and event kinds may
   be added without a version bump as long as old consumers still
   parse — a change that alters an existing field's meaning or shape
   bumps the schema string (`de.worldstream/2`).
3. **Check `schema` on hello.** A consumer should refuse (or warn)
   on a `schema` it does not understand rather than half-parse it.
4. **Truth is append-only.** Events land in `truth.jsonl` in arrival
   order; late or out-of-order `t_ms` values are legal — assertions
   compare, they do not reorder.
