# stacks/ — declarative dev stacks

One yaml per stack. `./sim up <name>` / `./sim down <name>` /
`./sim logs <name> [proc]` / `./sim servers` and `./sim list stacks`
pick every file up — no engine edit. Format:

```yaml
name: de_servers            # must equal the file name minus .yaml
up_timeout_s: 30            # per-proc port-wait budget (default 30)
open_url: "{webclient_url}" # optional: what --open-web opens and
                            # `sim status` shows for this stack.
                            # {port} substitutes the first proc's port
up_note: "..."              # optional line printed after a successful up
provides:                   # optional: capabilities this stack lends
  render_web: m3d_web       #   sims - <what>: <proc name>. A live
                            #   stack's `render_web` replaces a render:
                            #   sim's own vite page (see below)
procs:                      # launch order - first entry starts first
  - name: de_auth           # proc name: pids/<name>.pid, logs/<name>.log
    kind: exec              # exec (default): run a built binary at
                            # <repo>/<bin> (the Rust servers; the
                            # --<stack> marker rides argv - they
                            # ignore unknown args)
                            # npm: exec `npm start` (react-scripts dev
                            # server, port from $PORT/SIM_WEBCLIENT_URL)
                            # node: exec `node --require <marker.cjs>
                            # <script> <args>` - the --<stack> marker
                            # rides the preload file's NAME so tools
                            # that reject unknown args (vite) stay
                            # stack-marked
    bin: target/release/x   # kind exec: binary path inside the repo
    repo_env: DE_AUTH_DIR   # sim.local.env / env override key (optional)
    repo: droneengage_authenticator
                            # repo dir: with repo_env this is the
                            # fallback under $DE_CODE; without repo_env
                            # it is used as-is (absolute or under DE_CODE)
    port_env: some_port_key # optional env var that overrides the port
                            # read from the repo's server.config(.local)
                            # (kind node: honoured via sim.local.env too)
    cwd: frontend           # kind node: working dir inside the repo
    script:                 # kind node: js entry, or a candidate list -
      - ../node_modules/vite/bin/vite.js   # relative to cwd, first
      - node_modules/vite/bin/vite.js      # existing wins
    args: ["--port", "{port}", "--strictPort"]
                            # kind node: argv after the script; {port}
                            # substitutes the resolved port
    port: 8070              # kind node: the port (port_env overrides)
```

Stacks are deliberately NOT sims: procs carry a `--<stack>` argv marker
so `simlib.pid_is_ours` stays False — `sim down`, `sim down --all`,
`sim run`'s teardown and `sim clean` can never sweep them. Only
`./sim down <stack>` stops a stack. State lives under `var/<stack
minus any de_ prefix>/`: `launch.<proc>.sh`, `pids/`, `logs/`.

## de_mavlink3dmap — the shared render page (SL-0154)

A `render:` sim normally launches its own Mavlink3DMap vite page on
`8070 + slot_base` next to its per-sim `mav_bridge`. The page itself is
slot-agnostic — the viewer URL selects feeds through `?mavlink=` and
`?world=` — so ONE shared page can serve every sim, the same way
`./sim up de_webclient` serves the GCS.

`./sim up de_mavlink3dmap` runs just that page (port `8070` by
default — below the `>= 10` slot range, so it never shadows a sim's
own `web_port`; override with `SIM_M3D_WEB_PORT`). While it is up, a
`render:` sim starts **no `web` proc**: its `gen_render` sees the
live `provides: render_web` and bakes the stack port into the
manifest's `render.page_port`, so the printed `render viewer:` URL
points at the shared page with that sim's own `?mavlink=` (and
`?world=` when a world stream runs). Two sims on different slots then
print URLs on the same page port with different `?mavlink=` ports —
each viewer shows its own vehicles.

The **mav_bridge (and render_sink) stay per-sim on purpose**: their
UDP input port is baked into each SITL's `--serial4 udpclient` argument
at launch and must follow the sim's slot — they cannot be shared.

Stack down or absent → `render:` behaves exactly as before: the sim
runs its own vite on its slot-derived `web_port`, torn down with the
sim. Nothing requires the stack: `sim doctor` and every suite pass
without Mavlink3DMap, and no test may depend on it. Stop it only with
`./sim down de_mavlink3dmap`.

## `./sim vcam` — virtual camera devices (DE-0050)

Not a stack — a one-shot setup command. `./sim vcam <sim.yaml>`
derives the `/dev/videoN` indices a sim's camera units open (merged
`camera_start_index`/`camera_end_index` / `camera_list device_num`)
and `sudo modprobe`s the missing v4l2loopback devices itself — the
only sudo path in the engine. Real devices are never touched;
`--print` shows the modprobe line, `--reload` replaces an
already-loaded module's devices (drops them).

Pair it with `camera_feed:` in the sim yaml — an ffmpeg lavfi testsrc
producer per configured index, launched before the units and killed by
`sim down` (required with `exclusive_caps`: a loopback device only
advertises CAPTURE while a producer streams). See
`sims/copter_and_cameras.md` for the full flow.
