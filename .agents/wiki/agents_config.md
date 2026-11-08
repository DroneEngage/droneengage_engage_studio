# droneengage_studio — config build & ports

## How config is built (precedence, lowest -> highest)

1. module base template: its repo's `*.config.module.json`; de_comm uses
   `modules/templates/de_comm.json`
2. catalog `set:` defaults
3. block overlays along the `extends:` chain (`<module>.json` files, then
   inline `config:`)
4. per-unit `config:` / `fc:` in the sim
5. engine wiring and catalog `wire:` - ports, unitID/party, server and auth
   settings, state files, journal dir. Never put these in a block;
   they are overwritten.

Machine paths come only from `simlib.setting()`: environment variable >
`sim.local.env` > default. **Never hard-code `~/de_code/...` or
`~/code/ardupilot` in engine code, blocks or sims.**

## Ports

Slot `k = slot_base (10) + unit index` is the fleet's IDENTITY only
(SITL `-I k`, sysid `k`); no port derives from it. **Every** local port
a sim binds is a named request to the machine-wide allocator - there
are no formulas left: SITL binds a leased 10-port block (serial0 pilot
+0, serial2 aux +2, serial5 pose +5) plus separate leases for fcb
(serial1), RC-in, sim-link in/out and irlock; the fcb proxy
(`fcb_proxy: true`, SL-0008), link proxy (SL-0009), udp_proxy
(SL-0010), module s2s ports (de_comm, de_mavlink, ...), server wss and
control, authenticator https/s2s, the relay wss and its UDP window,
and the five render ports are each their own lease. Re-run stability
comes from the allocator's remembered ports (it tries the last run's
port first); the only static preferences are the viewer's own defaults
(render.mav_ws 8811, render.web 8080). SITL launches with
`--base-port/--rc-in-port/--sim-port-*/--irlock-port` before `-I`, so
every socket is the leased value. The committed map is
`var/run/<sim>/sim.json` `ports` - read it (or `./sim ports` /
`./sim ctl status`) for what a live sim actually bound. Only
yaml-explicit ports pin - exact or fail.

**Port leases (allocator):** `engine/port_allocation_management.py` keeps
the machine-wide registry every worktree shares -
`$XDG_RUNTIME_DIR/de_sim/ports.json` (fallback `~/.cache/de_sim/ports.json`;
`SIM_PORT_REGISTRY`/`SIM_PORT_LAST` override; never under `var/`). `sim up`
leases the whole port set in one atomic transaction
(`simlib.lease_requests` -> `lease_commit` -> `_apply_ports` write-back
before `generate`). `down`/`clean` release; stale leases (dead holder,
dead sim, gone worktree) are swept by the next transaction or
`./sim ports gc`. `sim_hash` covers the request set, not the allocated
values, so a moved fleet still warm-reuses. See
`docs/port_allocation.md` before touching port logic.

A unit can run several instances of the same module (SL-0163
`cameras:`; SL-0165 `ctl add_module`): instance `i` (1-based,
`de_camera` = 1, `de_camera{i}` after) gets its own s2s lease plus its
SL-0010 udp_proxy lease; max 5 instances per unit.
