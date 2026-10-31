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

Slot `k = slot_base (10) + unit index`. SITL `-I k`, sysid `k`, pilot
`5760+10k`, fcb `7660+10k`. The opt-in fcb proxy (`fcb_proxy: true`,
SL-0008) takes `7760+10k` (de_mavlink dials it instead of `fcb_port`) and
`7860+10k` (fault control port). Module s2s ports are `port_base + 10k`
(de_comm 62000, de_mavlink 63000, de_camera 64000, de_tracker 65000). Server
wss 62008, control 62009, authenticator +10/+11. The opt-in link proxy
(SL-0009) takes `61500+10k` (de_comm dials it instead of the server wss)
and `61501+10k` (fault control port). Keep `slot_base` >= 10 so
sim ports never collide with the developer's own servers.

A unit can run several instances of the same module (SL-0163
`cameras:`; SL-0165 `ctl add_module`): instance `i` (1-based,
`de_camera` = 1, `de_camera{i}` after) takes `port_base + 10k +
2*(i-1)` — the odd offset under each instance stays that instance's
SL-0010 udp_proxy port; max 5 instances per unit.
