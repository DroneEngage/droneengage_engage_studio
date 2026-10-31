# droneengage_studio — extending

**New unit type → a block (no code).** `blocks/<name>/block.yaml`:

    description: ...
    extends: copter            # optional
    add_modules: [de_camera]   # | modules: [...] | remove_modules: [...]
    # `cameras:` on a unit runs that many de_camera instances (SL-0163):
    #   cameras: 2                     # auto-binds next free DE-CAM labels
    #   cameras: [DE-CAM1, 0, {label: DE-CAM3, name: front}]
    # A de_camera unit with no `cameras:`/`camera:` selection defaults
    # to one auto pool label. Devices come from the named pool, created
    # once with root by tools/setup_v4l2loopback.sh (./sim vcam --pool,
    # --persist for across reboots): DE-CAM1..10, DE-SRC/DE-AI render
    # pairs, and the camera repo's named-VC labels - indices are
    # kernel-picked so real webcams are never collided with./sim up
    # <sim> --cam <dev> pins a single-camera sim to a real device or
    # label; ./sim ctl add_module <unit> de_camera hot-adds an instance
    # on a live unit (remove_module reverses; hot-added procs live in
    # the manifest and survive a same-spec `sim up`).
    fc: {vehicle: copter|plane|rover, model: quad, params: {PARAM: value}}   # fc: null = no SITL
    config: {de_comm: {sv_comm_lost_s: 20}}
    location: {north_m: 0, east_m: 0}

Put larger overlays in `blocks/<name>/<module>.json` (comments allowed) and
extra SITL params in `blocks/<name>/sitl.parm`. Check the result with
`./sim block <name>`.

**New DE module → a catalog entry (no code)** in `modules/catalog.yaml`:
`repo`, `dir_env`, `bin` (`bin_env` if built elsewhere), `template`,
`port_base` (pick an unused range; slot ports must stay < 65535),
`start_delay`, `needs_fc`, `local` (`arg: -b` only if the binary's getopt
accepts it, otherwise `file:` in the cwd), `set`, `wire` (placeholders
`{fcb_port} {pilot_port} {sysid} {comm_port} {port} {unit}` fill in
both), and `ready_re`
(a journal line proving the module is up; `sim up` waits for it). Add the
`dir_env`/`bin_env` keys to `sim.local.env.example`.

**New fleet → a sim (no code).** `units:` entries take `block`, `name`,
`count` (→ name1..nameN), `at`, `config`, `fc`. Use `extends: other.yaml`
to reuse a fleet with a different `server:`. A one-twist variant is an
overlay, not a new file: `sims/overlays/<name>.yaml` +
`--with <name>` (see `sims/overlays/README.md`). A scenario takes the
fleet from a sims file with `sim: {base: copters3, with: [...]}` instead
of `fleet:`/`transport:`.

**New test/scenario → a file (no code)** when existing steps, faults and
assertions suffice. Tests are de.scenario/1 (`fleet:` + `timeline:`,
docs/SCENARIOS.md) — `./sim new <name>` scaffolds one, `./sim validate`
checks it; the legacy `sim:`+`steps:` form was removed (SL-0020).
Automated cases go under `tests/<area>/`, human ones under
`scenarios/`; every file opens with a `#`-comment "what it proves"
paragraph (CI enforces it).
Start mission cases with `stop`, `clear`, `mode: {unit: all, mode: GUIDED}`.

**New step / assertion / ctl command (code):**
1. the operator action → `ctl.py` `main()`, also listed in its docstring
   (that docstring IS the `./sim ctl` help - a dispatched subcommand
   missing from it is invisible);
2. the step → `runner.run_step` calls it through `run_ctl`, plus an entry in
   `_UNIT_STEPS` if it takes `unit:`;
3. the assertion → `runner.check_assertion`. Read journals only through
   `simlib.journal_match` / `journal_lines` with the test's `marks`;
4. document it in the runner docstring and README.

**New command / flag → update the help in the same change.** `./sim`'s
overview and per-command `help` are generated from
`engine/sim_commands.py` `COMMANDS` (summary / usage / flags / examples) -
a command without a registry entry fails `engine/test_sim_commands.py`,
and a flag missing from the entry is refused by `_bad_flags` even when
the dispatch parses it. The overview prints only `usage[0]`, so a flag
the user should see goes in the usage line, not just the flags table.
`./sim ctl`'s help is the `ctl.py` module docstring; README.md's command
list stays in step too.

**New vehicle type**: add to `simlib.FC_KINDS` (SITL binary, model, defaults,
pymavlink mode table, arm mode, and how `land` brings it down).

**New server behaviour**: the bridge and the fake server share one control
protocol. Implement a command in both, or have the bridge answer `ERR ...`
and make `ctl.ctrl` refuse it for that server type.
