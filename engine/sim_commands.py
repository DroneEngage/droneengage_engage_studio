"""sim_commands.py - the central command registry for the `sim` CLI.

Every command `./sim` dispatches is described here in one place: a
one-line summary, its usage line(s), a longer description, the flags it
accepts, whether it takes the shared flag set, and examples. The `sim`
front-end builds its top-level help (overview) and per-command help
(command_help) from this registry, and asks allowed_flags() for the set
a command's flag validation checks against - so a new command without a
registry entry fails engine/test_sim_commands.py.

The wording was moved out of the `sim` script's module docstring; the
text is kept as-is (no behaviour invented). ctl's subcommands are not
duplicated here - its entry just points at `./sim ctl`.
"""

# The intro paragraph that opens `./sim` / `./sim help` (from the
# original module docstring).
INTRO = (
    "DroneEngageStudio - sim: simulation and ground-truth testing from "
    "reusable\n"
    "blocks. DroneEngage is the flagship integration (not the owner)."
)

# Trailing notes that close the overview - the three-axis model
# (SL-0135): what you compose, why you run it, where it runs.
LAYOUT_NOTE = (
    "Model - compose: blocks -> sims (+worlds/, packages/, stacks); "
    "execute: tests/ (machine-checked), scenarios/ (human-run);\n"
    "substrate: host SITL, ./sim up <stack>, tools/rpi, tools/hil.\n"
    "docs/SCENARIOS.md is the authoring reference. See README.md."
)

# Intent groups for the overview (SL-0135): every COMMANDS entry has a
# "group" naming one of these; overview() prints a section header per
# group in this order.
GROUPS = [
    ("DISCOVER", "see what exists - blocks, sims, stacks, ports"),
    ("COMPOSE", "author and check sims and scenarios"),
    ("LIVE", "bring sims and dev stacks up, watch and drive them"),
    ("EXECUTE", "run tests and suites"),
    ("CAPTURE", "record a live session into a scenario"),
    ("HARDWARE", "boards and rigs (tools/hil)"),
    ("HOUSE", "housekeeping"),
]

MACHINE_NOTE = (
    "Machine output: show, sims, status (and ./sim ctl status), list,\n"
    "validate, expand, doctor and verbs take --json (docs/MACHINE_API.md).\n"
    "A run's last line is `RESULT <dir>/summary.json` - the run's full\n"
    "report."
)

# Flags shared across several commands. --force is valid on every command
# (stripped before per-command parsing); the rest are pulled out by
# _cli_flags for the commands whose shared_flags is true. Each value is a
# one-line help.
SHARED_FLAGS = {
    "--user": "the one account every unit and the GCS bridge log in with "
              "(overrides per-unit account: too)",
    "--password": "the login code for --user",
    "--auth-server": "the authenticator host[:port] (external server with "
                     "auth: true only)",
    "--slot-base": "the slot every port derives from (wins over the sim "
                   "yaml's slot_base: and SIM_SLOT_BASE); `any` picks the "
                   "lowest free base",
    "--with": "sims/overlays/<name>.yaml twists applied on top of the sim",
    "--set": "override a sim yaml params: entry (name=value, repeatable; "
             "SL-0116)",
    "--shared": "run/view/suite stay additive next to live sims instead of "
                "downing them (= SIM_SHARED=1)",
    "--force": "take over a sim another live session owns (every command "
               "takes it; = SIM_FORCE=1)",
}

# name -> {summary, usage, description, flags, shared_flags, examples,
#          takes_sim}
COMMANDS = {
    "blocks": {
        "group": "DISCOVER",
        "summary": "list blocks (reusable unit types)",
        "usage": ["./sim blocks"],
        "description": "List every block under blocks/ with its flight "
                       "controller kind, the modules it runs and any "
                       "parent it extends.",
        "flags": {},
        "shared_flags": False,
        "examples": ["./sim blocks"],
        "takes_sim": False,
    },
    "block": {
        "group": "DISCOVER",
        "summary": "show a block resolved through `extends`",
        "usage": ["./sim block <name>"],
        "description": "Resolve a block with its `extends:` chain and print "
                       "the full expansion (modules, fc, config overlays, "
                       "parm files) as JSON.",
        "flags": {},
        "shared_flags": False,
        "examples": ["./sim block copter"],
        "takes_sim": False,
    },
    "show": {
        "group": "COMPOSE",
        "summary": "units, modules and ports a sim expands to",
        "usage": ["./sim show <sim.yaml> [--json]"],
        "description": "Resolve a sim file into concrete units, modules and "
                       "ports without starting anything. --json prints the "
                       "machine-readable expansion.",
        "flags": {
            "--json": "machine-readable json (docs/MACHINE_API.md)",
        },
        "shared_flags": True,
        "examples": ["./sim show copters3", "./sim show sims/copters3.yaml "
                     "--json"],
        "takes_sim": True,
    },
    "up": {
        "group": "LIVE",
        "summary": "start a sim (restarts the same-named one)",
        "usage": ["./sim up <sim.yaml|de_servers|de_webclient> [--fresh] "
                  "[--open-web] [--cam dev]"],
        "description": "Start a sim. Restarts the same-named sim; refuses "
                       "ports of a live sim - sims run in parallel; unit "
                       "names may repeat on per-sim servers but stay unique "
                       "on a shared external stack or rendered cameras. "
                       "--open-web xdg-opens the webclient link. "
                       "de_servers and de_webclient are the developer's "
                       "real servers/webclient, not sims (only `./sim "
                       "down <stack>` stops them).",
        "flags": {
            "--fresh": "fresh bring-up (tear down any same-named sim first)",
            "--open-web": "xdg-open the webclient link once the sim is up",
            "--cam": "pin THE camera of a single-camera sim: --cam 0 binds "
                    "/dev/video0, --cam DE-CAM3 binds that card label",
        },
        "shared_flags": True,
        "examples": ["./sim up copters3", "./sim up copters3 --fresh",
                     "./sim up de_servers"],
        "takes_sim": True,
    },
    "vcam": {
        "group": "LIVE",
        "summary": "create the v4l2loopback devices a camera sim needs",
        "usage": ["./sim vcam <sim.yaml> [--print|--dry-run] [--reload] "
                  "| --pool | --list"],
        "description": "Derive the /dev/videoN indices a sim's camera "
                       "units open from their merged de_camera configs "
                       "and sudo-modprobe v4l2loopback for the missing "
                       "ones (real devices are never touched). The only "
                       "sudo path in the engine - `sim up` stays "
                       "unprivileged. --pool instead runs "
                       "tools/setup_v4l2loopback.sh - the one-time "
                       "named-label pool (DE-CAM*, DE-SRC/DE-AI, ...) "
                       "label-bound cameras and hot-added de_camera "
                       "instances need; its other args (--persist, "
                       "--force, counts) pass through to the script. "
                       "--list is read-only and unprivileged: every "
                       "/dev/videoN with its label, kind, the live-sim "
                       "unit.proc bound to it and the feed producing "
                       "into it, plus the free pool labels.",
        "flags": {
            "--print": "show the modprobe line without running it",
            "--dry-run": "alias for --print",
            "--reload": "unload the module first (drops its existing "
                        "devices) when it is already loaded",
            "--pool": "run tools/setup_v4l2loopback.sh - the one-time "
                      "named-label pool (DE-CAM*, DE-SRC/DE-AI, ...)",
            "--list": "list every /dev/videoN: label, real/loopback, "
                      "bound unit.proc and feed proc, plus free labels",
        },
        "shared_flags": False,
        "examples": ["./sim vcam sims/copter_and_cameras.yaml",
                     "./sim vcam copter_and_cameras --print",
                     "./sim vcam --pool --persist",
                     "./sim vcam --list"],
        "takes_sim": True,
    },
    "down": {
        "group": "LIVE",
        "summary": "stop one sim, or every sim",
        "usage": ["./sim down [name|--all]"],
        "description": "Stop one sim by name, or every sim with --all. "
                       "`down de_servers` / `down de_webclient` stop the "
                       "dev stacks.",
        "flags": {
            "--all": "stop every live sim",
        },
        "shared_flags": False,
        "examples": ["./sim down", "./sim down copters3",
                     "./sim down --all", "./sim down de_servers"],
        "takes_sim": False,
    },
    "sims": {
        "group": "DISCOVER",
        "summary": "generated sims and live process counts",
        "usage": ["./sim sims [--json]"],
        "description": "List every generated sim with its unit count, "
                       "server type and live process count.",
        "flags": {
            "--json": "machine-readable json",
        },
        "shared_flags": False,
        "examples": ["./sim sims", "./sim sims --json"],
        "takes_sim": False,
    },
    "servers": {
        "group": "DISCOVER",
        "summary": "both stacks' procs and ports",
        "usage": ["./sim servers [--json]"],
        "description": "Show the dev server stack (de_servers) and webclient "
                       "(de_webclient) procs and ports.",
        "flags": {
            "--json": "machine-readable json",
        },
        "shared_flags": False,
        "examples": ["./sim servers"],
        "takes_sim": False,
    },
    "ports": {
        "group": "DISCOVER",
        "summary": "live sims' declared ports vs what's bound right now",
        "usage": ["./sim ports [name|--all]"],
        "description": "Show which of a sim's declared ports are actually "
                       "bound right now. --all lists every generated sim; "
                       "udp_relay shows its whole udp range n/N.",
        "flags": {
            "--all": "list every generated sim (not just live ones)",
        },
        "shared_flags": False,
        "examples": ["./sim ports", "./sim ports --all"],
        "takes_sim": False,
    },
    "status": {
        "group": "LIVE",
        "summary": "= ./sim ctl status (live sims only)",
        "usage": ["./sim status [-s <sim>] [live|all] [--open-web]"],
        "description": "Live sims only; `all` also lists downed ones. "
                       "--open-web xdg-opens the sim's link. Also surfaces "
                       "the dev stacks and orphaned scenario runners.",
        "flags": {
            "--open-web": "xdg-open the sim's webclient link",
        },
        "shared_flags": False,
        "examples": ["./sim status", "./sim status all"],
        "takes_sim": False,
    },
    "ctl": {
        "group": "LIVE",
        "summary": "operator console (./sim ctl for help)",
        "usage": ["./sim ctl [-s <sim>] <cmd> ..."],
        "description": "The operator console: live procs, units online, "
                       "mode/arm/mission control and link faults. Run "
                       "`./sim ctl` with no command for its own help "
                       "(ctl's subcommands are documented there, not here).",
        "flags": {},
        "shared_flags": False,
        "examples": ["./sim ctl status", "./sim ctl -s copters3 mode "
                     "copter1 GUIDED"],
        "takes_sim": False,
    },
    "watch": {
        "group": "LIVE",
        "summary": "party watch on any reachable stack",
        "usage": ["./sim watch --auth-ip H --auth-port P --account U "
                  "--code C"],
        "description": "Party watch on any reachable server stack - no sim "
                       "manifest needed (e.g. the external de_servers). "
                       "Prints join/leave/id events. --no-auth + "
                       "--server-ip/--server-port for auth-off servers; "
                       "--party NAME; --ctl-port P keeps the list/listen/req "
                       "control port too.",
        "flags": {},
        "shared_flags": False,
        "examples": ["./sim watch --auth-ip 127.0.0.1 --auth-port 62018 "
                     "--account demo --code 1234"],
        "takes_sim": False,
    },
    "run": {
        "group": "EXECUTE",
        "summary": "run a test/scenario - owns the machine",
        "usage": ["./sim run <file.yaml> [--seed N] [--keep] [--hold|--view] "
                 "[--record <file>] [--open-web] [--module <name>=<path>] ..."],
        "description": "Run a test/scenario - owns the machine: downs every "
                       "other sim (the CI posture; --shared / SIM_SHARED=1 "
                       "keeps it additive on a shared box). --keep skips the "
                       "teardown, --hold parks after the timeline (world "
                       "stream + Mavlink3DMap viewer live) until Ctrl-C. "
                       "Interactive work is additive by default: a bare "
                       "sims/*.yaml just comes up (hash-reused, like up). "
                       "--open-web opens the webclient link once the sim is "
                       "up. Seed: the file's `seed:` applies; --seed N "
                       "overrides it. --module <name>=<abs path> (repeatable) "
                       "swaps a catalog module / de_comm binary for this run "
                       "only: same fleet spec -> just that proc restarts on "
                       "the warm sim.",
        "flags": {
            "--seed": "override the file's seed: with N",
            "--keep": "skip the teardown (leave the sim up)",
            "--hold": "park after the timeline until Ctrl-C",
            "--view": "alias for --hold",
            "--record": "record the session window to <file>",
            "--open-web": "xdg-open the webclient link once the sim is up",
            "--module": "swap a module binary for this run "
                        "(<name>=<abs path>, repeatable)",
        },
        "shared_flags": True,
        "examples": ["./sim run tests/smoke_copters.yaml",
                     "./sim run tests/smoke_copters.yaml --keep --seed 2",
                     "./sim try tests/smoke_copters.yaml --module "
                     "de_mavlink=$HOME/de_code/droneengage_mavlink/bin/"
                     "de_ardupilot"],
        "takes_sim": True,
    },
    "try": {
        "group": "EXECUTE",
        "summary": "= run --keep: the build->try loop",
        "usage": ["./sim try <file.yaml> [--module <name>=<path>] ..."],
        "description": "The iteration form of run: it always keeps the sim "
                       "(no teardown) so the next try lands on a warm fleet. "
                       "Rebuilt de_mavlink? Point --module at the new bin and "
                       "the scenario runs against it on the warm sim (no "
                       "SITL warm-up).",
        "flags": {
            "--seed": "override the file's seed: with N",
            "--keep": "no teardown after the run (always on for try)",
            "--hold": "park after the timeline until Ctrl-C",
            "--view": "hold + the Mavlink3DMap viewer",
            "--record": "record the session window to <file>",
            "--open-web": "xdg-open the webclient link once the sim is up",
            "--module": "swap a module binary for this run "
                        "(<name>=<abs path>, repeatable)",
        },
        "shared_flags": True,
        "examples": ["./sim try tests/smoke_copters.yaml --module "
                     "de_mavlink=$HOME/de_code/droneengage_mavlink/bin/"
                     "de_ardupilot"],
        "takes_sim": True,
    },
    "suite": {
        "group": "EXECUTE",
        "summary": "run every yaml under dir(s) once per seed",
        "usage": ["./sim suite [dir ...] [--seeds 1,2,3] [--keep] "
                 "[--tier smoke] [--task <id>]"],
        "description": "Run every yaml under dir(s) once per seed (default "
                       "dir: tests) - the nightly run: owns the machine "
                       "serially, unless --shared. Verdicts + junit in "
                       "var/results/nightly/; --tier smoke runs only the "
                       "marked fast first-pass set; --task selects only "
                       "files tagged `tasks: [<id>]`.",
        "flags": {
            "--seeds": "comma-separated seed list (default 1,2,3)",
            "--keep": "skip teardowns (leave sims up)",
            "--tier": "run only the marked tier (smoke|regression)",
            "--task": "run only files tagged `tasks: [<id>]` (e.g. "
                      "SL-0099)",
        },
        "shared_flags": True,
        "examples": ["./sim suite", "./sim suite --seeds 1,2,3 --tier smoke"],
        "takes_sim": False,
    },
    "list": {
        "group": "DISCOVER",
        "summary": "catalog: files + one-line descriptions",
        "usage": ["./sim list [--json] [scenarios|tests|sims|stacks] ..."],
        "description": "List scenario/test/sim files with their one-line "
                       "header-comment descriptions (default: "
                       "scenarios+tests); stacks lists the dev stacks "
                       "(de_servers, de_webclient) and their procs. "
                       "--task/--kind/--tier filter with AND (kind is "
                       "inferred from the path: test|demo|example|"
                       "challenge).",
        "flags": {
            "--json": "machine-readable json (rows carry kind + tasks)",
            "--task": "only files tagged `tasks: [<id>]` (e.g. SL-0099)",
            "--kind": "only files of a kind "
                      "(test|demo|example|challenge)",
            "--tier": "only files marked `tier: <name>`",
        },
        "shared_flags": False,
        "examples": ["./sim list", "./sim list tests", "./sim list sims",
                     "./sim list --task SL-0099"],
        "takes_sim": False,
    },
    "new": {
        "group": "COMPOSE",
        "summary": "scaffold scenarios/<name>.yaml from the template",
        "usage": ["./sim new <name> [--sensor]"],
        "description": "Scaffold scenarios/<name>.yaml from "
                       "templates/scenario.yaml; --sensor also writes "
                       "blocks/<name>/ a world-driven sensor stub + "
                       "contract (docs/SENSORS.md).",
        "flags": {
            "--sensor": "also scaffold a world-driven sensor block",
        },
        "shared_flags": False,
        "examples": ["./sim new my_scenario", "./sim new my_sensor "
                     "--sensor"],
        "takes_sim": False,
    },
    "validate": {
        "group": "COMPOSE",
        "summary": "schema + referenced-file check, starts nothing",
        "usage": ["./sim validate <file.yaml> [--json]"],
        "description": "Validate a scenario/sim yaml against the schema and "
                       "check referenced files exist. Starts nothing.",
        "flags": {
            "--json": "machine-readable json",
        },
        "shared_flags": True,
        "examples": ["./sim validate tests/smoke_copters.yaml",
                     "./sim validate scenarios/demos/x.yaml --json"],
        "takes_sim": True,
    },
    "expand": {
        "group": "COMPOSE",
        "summary": "print the chaos: schedule a run would execute",
        "usage": ["./sim expand <file.yaml> [--seed N] [--json]"],
        "description": "Print the concrete chaos: schedule a run would "
                       "execute (concrete fault steps inlined; same seed, "
                       "same schedule). Runs nothing.",
        "flags": {
            "--seed": "override the file's seed: with N",
            "--json": "machine-readable json",
        },
        "shared_flags": False,
        "examples": ["./sim expand tests/faults/link_loss.yaml",
                     "./sim expand tests/faults/link_loss.yaml --seed 2"],
        "takes_sim": True,
    },
    "record": {
        "group": "CAPTURE",
        "summary": "capture a live webclient session as a scenario",
        "usage": ["./sim record <sim|live-sim> -o <file.yaml>"],
        "description": "Capture a live webclient session through the real "
                       "server's command_plugin seam and write it as a "
                       "de.scenario/1 scenario (+ .md runbook stub). "
                       "--record <file> on run/view records the window "
                       "around a scenario run or its --hold.",
        "flags": {
            "-o": "output scenario file (required)",
        },
        "shared_flags": True,
        "examples": ["./sim record copters3 -o scenarios/my.yaml",
                     "./sim record copters3 -o scenarios/my.yaml"],
        "takes_sim": True,
    },
    "studio": {
        "group": "LIVE",
        "summary": "the human session in one command",
        "usage": ["./sim studio <sim.yaml> [--world <w>] [--record <file>] "
                 "[--down]"],
        "description": "The human session in one command: the sim up (reuse "
                       "if warm), the webclient planning mode opened in the "
                       "browser, the de.worldstream/1 feed + Mavlink3DMap "
                       "viewer when available (studio still runs without it), "
                       "the session recorded through the record seam; Ctrl-C "
                       "finalizes the recording and leaves the sim up unless "
                       "--down.",
        "flags": {
            "--world": "seeded ground-truth world (worlds/<name>.yaml)",
            "--record": "record the session to <file>",
            "--down": "tear the sim down on Ctrl-C",
        },
        "shared_flags": True,
        "examples": ["./sim studio copters3", "./sim studio copters3 "
                     "--world canopy --record out.yaml --down"],
        "takes_sim": True,
    },
    "view": {
        "group": "LIVE",
        "summary": "sim + Mavlink3DMap viewer (= run --hold)",
        "usage": ["./sim view <scenario.yaml> [--record <file>] "
                 "[--open-web]"],
        "description": "One command for sim + Mavlink3DMap viewer: = run "
                       "--hold (the scenario needs render: {mode: viewer}); "
                       "same shared/exclusive rules as run.",
        "flags": {
            "--record": "record the session window to <file>",
            "--open-web": "xdg-open the webclient link once the sim is up",
        },
        "shared_flags": True,
        "examples": ["./sim view scenarios/demos/world_canopy_viewer.yaml",
                     "./sim view scenarios/demos/x.yaml --open-web"],
        "takes_sim": True,
    },
    "logs": {
        "group": "LIVE",
        "summary": "tail a process log",
        "usage": ["./sim logs [-s <sim>] <unit|server> [proc]"],
        "description": "Tail a process log. The owner is a unit name, "
                       "`server`, or a dev stack (`de_servers` / "
                       "`de_webclient`). An optional proc narrows to one "
                       "module log.",
        "flags": {},
        "shared_flags": False,
        "examples": ["./sim logs copter1 de_comm", "./sim logs de_servers",
                     "./sim logs -s copters3 copter1"],
        "takes_sim": False,
    },
    "doctor": {
        "group": "DISCOVER",
        "summary": "machine-readiness check",
        "usage": ["./sim doctor [sim.yaml] [--json] [--esp32]"],
        "description": "Machine-readiness check: one OK/MISSING/OPTIONAL line "
                       "per item with a fix line; a sim file scopes what is "
                       "required to what that sim needs; --esp32 makes the "
                       "ESP32 HIL rows required and probes the board.",
        "flags": {
            "--json": "machine-readable json",
            "--esp32": "make ESP32 HIL rows required and probe the board",
        },
        "shared_flags": False,
        "examples": ["./sim doctor", "./sim doctor copters3.yaml --json"],
        "takes_sim": False,
    },
    "esp32": {
        "group": "HARDWARE",
        "summary": "the tools/hil/ ESP32 firmwares",
        "usage": ["./sim esp32 <list|build|upload|check> [fw] [--port DEV] "
                 "[--baud N] [--no-check]"],
        "description": "The tools/hil/ ESP32 firmwares (default "
                       "esp32_fake_fc, a fake ArduCopter for a phone's "
                       "USB-OTG FCB link): upload = build + flash + run the "
                       "firmware's check_*.py.",
        "flags": {
            "--port": "serial device for upload",
            "--baud": "baud rate for upload",
            "--no-check": "skip the firmware's check_*.py after upload",
        },
        "shared_flags": False,
        "examples": ["./sim esp32 list", "./sim esp32 upload --port "
                     "/dev/ttyUSB0"],
        "takes_sim": False,
    },
    "verbs": {
        "group": "DISCOVER",
        "summary": "the de.scenario/1 verb registry",
        "usage": ["./sim verbs [--json]"],
        "description": "The de.scenario/1 verb registry: every do: step, "
                       "do: probe, fault: kind and assert: check with its "
                       "args, a one-line doc and an example.",
        "flags": {
            "--json": "machine-readable json",
        },
        "shared_flags": False,
        "examples": ["./sim verbs", "./sim verbs --json"],
        "takes_sim": False,
    },
    "clean": {
        "group": "HOUSE",
        "summary": "prune var/run/ and var/results/",
        "usage": ["./sim clean [--days N] [--all] [--dry-run]"],
        "description": "Prune var/run/ and var/results/ (never a live sim; "
                       "default keeps 7 days).",
        "flags": {
            "--days": "keep the last N days (default 7)",
            "--all": "prune everything (downed sims too)",
            "--dry-run": "print what would be pruned without removing it",
        },
        "shared_flags": False,
        "examples": ["./sim clean", "./sim clean --days 30",
                     "./sim clean --dry-run"],
        "takes_sim": False,
    },
}


def overview():
    """Top-level help: the intro paragraph, the commands under their
    intent-group headers (SL-0135), then the shared-flags section and
    the trailing layout/machine notes. This is what `./sim`,
    `./sim help`, `./sim -h` and `./sim --help` print."""
    lines = [INTRO, ""]
    for group, blurb in GROUPS:
        lines.append(f"{group} - {blurb}")
        for name, cmd in COMMANDS.items():
            if cmd["group"] != group:
                continue
            usage = cmd["usage"][0]
            summary = cmd["summary"]
            # pad the usage column so summaries align (like the old
            # docstring)
            pad = " " * max(2, 36 - len(usage))
            lines.append(f"  {usage}{pad}{summary}")
        lines.append("")
    lines.append("Shared flags (show/up/run/try/view/suite/validate/"
                 "record/studio):")
    for flag, help_text in SHARED_FLAGS.items():
        pad = " " * max(2, 20 - len(flag))
        lines.append(f"  {flag}{pad}{help_text}")
    lines.append("")
    lines.append(LAYOUT_NOTE)
    lines.append("")
    lines.append(MACHINE_NOTE)
    return "\n".join(lines)


def command_help(name):
    """Per-command help: usage line(s), the description, the command's
    own flags, the shared flags it accepts (if any), and examples. This
    is what `./sim help <command>` and `./sim <command> help` print."""
    if name not in COMMANDS:
        return f"unknown command {name!r} - ./sim help"
    cmd = COMMANDS[name]
    lines = []
    for u in cmd["usage"]:
        lines.append(f"usage: {u}")
    lines.append("")
    lines.append(cmd["description"])
    if cmd["flags"]:
        lines.append("")
        lines.append("flags:")
        for flag, help_text in cmd["flags"].items():
            pad = " " * max(2, 14 - len(flag))
            lines.append(f"  {flag}{pad}{help_text}")
    if cmd["shared_flags"]:
        lines.append("")
        lines.append("shared flags:")
        for flag, help_text in SHARED_FLAGS.items():
            pad = " " * max(2, 14 - len(flag))
            lines.append(f"  {flag}{pad}{help_text}")
    if cmd["examples"]:
        lines.append("")
        lines.append("examples:")
        for eg in cmd["examples"]:
            lines.append(f"  {eg}")
    return "\n".join(lines)


def allowed_flags(name):
    """The flag strings a command's bad-flag check validates against: the
    command's own flags plus the shared flags when it accepts them. Used
    by the `sim` front-end's _bad_flags() so a typo like --view fails
    instead of being silently ignored."""
    if name not in COMMANDS:
        return ()
    cmd = COMMANDS[name]
    flags = tuple(cmd.get("flags", {}))
    if cmd.get("shared_flags"):
        flags = flags + tuple(SHARED_FLAGS)
    return flags
