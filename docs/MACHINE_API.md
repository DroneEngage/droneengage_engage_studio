# MACHINE_API.md — `--json` output contract

> DroneEngageStudio doc. DroneEngage is the flagship integration, not the owner.

This is the agent-facing machine interface for `sim`. Every read-only
command takes `--json`. It emits one JSON document on stdout. Nothing
else goes to stdout. Without `--json`, the human output is unchanged.
`json.tool` and `jq` parse every document below.

General rules:

- Objects use snake_case keys. `null` means the field does not apply.
  Example: a unit with `fc: null` has no SITL.
- Ports and pids are integers. A `null` pid means that process is DOWN.
- Usage and schema errors still print text and exit non-zero. They are
  not JSON. Exit codes are listed per command. `0` means the answer is
  in the JSON body.

## `./sim show <sim.yaml> --json`

The resolved sim. It is the same data that `./sim show` prints as
text:

```json
{
  "file": "sims/copters3.yaml",
  "name": "copters3",
  "server": {"type": "real", "auth": false, "port": 20000,
             "control_port": 20001, "auth_port": null,
             "auth_s2s_port": null, "ip": "127.0.0.1",
             "auth_ip": "127.0.0.1", "gcs_party": "GCS_FLEET"},
  "slot_base": 40,
  "speedup": 1.0,
  "units": [
    {"name": "copter1", "block": "copter",
     "fc": {"vehicle": "copter", "model": "quad", "sysid": 40,
            "pilot_port": 20002, "fcb_port": 20012, "...": "..."},
     "modules": ["de_comm", "de_mavlink"],
     "ports": {"de_comm": 20000, "de_mavlink": 20001},
     "account": "unit1_or_null",
     "position": {"lat": ..., "lng": ...}}
  ]
}
```

`fc` is `null` for units with no flight controller. Ports are
allocator leases, not formulas. Before a sim has ever run, they show
`null`. At that point there are no live or remembered leases. Once
leased, they show the values `./sim ports` lists. Exit 0. Non-zero
exit with a `SimError` text when the sim does not resolve.

Camera bindings live under `units[].config.<proc>.camera.camera_list`.
There is one entry per `de_camera` instance (`de_camera`,
`de_camera2`, ...). A pool label gives `{"name": "CAM0", "key":
"cam_0", "device_name": "DE-CAM1"}`. A real index gives `"device_num":
N`. Instances listed via `cameras:` get suffix ports. The formula is
`port_base + 10*slot + 2*(i-1)`. A manifest
(`var/run/<sim>/sim.json`) also carries `units[].hot_added`. These are
procs created by `./sim ctl add_module`. The spec itself does not
declare them. They survive a same-hash `sim up` reuse. They are
dropped on any spec-change restart.

## `./sim sims --json`

Every generated sim, live or downed:

```json
{"sims": [{"name": "copters3", "units": 3, "server_type": "real",
           "procs_alive": 0}]}
```

The list is empty when nothing was generated. Always exit 0.

## `./sim status [live|all] --json` / `./sim ctl status --json`

This lists live sims. With `all` it lists all known sims. Process
state is included:

```json
{"sims": [
  {"name": "smoke_copters", "sim_file": "sims/copters3.yaml",
   "sim_hash": "84d4157c13874177", "live": true,
   "server": {"type": "real", "auth": false, "port": 20000,
              "control_port": 20001, "auth_port": null, "up": true},
   "procs": {"server.de_server": 240366, "server.gcs_bridge": 240377},
   "units": [
     {"name": "copter1", "block": "copter", "fc": {"...": "..."},
      "udp": 20000,
      "procs": {"sitl": 240388, "de_comm": 240389,
                "de_mavlink": 240397}}],
   "online": ["copter1", "copter2"],
   "webclient": "https://localhost:3000/...",
   "planning": "https://localhost:3000/planning?..."}
]}
```

- `online`, `webclient` and `planning` appear only while the sim's
  server proc is alive (`server.up`).
- An `external` server adds `server.conn`. Its value is
  `"external <ip>:<port> via auth <ip>:<port>"`.
- No live sims: `{"sims": [], "downed": <n>}`. Nothing generated at
  all: `{"sims": []}`. Always exit 0.

## `./sim list [roots...] --json`

This is the scenario, test and sim catalog:

```json
{"roots": [
  {"root": "scenarios",
   "files": [{"file": "scenarios/examples/link_fault.yaml",
              "description": "Runbook: ..."}]}]}
```

`roots` defaults to `scenarios` + `tests`. Always exit 0.

## `./sim validate <file.yaml> --json`

```json
{"file": "tests/fleet_bus/p4a08_targets.yaml", "ok": true, "problems": []}
```

`problems` lists every finding (`ok: false`). The exit code is `1`
when problems exist. The JSON is still printed first. Parse it before
you check the exit code.

## `./sim expand <file.yaml> [--seed N] --json`

The chaos-expanded scenario as a `de.scenario/1` object. It is the
human mode's yaml text in structured form:

```json
{"file": "tests/soak/soak_copters3.yaml", "name": "soak_copters3",
 "seed": 1,
 "expanded": {"version": 1, "name": "...", "timeline": [...]}}
```

It exits non-zero with text when the file has no `chaos:` block.

## `./sim doctor [sim.yaml] --json`

```json
[{"item": "de_comm", "status": "ok",
  "detail": "/path/bin/de_comm", "fix": ""}]
```

`status` is `ok` | `missing` | `optional`. Exit `1` when any required
item is missing.

## `./sim verbs --json`

The de.scenario/1 verb registry (engine/registry.py). It lists every
`do:` step, `do:` probe, `fault:` kind and `assert:` check:

```json
{"verbs": [{"name": "wait_journal", "kind": "probe",
            "args": {"required": ["unit", "re"],
                     "optional": ["timeout", "any"]},
            "doc": "block until the unit's journal matches re ...",
            "example": "{t: 15, do: wait_journal, unit: c1, re: '...'}"}]}
```

`kind` is `step` | `probe` | `fault` | `check`. `./sim validate`
rejects a `do:`, `fault:` or `assert:` name missing from this list.
It also rejects a known verb's missing-required or unknown args. The
error names the verb and its expected args. `ci/check_verbs.py`
asserts that every verb used under `tests/` and `scenarios/` is
registered and documented.

## `./sim run <file.yaml>` — the `RESULT` line + summary.json

The **last stdout line** of a run is always:

```
RESULT /abs/path/var/results/<ts>-<name>/summary.json
```

That file is the full run report:

```json
{"scenario": "smoke_copters", "file": "...yaml", "seed": 1,
 "started_utc": "2026-10-12T15:48:10Z", "wall_s": 240.1,
 "commits": {"droneengage_studio": "...", "...": "..."},
 "speedup": {...}, "speedup_requested": 1,
 "realtime": {"required": false, "ok": true, "note": ""},
 "step_ok": true,
 "assertions": [{"name": "journal_count ...", "status": "PASS",
                 "value": ..., "detail": "..."}],
 "counts": {"pass": 8, "fail": 0, "skip": 0},
 "exit": 0}
```

- `assertions[].status` is `PASS` | `FAIL` | `SKIP`. `value` and
  `detail` are the measured value and the explanation. They may be
  `null` or `""`.
- `exit` mirrors the process exit code. `0` means every step and
  assertion passed. The directory also holds `junit.xml`, per-unit
  journals and logs.
- A run can stop before its timeline ends. Examples: verifier failed,
  sim bring-up failed. The run still writes `summary.json` with
  `"exit": 1`, an `"error"` string and empty `assertions`. It still
  ends with the `RESULT` line. A scenario file that fails to load
  exits non-zero with no `RESULT`. Run `./sim validate --json` first.
- `./sim suite` writes the same `summary.json` per run. It also
  writes a junit report under `var/results/nightly/`.

## `./sim prelaunch` — `report.json`

`./sim prelaunch --target <t> [--stage 0,2] [--version <v>]` writes
`var/results/prelaunch/<version>/<target>/report.json` and
`report.md`. With `--json` it also prints `report.json` on stdout.
`--list --json` prints the feature register. It has no side effects.

```json
{
  "version": "1.2.0",
  "target": "airgap",
  "role": "airgap",
  "required_stages": ["0", "1", "install", "2", "3", "4"],
  "updated": "2026-11-05T13:00:00Z",
  "verdict": "NO-GO",
  "override": null,
  "features": [{"id": "udp_proxy_qgc", "title": "...", "severity": "P0",
                "status": "FAIL", "checks": {"relay.range_edges": "FAIL"}}],
  "checks": {"relay.range_edges": {"status": "FAIL", "detail": "...",
             "stage": "3", "kind": "probe", "title": "...",
             "at": "2026-11-05T13:00:00Z", "wall_s": 12.1}},
  "versions": {"de_comm": "abc1234", "de_udp": "def5678+dirty"}
}
```

`verdict` is `GO` when every P0 row in the required stages is PASS.
`verdict` is `NO-GO` on a P0 FAIL or SKIP. `verdict` is `INCOMPLETE`
when a required P0 check has not run. `override` is `{reason, at}`
when a human accepted a no-go. `verdict` keeps its real value.
Results of the same version and target merge between runs. A later
`--stage 4` or `--signoff` adds to the report. `--fresh` starts again.

Exit code is `0` when every P0 row of the stages this run covered is
PASS. It is also `0` when an override exists. Otherwise it is `1`.
With only `--signoff` or `--override`, `0` means the verdict is `GO`.
