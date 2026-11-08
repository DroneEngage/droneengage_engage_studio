"""registry.py - the de.scenario/1 verb registry (TASK-21 / SL-0024).

Every timeline `do:` step, `fault:` kind and `assert:` name is declared
once here with its kind, args, a one-line doc and an example.
scenario.py's _validate refuses unknown verbs and wrong args through
this table, `./sim verbs [--json]` prints it, and ci/check_verbs.py
keeps tests//scenarios usage and the docs/SCENARIOS.md verb reference
in sync with it. The runner/scenario_runner dispatch is unchanged -
the registry describes it, it does not replace it.

    V("arm", "step", ["unit"], doc="arm the unit(s)",
      eg="{t: 6, do: arm, unit: c1}")

kind: step  - does something (deploy, arm, kill, mode, ...)
      probe - observes only (reads journals / logs / telemetry / server
              state; changes nothing on the fleet)
      fault - a `fault:` kind (timeline and chaos.faults)
      check - an `assert:` name

`req`/`opt` list the verb's own args. A step's structural keys
(t/on/do/fault/once) and a fault step's `dur_s`/`seconds` window are
never listed - `dur_s` is implicit on every fault. `free: True` marks
verbs whose args are a free-form message body (send/event/write_file/
opmsg) - validate then only checks the name.
"""


def V(name, kind, req=(), opt=(), doc="", eg=None, free=False):
    return {"name": name, "kind": kind, "req": list(req),
            "opt": list(opt), "doc": doc,
            "eg": eg if eg is not None else {}, "free": free}


VERBS = [
    # ------------------------------------------------ steps (do: verbs)
    V("deploy", "step", opt=["operation", "package", "unit", "binding",
                             "authorize", "timeout"],
      doc="push an op/package to the fleet (1094 spec) or one unit",
      eg="{t: 0, do: deploy}"),
    V("prepare", "step", opt=["operation", "package", "unit", "timeout"],
      doc="PREPARE an op without starting it (1094)",
      eg="{t: 0, do: prepare}"),
    V("start", "step",
      opt=["lead_ms", "in_ms", "t_start", "run", "start", "targets",
           "expect_fail", "timeout", "mission"],
      doc="start the deployed run (1094 op start, or ctl mission start)",
      eg="{t: 9, do: start}"),
    V("abort", "step", opt=["run", "why"],
      doc="abort the running op (1094)", eg="{t: 60, do: abort}"),
    V("opstart", "step", opt=["run", "lead_ms", "in_ms", "t_start",
                              "start", "targets"],
      doc="fleet START over the 1094 op channel (ctl opstart)",
      eg="{t: 9, do: opstart}"),
    V("opabort", "step", opt=["run", "why"],
      doc="fleet ABORT over the 1094 op channel",
      eg="{t: 60, do: opabort, why: abort probe}"),
    V("authorize", "step", req=["unit"],
      opt=["valid_for_s", "max_launches", "timeout"],
      doc="send/replace a unit's launch grant mid-run (1094)",
      eg="{t: 30, do: authorize, unit: c1, max_launches: 1}"),
    V("revoke_launch", "step", req=["unit"], opt=["timeout"],
      doc="void the unit's launch grant (1094)",
      eg="{t: 60, do: revoke_launch, unit: c1}"),
    V("operator", "step", req=["act"],
      opt=["unit", "req", "kind", "tid", "event", "payload", "mission",
           "hold", "wait_s", "package", "binding", "timeout", "run",
           "why", "lead_ms", "in_ms", "t_start", "start", "targets",
           "expect_fail"],
      doc="act as the operator: approve/deny/confirm/dismiss/fire/"
          "activate/pause/resume/stop/clear/deploy/start/abort",
      eg="{on: {unit: c1, re: 'APPROVAL_REQ'}, do: operator, "
         "act: deny, once: true}"),
    V("arm", "step", opt=["unit"], doc="arm the unit(s)",
      eg="{t: 6, do: arm, unit: c1}"),
    V("disarm", "step", opt=["unit"], doc="disarm the unit(s)",
      eg="{t: 90, do: disarm, unit: all}"),
    V("land", "step", opt=["unit"], doc="land the unit(s)",
      eg="{t: 120, do: land, unit: all}"),
    V("reset", "step", opt=["unit"],
      doc="reset the unit(s): SITL + de_comm + modules restart together",
      eg="{t: 0, do: reset, unit: c1}"),
    V("journal", "step", opt=["unit"],
      doc="ask de_comm to reopen its journal sink",
      eg="{t: 30, do: journal, unit: c1}"),
    V("kick", "step", opt=["unit"],
      doc="kick the unit's server session (it re-dials)",
      eg="{t: 20, do: kick, unit: c1}"),
    V("link_clear", "step", opt=["unit"],
      doc="clear every link fault knob on the unit(s)",
      eg="{t: 40, do: link_clear, unit: all}"),
    V("fc", "step", opt=["unit"],
      doc="sync the unit's FC link state (ctl fc)",
      eg="{t: 5, do: fc, unit: c1}"),
    V("mode", "step", req=["unit", "mode"],
      doc="set the FC mode (GUIDED, AUTO, RTL, ...)",
      eg="{t: 0, do: mode, unit: c1, mode: GUIDED}"),
    V("takeoff", "step", opt=["unit", "alt"],
      doc="NAV_TAKEOFF, waits until ~80% of alt (default 10)",
      eg="{t: 8, do: takeoff, unit: c1, alt: 20}"),
    V("param", "step", req=["unit", "name", "value"],
      doc="verified SITL param set (PARAM_VALUE echo)",
      eg="{t: 0, do: param, unit: c1, name: SIM_GPS1_ENABLE, value: 0}"),
    V("param_get", "probe", req=["unit", "name"],
      doc="live param read-back off the FC",
      eg="{t: 5, do: param_get, unit: c1, name: BATT_CAPACITY}"),
    V("wait", "step", req=["seconds"], doc="sleep the timeline",
      eg="{t: 30, do: wait, seconds: 5}"),
    V("wait_journal", "probe", req=["unit", "re"],
      opt=["timeout", "any"],
      doc="block until the unit's journal matches re (a re-matched "
          "line's groups become {M1}..{M9} for later steps)",
      eg="{t: 15, do: wait_journal, unit: c1, re: 'RUN run .* started',"
         " timeout: 60}"),
    V("wait_log", "probe", req=["unit", "module", "re"], opt=["timeout"],
      doc="wait_journal on a module's stdout log, not the journal",
      eg="{t: 10, do: wait_log, unit: c1, module: de_precland, "
         "re: 'PRECLAND state'}"),
    V("wait_ready", "probe", opt=["unit", "timeout"],
      doc="wait for the unit(s) to report ready on the bus",
      eg="{t: 0, do: wait_ready, unit: all, timeout: 120}"),
    V("wait_online", "probe", opt=["unit", "timeout"],
      doc="wait until the unit(s) show online to the GCS",
      eg="{t: 0, do: wait_online, unit: s1, timeout: 60}"),
    V("opstatus", "probe", opt=["seconds"],
      doc="dump the fleet op state (1094 status, `seconds` window)",
      eg="{t: 40, do: opstatus, seconds: 8}"),
    V("opmsg", "probe", free=True,
      opt=["tg", "to", "mt", "ms", "expect", "secs", "seconds"],
      doc="inject one 1094 frame at tg and check replies (expect re)",
      eg="{t: 20, do: opmsg, tg: c1, mt: 1094, ms: {a: status}, "
         "expect: 'RUN'}"),
    V("beacon", "probe", opt=["unit", "seconds", "min"],
      doc="count the unit's mt=1004 presence beacons on a fresh GCS "
          "listen (bounded comms evidence inside a fault window)",
      eg="{t: 40, do: beacon, unit: c1, seconds: 5, min: 1}"),
    V("roster_peers", "probe", opt=["unit", "count", "seconds"],
      doc="print each unit's roster peers (server-side)",
      eg="{t: 30, do: roster_peers}"),
    V("links", "probe", opt=["unit"],
      doc="print the server link table (loss/delay/cut state)",
      eg="{t: 30, do: links}"),
    V("reboot", "step", opt=["unit"],
      doc="de_comm restart under a fresh boot id",
      eg="{t: 45, do: reboot, unit: c1}"),
    V("stop", "step", opt=["mission"],
      doc="mission control STOP (ctl)", eg="{t: 50, do: stop}"),
    V("pause", "step", opt=["mission"],
      doc="mission control PAUSE", eg="{t: 30, do: pause}"),
    V("resume", "step", opt=["mission"],
      doc="mission control RESUME", eg="{t: 40, do: resume}"),
    V("clear", "step", opt=["mission"],
      doc="mission control CLEAR", eg="{t: 5, do: clear}"),
    V("kill", "step", opt=["unit", "proc"],
      doc="SIGKILL unit procs (proc defaults all; `server` works too)",
      eg="{t: 30, do: kill, unit: c1, proc: de_mavlink}"),
    V("restart", "step", opt=["unit", "proc"],
      doc="restart unit procs", eg="{t: 36, do: restart, unit: c1}"),
    V("freeze", "step", opt=["unit", "proc", "module", "dur_s"],
      doc="SIGSTOP a proc (alive but silent; dur_s self-resumes)",
      eg="{t: 20, do: freeze, unit: c1, proc: de_comm, dur_s: 30}"),
    V("unfreeze", "step", opt=["unit", "proc", "module"],
      doc="SIGCONT a frozen proc", eg="{t: 50, do: unfreeze, unit: c1}"),
    V("delay", "step", req=["unit", "ms"],
      doc="one-shot link delay (fake transport or real+link_proxy)",
      eg="{t: 10, do: delay, unit: c1, ms: 500}"),
    V("loss", "step", req=["unit", "percent"],
      doc="set message-level link loss % (fake transport only)",
      eg="{t: 3, do: loss, unit: s1, percent: 100}"),
    V("link_cut", "step", req=["unit", "seconds"],
      doc="sever the unit's server link for seconds",
      eg="{t: 10, do: link_cut, unit: c1, seconds: 30}"),
    V("commline", "step", req=["unit", "seconds"],
      doc="in-band comm-line drop for seconds (any transport)",
      eg="{t: 10, do: commline, unit: c1, seconds: 30}"),
    V("link", "step", req=["unit", "set"],
      doc="raw link-model sub-line(s): {set: 'ge 0.05 0.2 0 90 stall'} "
          "or a list; fake transport or real+link_proxy (loss* subs "
          "stay fake-only)",
      eg="{t: 10, do: link, unit: c1, set: 'stall 30'}"),
    V("write_file", "step", free=True,
      opt=["unit", "module", "path", "json", "also", "delay_ms"],
      doc="write a json file into a unit's config dir (module: scopes "
          "it; also: [] extra files; delay_ms paces the write)",
      eg="{t: 0, do: write_file, unit: c1, module: de_comm, "
         "path: extra.json, json: {a: 1}}"),
    V("udp_chunks", "step", req=["unit", "module"],
      opt=["drop_pct", "reorder", "dup_pct", "seed", "clear"],
      doc="knobs on the unit's udp_proxy chunk fault line (needs "
          "udp_proxy: on the unit)",
      eg="{t: 20, do: udp_chunks, unit: c1, module: de_mavlink, "
         "drop_pct: 30}"),
    V("server_up", "step",
      doc="relaunch the sim's server procs",
      eg="{t: 60, do: server_up}"),
    V("mission_save", "step", req=["unit", "name"],
      opt=["missionId", "data"],
      doc="save a mission through the comm server's storage link "
          "(server: {type: real, storage: true}; system message 9011)",
      eg="{t: 10, do: mission_save, unit: c1, name: my_mission, "
         "missionId: m1, data: {waypoints: []}}"),
    V("mission_load", "step", req=["unit"],
      opt=["missionId", "expect"],
      doc="load a mission back from storage (9010); expect: a regex "
          "some reply must match",
      eg="{t: 15, do: mission_load, unit: c1, missionId: m1, "
         "expect: my_mission}"),
    V("clock_offset", "step", req=["ms"],
      doc="shift the fake server's stamped fleet clock by ms",
      eg="{t: 5, do: clock_offset, ms: 3600000}"),
    V("fire", "step", req=["event"],
      opt=["unit", "payload", "until"],
      doc="fire a bus event (until: {re, count, tries, every_s} waits "
          "for the journal echo)",
      eg="{t: 10, do: fire, unit: c1, event: my.event}"),
    V("event", "step", req=["event"],
      doc="fire a GCS event on the ctl session",
      eg="{t: 10, do: event, event: my.event}"),
    V("send", "step", free=True, opt=["ty", "mt", "tg", "ms"],
      doc="send one raw andruav frame {ty, mt, tg, ms} (PKG:/HASH: "
          "placeholders expand)",
      eg="{t: 5, do: send, ty: i, mt: 1085, tg: c1, ms: {...}}"),
    V("fcb_cut", "step", req=["unit"], opt=["seconds", "dur_s"],
      doc="sever the de_mavlink<->SITL link via the fcb proxy",
      eg="{t: 10, do: fcb_cut, unit: c1, seconds: 30}"),
    V("fcb_stall", "step", req=["unit", "ms"],
      doc="hold the fcb link silent for ms",
      eg="{t: 10, do: fcb_stall, unit: c1, ms: 5000}"),
    V("fcb_corrupt", "step", req=["unit"],
      opt=["drop_pct", "flip_pct", "seed"],
      doc="seeded frame-aware noise on the fcb link",
      eg="{t: 0, do: fcb_corrupt, unit: c1, drop_pct: 1, seed: 1}"),
    V("fcb_drop", "step", req=["unit"], opt=["msgid", "percent",
                                            "seconds"],
      doc="drop one MAVLink msgid on the fcb link",
      eg="{t: 10, do: fcb_drop, unit: c1, msgid: all}"),
    V("fcb_inject", "step", req=["unit", "hex"],
      doc="inject a raw hex frame on the fcb link",
      eg="{t: 10, do: fcb_inject, unit: c1, hex: 'fe09...'}"),
    V("fcb_clear", "step", opt=["unit"],
      doc="remove the unit's fcb rules, end any cut/stall",
      eg="{t: 60, do: fcb_clear, unit: c1}"),
    V("fcb_stats", "probe", opt=["unit"],
      doc="the unit's fcb proxy counters + active rules",
      eg="{t: 60, do: fcb_stats, unit: c1}"),
    V("cpu_starve", "step", opt=["unit", "proc", "module", "duty_pct",
                                 "cycle_ms", "dur_s"],
      doc="SIGSTOP/SIGCONT duty cycle on a proc (SL-0014)",
      eg="{t: 10, do: cpu_starve, unit: c1, proc: de_comm, dur_s: 60}"),
    V("cpu_unstarve", "step", opt=["unit", "proc", "module"],
      doc="end a cpu_starve cycle early",
      eg="{t: 70, do: cpu_unstarve, unit: c1, proc: de_comm}"),
    V("journal_unwritable", "step", opt=["unit"],
      doc="make the unit's journal dir read-only (SL-0014)",
      eg="{t: 10, do: journal_unwritable, unit: c1}"),
    V("journal_writable", "step", opt=["unit"],
      doc="restore the unit's journal dir",
      eg="{t: 70, do: journal_writable, unit: c1}"),
    V("clock_step", "step", req=["unit", "offset_s"],
      doc="step a faketime:true unit's de_comm clock (SL-0014)",
      eg="{t: 5, do: clock_step, unit: c1, offset_s: 3600}"),

    # ------------------------------------------------- fault: kinds
    V("link_cut", "fault", opt=["unit"],
      doc="sever the server link (fake, or real+link_proxy - the "
          "proxy kills the session and accept-drops re-dials)",
      eg="{t: 60, fault: link_cut, unit: s1, dur_s: 30}"),
    V("link_stall", "fault", opt=["unit"],
      doc="hold every byte in order for the window (a buffered "
          "blackout - the socket stays up)",
      eg="{t: 60, fault: link_stall, unit: c1, dur_s: 30}"),
    V("link_profile", "fault", opt=["unit", "profile", "cmds"],
      doc="a link_model.PROFILES name (lte_edge, lte_stall, "
          "uplink_dead, dial_up) and/or raw link cmds; loss-knob "
          "profiles stay fake-only",
      eg="{t: 60, fault: link_profile, unit: c1, profile: lte_stall}"),
    V("loss", "fault", opt=["unit", "percent"],
      doc="seeded message-level packet loss (fake transport only)",
      eg="{t: 3, fault: loss, unit: s1, percent: 100, dur_s: 20}"),
    V("delay", "fault", opt=["unit", "ms"],
      doc="link delay (fake, or real+link_proxy)",
      eg="{t: 30, fault: delay, unit: c1, ms: 500, dur_s: 60}"),
    V("gcs_disconnect", "fault",
      doc="drop the operator's GCS session",
      eg="{t: 60, fault: gcs_disconnect, dur_s: 30}"),
    V("server_down", "fault", opt=["proc"],
      doc="kill the server proc(s)",
      eg="{t: 60, fault: server_down, dur_s: 15}"),
    V("kill", "fault", opt=["unit", "proc"],
      doc="SIGKILL unit processes (dur_s restarts them)",
      eg="{t: 40, fault: kill, unit: c1, proc: all}"),
    V("restart", "fault", opt=["unit", "proc"],
      doc="restart unit processes",
      eg="{t: 40, fault: restart, unit: c1, proc: de_comm}"),
    V("module_kill", "fault", opt=["unit", "module"],
      doc="kill one module proc (dur_s restarts it)",
      eg="{t: 30, fault: module_kill, unit: c1, module: de_mavlink}"),
    V("freeze", "fault", opt=["unit", "proc", "module"],
      doc="SIGSTOP a proc - a hang, not a crash; dur_s SIGCONTs",
      eg="{t: 20, fault: freeze, unit: c1, proc: de_comm, dur_s: 30}"),
    V("param", "fault", req=["unit", "set"],
      doc="FC param write with read-before-set; dur_s restores the "
          "read values",
      eg="{t: 20, fault: param, unit: c1, set: {SIM_GPS1_ENABLE: 0},"
         " dur_s: 60}"),
    V("battery", "fault",
      opt=["unit", "volts", "cap_mah", "res_ohm", "ok_volts",
           "ok_cap_mah"],
      doc="FC preset: SIM_BATT_VOLTAGE (+BATT_CAPACITY/SIM_BATT_RES_OHM)",
      eg="{t: 60, fault: battery, unit: c1, volts: 10.5}"),
    V("gps_off", "fault", opt=["unit", "fixtype"],
      doc="FC preset: SIM_GPS1_FIXTYPE (0 = no fix)",
      eg="{t: 30, fault: gps_off, unit: c1, dur_s: 60}"),
    V("gps_jam", "fault", opt=["unit", "jam"],
      doc="FC preset: SIM_GPS1_JAM - sats/lock/velocity go random",
      eg="{t: 30, fault: gps_jam, unit: c1, dur_s: 60}"),
    V("gps_glitch", "fault", opt=["unit", "x", "y", "z"],
      doc="FC preset: SIM_GPS1_GLTCH_* position offset (40/0/0 m)",
      eg="{t: 30, fault: gps_glitch, unit: c1, dur_s: 60}"),
    V("gps_degrade", "fault", opt=["unit", "sats", "noise"],
      doc="FC preset: SIM_GPS1_NUMSATS + SIM_GPS1_NOISE",
      eg="{t: 30, fault: gps_degrade, unit: c1, dur_s: 60}"),
    V("motor_out", "fault", opt=["unit", "motors", "mul"],
      doc="FC preset: SIM_ENGINE_FAIL + SIM_ENGINE_MUL (bitmask 1)",
      eg="{t: 40, fault: motor_out, unit: c1, dur_s: 30}"),
    V("rc_fail", "fault", opt=["unit", "how"],
      doc="FC preset: SIM_RC_FAIL (1 no pulses, 2 throttle-950, "
          "3 failsafe bit)",
      eg="{t: 40, fault: rc_fail, unit: c1, dur_s: 30}"),
    V("compass_fail", "fault",
      opt=["unit", "how", "ofs_x", "ofs_y", "ofs_z", "noise"],
      doc="FC preset: SIM_MAG1_FAIL + optional offsets/noise",
      eg="{t: 40, fault: compass_fail, unit: c1, dur_s: 30}"),
    V("vibration", "fault", opt=["unit", "freq", "amp", "noise"],
      doc="FC preset: SIM_VIB_FREQ_* + SIM_VIB_MOT_MAX + SIM_ACC1_RND "
          "(bites only with the motors on)",
      eg="{t: 40, fault: vibration, unit: c1, dur_s: 30}"),
    V("server_clock", "fault", opt=["offset_ms"],
      doc="skew the fleet clock (fake transport only)",
      eg="{t: 10, fault: server_clock, offset_ms: -3600000}"),
    V("no_time_sync", "fault", opt=["unit"],
      doc="units stop carrying `ts` (fake transport only)",
      eg="{t: 10, fault: no_time_sync, unit: c1, dur_s: 60}"),
    V("rc_override", "fault", opt=["unit", "chan", "channels", "pwm"],
      doc="RC_CHANNELS_OVERRIDE on the FC",
      eg="{t: 30, fault: rc_override, unit: c1, chan: 3, pwm: 1600,"
         " dur_s: 15}"),
    V("mode", "fault", req=["unit", "mode"],
      doc="force an FC mode (dur_s returns to the scripted mode)",
      eg="{t: 60, fault: mode, unit: c1, mode: RTL, dur_s: 30}"),
    V("fcb_cut", "fault", opt=["unit"],
      doc="sever the de_mavlink<->SITL link via the fcb proxy "
          "(needs fcb_proxy: true)",
      eg="{t: 30, fault: fcb_cut, unit: c1, dur_s: 30}"),
    V("fcb_stall", "fault", opt=["unit", "ms"],
      doc="hold the fcb link silent (sockets stay open)",
      eg="{t: 30, fault: fcb_stall, unit: c1, dur_s: 10}"),
    V("fcb_rate", "fault", opt=["unit", "baud"],
      doc="UART pacing on the fcb link: baud/10 bytes/s per direction "
          "(SL-0101); dur_s clears it",
      eg="{t: 10, fault: fcb_rate, unit: c1, baud: 9600}"),
    V("fcb_corrupt", "fault",
      opt=["unit", "byte_drop_pct", "bit_flip_pct", "seed"],
      doc="seeded frame-aware noise on the fcb link",
      eg="{t: 0, fault: fcb_corrupt, unit: c1, byte_drop_pct: 1}"),
    V("fcb_unplug", "step", req=["unit"],
      doc="pull the serial device an `fcb_link` unit's client sits "
          "on - one-shot, the fault: form's dur_s replugs on a "
          "timer instead (SL-0103)",
      eg="{t: 30, do: fcb_unplug, unit: c1}"),
    V("fcb_plug", "step", req=["unit"],
      doc="replug the serial bridge an fcb_unplug killed (same link "
          "path/channel)",
      eg="{t: 60, do: fcb_plug, unit: c1}"),
    V("fcb_unplug", "fault", opt=["unit"],
      doc="pull the serial device an `fcb_link` unit's de_mavlink sits "
          "on (SL-0103); dur_s replugs it",
      eg="{t: 30, fault: fcb_unplug, unit: c1, dur_s: 10}"),
    V("cpu_starve", "fault",
      opt=["unit", "proc", "module", "duty_pct", "cycle_ms"],
      doc="SIGSTOP/SIGCONT duty cycle on a proc (SL-0014)",
      eg="{t: 10, fault: cpu_starve, unit: c1, proc: de_comm,"
         " dur_s: 60}"),
    V("journal_unwritable", "fault", opt=["unit"],
      doc="the unit's journal dir goes read-only (SL-0014); dur_s "
          "restores, sim down always restores",
      eg="{t: 10, fault: journal_unwritable, unit: c1, dur_s: 60}"),
    V("clock_step", "fault", req=["unit", "offset_s"],
      doc="step a `faketime: true` unit's de_comm clock by signed "
          "seconds via FAKETIME_TIMESTAMP_FILE (SL-0014)",
      eg="{t: 5, fault: clock_step, unit: c1, offset_s: 3600}"),
    V("udp_chunks", "fault", req=["unit", "module"],
      opt=["drop_pct", "reorder", "dup_pct", "seed"],
      doc="seeded chunk faults on the module<->de_comm s2s proxy "
          "(any transport; needs udp_proxy:)",
      eg="{t: 20, fault: udp_chunks, unit: c1, module: de_mavlink,"
         " drop_pct: 30, dur_s: 60}"),

    # ------------------------------------------------- assert: checks
    V("journal_count", "check", req=["re"],
      opt=["unit", "min", "total"],
      doc="journal line count matching re",
      eg="{journal_count: {unit: all, re: 'BUS_IN x', eq: 1}}"),
    V("module_log_count", "check", req=["module", "re"],
      opt=["unit", "total"],
      doc="journal_count on a module's stdout log",
      eg="{module_log_count: {unit: c1, module: de_mavlink, "
         "re: 'x', at_least: 1}}"),
    V("no_journal", "check", req=["re"], opt=["unit"],
      doc="the pattern never appears in the journal",
      eg="{no_journal: {unit: all, re: 'comm line offline'}}"),
    V("journal_order", "check", req=["first", "then"],
      opt=["unit", "proc"],
      doc="ordering of two journal patterns (`proc` reads the module "
          "stdout instead)",
      eg="{journal_order: {unit: c1, first: 'a', then: 'b'}}"),
    V("journal_time_delta", "check", req=["u1", "re1", "u2", "re2"],
      opt=["max_s", "min_s", "ordered", "any"],
      doc="timing between a journal line on u1 and one on u2",
      eg="{journal_time_delta: {u1: c1, re1: 'a', u2: c2, re2: 'b',"
         " max_s: 5}}"),
    V("log_match_delta", "check",
      req=["unit", "module", "re1", "re2"],
      opt=["min_s", "max_s"],
      doc="elapsed between the runner observing two wait_log/"
          "wait_journal matches on the unit (module 'journal' reads "
          "the journal); proves a paced path really took the time",
      eg="{log_match_delta: {unit: c1, module: de_mavlink,"
         " re1: 'TCP Client connected', re2: 'Parameter LOADED',"
         " min_s: 30}}"),
    V("clock_spread", "check", opt=["unit", "max_ms", "min_ms"],
      doc="spread of the units' latest CLOCK fleet_minus_unix_ms "
          "(min_ms proves a clock_step landed)",
      eg="{clock_spread: {max_ms: 100}}"),
    V("mission_current", "check", req=["unit"],
      opt=["seq", "equals", "at_least"],
      doc="the FC's current mission seq",
      eg="{mission_current: {unit: c1, at_least: 2}}"),
    V("distance_to", "check", req=["unit", "re"],
      opt=["truth", "truth_file", "detected_cls", "min_m", "max_m"],
      doc="a journal event's position vs world truth",
      eg="{distance_to: {unit: c1, re: 'release', truth: person_1,"
         " max_m: 15}}"),
    V("payload_landed", "check",
      opt=["truth", "pos", "max_m", "max_impact_mps", "hit", "unit"],
      doc="landing point vs truth/a fixed point (a `hit` mismatch "
          "fails with the reason)",
      eg="{payload_landed: {truth: person_1, max_m: 5}}"),
    V("drop_drift", "check", req=["unit"], opt=["max_m"],
      doc="release -> landing distance",
      eg="{drop_drift: {unit: c1, max_m: 8}}"),
    V("no_collision", "check", opt=["unit", "expect_collision"],
      doc="zero collision events (expect_collision: true inverts)",
      eg="{no_collision: {}}"),
    V("no_sep_violation", "check", opt=["unit", "min_m", "h_m", "v_m"],
      doc="units never closer than min_m",
      eg="{no_sep_violation: {min_m: 5}}"),
    V("coverage_complete", "check",
      opt=["unit", "area", "cell_m", "min", "entity", "truth_file"],
      doc="coverage grading over the search area",
      eg="{coverage_complete: {min: 0.9}}"),
    V("pod_mean", "check",
      opt=["unit", "area", "cell_m", "min", "entity", "truth_file"],
      doc="mean probability-of-detection over cells",
      eg="{pod_mean: {min: 0.7}}"),
    V("pod_at", "check", opt=["unit", "truth", "min", "max", "entity", "poly",
           "truth_file"],
      doc="probability-of-detection at a truth entity",
      eg="{pod_at: {truth: person_1, min: 0.5}}"),
    V("detection_precision", "check",
      opt=["src", "truth", "unit", "cls", "match_m", "min", "max",
           "min_px", "min_frames", "iou"],
      doc="precision of the detection stream vs truth",
      eg="{detection_precision: {min: 0.8}}"),
    V("detection_precision_render", "check",
      opt=["src", "truth", "unit", "cls", "match_m", "min", "max",
           "min_px", "min_frames", "iou"],
      doc="detection_precision on the rendered-camera stream",
      eg="{detection_precision_render: {min: 0.8}}"),
    V("detection_recall", "check",
      opt=["src", "truth", "unit", "cls", "match_m", "min", "max",
           "min_px", "min_frames", "iou"],
      doc="recall of the detection stream vs truth",
      eg="{detection_recall: {min: 0.9}}"),
    V("render_latency", "check", opt=["unit", "max_ms", "p95_ms", "p95_max_ms"],
      doc="render pipeline frame latency",
      eg="{render_latency: {max_ms: 200}}"),
    V("visible_px", "check", opt=["truth", "unit", "min_px", "max_px", "min", "min_frames"],
      doc="truth entity's rendered pixel footprint",
      eg="{visible_px: {truth: person_1, min_px: 8}}"),
    V("time_to_detect", "check", opt=["max_s", "truth", "src", "unit"],
      doc="first detection latency from the first start step",
      eg="{time_to_detect: {max_s: 60}}"),
    V("stub_deterministic", "check",
      opt=["unit", "module", "dur_s", "poses"],
      doc="same seed -> identical stub sensor output",
      eg="{stub_deterministic: {module: stub_precland}}"),
    V("signals_in_range", "check", opt=["unit", "truth", "re", "max_m", "min_count"],
      doc="SDR signals the stub reported in range",
      eg="{signals_in_range: {min: 1}}"),
    V("sensor_error", "check",
      opt=["src", "unit", "truth_file", "truth_tol_ms", "mean_le",
           "p95_le", "max_le", "min_count"],
      doc="sensor-vs-truth position error stats (mean/p95/max m) into "
          "summary.json",
      eg="{sensor_error: {src: demo_range, mean_le: 5, p95_le: 10}}"),
    V("journal_consistent", "check", opt=["unit", "re", "min_count"],
      doc="journal invariants hold across the run",
      eg="{journal_consistent: {unit: all}}"),
    V("final_state", "check",
      opt=["unit", "state", "in", "fc_mode", "mis", "re", "cause_re"],
      doc="the unit's final journal state (state|in, fc_mode, mis, re "
          "filters over the GCS status)",
      eg="{final_state: {unit: c1, state: landed}}"),
    V("fc_items_unchanged_without_cause", "check",
      opt=["unit", "cause_re"],
      doc="FC mission items identical before/after with no upload "
          "in between",
      eg="{fc_items_unchanged_without_cause: {unit: c1}}"),
    V("mission_items", "check", req=["unit"],
      opt=["equals", "at_least", "at_most"],
      doc="FC mission count, serial0 read-back",
      eg="{mission_items: {unit: c1, equals: 5}}"),
    V("mission_pull", "check", req=["unit"],
      opt=["secs", "min", "equals"],
      doc="the GCS 501->1027 plan read-back a WebClient draws",
      eg="{mission_pull: {unit: c1, min: 5}}"),
    V("udp_proxy_mavlink", "check", req=["unit"],
      opt=["secs", "at_least"],
      doc="FC HEARTBEATs counted through the unit's UDP proxy (udp_relay)",
      eg="{udp_proxy_mavlink: {unit: c1, secs: 15, at_least: 10}}"),
    V("udp_proxy_rejects", "check", req=["unit"],
      opt=["from_ip", "secs"],
      doc="a stranger at from_ip cannot inject ARM/mission writes "
          "through the proxy pair",
      eg="{udp_proxy_rejects: {unit: c1, from_ip: '127.0.0.2'}}"),
    V("relay_gcs_telemetry", "check", req=["gcs"],
      opt=["secs", "at_least"],
      doc="the named fake GCS (`relay gcs start`) keeps receiving "
          "telemetry back through the relay pair",
      eg="{relay_gcs_telemetry: {gcs: qgc1, secs: 5, at_least: 3}}"),
    V("relay_gcs_no_telemetry", "check", req=["gcs"],
      opt=["secs"],
      doc="the named fake GCS receives no telemetry for the window",
      eg="{relay_gcs_no_telemetry: {gcs: qgc2, secs: 5}}"),
    V("relay", "step", opt=["act", "unit", "gcs", "ip", "port", "secs",
                           "cmd"],
      doc="9021 operator act as the sim's GCS: approve|reject|kick|"
          "change_port|query (SL-0179)",
      eg="{t: 10, do: relay, act: approve, unit: c1, ip: 127.0.0.2, port: 50001}"),
    V("relay_flood_msgs", "check", req=["unit", "secs"],
      opt=["at_most"]),
    V("relay_reject_bound", "check", req=["unit", "at_most"], opt=[]),
    V("relay_rejected_frozen", "check", req=["unit", "secs"],
      opt=["trigger"],
      doc="the relay's gcs_* rejected counters do not grow in the "
          "window - the stray source cannot reach the relay (DE-0067)",
      eg="{relay_rejected_frozen: {unit: c1, secs: 8}}"),
    V("relay_unit_locked", "check", req=["unit"],
      doc="the pair's unit side is pinned through its DEU1 token "
          "(pinned.unit mode token with ip/port) (DE-0067)",
      eg="{relay_unit_locked: {unit: c1}}"),
    V("relay_acct2_denied", "check", req=["unit"],
      opt=["act", "trigger"]),
    V("relay_acct2_quiet", "check", req=["unit", "secs"],
      opt=["trigger"]),
    V("relay_gcs_attempt", "check", req=["secs", "ip"],
      opt=["port", "others", "trigger", "unit"],
      doc="a 9021 gcs_attempt push for this source arrives in secs; "
          "port/others pin it down (SL-0179)",
      eg="{relay_gcs_attempt: {secs: 5, ip: '127.0.0.2', others: 0}}"),
    V("relay_no_gcs_attempt", "check", req=["secs"], opt=["trigger", "unit"],
      doc="no 9021 gcs_attempt push arrives in the window (SL-0179)",
      eg="{relay_no_gcs_attempt: {secs: 5}}"),
    V("relay_gcs_switched", "check", req=["secs"], opt=["reason",
                                                      "trigger", "unit"],
      doc="a 9021 gcs_switched push arrives in secs, reason when given "
          "(SL-0179)",
      eg="{relay_gcs_switched: {secs: 5, reason: idle}}"),
    V("relay_gcs_locked", "check", req=["unit", "gcs"],
      doc="the relay pair's pinned GCS address is the named fake GCS",
      eg="{relay_gcs_locked: {unit: c1, gcs: qgc1}}"),
    V("no_fc_command_to_static", "check", opt=["unit"],
      doc="no MAVLink command was ever sent to a no-FC unit",
      eg="{no_fc_command_to_static: {unit: s1}}"),
    V("max_latency", "check", req=["u1", "re1", "u2", "re2"],
      opt=["max_ms"],
      doc="journal_time_delta in milliseconds",
      eg="{max_latency: {u1: c1, re1: 'a', u2: c2, re2: 'b',"
         " max_ms: 500}}"),
    V("proc_alive", "check", req=["unit"], opt=["proc"],
      doc="the named proc is alive at the end",
      eg="{proc_alive: {unit: c1, proc: de_comm}}"),
    V("proc_dead", "check", req=["unit"], opt=["proc"],
      doc="the named proc is dead at the end",
      eg="{proc_dead: {unit: c1, proc: de_mavlink}}"),
    V("operator_log_count", "check", req=["re"],
      opt=["equals", "at_least", "at_most"],
      doc="journal_count over the operator bot's decision log",
      eg="{operator_log_count: {re: 'deny', at_least: 1}}"),
    V("link_offline", "check", req=["unit"],
      opt=["at_least", "min_s"],
      doc="the link monitor saw the unit offline",
      eg="{link_offline: {unit: s1, at_least: 1, min_s: 5}}"),
    V("no_unexpected_restart", "check", opt=["unit", "proc", "tol_s"],
      doc="every pid change sits inside a recorded fault window "
          "(invariant)",
      eg="{no_unexpected_restart: {}}"),
    V("rss_growth", "check", opt=["unit", "proc", "max_mb_per_h",
                                  "warmup_s"],
      doc="least-squares RSS slope after warm-up (invariant)",
      eg="{rss_growth: {max_mb_per_h: 50}}"),
    V("fd_growth", "check", opt=["unit", "proc", "max"],
      doc="open-fd count growth over the run (invariant)",
      eg="{fd_growth: {max: 20}}"),
    V("journal_size", "check", opt=["unit", "max_mb"],
      doc="journal dir total size (invariant)",
      eg="{journal_size: {max_mb: 50}}"),
    V("recovers_after_fault", "check", opt=["within_s"],
      doc="every timed fault restore is followed by link online + "
          "fresh telemetry (invariant)",
      eg="{recovers_after_fault: {within_s: 60}}"),
    V("no_stuck_state", "check", req=["unit", "re"],
      opt=["max_s"],
      doc="a matching journal state line changes at least every "
          "max_s (invariant)",
      eg="{no_stuck_state: {unit: c1, re: 'state', max_s: 30}}"),
    V("roster_value", "check", req=["unit", "key"],
      opt=["equals", "at_least", "at_most", "re"],
      doc="a field of the unit's latest roster entry",
      eg="{roster_value: {unit: c1, key: state, equals: ONLINE}}"),
    V("telemetry_value", "check", req=["unit", "field"],
      opt=["pick", "re", "max_age_ms"],
      doc="tlog sample bounds (pick: min|max)",
      eg="{telemetry_value: {unit: c1, field: rel_alt_m, min: 10}}"),
    V("fc_mode", "check", req=["unit", "mode"],
      doc="the FC's last mode",
      eg="{fc_mode: {unit: c1, mode: AUTO}}"),
    V("status_field", "check", req=["unit", "path"],
      opt=["equals", "re", "min", "max", "at_least", "at_most"],
      doc="a field of the latest 1087 status (e.g. launch.grant)",
      eg="{status_field: {unit: c1, path: launch.grant, "
         "equals: true}}"),
    V("fcb_stats", "check", opt=["unit", "direction",
                                 "min_bytes", "min_frames"],
      doc="the unit's fcb proxy counters read live off its ctrl port "
          "(SL-0104: bytes moved through a bridged serial adapter)",
      eg="{fcb_stats: {unit: c1, direction: both, min_bytes: 1}}"),
    V("bt_no_squatters", "check", opt=["unit"],
      doc="no foreign process holds an RFCOMM socket - a squatter "
          "(kdeconnectd is the known thief) steals a bt_spp unit's "
          "dials while fcb_stats reads zero (SL-0105)",
      eg="{bt_no_squatters: {unit: c1}}"),
    V("adsb_vehicle", "check", req=["icao"],
      opt=["tid", "at_least", "max_m", "log", "unit"],
      doc="the GCS bridge (or log:/unit stdout) saw >= at_least "
          "ADSB_VEHICLE frames with this ICAO each within max_m of "
          "truth",
      eg="{adsb_vehicle: {icao: 'a1b2c3', at_least: 1}}"),
    V("gcs_msg", "check", req=["mt", "re"],
      opt=["secs"],
      doc="the GCS session receives an mt whose ms body matches re "
          "(compact-dumped ms; use (?=...) lookaheads) within secs "
          "(SL-0067)",
      eg="{gcs_msg: {mt: 1008, re: '\"MI\":3', secs: 20}}"),
    V("obs_frames", "check", req=["unit"],
      opt=["min_frames", "cls", "trk_stable", "attrs"],
      doc="the stub's 6546 obs frames: cls multiset per frame, stable "
          "trk set, attrs bounds (SL-0034)",
      eg="{obs_frames: {unit: cam1, min_frames: 20, "
         "cls: [person], trk_stable: true}}"),
]

KINDS = ("step", "probe", "fault", "check")
BY_NAME = {}
for _v in VERBS:
    BY_NAME.setdefault(_v["name"], []).append(_v)

# the same verb names surface as both do: steps and fault: kinds
# (kill/restart/freeze/loss/delay/link_cut/mode/udp_chunks/...). A
# `fault:` step dispatches on the fault kind; a `do:` step on the
# step name - lookup is always (name, kind) so the two coexist.

_STEP_KINDS = ("step", "probe")


def by_kind(kind):
    return [v for v in VERBS if v["kind"] == kind]


def lookup(name, kind):
    """one (kind, name) pair -> the entry or None. kind: 'do' covers
    step+probe, 'fault' fault kinds, 'assert'/'check' assert names.
    BY_NAME holds a list per name - verb names repeat across kinds
    (kill/mode/loss/... exist as both steps and faults)."""
    vs = BY_NAME.get(name) or []
    for v in vs:
        if kind in ("do", "step") and v["kind"] in _STEP_KINDS:
            return v
        if kind == "fault" and v["kind"] == "fault":
            return v
        if kind in ("assert", "check") and v["kind"] == "check":
            return v
    return None


# every numeric `assert:` check funnels its bounds through checks._cmp,
# which reads this shared comparator vocabulary - listed once here so
# each entry's opt list can stay on the verb-specific args
_CHECK_CMPS = ("eq", "equals", "at_least", "at_most",
               "lt", "gt", "le", "ge")


def check_args(v, args):
    """verb entry + the step's args -> an error string or None:
    missing required args and unknown args fail, naming the expected
    list. free-form verbs only check what they declare."""
    known = set(v["req"]) | set(v["opt"])
    hint = ", ".join(v["req"] + v["opt"])
    if v["kind"] == "check":
        known |= set(_CHECK_CMPS)
        hint += " + " + "/".join(_CHECK_CMPS)
    missing = [k for k in v["req"] if k not in args]
    if missing:
        return f"{v['name']}: missing required arg(s) {missing} " \
               f"(args: {hint})"
    if not v["free"]:
        unknown = sorted(set(args) - known)
        if unknown:
            return f"{v['name']}: unknown arg(s) {unknown} " \
                   f"(args: {hint})"
    return None
