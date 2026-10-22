"""de_adapter.py - the stock DroneEngage vehicle adapter, shipped as the
`droneengage` plugin of the `simland.adapters` entry-point group
(pyproject.toml). The seam itself (VehicleAdapter, new_adapter) lives in
adapter.py; docs/ADAPTER.md is the spec."""
import json
import os
import time

import ctl
import runner
import simlib
from adapter import VehicleAdapter


class DEAdapter(VehicleAdapter):
    """DroneEngage - the stock adapter. No behavior of its own: every
    branch delegates to the same bot / ctl / runner calls the runner
    has always made; the seam only names them."""

    name = "droneengage"

    def do_step(self, name, a, step, scn, ctx):
        bot = ctx["bot"]
        if name in ("deploy", "prepare"):
            op = a.get("operation") or a.get("package") or \
                scn.get("operation")
            if not op:
                print("  deploy: no operation:", flush=True)
                return False
            op = op if os.path.isabs(op) else os.path.join(
                os.path.dirname(scn["file"]), op)
            if op.endswith(".deop"):
                # TASK-23: the run-start build already compiled the
                # scenario's own op; a step-level operation: override
                # builds on demand here (the scenario fleet supplies
                # the role binding for ctx-less ops)
                if op == scn.get("operation"):
                    op = scn.get("operation_spec") or op
                if op.endswith(".deop"):
                    import opbuild
                    op = opbuild.build_deop(op, scn.get("fleet"))
            if ctl._is_deploy_spec(op):
                bot.log("DEPLOY", f"spec {os.path.basename(op)}")
                # P4F-07: `authorize: {unit: {valid_for_s,
                # max_launches}}` on the deploy step - a launch grant
                # per unit after its READY
                authorize = None
                if isinstance(a.get("authorize"), dict):
                    authorize = {
                        (ctx["units_by_name"].get(str(n)) or {})
                        .get("party_id", str(n)): (v or {})
                        for n, v in a["authorize"].items()}
                return bot.op_deploy(op, float(a.get("timeout", 60)),
                                     authorize=authorize)
            # `unit:`/`package:` (one compiled package to one unit,
            # e.g. P4A-07's scout + workers) is the runner step's own
            # per-unit binding path; the plain form stays a fleet-wide
            # ctl deploy
            if a.get("unit") not in (None, "all") or a.get("package"):
                return runner.run_step(
                    {"deploy": {k: v for k, v in a.items()
                                if k in ("package", "unit", "vehicle")}},
                    {"__file": scn["file"],
                     "operation": scn.get("operation")}, ctx["marks"])
            return runner.run_ctl("deploy", op)
        if name == "start":
            # the scenario's own t0: time_to_detect measures from the
            # first start step, not from the earlier fleet-up world
            # epoch (the arm/EKF wait in between is not detection
            # latency)
            ctx.setdefault("start_ms", int(time.time() * 1000))
            st = {}
            try:
                st = json.load(open(simlib.op_state_path(ctx["sim"])))
            except (OSError, ValueError):
                pass
            if st.get("run"):
                ov = {k: a[k] for k in
                      ("lead_ms", "in_ms", "t_start", "run") if k in a}
                if isinstance(a.get("start"), dict):
                    ov["start"] = a["start"]    # {role: mission} (1094)
                if isinstance(a.get("targets"), list):
                    ov["targets"] = [
                        (ctx["units_by_name"].get(str(n)) or {})
                        .get("party_id", str(n)) for n in a["targets"]]
                if isinstance(a.get("expect_fail"), list):
                    ov["expect_fail"] = [
                        (ctx["units_by_name"].get(str(n)) or {})
                        .get("party_id", str(n))
                        for n in a["expect_fail"]]
                return bot.op_start(
                    ov, float(a["timeout"]) if "timeout" in a else None)
            return runner.run_ctl("start")
        if name == "abort":
            return bot.op_abort(a.get("run"), a.get("why", ""))
        if name in ("authorize", "revoke_launch"):
            # P4F-07: mid-run launch-grant verbs through the bot's
            # session
            party = (ctx["units_by_name"].get(str(a.get("unit"))) or
                     {}).get("party_id", a.get("unit"))
            if name == "authorize":
                choice = {k: a[k] for k in ("valid_for_s", "max_launches")
                          if k in a}
                return bot.op_authorize(party, choice,
                                        float(a.get("timeout", 10)))
            return bot.op_revoke_launch(party, float(a.get("timeout", 10)))
        if name == "operator":
            return self.operator_act(a, ctx)
        # infra lifecycle steps: an operator action rather than a
        # `fault:`, but the pids they re-create are expected to change
        if name in ("reset", "server_up", "land"):
            ctx["proc_events"].append(
                {"t_ms": int(time.time() * 1000),
                 "unit": "server" if name == "server_up"
                 else str(a.get("unit", "all")),
                 "proc": "all"})
        # every other name: the runner.py step, unchanged semantics
        t = {"__file": scn["file"], "operation": scn.get("operation")}
        return runner.run_step({name: a}, t, ctx["marks"])

    def operator_act(self, a, ctx):
        """do: operator - act: approve|deny|confirm|dismiss|fire|<1086 a>."""
        bot = ctx["bot"]
        act = a.get("act")
        unit = a.get("unit")
        tg = "_AGN_" if not unit or str(unit) == "all" else \
            ctl.resolve(str(unit), ctx["manifest"])[0]["party_id"]
        if act in ("approve", "deny"):
            if not unit:
                print("  operator approve/deny needs unit:", flush=True)
                return False
            return bot.decide(tg, req=a.get("req"), kind=a.get("kind"),
                              decision=act, wait_s=float(a.get("wait_s", 10)))
        if act in ("confirm", "dismiss"):
            state = "confirmed" if act == "confirm" else "dismissed"
            return bot.confirm(a["tid"], tg=tg, state=state)
        if act == "fire":
            return bot.fire(a["event"], a.get("payload"), tg)
        if act in ("activate", "pause", "resume", "stop", "clear"):
            ms = {"a": act}
            if a.get("mission"):
                ms["mission"] = a["mission"]
            if a.get("hold") is not None:
                ms["hold"] = bool(a["hold"])
            return bot.action(ms, tg)
        if act in ("deploy", "start", "abort"):
            return self.do_step(act,
                                {k: v for k, v in a.items()
                                 if k != "act"},
                                {"do": act}, ctx["scn"], ctx)
        print(f"  operator: unknown act {act!r}", flush=True)
        return False
