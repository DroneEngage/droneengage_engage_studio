#!/usr/bin/env python3
"""operator_bot.py - the scripted GCS operator (TASK-P5-07).

Holds one GCS session through engine/gcs_client.py and acts like the
person at the ground station:

- watches 1087 status frames for pending approvals and decides them per
  the scenario `operator:` policy (`approve`/`deny` + `after_s`),
- watches 1090 detection envelopes for world-model targets and
  confirms/dismisses them (1093) per `confirm:`/`dismiss:` rules,
- sends 1094 deploy/start/abort, 1086 fire/activate/pause/resume/stop and
  `sys.approval` decisions on demand (timeline `do: operator` / `do:` GCS
  actions route through here so every operator action lands in
  operator.log).

Every action is journaled to operator.log as
`<fleet_ms> <KIND> <text>` - fleet ms is the server clock the session
synced to (CLOCK.md T2; 0 when the server gives no stamp).

The bot runs its own asyncio loop on a thread; all public methods are
thread-safe and callable from the scenario runner.
"""

import asyncio
import hashlib
import json
import os
import threading
import time
import uuid

import simlib
import gcs_client


class OperatorBot:
    def __init__(self, manifest, policy=None, log_path=None,
                 party="GCS_OPERATOR"):
        self.m = manifest
        self.party = party
        self.policy = policy or {}
        self.log_path = log_path or "/dev/null"
        self._loop = None
        self._thread = None
        self._task = None
        self._tick_task = None
        self._client = gcs_client.GCSClient(
            manifest["server"], party, on_frame=self._on_frame)
        self._lock = threading.Lock()
        self._status = {}             # party -> last 1087 ms
        self._roster = {}             # party -> last 1092 ms (tq, bat, st)
        self._pending = {}            # party -> {req: {kind, first_seen}}
        self._targets = {}            # tid -> {cls, first_seen, decided}
        self._event_targets = {}      # event name -> wm class (deploy pkgs)
        self._op_oh = ""              # deploy's operation hash (1090 oh)
        self._awaited = []            # [(deadline, pred, act) pending decisions]
        self._ev_seq = {}             # run -> last seq the bot used (1090)
        self._op_q = 0                # 1093 author sequence
        # the WebClient's 1093 resend watch (js_worldModel.js): every
        # decision of a run is kept and re-sent unchanged while the
        # units' roster "wm" digests disagree - a unit that missed the
        # one-shot 1093 (lost frame, run started late) only gets it so
        self._decisions = {}          # run -> [op] (sent as-is again)
        self._resend = {"next": 0.0, "backoff": 2.0}
        self._replies = []            # recent non-roster frames (max 5000)
        self._was_connected = True    # start() logs the first connect

    # ------------------------------------------------------------- log

    def fleet_ms(self):
        return self._client.fleet_ms()

    def log(self, kind, text):
        line = f"{self.fleet_ms()} {kind} {text}\n"
        try:
            with open(self.log_path, "a") as f:
                f.write(line)
        except OSError:
            pass
        print(f"  [operator] {kind} {text}", flush=True)

    # --------------------------------------------------------- lifecycle

    def start(self, timeout=25):
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._client.connected.is_set():
                self.log("SESSION", f"{self.party} online")
                return True
            time.sleep(0.2)
        print("  [operator] GCS session did not connect - "
              "bot actions will fail", flush=True)
        return False

    def _run(self):
        asyncio.set_event_loop(self._loop)
        self._task = self._loop.create_task(self._client.run())
        self._tick_task = self._loop.create_task(self._tick())
        self._loop.run_forever()

    def stop(self):
        if not self._loop:
            return

        async def shutdown():
            for t in (self._task, self._tick_task):
                if t:
                    t.cancel()
            await asyncio.gather(
                *[t for t in (self._task, self._tick_task) if t],
                return_exceptions=True)
            try:
                await self._client.close()
            except Exception:
                pass
            self._loop.stop()

        try:
            self._loop.call_soon_threadsafe(
                lambda: asyncio.ensure_future(shutdown()))
            self._thread.join(5)
        except Exception:
            pass
        self.log("SESSION", f"{self.party} offline")

    def _submit(self, coro):
        if not self._loop:
            return None
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def wait_connected(self, timeout=10):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._client.connected.is_set():
                return True
            time.sleep(0.2)
        return False

    def disconnect_for(self, secs):
        """gcs_disconnect fault (fake server: the bot IS the GCS). The
        reconnect loop stays suppressed for `secs`."""
        self.log("FAULT", f"gcs_disconnect {secs}s")
        self._client.hold_until = time.time() + secs

        async def drop():
            ws = self._client.ws
            if ws:
                await ws.close()

        self._submit(drop())

    # ------------------------------------------------------- frame intake

    def _on_frame(self, msg):
        mt = msg.get("mt")
        sd = msg.get("sd")
        ms = msg.get("ms") or {}
        now = time.monotonic()
        with self._lock:
            if mt != 1092:               # roster churn is not a reply
                self._replies.append(msg)
                # an 8-unit fleet's 1087/1090 traffic evicted a 400-frame
                # ring before the sequential deploy/start waits read the
                # later units' replies (P4B-10 scenario I)
                del self._replies[:-5000]
            if mt == 1092 and sd:
                self._roster[sd] = ms
            elif mt == 1087 and sd:
                self._status[sd] = ms
                seen = self._pending.setdefault(sd, {})
                live = set()
                for row in ms.get("approvals") or []:
                    req = row.get("req")
                    if not req:
                        continue
                    live.add(req)
                    if req not in seen:
                        seen[req] = dict(row)
                        seen[req]["first_seen"] = now
                for req in list(seen):
                    if req not in live:
                        del seen[req]
            elif mt == 1090 and isinstance(ms, dict):
                p = ms.get("p") or {}
                name = ms.get("n") or ""
                det_cls = self._event_targets.get(name)
                tid = ms.get("tgt") or p.get("tid")
                if not tid and det_cls:
                    # P4B-10: a detection envelope carries no tid - the
                    # world model keys each cluster on its earliest
                    # member's envelope id (wm_core), which is the first
                    # detection of the class on the wire. Only that one
                    # is a confirm candidate; later envelopes of the
                    # class are cluster members (or new clusters the
                    # policy already decided - confirming either would
                    # act on the wrong/world-less tid).
                    if not any(r.get("cls") == det_cls
                               for r in self._targets.values()):
                        tid = ms.get("id")
                if tid and tid not in self._targets:
                    self._targets[tid] = {
                        "cls": det_cls or p.get("class") or p.get("cls")
                               or (p.get("target") or {}).get("cls"),
                        "t_fleet": ms.get("t") or 0,
                        # P5-08: the reported position + arrival wall ms -
                        # distance_to(detected_cls:) reads them
                        "pos": p.get("pos") or ms.get("pos"),
                        "t_ms": int(time.time() * 1000),
                        "src": ms.get("src") or sd,
                        "run": ms.get("run", ""),
                        "first_seen": now, "decided": False}

    # ------------------------------------------------------------- state

    def approvals(self):
        with self._lock:
            return {p: {r: dict(row) for r, row in rows.items()}
                    for p, rows in self._pending.items()}

    def status_of(self, party):
        with self._lock:
            return dict(self._status.get(party) or {})

    def roster_of(self, party):
        with self._lock:
            return dict(self._roster.get(party) or {})

    def targets(self):
        with self._lock:
            return {t: dict(r) for t, r in self._targets.items()}

    def _party_of(self, unit):
        return unit["party_id"] if isinstance(unit, dict) else str(unit)

    def _run_of(self, party):
        return (self._status.get(party) or {}).get("run") or ""

    def _next_seq(self, run):
        n = self._ev_seq.get(run, 0) + 1
        self._ev_seq[run] = n
        return n

    def _oh(self, party):
        """the deploy's operation hash - the 1090 validator compares it
        verbatim to the package op_hash (P4B-10: the status pkg_hash is
        a different digest - using it drops the decision 'other
        operation')."""
        return self._op_oh or None

    # ------------------------------------------------------------ actions

    async def _send_async(self, msg, kind, text):
        try:
            await self._client.send(msg)
            ok = True
        except Exception as e:
            ok = False
            text += f" [send failed: {e}]"
        self.log(kind, text)
        return ok

    def _send(self, msg, kind, text):
        """Synchronous send for the runner thread."""
        fut = self._submit(self._send_async(msg, kind, text))
        try:
            return bool(fut and fut.result(timeout=6))
        except Exception:
            return False

    def action(self, ms, tg="_AGN_", note=""):
        """1086 DE_Supervisor_Action (fire/activate/pause/resume/stop/...)."""
        return self._send({"ty": "i", "mt": 1086, "tg": tg, "ms": ms},
                          "ACTION", f"tg={tg} {json.dumps(ms)} {note}".strip())

    def op_control(self, ms, tg, note=""):
        """1094 DE_OP_CONTROL (prepare/start/abort/status)."""
        return self._send({"ty": "i", "mt": 1094, "tg": tg, "ms": ms},
                          "OP", f"tg={tg} {json.dumps(ms)} {note}".strip())

    def fire(self, event, payload=None, tg="_AGN_"):
        ms = {"a": "fire", "event": event}
        if payload is not None:
            ms["payload"] = payload
        return self.action(ms, tg, f"fire {event}")

    def _approval_envelope(self, party, req, decision, kind=None):
        run = self._run_of(party)
        env = {"v": 1, "id": str(uuid.uuid4()), "src": self.party,
               "run": run, "seq": self._next_seq(run),
               "t": self.fleet_ms(), "age": 0, "ttl": 30,
               "n": "sys.approval",
               "p": {"req": req, "decision": decision,
                     "by": self.party, "to": party}}
        st = self._status.get(party) or {}
        if st.get("op"):
            env["op"] = st["op"]
        oh = self._oh(party)
        if oh:
            env["oh"] = oh
        return env

    async def _decide_async(self, party, req, kind, decision, wait_s):
        t0 = time.monotonic()
        while True:
            with self._lock:
                rows = dict(self._pending.get(party) or {})
            if req:
                pick = req          # an explicit req needs no 1087 row
            elif kind:
                pick = next((r for r, row in rows.items()
                             if row.get("kind") == kind), None)
            else:
                pick = next(iter(rows), None)
            if pick is not None:
                break
            if time.monotonic() - t0 >= wait_s:
                self.log("APPROVAL_MISS",
                         f"{party}: no pending request "
                         f"kind={kind} req={req} after {wait_s:.0f}s")
                return False
            await asyncio.sleep(0.4)
        env = self._approval_envelope(party, pick, decision, kind)
        return await self._send_async(
            {"ty": "i", "mt": 1090, "tg": party, "ms": env},
            "APPROVAL", f"{decision} {party} req={pick} kind={kind}")

    def decide(self, unit, req=None, kind=None, decision="approve",
               wait_s=10.0):
        """sys.approval decision for a unit's pending request. `req` is the
        exact request id, `kind` picks the pending request of that kind;
        with neither, the first pending row is used. Waits up to wait_s
        for a matching request to appear (the 1087 can lag the journal's
        APPROVAL_REQ line)."""
        party = self._party_of(unit)
        fut = self._submit(self._decide_async(party, req, kind,
                                              decision, wait_s))
        try:
            return bool(fut and fut.result(timeout=wait_s + 10))
        except Exception:
            return False

    async def _confirm_async(self, tid, tg, state, run):
        self._op_q += 1
        op = {"o": "state", "tid": tid, "st": state,
              "t": self.fleet_ms(), "by": self.party, "q": self._op_q}
        ms = {"v": 1, "run": run if run is not None else (
            self._run_of(tg) if tg not in ("_AGN_", "_GD_") else ""),
              "ops": [op]}
        if ms["run"]:
            with self._lock:
                self._decisions.setdefault(ms["run"], []).append(op)
                self._resend = {"next": 0.0, "backoff": 2.0}
        return await self._send_async(
            {"ty": "i", "mt": 1093, "tg": tg, "ms": ms},
            "WORLD", f"{state} tid={tid} tg={tg}")

    def confirm(self, tid, tg="_AGN_", state="confirmed", run=None):
        """1093 DE_WORLD operator decision (confirm/dismiss a target)."""
        ms = {"v": 1,
              "run": run if run is not None else (
                  self._run_of(tg) if tg not in ("_AGN_", "_GD_") else ""),
              "ops": [self._next_op(tid, state)]}
        if ms["run"]:
            with self._lock:
                self._decisions.setdefault(ms["run"], []).extend(ms["ops"])
                self._resend = {"next": 0.0, "backoff": 2.0}
        return self._send({"ty": "i", "mt": 1093, "tg": tg, "ms": ms},
                          "WORLD", f"{state} tid={tid} tg={tg}")

    def _next_op(self, tid, state):
        self._op_q += 1
        return {"o": "state", "tid": tid, "st": state,
                "t": self.fleet_ms(), "by": self.party, "q": self._op_q}

    # -------------------------------------------------- op control (1094)

    def wait_reply(self, pred, timeout):
        """Latest received frame pred(msg)->True, or None. Polls the
        reply ring from the caller's thread."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                frames = list(self._replies)
            hit = next((m for m in reversed(frames) if pred(m)), None)
            if hit is not None:
                return hit
            time.sleep(0.25)
        return None

    def _party(self, name):
        for u in self.m["units"]:
            if u["name"] == name:
                return u["party_id"]
        raise KeyError(f"operator: no unit {name!r}")

    def _op_state_file(self):
        return simlib.op_state_path(self.m["name"])

    def _op_state(self):
        try:
            return json.load(open(self._op_state_file()))
        except (OSError, ValueError):
            return {}

    def op_deploy(self, spec_path, timeout=60.0):
        """Two-phase deploy (P4A-09): one PREPARE per bound unit through
        the bot's own GCS session (the units see GCS_OPERATOR as the
        deployer - the same party their approval requests go back to)."""
        spec_path = os.path.abspath(spec_path)
        spec = simlib.load_config(spec_path)
        if not isinstance(spec.get("roles"), dict):
            self.log("DEPLOY", f"{spec_path}: no roles map - not a "
                     "deploy spec")
            return False
        base = os.path.dirname(spec_path)
        run = spec.get("run") or str(uuid.uuid4())
        op, ohs, jobs = spec.get("op", ""), [], []
        for role, r in spec["roles"].items():
            text = open(os.path.join(base, r["pkg"])).read()
            pkg = json.loads(simlib.strip_comments(text))
            h = "sha256:" + hashlib.sha256(text.encode()).hexdigest()
            # P4B-10: event name -> wm class map for the detection->tid
            # bookkeeping in _on_frame (mirrors de_comm's wmIngest)
            for ev_name, ev_cls in (
                    (pkg.get("fleet") or {}).get("event_targets")
                    or {}).items():
                if isinstance(ev_cls, str):
                    self._event_targets.setdefault(ev_name, ev_cls)
            ohs.append(pkg.get("op_hash", ""))
            if not op:
                op = pkg.get("op", "")
            for tok in r.get("units", []):
                jobs.append((self._party(str(tok)), role, text, h))
        oh = spec.get("oh") or next((h for h in ohs if h), "")
        self._op_oh = oh              # 1090 decisions must carry it
        bind = {p: role for p, role, _, _ in jobs}
        if not jobs or len(bind) != len(jobs):
            self.log("DEPLOY", "spec binds no units (or a unit twice)")
            return False
        self.log("DEPLOY", f"op {op} run {run}: PREPARE -> "
                 + ", ".join(f"{p}={r}" for p, r, _, _ in jobs))
        for p, role, text, h in jobs:
            if not self._send(
                    {"ty": "i", "mt": 1094, "tg": p,
                     "ms": {"a": "prepare", "op": op, "oh": oh, "run": run,
                            "bind": bind, "pkg": text, "hash": h}},
                    "PREPARE", f"{p} role={role}"):
                return False
        ok = True
        for p, role, _t, _h in jobs:
            ans = self.wait_reply(
                lambda m, p=p: m.get("mt") == 1094 and m.get("sd") == p
                and (m.get("ms") or {}).get("run") == run
                and (m.get("ms") or {}).get("a") in ("ready", "fail"),
                timeout)
            if ans is None:
                self.log("DEPLOY", f"{p}: no PREPARE reply")
                ok = False
            elif (ans.get("ms") or {}).get("a") == "ready":
                self.log("READY", f"{p} as {role}")
            else:
                self.log("DEPLOY",
                         f"{p}: FAIL ({ans['ms'].get('why', '?')})")
                ok = False
        json.dump({"run": run, "op": op, "bind": bind},
                  open(self._op_state_file(), "w"), indent=1)
        return ok

    def op_start(self, ov=None, timeout=None):
        """Fleet START on the deploy's run; t_start comes from the bot's
        synced fleet clock, start_in_ms is the tq=0 fallback."""
        ov = ov or {}
        st = self._op_state()
        run = ov.get("run") or st.get("run")
        if not run:
            self.log("START", "no op deploy - nothing to start")
            return False
        parties = ov.get("targets") or sorted(st.get("bind") or {})
        lead = int(ov.get("lead_ms", 5000))
        ms = {"a": "start", "run": run,
              "start_in_ms": int(ov.get("in_ms", lead))}
        ms["t_start"] = int(ov["t_start"]) if isinstance(
            ov.get("t_start"), (int, float)) else self.fleet_ms() + lead
        if isinstance(ov.get("start"), dict):
            ms["start"] = ov["start"]
        self.log("START", f"run {run} t_start={ms['t_start']} "
                 f"-> {', '.join(parties)}")
        for p in parties:
            self._send({"ty": "i", "mt": 1094, "tg": p, "ms": dict(ms)},
                       "START", f"{p} run={run}")
        secs = timeout or max(15.0, (lead + ms["start_in_ms"])
                              / 1000.0 + 30.0)
        ok = True
        for p in parties:
            ans = self.wait_reply(
                lambda m, p=p: m.get("mt") == 1094 and m.get("sd") == p
                and (m.get("ms") or {}).get("run") == run
                and (m.get("ms") or {}).get("a") in ("started", "fail"),
                secs)
            if ans is None:
                self.log("START", f"{p}: no START reply")
                ok = False
            elif (ans.get("ms") or {}).get("a") == "started":
                self.log("STARTED", p)
            elif p in (ov.get("expect_fail") or []):
                self.log("START", f"{p}: refused as expected "
                         f"({ans['ms'].get('why','?')})")
            else:
                self.log("START", f"{p}: FAIL ({ans['ms'].get('why','?')})")
                ok = False
        return ok

    def op_abort(self, run=None, why="", timeout=8.0):
        """Fleet ABORT: idempotent {a:"abort"} to every bound unit."""
        st = self._op_state()
        run = st.get("run", "") if not run or run == "-" else run
        parties = sorted(st.get("bind") or {})
        ok = True
        for p in parties:
            self._send({"ty": "i", "mt": 1094, "tg": p,
                        "ms": {"a": "abort", "run": run, "why": why}},
                       "ABORT", f"{p} run={run or '-'} {why}".strip())
        for p in parties:
            ans = self.wait_reply(
                lambda m, p=p: m.get("mt") == 1094 and m.get("sd") == p
                and (m.get("ms") or {}).get("a") == "aborted", timeout)
            self.log("ABORTED" if ans else "ABORT",
                     p if ans else f"{p}: no reply")
            ok &= ans is not None
        return ok

    # -------------------------------------------------------------- tick

    async def _tick(self):
        """Standing policy: decide pending approvals and world targets
        whose `after_s` has elapsed."""
        while True:
            await asyncio.sleep(0.5)
            try:
                self._policy_once()
            except Exception as e:
                print(f"  [operator] policy tick error: {e!r}", flush=True)

    def _resend_watch(self, now):
        """Re-send the run's decisions (unchanged ops - the units dedupe
        them) while the alive units' "wm" digests differ, with a 2..30 s
        backoff; idle once they agree."""
        with self._lock:
            if not self._decisions:
                return
            digests = {str(r.get("wm")) for r in self._roster.values()
                       if isinstance(r, dict) and r.get("wm")}
            if len(digests) <= 1:
                self._resend = {"next": 0.0, "backoff": 2.0}
                return
            if now < self._resend["next"]:
                return
            self._resend["next"] = now + self._resend["backoff"]
            self._resend["backoff"] = min(30.0, self._resend["backoff"] * 2)
            batches = {run: list(ops) for run, ops in self._decisions.items()}
        for run, ops in batches.items():
            self._submit(self._send_async(
                {"ty": "i", "mt": 1093, "tg": "_AGN_",
                 "ms": {"v": 1, "run": run, "ops": ops}},
                "WORLD", f"resend {len(ops)} op(s) run={run}"))

    def _policy_once(self):
        now = time.monotonic()
        self._resend_watch(now)
        up = self._client.connected.is_set()
        if up and not self._was_connected:
            self.log("SESSION", f"{self.party} online (reconnect)")
        self._was_connected = up
        with self._lock:
            pend = {p: dict(rows) for p, rows in self._pending.items()}
            tgts = {t: dict(r) for t, r in self._targets.items()
                    if not r["decided"]}
        for party, rows in pend.items():
            for req, row in rows.items():
                kind = row.get("kind")
                for act in ("approve", "deny"):
                    rule = (self.policy.get(act) or {}).get(kind)
                    if rule is None:
                        continue
                    if now - row["first_seen"] >= float(rule.get("after_s", 0)):
                        self._submit(self._decide_async(
                            party, req, kind, act, 0))
                        break
        for tid, row in tgts.items():
            cls = row.get("cls") or "*"
            for act in ("confirm", "dismiss"):
                rules = self.policy.get(act) or {}
                rule = rules.get(cls, rules.get("*"))
                if rule is None:
                    continue
                if now - row["first_seen"] >= float(rule.get("after_s", 0)):
                    state = "confirmed" if act == "confirm" else "dismissed"
                    self._submit(self._confirm_async(
                        tid, "_AGN_", state, row.get("run")))
                    with self._lock:
                        if tid in self._targets:
                            self._targets[tid]["decided"] = True
                    break
