#!/usr/bin/env python3
"""port_allocation_management.py - machine-wide port leases for the sim.

One registry per machine, one flock-serialized transaction per sim
bring-up: ./sim up queues every port it wants (units, servers, proxies,
SITL blocks), commit() picks them all under ports.lock and writes
ports.json in one replace, so two sims coming up at the same moment can
never be handed the same port. See docs/port_allocation.md; this is the
phase 0 allocator, standalone (stdlib only, no simlib import).

Registry: SIM_PORT_REGISTRY > $XDG_RUNTIME_DIR/de_sim/ports.json >
~/.cache/de_sim/ports.json (tmpfs there suits leases; they need not
survive logout). The previous-port memory does survive:
SIM_PORT_LAST > ~/.cache/de_sim/last_ports.json. Locking is
fcntl.flock(LOCK_EX) on ports.lock next to the registry for the whole
read -> mutate -> write; writes go to ports.json.tmp + os.replace. A
missing registry starts empty; a corrupt one is moved aside to
ports.json.bad-<epoch> first.

Lease lifecycle: "reserved" at commit (holder = the ./sim up pid + its
/proc starttime), "bound" once the launcher reports the process up
(mark_bound), deleted on release (./sim down). A stale-lease sweep runs
at the start of every transaction and via gc(); a lease is reclaimed
when its worktree is gone, or its sim has no live pids under
<worktree>/var/run/<sim>/pids and its holder pid is dead, or it is
still "reserved" with a dead holder (./sim up killed mid bring-up).
Pid liveness compares /proc/<pid>/stat field 22 to the recorded
starttime, so a reused pid never matches; proc_root is injectable for
tests. The sweep's reclaim is persisted by the next registry write -
an aborted transaction writes nothing, so dead leases wait one more
round, harmlessly.

Requests: kind "pin" = that port or fail (ignores the deny list,
still test-binds), "prefer" = that port then the remembered one then
the pool, "free" = remembered then pool, "block" = a run of size
consecutive ports (members named <name>.p<i>). "Usable" = not leased,
not on the deny list, not foreign-marked within foreign_ttl, and
test-bindable on 0.0.0.0; a failed test-bind marks the port "foreign"
so later picks skip it. The pool (SIM_PORT_POOL > 20000-32767) never
overlaps Linux's ephemeral range. Sticky by name: a live lease for
(worktree, sim, name) answers any request unchanged. alloc_id() hands
out non-port IDs (space "sysid" = 1..250) that never hit the registry -
they live in the sim manifest. commit() returns one flat {name: int}
covering request names, block members and ids; tx.ids is just the ids.

    alloc = Allocator()
    with alloc.transaction("copters3", SIM_ROOT) as tx:
        tx.alloc("server.hq.wss", proto="tcp")
        tx.alloc("unit.c1.de_comm", proto="udp", prefer=62100)
        tx.alloc("unit.c1.de_comm2", proto="udp", pin=62111)
        tx.alloc_block("unit.c1.sitl", size=10, proto="tcp")
        tx.alloc_id("unit.c1.sysid", prefer=11)
        ports = tx.commit()                # or AllocError(.conflicts)

    alloc.check(sim, worktree, requests)   dry run -> [conflict dicts]
    alloc.hints(sim, worktree)             -> {name: port} a request
                                           would most likely get (live
                                           lease, else remembered)
    alloc.mark_bound(sim, names)           launcher, after readiness
    alloc.lookup(sim, name)                -> port or None
    alloc.leases()                         -> [lease, ...] (+ "stale")
    alloc.release(sim, worktree)           ./sim down
    alloc.reassign(sim, name, worktree)    bind-failure recovery
    alloc.gc()                             sweep by hand

Timestamps: created/touched are ISO-8601 UTC strings; "foreign" marks
and last_ports entries keep float epoch seconds.
"""

import contextlib
import fcntl
import json
import os
import re
import socket
import sys
import time

REGISTRY_VERSION = 1
ID_SPACES = {"sysid": (1, 250)}
DEFAULT_POOL = "20000-32767"
DEFAULT_DENY = "3000,9966,19408,19001,9000,8089,5760-5859"
PROTOS = ("tcp", "udp", "both")


def _now_iso():
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _parse_ranges(spec):
    """"20000-32767,40000" -> sorted merged [(lo, hi), ...]; also accepts
    a ready list of ints / (lo, hi) pairs."""
    out = []
    items = spec.split(",") if isinstance(spec, str) else spec
    for it in items:
        try:
            if isinstance(it, str):
                it = it.strip()
                if not it:
                    continue
                lo_s, _, hi_s = it.partition("-")
                lo, hi = int(lo_s), int(hi_s or lo_s)
            elif isinstance(it, int):
                lo = hi = it
            else:
                lo, hi = int(it[0]), int(it[1])
        except (TypeError, ValueError, IndexError):
            raise ValueError(f"bad port range {it!r}")
        if lo > hi:
            lo, hi = hi, lo
        out.append((lo, hi))
    out.sort()
    merged = []
    for lo, hi in out:
        if merged and lo <= merged[-1][1] + 1:
            merged[-1] = (merged[-1][0], max(merged[-1][1], hi))
        else:
            merged.append((lo, hi))
    return merged


def _in_ranges(ranges, port):
    return any(lo <= port <= hi for lo, hi in ranges)


def _subtract_range(ranges, lo, hi):
    out = []
    for a, b in ranges:
        if b < lo or a > hi:
            out.append((a, b))
            continue
        if a < lo:
            out.append((a, lo - 1))
        if b > hi:
            out.append((hi + 1, b))
    return out


def _proto_clash(a, b):
    """A lease blocks a request when the transport spaces overlap."""
    return a == "both" or b == "both" or a == b


def _as_request(r):
    """check() accepts Request objects and plain dicts alike."""
    if isinstance(r, Request):
        return r
    if not isinstance(r, dict) or "name" not in r:
        raise ValueError(f"bad request {r!r}: need a Request or a dict "
                         "with a 'name'")
    return Request(**{k: r[k] for k in
                      ("name", "proto", "kind", "prefer", "pin", "size",
                       "path")
                      if k in r})


def _remove_lease_dirs(leases):
    """rmtree the "path" of every path-kind lease - the per-unit
    unix-socket dirs live under de_sim/s/ and belong to the lease alone."""
    import shutil
    for l in leases:
        p = l.get("path")
        if l.get("kind") == "path" and isinstance(p, str) and p:
            shutil.rmtree(p, ignore_errors=True)


def _conflict_msg(sim, conflicts):
    parts = []
    for c in conflicts:
        h = c["holder"]
        if h == "foreign":
            who = "foreign"
        elif isinstance(h, dict):
            who = f"{h.get('sim')}@{h.get('worktree')}"
        else:
            who = "-"
        parts.append(f"{c['name']}:{c['port']} {c['reason']} ({who})")
    return f"port allocation for sim {sim!r} failed: " + "; ".join(parts)


class AllocError(Exception):
    """One or more requests could not be satisfied. .conflicts is a list
    of {"name", "port", "reason", "holder"} - holder is {"sim",
    "worktree"} for a registry lease, "foreign" for a port bound by a
    non-sim process, None when the pool / id space is exhausted."""

    def __init__(self, message, conflicts=()):
        super().__init__(message)
        self.conflicts = list(conflicts)


class Request:
    """One queued request - a plain record, no behaviour."""

    def __init__(self, name, proto="tcp", kind="free",
                 prefer=None, pin=None, size=None, path=None):
        self.name = name            # dotted: <owner>.<id>.<role>
        self.proto = proto          # tcp | udp | both | path
        self.kind = kind            # free | block | prefer | pin | path
        self.prefer = prefer        # try this port/base first
        self.pin = pin              # exactly this port/base, or fail
        self.size = size            # block length, for kind "block"
        self.path = path            # directory to lease, for kind "path"

    def __repr__(self):
        return (f"Request({self.name!r} proto={self.proto!r} "
                f"kind={self.kind!r} prefer={self.prefer} "
                f"pin={self.pin} size={self.size} path={self.path!r})")


class Allocator:
    """All picks happen inside the ports.lock flock. Read helpers
    (lookup/leases) go lock-free: registry writes are atomic os.replace.
    worktree args normalize through realpath; the constructor's worktree
    is a fallback default, and for lookup/leases/mark_bound None means
    'any worktree'."""

    def __init__(self, registry=None, last=None, pool=None, deny=None,
                 worktree=None, proc_root="/proc", foreign_ttl=60.0,
                 last_ttl=30 * 86400, last_max=4096):
        self.registry = os.path.abspath(os.path.expanduser(
            registry or os.environ.get("SIM_PORT_REGISTRY") or
            self._default_registry()))
        d = os.path.dirname(self.registry)
        self.lock_path = os.path.join(d or ".", "ports.lock")
        self.last_path = os.path.abspath(os.path.expanduser(
            last or os.environ.get("SIM_PORT_LAST") or
            "~/.cache/de_sim/last_ports.json"))
        self.worktree = os.path.realpath(worktree) if worktree else None
        self.proc_root = proc_root
        self.foreign_ttl = float(foreign_ttl)
        self.last_ttl = float(last_ttl)
        self.last_max = int(last_max)

        self.pool = _parse_ranges(
            pool if pool is not None
            else os.environ.get("SIM_PORT_POOL", DEFAULT_POOL))
        eph = self._ephemeral_range()
        if eph:
            self.pool = _subtract_range(self.pool, eph[0], eph[1])
        if not self.pool:
            raise AllocError("port pool empty after subtracting the "
                             "ephemeral range")
        self.deny = _parse_ranges(
            deny if deny is not None
            else os.environ.get("SIM_PORT_DENY", DEFAULT_DENY))

    @staticmethod
    def _default_registry():
        xdg = os.environ.get("XDG_RUNTIME_DIR")
        base = xdg if xdg else os.path.expanduser("~/.cache")
        return os.path.join(base, "de_sim", "ports.json")

    def _ephemeral_range(self):
        """Linux's ephemeral range; free picks never land in it (the
        module blocks 54000-60999 sit inside it on this host)."""
        try:
            with open(os.path.join(
                    self.proc_root,
                    "sys/net/ipv4/ip_local_port_range")) as f:
                lo, hi = f.read().split()[:2]
            return int(lo), int(hi)
        except (OSError, ValueError):
            return None

    def _wt(self, worktree, required=False):
        """worktree arg > the Allocator default; realpath'd. None means
        'any' where the caller allows it."""
        w = worktree if worktree is not None else self.worktree
        if w is None:
            if required:
                raise ValueError("worktree is required")
            return None
        return os.path.realpath(w)

    # ----------------------------------------------------------- files

    @contextlib.contextmanager
    def _locked(self):
        """flock(LOCK_EX) on ports.lock for a whole read-mutate-write."""
        d = os.path.dirname(self.lock_path)
        if d:
            os.makedirs(d, exist_ok=True)
        fd = os.open(self.lock_path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX)
            yield
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)

    def _read_registry(self):
        empty = {"version": REGISTRY_VERSION, "leases": [], "foreign": {}}
        try:
            with open(self.registry) as f:
                reg = json.load(f)
        except FileNotFoundError:
            return empty
        except (OSError, ValueError):
            self._backup_corrupt()
            return empty
        if (not isinstance(reg, dict) or
                not isinstance(reg.get("leases"), list) or
                not isinstance(reg.get("foreign", {}), dict)):
            self._backup_corrupt()
            return empty
        foreign = {}
        for k, v in reg.get("foreign", {}).items():
            try:
                foreign[int(k)] = float(v)
            except (TypeError, ValueError):
                pass
        reg["foreign"] = foreign
        reg["version"] = REGISTRY_VERSION
        return reg

    def _backup_corrupt(self):
        bak = f"{self.registry}.bad-{int(time.time())}"
        with contextlib.suppress(OSError):
            os.replace(self.registry, bak)

    def _write_registry(self, reg):
        """tmp file + os.replace: readers never see a half-written file.
        Caller holds the lock."""
        now = time.time()
        reg["version"] = REGISTRY_VERSION
        reg["foreign"] = {p: t for p, t in reg["foreign"].items()
                          if now - t < self.foreign_ttl}
        d = os.path.dirname(self.registry)
        if d:
            os.makedirs(d, exist_ok=True)
        tmp = self.registry + ".tmp"
        with open(tmp, "w") as f:
            json.dump(reg, f, indent=2, sort_keys=True)
        os.replace(tmp, self.registry)

    def _read_last(self):
        try:
            with open(self.last_path) as f:
                d = json.load(f)
            return d if isinstance(d, dict) else {}
        except (OSError, ValueError):
            return {}

    def _write_last(self, last):
        """Caller holds ports.lock: memory and registry update together.
        Entries older than last_ttl are pruned, the file is capped at
        last_max by dropping the oldest."""
        now = time.time()
        last = {k: v for k, v in last.items()
                if now - v.get("t", 0) <= self.last_ttl}
        if len(last) > self.last_max:
            newest = sorted(last.items(),
                            key=lambda kv: kv[1].get("t", 0))[-self.last_max:]
            last = dict(newest)
        d = os.path.dirname(self.last_path)
        if d:
            os.makedirs(d, exist_ok=True)
        tmp = self.last_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(last, f, indent=2, sort_keys=True)
        os.replace(tmp, self.last_path)

    # ------------------------------------------------------------ /proc

    def _stat_start(self, pid):
        """Field 22 (starttime, jiffies) of <proc>/<pid>/stat, or None.
        Field 1 (comm) may contain spaces/parens, so parse from the last
        ')' - rest[0] is field 3, rest[19] is field 22."""
        try:
            with open(os.path.join(self.proc_root, str(pid), "stat")) as f:
                line = f.read()
            rest = line[line.rindex(")") + 2:].split()
            return int(rest[19])
        except (OSError, ValueError, IndexError):
            return None

    def _pid_alive(self, pid, start):
        """Alive iff /proc/<pid>/stat exists and starttime matches the
        recorded one - a reused pid never matches."""
        return (pid is not None and start is not None and
                self._stat_start(pid) == start)

    def _sim_has_live_pids(self, worktree, sim):
        """Any *.pid under <worktree>/var/run/<sim>/pids naming a live
        process whose cmdline points into that worktree."""
        d = os.path.join(worktree, "var", "run", sim, "pids")
        if not os.path.isdir(d):
            return False
        for f in os.listdir(d):
            if not f.endswith(".pid"):
                continue
            try:
                with open(os.path.join(d, f)) as fh:
                    pid = int(fh.read().strip())
            except (OSError, ValueError):
                continue
            if self._sim_pid_alive(pid, worktree):
                return True
        return False

    def _sim_pid_alive(self, pid, worktree):
        if self._stat_start(pid) is None:
            return False
        try:
            with open(os.path.join(self.proc_root, str(pid), "cmdline"),
                      "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except OSError:
            return False
        return (worktree + "/var/run" in cmd or
                worktree + "/engine" in cmd)

    # ------------------------------------------------------------ sweep

    def _stale(self, lease):
        """Would the sweep reclaim this lease? Worktree gone, or dead
        holder + no live sim pids, or dead holder on a still-'reserved'
        lease (./sim up killed mid bring-up)."""
        wt = lease.get("worktree") or ""
        if not wt or not os.path.isdir(wt):
            return True
        holder = lease.get("holder") or {}
        if self._pid_alive(holder.get("pid"), holder.get("start")):
            return False
        if lease.get("state") == "reserved":
            return True
        return not self._sim_has_live_pids(wt, lease.get("sim") or "")

    def _sweep(self, reg):
        """Drop stale leases and expired foreign marks in place; returns
        (reclaimed lease dicts, changed). A reclaimed "path" lease's
        directory is removed - the sim that owned it is gone."""
        now = time.time()
        dead = [l for l in reg["leases"] if self._stale(l)]
        if dead:
            reg["leases"] = [l for l in reg["leases"]
                             if not self._stale(l)]
            _remove_lease_dirs(dead)
        before_f = len(reg["foreign"])
        reg["foreign"] = {p: t for p, t in reg["foreign"].items()
                          if now - t < self.foreign_ttl}
        return dead, bool(dead) or len(reg["foreign"]) != before_f

    # ------------------------------------------------------------- bind

    @staticmethod
    def _try_bind(sock_type, port):
        s = socket.socket(socket.AF_INET, sock_type)
        try:
            if sock_type == socket.SOCK_STREAM:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", port))
            return True
        except OSError:
            return False
        finally:
            s.close()

    def _test_bind(self, port, proto):
        ok = True
        if proto in ("tcp", "both"):
            ok = self._try_bind(socket.SOCK_STREAM, port) and ok
        if proto in ("udp", "both"):
            ok = self._try_bind(socket.SOCK_DGRAM, port) and ok
        return ok

    # -------------------------------------------------------- evaluation

    def _evaluate(self, sim, worktree, reqs, id_reqs, reg, last_map):
        """Pick ports for every queued request against the post-sweep
        registry (in memory). Returns (result, new_leases, dropped,
        idmap, conflicts); reg["foreign"] may gain marks. Dry-run safe.
        `dropped` lists superseded leases - a request that re-pins a
        name to a different port - for the caller to delete on write."""
        now = time.time()
        now_iso = _now_iso()
        holder = {"pid": os.getpid(),
                  "start": self._stat_start(os.getpid())}
        held = list(reg["leases"])
        new_leases = []
        dropped = []
        result = {}
        idmap = {}
        conflicts = []
        used_ids = {}

        def sticky(name):
            for l in held:
                if (l["worktree"] == worktree and l["sim"] == sim and
                        l["name"] == name):
                    return l
            return None

        def supersede(name, lease):
            """Drop `lease` from `held` and queue it for deletion - its
            name is being re-pinned to a different port, so the lease
            must move or the registry no longer names what's bound."""
            if lease in held:
                held.remove(lease)
            dropped.append(lease)

        def held_by(port, proto):
            for l in held:
                if l["port"] == port and _proto_clash(l["proto"], proto):
                    return l
            return None

        def take(name, port, proto, kind, block):
            lease = {"port": port, "proto": proto, "kind": kind,
                     "block": block, "sim": sim, "worktree": worktree,
                     "name": name, "state": "reserved",
                     "holder": dict(holder), "bound_by": None,
                     "created": now_iso, "touched": now_iso}
            new_leases.append(lease)
            held.append(lease)
            result[name] = port

        def conflict(name, port, reason, who):
            conflicts.append({"name": name, "port": port,
                              "reason": reason, "holder": who})

        def foreign_fresh(port):
            t = reg["foreign"].get(port)
            return t is not None and now - t < self.foreign_ttl

        def takeable(port, proto):
            """Free/prefer/block-scan candidate usable right now; a
            failed bind is remembered 'foreign' so later picks skip it."""
            if held_by(port, proto):
                return False
            if _in_ranges(self.deny, port):
                return False
            if foreign_fresh(port):
                return False
            if not self._test_bind(port, proto):
                reg["foreign"][port] = now
                return False
            return True

        def remembered(name):
            e = last_map.get(f"{worktree}|{sim}|{name}")
            if e and now - e.get("t", 0) <= self.last_ttl:
                return e.get("port")
            return None

        def block_members_sticky(name):
            pat = re.compile(re.escape(name) + r"\.p\d+$")
            return [l for l in held
                    if l["worktree"] == worktree and l["sim"] == sim and
                    l.get("block") and pat.fullmatch(l["name"])]

        def block_supersede(name, pin, members):
            """A re-pinned block base moves every member lease."""
            if pin is not None and members and \
                    members[0]["block"]["base"] != pin:
                for m in members:
                    supersede(name, m)
                return []
            return members

        def eval_block(req):
            size = req.size
            if not isinstance(size, int) or size < 1:
                conflict(req.name, None, "bad-request", None)
                return
            members = block_supersede(req.name, req.pin,
                                      block_members_sticky(req.name))
            if members:
                result[req.name] = members[0]["block"]["base"]
                for m in members:
                    result[m["name"]] = m["port"]
                return
            if req.pin is not None:
                base = req.pin
                if (not isinstance(base, int) or base < 1 or
                        base + size - 1 > 65535):
                    conflict(req.name, base, "bad-port", None)
                    return
                bad = 0
                for i in range(size):
                    p = base + i
                    l = held_by(p, req.proto)
                    if l is not None:
                        conflict(f"{req.name}.p{i}", p, "leased",
                                 {"sim": l["sim"],
                                  "worktree": l["worktree"]})
                        bad += 1
                    elif not self._test_bind(p, req.proto):
                        reg["foreign"][p] = now
                        conflict(f"{req.name}.p{i}", p, "held", "foreign")
                        bad += 1
                if bad:
                    return
            else:
                base = None
                seeds = []
                if req.prefer is not None:
                    seeds.append(req.prefer)
                mem = remembered(req.name)
                if mem is not None:
                    seeds.append(mem)
                for b in seeds:
                    if (isinstance(b, int) and 1 <= b and
                            b + size - 1 <= 65535 and
                            all(takeable(b + i, req.proto)
                                for i in range(size))):
                        base = b
                        break
                if base is None:
                    base = self._pool_block(size, req.proto, takeable)
                if base is None:
                    conflict(req.name, None, "no-free-block", None)
                    return
            blk = {"base": base, "size": size}
            result[req.name] = base
            for i in range(size):
                take(f"{req.name}.p{i}", base + i, req.proto, "block", blk)

        def eval_path(req):
            """kind "path" - a directory lease (SL-0086, e.g. a unit's
            unix-socket dir). No port is picked: the lease records the
            caller's path so ./sim ports shows it and release()/the sweep
            remove the directory. Two live leases cannot hold the same
            path."""
            if not isinstance(req.path, str) or not req.path:
                conflict(req.name, None, "bad-request", None)
                return
            l = sticky(req.name)
            if l is not None:
                if l.get("kind") != "path" or l.get("path") != req.path:
                    supersede(req.name, l)
                    l = None
            if l is not None:
                result[req.name] = l["path"]
                return
            for o in held:
                if o.get("path") == req.path:
                    conflict(req.name, None, "leased",
                             {"sim": o["sim"], "worktree": o["worktree"]})
                    return
            lease = {"port": None, "path": req.path, "proto": "path",
                     "kind": "path", "block": None, "sim": sim,
                     "worktree": worktree, "name": req.name,
                     "state": "reserved", "holder": dict(holder),
                     "bound_by": None,
                     "created": now_iso, "touched": now_iso}
            new_leases.append(lease)
            held.append(lease)
            result[req.name] = req.path

        for req in reqs:
            if req.kind == "path" or req.proto == "path":
                eval_path(req)
                continue
            if req.proto not in PROTOS:
                conflict(req.name, None, "bad-request", None)
                continue
            # sticky by name: a live lease for (worktree, sim, name)
            # answers every kind of request unchanged - except an
            # explicit pin to a DIFFERENT port or a kind change, which
            # re-pins it
            l = sticky(req.name)
            if l is not None:
                block_req = req.size is not None or req.kind == "block"
                if (block_req and not l.get("block")) or \
                        (req.pin is not None and not l.get("block") and
                         l["port"] != req.pin):
                    supersede(req.name, l)
                    l = None
            if l is not None:
                result[req.name] = l["port"]
                continue
            if req.size is not None or req.kind == "block":
                eval_block(req)
            elif req.pin is not None or req.kind == "pin":
                p = req.pin
                if not isinstance(p, int) or not 1 <= p <= 65535:
                    conflict(req.name, p, "bad-port", None)
                    continue
                l = held_by(p, req.proto)
                if l is not None:
                    conflict(req.name, p, "leased",
                             {"sim": l["sim"], "worktree": l["worktree"]})
                elif not self._test_bind(p, req.proto):
                    reg["foreign"][p] = now
                    conflict(req.name, p, "held", "foreign")
                else:
                    reg["foreign"].pop(p, None)
                    take(req.name, p, req.proto, "pin", None)
            else:
                cands = []
                if req.prefer is not None:
                    cands.append(req.prefer)
                mem = remembered(req.name)
                if mem is not None:
                    cands.append(mem)
                port = None
                for c in cands:
                    if (isinstance(c, int) and 1 <= c <= 65535 and
                            takeable(c, req.proto)):
                        port = c
                        break
                if port is None:
                    port = self._pool_take(req.proto, takeable)
                if port is None:
                    conflict(req.name, None, "no-free-port", None)
                else:
                    take(req.name, port, req.proto,
                         "prefer" if req.prefer is not None else "free",
                         None)

        # ids are not ports: they never touch the registry, they only
        # need to be unique inside this transaction
        for name, space, prefer in id_reqs:
            if name in result:
                continue
            lo_hi = ID_SPACES.get(space)
            if lo_hi is None:
                conflict(name, None, "bad-space", None)
                continue
            lo, hi = lo_hi
            used = used_ids.setdefault(space, set())
            value = None
            if (isinstance(prefer, int) and lo <= prefer <= hi and
                    prefer not in used):
                value = prefer
            else:
                for cand in range(lo, hi + 1):
                    if cand not in used:
                        value = cand
                        break
            if value is None:
                conflict(name, None, "space-exhausted", None)
            else:
                used.add(value)
                result[name] = value
                idmap[name] = value

        return result, new_leases, dropped, idmap, conflicts

    def _pool_take(self, proto, takeable):
        for lo, hi in self.pool:
            for p in range(lo, hi + 1):
                if takeable(p, proto):
                    return p
        return None

    def _pool_block(self, size, proto, takeable):
        """Lowest pool base with a run of `size` usable ports; the scan
        jumps past the first bad member each round."""
        for lo, hi in self.pool:
            base = lo
            while base + size - 1 <= hi:
                bad = None
                for i in range(size):
                    if not takeable(base + i, proto):
                        bad = base + i
                        break
                if bad is None:
                    return base
                base = bad + 1
        return None

    # -------------------------------------------------------------- api

    def transaction(self, sim, worktree=None):
        """One flock for the whole read -> sweep -> mutate -> write."""
        return _Transaction(self, sim, self._wt(worktree, required=True))

    def check(self, sim, worktree, requests):
        """Dry-run of commit() over Request objects or plain dicts: runs
        the sweep, test-binds, returns [conflict dicts], writes nothing."""
        w = self._wt(worktree, required=True)
        reqs = [_as_request(r) for r in requests]
        with self._locked():
            reg = self._read_registry()
            self._sweep(reg)
            _res, _nl, _dd, _ids, conflicts = self._evaluate(
                sim, w, reqs, [], reg, self._read_last())
        return conflicts

    def hints(self, sim, worktree=None):
        """name -> the port a fresh request would most likely get right
        now: a live (worktree, sim, name) lease first (a block's name
        maps to its base), else the remembered port still inside
        last_ttl. Display-only (./sim show, doctor): the commit itself
        re-derives the same preferences under the lock."""
        w = self._wt(worktree)
        out = {}
        for l in self._read_registry()["leases"]:
            if (l["sim"] != sim or
                    (w is not None and l["worktree"] != w) or
                    self._stale(l)):
                continue
            if l.get("block"):
                out[re.sub(r"\.p\d+$", "", l["name"])] = l["block"]["base"]
            else:
                out[l["name"]] = l["port"]
        now = time.time()
        for k, e in self._read_last().items():
            wt_s, sim_s, name = k.rsplit("|", 2)
            if (sim_s != sim or name in out or
                    (w is not None and os.path.realpath(wt_s) != w) or
                    now - e.get("t", 0) > self.last_ttl):
                continue
            out[name] = e["port"]
        return out

    def mark_bound(self, sim, names, worktree=None):
        """reserved -> bound (+touched) on matching (worktree, sim, name)
        leases; names=None marks every lease the sim holds - the launcher
        uses it once the whole fleet is verified up."""
        w = self._wt(worktree)
        names = None if names is None else set(names)
        with self._locked():
            reg = self._read_registry()
            _dead, changed = self._sweep(reg)
            for l in reg["leases"]:
                if (l["sim"] == sim and
                        (names is None or l["name"] in names) and
                        (w is None or l["worktree"] == w)):
                    l["state"] = "bound"
                    l["touched"] = _now_iso()
                    changed = True
            if changed:
                self._write_registry(reg)

    def lookup(self, sim, name, worktree=None):
        """Live port for (worktree, sim, name) - the block's base for a
        block name, a member's port for a <name>.p<i> - else None."""
        w = self._wt(worktree)
        pat = re.compile(re.escape(name) + r"\.p\d+$")
        for l in self._read_registry()["leases"]:
            if (l["sim"] != sim or
                    (w is not None and l["worktree"] != w) or
                    self._stale(l)):
                continue
            if l["name"] == name:
                return l["path"] if l.get("kind") == "path" else l["port"]
            if l.get("block") and pat.fullmatch(l["name"]):
                return l["block"]["base"]
        return None

    def leases(self, sim=None, worktree=None):
        """Lease dicts as stored, plus "stale": true on ones the sweep
        would reclaim (staleness is evaluated read-only, nothing is
        written)."""
        w = self._wt(worktree)
        out = []
        for l in self._read_registry()["leases"]:
            if sim is not None and l["sim"] != sim:
                continue
            if w is not None and l["worktree"] != w:
                continue
            d = dict(l)
            if self._stale(l):
                d["stale"] = True
            out.append(d)
        return out

    def release(self, sim, worktree=None):
        """./sim down: delete the (worktree, sim) leases and remember
        each port in last_ports so the next run prefers it back."""
        w = self._wt(worktree, required=True)
        with self._locked():
            reg = self._read_registry()
            # `gone` is picked BEFORE the sweep: `sim down` kills the sim
            # processes before releasing, so its leases already look stale
            # (dead holder) and a sweep first would reclaim them without
            # ever remembering their ports.
            gone = [l for l in reg["leases"]
                    if l["sim"] == sim and l["worktree"] == w]
            if gone:
                reg["leases"] = [l for l in reg["leases"]
                                 if not (l["sim"] == sim and
                                         l["worktree"] == w)]
            _dead, changed = self._sweep(reg)
            if gone:
                changed = True
            if changed:
                self._write_registry(reg)
            if gone:
                _remove_lease_dirs(gone)
                last = self._read_last()
                now = time.time()
                for l in gone:
                    if l.get("kind") == "path":
                        continue
                    last[f"{w}|{sim}|{l['name']}"] = {
                        "port": l["port"], "t": now}
                    if l.get("block"):
                        m = re.fullmatch(r"(.*)\.p\d+", l["name"])
                        if m:
                            last[f"{w}|{sim}|{m.group(1)}"] = {
                                "port": l["block"]["base"], "t": now}
                self._write_last(last)
        return gone

    def reassign(self, sim, name, worktree=None):
        """Drop the named lease and take a fresh free port under the same
        name/proto - the launcher's recovery when a process dies on bind
        at start. AllocError when the pool has nothing."""
        w = self._wt(worktree, required=True)
        with self._locked():
            reg = self._read_registry()
            self._sweep(reg)
            old = next((l for l in reg["leases"]
                        if l["sim"] == sim and l["worktree"] == w and
                        l["name"] == name), None)
            if old is None:
                raise AllocError(
                    f"reassign: no lease {name!r} for sim {sim!r}")
            reg["leases"] = [l for l in reg["leases"] if l is not old]
            result, new_leases, dropped, _ids, conflicts = \
                self._evaluate(
                    sim, w, [Request(name, proto=old["proto"])], [], reg,
                    self._read_last())
            if conflicts:
                raise AllocError(_conflict_msg(sim, conflicts), conflicts)
            reg["leases"] = [l for l in reg["leases"] if l not in dropped]
            reg["leases"].extend(new_leases)
            self._write_registry(reg)
        return result[name]

    def gc(self):
        """Run the stale-lease sweep by hand -> [reclaimed lease dicts]."""
        with self._locked():
            reg = self._read_registry()
            dead, changed = self._sweep(reg)
            if changed:
                self._write_registry(reg)
        return dead


class _Transaction:
    """One sim's queued requests under one flock. alloc/alloc_block/
    alloc_id only queue; commit() picks under the lock and writes
    all-or-nothing, returning one flat {name: int} covering request
    names, block members (<name>.p<i>) and ids - tx.ids is just the id
    part. The lock is taken on __enter__ (creating ports.lock), the
    sweep runs then; an exception before commit writes nothing."""

    def __init__(self, allocator, sim, worktree):
        self.allocator = allocator
        self.sim = sim
        self.worktree = worktree
        self.reqs = []
        self.id_reqs = []
        self.ids = {}
        self._cm = None
        self._registry = None
        self._last = {}
        self._committed = False

    # ---------------------------------------------------------- queueing

    def _open(self):
        if self._committed:
            raise RuntimeError("transaction already committed")

    def alloc(self, name, proto="tcp", kind=None, prefer=None, pin=None,
              size=None, path=None):
        """Queue one port request. kind comes from the args: pin set ->
        'pin', prefer set -> 'prefer', else 'free' ('block' via size,
        'path' via path)."""
        self._open()
        if pin is not None and prefer is not None:
            raise ValueError("prefer and pin are mutually exclusive")
        if kind is None:
            kind = ("path" if path is not None else
                    "pin" if pin is not None else
                    "prefer" if prefer is not None else "free")
        req = Request(name, proto=proto, kind=kind,
                      prefer=prefer, pin=pin, size=size, path=path)
        self.reqs.append(req)
        return req

    def alloc_path(self, name, path):
        """Queue a directory lease (kind "path") - no port is picked;
        release()/the sweep remove the directory."""
        self._open()
        req = Request(name, proto="path", kind="path", path=path)
        self.reqs.append(req)
        return req

    def alloc_block(self, name, size, proto="tcp", prefer=None, pin=None):
        """Queue a run of `size` consecutive ports; commit maps name ->
        base plus name.p0 .. name.p{size-1} -> each member."""
        self._open()
        if pin is not None and prefer is not None:
            raise ValueError("prefer and pin are mutually exclusive")
        req = Request(name, proto=proto, kind="block",
                      prefer=prefer, pin=pin, size=size)
        self.reqs.append(req)
        return req

    def alloc_id(self, name, space="sysid", prefer=None):
        """Queue a non-port ID (spaces: {"sysid": (1, 250)}); prefer wins
        if in range and unused in this tx, else the lowest free value.
        Never hits the registry - ids live in the sim manifest."""
        self._open()
        if space not in ID_SPACES:
            raise ValueError(f"unknown id space {space!r}")
        self.id_reqs.append((name, space, prefer))

    # -------------------------------------------------------------- lock

    def __enter__(self):
        if self._cm is None:
            self._cm = self.allocator._locked()
            self._cm.__enter__()
            self._registry = self.allocator._read_registry()
            self.allocator._sweep(self._registry)
            self._last = self.allocator._read_last()
        return self

    def __exit__(self, exc_type, exc, tb):
        self._release()
        return False

    def _release(self):
        cm, self._cm = self._cm, None
        if cm is not None:
            cm.__exit__(None, None, None)

    def commit(self):
        """Pick everything under the lock, write ports.json, return the
        flat {name: int} map (ports + block members + ids; the ids are
        also in tx.ids). AllocError(.conflicts) writes nothing."""
        if self._committed:
            raise RuntimeError("transaction already committed")
        self.__enter__()
        result, new_leases, dropped, idmap, conflicts = \
            self.allocator._evaluate(
                self.sim, self.worktree, self.reqs, self.id_reqs,
                self._registry, self._last)
        if conflicts:
            raise AllocError(_conflict_msg(self.sim, conflicts), conflicts)
        self._registry["leases"] = [l for l in self._registry["leases"]
                                    if l not in dropped]
        self._registry["leases"].extend(new_leases)
        self.allocator._write_registry(self._registry)
        self.ids = idmap
        self._committed = True
        self._release()
        return result


def main(argv):
    """`python3 port_allocation_management.py [leases [sim] | gc]`."""
    cmd = argv[1] if len(argv) > 1 else "leases"
    alloc = Allocator()
    if cmd == "gc":
        for l in alloc.gc():
            print(f"reclaimed {l['sim']}:{l['name']} "
                  f"{l.get('path') or 'port ' + str(l['port'])} "
                  f"({l['worktree']})")
        return 0
    if cmd == "leases":
        sim = argv[2] if len(argv) > 2 else None
        for l in alloc.leases(sim=sim):
            stale = "STALE" if l.get("stale") else ""
            what = l.get("path") if l.get("kind") == "path" else l["port"]
            print(f"{str(what):<40} {l['proto']:<5} {l['state']:<9} "
                  f"{stale:<6} {l['sim']:<20} {l['name']:<32} "
                  f"{l['worktree']}")
        return 0
    print(f"usage: {argv[0]} [leases [sim] | gc]", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
