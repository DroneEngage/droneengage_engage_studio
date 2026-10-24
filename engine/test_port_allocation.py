#!/usr/bin/env python3
"""Offline unit tests for port_allocation_management.py - the machine-wide
port-lease allocator (docs/port_allocation.md). No SITL, no real registry:
every test runs on a temp registry + temp last_ports and, where it matters,
a fake /proc tree:

    python3 engine/test_port_allocation.py
"""
import json
import multiprocessing
import os
import socket
import sys
import tempfile
import unittest

# bytecode under var/pycache from the first import on (same preamble as
# test_checks.py)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import port_allocation_management as pam


def _mk(proc_root="/proc", **kw):
    """An Allocator on a fresh temp registry + last_ports; the returned
    tempdir doubles as the sim's worktree (it exists, so the sweep keeps
    leases that point at it)."""
    d = tempfile.mkdtemp(prefix="pam_test_")
    kw.setdefault("registry", os.path.join(d, "de_sim", "ports.json"))
    kw.setdefault("last", os.path.join(d, "de_sim", "last_ports.json"))
    kw.setdefault("pool", "30000-30099")
    kw.setdefault("deny", "30050")
    kw.setdefault("proc_root", proc_root)
    return pam.Allocator(**kw), d


def _fake_proc(pid, start=None, cmdline=None):
    """A /proc/<pid> dir: stat carries `start` as field 22, cmdline the
    NUL-separated argv bytes."""
    d = tempfile.mkdtemp(prefix="pam_proc_")
    pd = os.path.join(d, str(pid))
    os.makedirs(pd)
    with open(os.path.join(pd, "stat"), "w") as f:
        # field 3 is state; fields 4..21 are 18 counters; field 22 is
        # the starttime the allocator compares
        f.write(f"{pid} (comm) S " + "0 " * 18 + f"{start or 1} 0\n")
    if cmdline is not None:
        with open(os.path.join(pd, "cmdline"), "wb") as f:
            f.write(cmdline)
    return d


def _bind(port, proto="tcp"):
    """A foreign holder: bound (and listening) without SO_REUSEADDR, so
    the allocator's probe - which always sets it - fails like it would
    on any real bound port."""
    s = socket.socket(socket.AF_INET,
                      socket.SOCK_STREAM if proto == "tcp"
                      else socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", port))
    if proto == "tcp":
        s.listen(1)
    return s


def _lease(**kw):
    """A lease dict as the allocator writes it (for seeding tests)."""
    l = {"port": 30100, "proto": "tcp", "kind": "pin", "block": None,
         "sim": "s1", "worktree": "/wt", "name": "server.ctrl",
         "state": "bound", "holder": {"pid": 4242, "start": 99},
         "bound_by": None, "created": "2026-01-01T00:00:00Z",
         "touched": "2026-01-01T00:00:00Z"}
    l.update(kw)
    return l


def _write_reg(path, leases, foreign=None):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump({"version": 1, "leases": leases,
                   "foreign": foreign or {}}, f)


def _race(sim, registry, wt, pins, barrier, done, q):
    """Child: one transaction, pin every port, report outcome. `done`
    holds both children until both finished, so the winner's holder pid
    is still alive while the loser evaluates (else its reserved leases
    would be swept and the loser would win too)."""
    a = pam.Allocator(registry=registry,
                      last=registry + ".last", pool="30000-30099",
                      deny="30050")
    barrier.wait()
    try:
        with a.transaction(sim, wt) as tx:
            for p in pins:
                tx.alloc(f"unit.x.p{p}", proto="tcp", pin=p)
            q.put(("ok", tx.commit()))
    except pam.AllocError as e:
        q.put(("fail", [c["holder"]["sim"] for c in e.conflicts
                        if isinstance(c.get("holder"), dict)]))
    done.wait()


class T(unittest.TestCase):

    # -------------------------------------------------------- registry

    def test_commit_and_sticky(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        with a.transaction("s1", wt) as tx:
            tx.alloc("server.ctrl", proto="tcp", pin=30010)
            tx.alloc("unit.c1.bus", proto="udp")          # free pick
            ports = tx.commit()
        self.assertEqual(ports["server.ctrl"], 30010)
        self.assertTrue(30000 <= ports["unit.c1.bus"] <= 30099)
        self.assertNotEqual(ports["unit.c1.bus"], 30050)  # deny listed
        # same-name request is sticky: the existing lease answers it
        with a.transaction("s1", wt) as tx:
            tx.alloc("server.ctrl", proto="tcp")          # free -> sticky
            ports2 = tx.commit()
        self.assertEqual(ports2["server.ctrl"], 30010)

    def test_pin_conflict_names_holder(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        with a.transaction("s1", wt) as tx:
            tx.alloc("x", proto="tcp", pin=30010)
            tx.commit()
        with self.assertRaises(pam.AllocError) as cm:
            with a.transaction("s2", wt2) as tx:
                tx.alloc("y", proto="tcp", pin=30010)
                tx.commit()
        c = cm.exception.conflicts[0]
        self.assertEqual(c["reason"], "leased")
        self.assertEqual(c["holder"]["sim"], "s1")

    def test_repinned_port_supersedes_own_lease(self):
        # a pin to a DIFFERENT port than the held lease moves the lease;
        # the registry must name what the configs will actually bind
        a, wt = _mk()
        with a.transaction("s1", wt) as tx:
            tx.alloc("server.wss", proto="tcp", pin=30010)
            tx.commit()
        with a.transaction("s1", wt) as tx:
            tx.alloc("server.wss", proto="tcp", pin=30020)
            ports = tx.commit()
        self.assertEqual(ports["server.wss"], 30020)
        leases = pam.Allocator(registry=a.registry).leases()
        self.assertEqual([(l["name"], l["port"]) for l in leases],
                         [("server.wss", 30020)])

    def test_repinned_port_conflicts_on_foreign(self):
        # same move, but the new port is taken: AllocError, and the old
        # lease survives the failed commit untouched
        a, wt = _mk()
        s = _bind(30020)
        try:
            with a.transaction("s1", wt) as tx:
                tx.alloc("server.wss", proto="tcp", pin=30010)
                tx.commit()
            with self.assertRaises(pam.AllocError):
                with a.transaction("s1", wt) as tx:
                    tx.alloc("server.wss", proto="tcp", pin=30020)
                    tx.commit()
            leases = pam.Allocator(registry=a.registry).leases()
            self.assertEqual([(l["name"], l["port"]) for l in leases],
                             [("server.wss", 30010)])
        finally:
            s.close()

    def test_failed_commit_writes_nothing(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        with self.assertRaises(pam.AllocError):
            with a.transaction("s1", wt) as tx:
                tx.alloc("ok", proto="tcp", pin=30010)
                tx.alloc("bad", proto="tcp", pin=-1)
                tx.commit()
        self.assertEqual(pam.Allocator(
            registry=a.registry).leases(), [])

    def test_foreign_bind_blocks_pin_and_free_pick(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        s = _bind(30020)
        try:
            with self.assertRaises(pam.AllocError) as cm:
                with a.transaction("s1", wt) as tx:
                    tx.alloc("x", proto="tcp", pin=30020)
                    tx.commit()
            self.assertEqual(cm.exception.conflicts[0]["holder"],
                             "foreign")
        finally:
            s.close()
        # the mark lives 60 s: a free pick must skip it even after the
        # socket closed
        with a.transaction("s1", wt) as tx:
            tx.alloc("y", proto="tcp")
            ports = tx.commit()
        self.assertNotEqual(ports["y"], 30020)

    def test_block_and_id(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        with a.transaction("s1", wt) as tx:
            tx.alloc_block("sitl", 4, proto="tcp", pin=30060)
            tx.alloc_id("sysid", prefer=11)
            tx.alloc_id("sysid2", prefer=11)    # taken - lowest free
            ports = tx.commit()
        self.assertEqual(ports["sitl"], 30060)
        self.assertEqual([ports[f"sitl.p{i}"] for i in range(4)],
                         [30060, 30061, 30062, 30063])
        self.assertEqual(ports["sysid"], 11)
        self.assertEqual(ports["sysid2"], 1)
        self.assertEqual(tx.ids, {"sysid": 11, "sysid2": 1})
        # a second sim pinning a member port is refused
        with self.assertRaises(pam.AllocError):
            with a.transaction("s2", wt2) as tx:
                tx.alloc("y", proto="tcp", pin=30061)
                tx.commit()
        # lookup: block name -> base, member -> its port
        self.assertEqual(a.lookup("s1", "sitl", wt), 30060)
        self.assertEqual(a.lookup("s1", "sitl.p2", wt), 30062)

    def test_release_and_remembered_port(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        with a.transaction("s1", wt) as tx:
            tx.alloc("unit.c1.bus", proto="udp")
            p = tx.commit()["unit.c1.bus"]
        gone = a.release("s1", wt)
        self.assertEqual(len(gone), 1)
        self.assertEqual(a.leases(), [])
        # the next free pick for the same name gets the port back
        with a.transaction("s1", wt) as tx:
            tx.alloc("unit.c1.bus", proto="udp")
            self.assertEqual(tx.commit()["unit.c1.bus"], p)

    def test_release_remembers_after_sim_death(self):
        """`sim down` order: kill the sim's processes first, THEN call
        release. By then the lease already looks stale (dead holder, no
        sim pids) - release must still remove it AND remember its port,
        not let the sweep reclaim it silently."""
        a, wt = _mk()
        proc = _fake_proc(4242, start=99)          # holder start
        _write_reg(a.registry,
                   [_lease(port=30100, name="unit.c1.bus", proto="udp",
                           worktree=wt,
                           holder={"pid": 4242, "start": 999})])  # dead
        a2 = pam.Allocator(registry=a.registry, last=a.last_path,
                           pool="30000-30099", deny="30050",
                           proc_root=proc)
        gone = a2.release("s1", wt)
        self.assertEqual([l["port"] for l in gone], [30100])
        self.assertEqual(a2.leases(), [])
        # remembered despite the dead holder: a same-name request
        # re-leases the released port, not a pool pick
        with a2.transaction("s1", wt) as tx:
            tx.alloc("unit.c1.bus", proto="udp")
            self.assertEqual(tx.commit()["unit.c1.bus"], 30100)

    def test_prefer_skips_foreign_listener(self):
        """a foreign bind on the prefer hint is skipped, not an error:
        the lease lands on the next takeable port (the pool) - and the
        same dry run reports no conflict"""
        a, wt = _mk()
        s = _bind(30020)
        try:
            with a.transaction("s1", wt) as tx:
                tx.alloc("unit.c1.link_proxy", proto="tcp",
                         prefer=30020)
                ports = tx.commit()
            self.assertNotEqual(ports["unit.c1.link_proxy"], 30020)
            self.assertTrue(30000 <= ports["unit.c1.link_proxy"] <= 30099)
        finally:
            s.close()
        s2 = _bind(30021)
        try:
            self.assertEqual(
                a.check("s2", wt, [{"name": "y", "proto": "tcp",
                                    "kind": "prefer",
                                    "prefer": 30021}]), [])
        finally:
            s2.close()

    def test_hints_report_live_and_remembered_ports(self):
        """hints() is the display-side guess at what a request would
        land (./sim show, doctor, simlib._apply_hints): a live same-sim
        lease wins, a block's name maps to its base, and a released
        name answers from the remembered-port file. Another sim's
        leases never leak in, and a stale lease is skipped."""
        a, wt = _mk()
        with a.transaction("s1", wt) as tx:
            tx.alloc("server.wss", proto="tcp", pin=30010)
            tx.alloc_block("unit.c1.sitl", 4, proto="tcp", pin=30060)
            tx.commit()
        self.assertEqual(a.hints("s1", wt),
                         {"server.wss": 30010, "unit.c1.sitl": 30060})
        # another sim sees nothing of s1's leases
        self.assertEqual(a.hints("s2", wt), {})
        # released: the remembered port answers the same name
        a.release("s1", wt)
        self.assertEqual(a.hints("s1", wt)["server.wss"], 30010)

    def test_hints_skip_stale_lease(self):
        """a lease whose worktree is gone is stale - hints() ignores it
        instead of predicting a port the commit would reclaim"""
        a, wt = _mk()
        _write_reg(a.registry,
                   [_lease(port=30100, worktree="/definitely/not/here",
                           holder={"pid": 4242, "start": 99})])
        self.assertEqual(a.hints("s1", wt), {})

    def test_check_dry_run_writes_nothing(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        with a.transaction("s1", wt) as tx:
            tx.alloc("x", proto="tcp", pin=30010)
            tx.commit()
        conflicts = a.check("s2", wt2,
                            [{"name": "y", "proto": "tcp", "pin": 30010},
                             {"name": "z", "proto": "tcp", "pin": 30011}])
        self.assertEqual(len(conflicts), 1)
        self.assertEqual(conflicts[0]["port"], 30010)
        # still just s1's lease - the dry run reserved nothing
        self.assertEqual(len(a.leases()), 1)

    def test_mark_bound_lookup_reassign(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        with a.transaction("s1", wt) as tx:
            tx.alloc("x", proto="tcp")
            tx.commit()
        self.assertEqual(a.leases()[0]["state"], "reserved")
        a.mark_bound("s1", None, wt)
        self.assertEqual(a.leases()[0]["state"], "bound")
        new = a.reassign("s1", "x", wt)
        self.assertIsInstance(new, int)
        self.assertEqual(a.lookup("s1", "x", wt), new)

    def test_corrupt_registry_moves_aside(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        os.makedirs(os.path.dirname(a.registry), exist_ok=True)
        with open(a.registry, "w") as f:
            f.write("{not json")
        self.assertEqual(a.leases(), [])
        self.assertTrue(any(n.startswith("ports.json.bad")
                            for n in os.listdir(
                                os.path.dirname(a.registry))))

    # ---------------------------------------------------------- /proc

    def test_sweep_dead_holder_reserved(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        wt = tempfile.mkdtemp(prefix="pam_wt_")
        # a 'reserved' lease whose holder pid is dead -> reclaimed even
        # though nothing else is wrong (kill -9'd ./sim up)
        dead_proc = _fake_proc(4242, start=555)
        _write_reg(a.registry,
                   [_lease(port=30100, state="reserved",
                           worktree=wt,
                           holder={"pid": 4242, "start": 999})])
        a2 = pam.Allocator(registry=a.registry, last=a.last_path,
                           proc_root=dead_proc)
        gone = a2.gc()
        self.assertEqual([l["port"] for l in gone], [30100])
        self.assertEqual(a2.leases(), [])

    def test_sweep_keeps_live_holder(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        wt = tempfile.mkdtemp(prefix="pam_wt_")
        proc = _fake_proc(4242, start=99)
        _write_reg(a.registry,
                   [_lease(port=30100, worktree=wt,
                           holder={"pid": 4242, "start": 99})])
        a2 = pam.Allocator(registry=a.registry, last=a.last_path,
                           proc_root=proc)
        self.assertEqual(a2.gc(), [])
        self.assertEqual(len(a2.leases()), 1)

    def test_sweep_bound_lease_needs_live_sim_pids(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        wt = tempfile.mkdtemp(prefix="pam_wt_")
        proc = _fake_proc(4242, start=99)          # holder alive? no:
        # start recorded is 999 -> _pid_alive mismatches -> dead holder
        pids = os.path.join(wt, "var", "run", "s1", "pids")
        os.makedirs(pids)
        with open(os.path.join(pids, "unit.c1.pid"), "w") as f:
            f.write("777")
        # sim pid 777's cmdline points into the worktree -> sim alive
        sim_proc = _fake_proc(777, start=1,
                              cmdline=(b"bash\x00" + wt.encode() +
                                       b"/var/run/s1/procs/x.sh\x00"))
        merged = tempfile.mkdtemp(prefix="pam_proc_all_")
        for src, pid in ((proc, 4242), (sim_proc, 777)):
            srcdir = os.path.join(src, str(pid))
            os.replace(srcdir, os.path.join(merged, str(pid)))
        _write_reg(a.registry,
                   [_lease(port=30100, worktree=wt,
                           holder={"pid": 4242, "start": 999})])
        a2 = pam.Allocator(registry=a.registry, last=a.last_path,
                           proc_root=merged)
        # bound lease, dead holder, live sim pids -> kept
        self.assertEqual(a2.gc(), [])
        # remove the pid file -> sim dead -> reclaimed
        os.unlink(os.path.join(pids, "unit.c1.pid"))
        self.assertEqual(len(a2.gc()), 1)

    def test_sweep_missing_worktree(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        _write_reg(a.registry,
                   [_lease(port=30100,
                           worktree="/definitely/not/here",
                           holder={"pid": 4242, "start": 99})])
        self.assertEqual(len(a.gc()), 1)

    # ----------------------------------------------------------- race

    def test_two_processes_race_same_pins(self):
        a, wt = _mk()
        wt2 = tempfile.mkdtemp(prefix='pam_wt2_')
        pins = [30010, 30011, 30012]
        barrier = multiprocessing.Barrier(2)
        done = multiprocessing.Barrier(2)
        q = multiprocessing.Queue()
        ps = [multiprocessing.Process(
                  target=_race, args=(f"s{i}", a.registry, wt, pins,
                                      barrier, done, q))
              for i in (1, 2)]
        for p in ps:
            p.start()
        for p in ps:
            p.join(30)
            self.assertEqual(p.exitcode, 0)
        results = [q.get(timeout=5), q.get(timeout=5)]
        ok = [r for r in results if r[0] == "ok"]
        fail = [r for r in results if r[0] == "fail"]
        # exactly one sim wins; the loser is told who holds the ports
        self.assertEqual(len(ok), 1)
        self.assertEqual(len(fail), 1)
        self.assertEqual(len(a.leases()), 3)


class TestPathLeases(unittest.TestCase):
    """SL-0086: kind "path" leases - a directory, not a port."""

    def test_path_lease_commit_and_release(self):
        a, wt = _mk()
        d = os.path.join(tempfile.mkdtemp(prefix="pam_sock_"), "u1")
        with a.transaction("s1", wt) as tx:
            tx.alloc("unit.c1.de_comm", proto="udp")
            tx.alloc_path("unit.c1.sockdir", d)
            ports = tx.commit()
        self.assertEqual(ports["unit.c1.sockdir"], d)
        lease = [l for l in a.leases(sim="s1")
                 if l["name"] == "unit.c1.sockdir"][0]
        self.assertEqual(lease["path"], d)
        self.assertIsNone(lease["port"])
        os.makedirs(d)
        open(os.path.join(d, "de_comm_broker.sock"), "w").close()
        gone = a.release("s1", wt)
        self.assertEqual(len(gone), 2)
        self.assertFalse(os.path.exists(d))           # dir removed
        self.assertEqual(a.leases(sim="s1"), [])

    def test_path_lease_sticky(self):
        a, wt = _mk()
        d = tempfile.mkdtemp(prefix="pam_sock_")
        for _ in range(2):
            with a.transaction("s1", wt) as tx:
                tx.alloc_path("unit.c1.sockdir", d)
                ports = tx.commit()
            self.assertEqual(ports["unit.c1.sockdir"], d)
        self.assertEqual(len(a.leases(sim="s1")), 1)

    def test_path_lease_conflict_same_dir(self):
        a, wt = _mk()
        d = tempfile.mkdtemp(prefix="pam_sock_")
        with a.transaction("s1", wt) as tx:
            tx.alloc_path("unit.c1.sockdir", d)
            tx.commit()
        conflicts = a.check("s2", wt,
                            [{"name": "unit.c2.sockdir", "kind": "path",
                              "proto": "path", "path": d}])
        self.assertEqual(len(conflicts), 1)
        self.assertEqual(conflicts[0]["reason"], "leased")

    def test_path_lease_no_port_memory(self):
        a, wt = _mk()
        d = tempfile.mkdtemp(prefix="pam_sock_")
        with a.transaction("s1", wt) as tx:
            tx.alloc_path("unit.c1.sockdir", d)
            tx.commit()
        a.release("s1", wt)
        # path leases leave no last_ports entry - there is no port to
        # remember; the path is recomputed deterministically each run
        last = json.load(open(a.last_path)) if os.path.exists(
            a.last_path) else {}
        self.assertFalse(any("sockdir" in k for k in last))


if __name__ == "__main__":
    unittest.main()
