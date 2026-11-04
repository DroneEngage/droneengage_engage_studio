#!/usr/bin/env python3
"""test_devstack.py - offline gates for dev stacks (SL-0154).

  - the shipped de_mavlink3dmap stack loads: kind node, the provides
    render_web mapping, its vite script candidates and port;
  - _load_stacks validation rejects malformed kind-node specs (missing
    script/port, node-only fields on other kinds, provides pointing at
    no proc) without a traceback;
  - the node launch script carries the marker--<stack>.cjs preload (the
    --<stack> substring _pid_is_stack matches), an absolute script path
    and {port}-substituted args under <repo>/<cwd>;
  - _port honours `port:` and a port_env override; provided_port()
    reports the stack's port only while its proc is stack-alive.

    python3 engine/test_devstack.py
"""
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

# bytecode under var/pycache from the first import on (simlib sets it
# too, but only after itself is compiled - that left __pycache__ dirs)
sys.pycache_prefix = os.environ.setdefault(
    "PYTHONPYCACHEPREFIX",
    os.path.join(os.environ.get("SIM_VAR_DIR") or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "var"), "pycache"))

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import devstack  # noqa: E402
import simlib   # noqa: E402

STACK = "de_mavlink3dmap"
PROC = "m3d_web"


def _spec(name=STACK):
    return devstack.STACKS[name]


def _proc(name=STACK, proc=PROC):
    return next(p for p in _spec(name)["procs"] if p["name"] == proc)


def _stacks_dir(*yamls):
    """A temp stacks/ dir holding the given yaml bodies."""
    d = tempfile.mkdtemp()
    for i, body in enumerate(yamls):
        with open(os.path.join(d, f"s{i}.yaml"), "w") as f:
            f.write(body)
    return d


NODE_YAML = """\
name: s0
procs:
  - name: web
    kind: node
    repo: /nonexistent
    cwd: frontend
    script: [a.js, b.js]
    args: ["--port", "{port}"]
    port: 8070
"""


class Out(unittest.TestCase):
    """./sim up <stack> --out <host>: generated configs, repo untouched."""

    def _proc(self, repo, out):
        return {"name": "srv", "repo": repo, "repo_env": None,
                "bin": "bin/srv", "port_env": None, "kind": "exec",
                "cwd": None, "script": None, "args": None, "port": None,
                "out": out}

    def _repo(self, local='{"keep": 1, "public_host": "127.0.0.1"}'):
        repo = tempfile.mkdtemp()
        with open(os.path.join(repo, "server.config"), "w") as f:
            f.write('{"server_port": 9, // comment\n "ssl": "./ssl/a"}')
        if local is not None:
            with open(os.path.join(repo, "server.config.local"), "w") as f:
                f.write(local)
        return repo

    def _script(self, repo, p, host):
        var = tempfile.mkdtemp()
        with mock.patch.object(simlib, "VAR_DIR", var):
            path = devstack._launch_script("st", p, host)
            return var, open(path).read()

    def test_config_overrides_on_a_copy(self):
        repo = self._repo()
        p = self._proc(repo, {"public_host": "{host}", "server_ip": "::"})
        var, text = self._script(repo, p, "10.0.0.5")
        cfg = os.path.join(var, "st", "out", "srv", "server.config")
        self.assertIn(f"'--config={cfg}'", text)
        self.assertIn("--st", text)
        self.assertIn(f"cd '{repo}'", text)
        self.assertEqual(simlib.load_config(cfg + ".local"),
                         {"keep": 1, "public_host": "10.0.0.5",
                          "server_ip": "::"})
        self.assertEqual(simlib.load_config(cfg)["server_port"], 9)
        self.assertEqual(os.stat(cfg + ".local").st_mode & 0o777, 0o600)
        self.assertEqual(
            simlib.load_config(os.path.join(repo, "server.config.local"))
            ["public_host"], "127.0.0.1")

    def test_repo_without_local(self):
        repo = self._repo(local=None)
        var, _ = self._script(repo, self._proc(repo, {"a": "{host}"}), "h1")
        self.assertEqual(simlib.load_config(os.path.join(
            var, "st", "out", "srv", "server.config.local")), {"a": "h1"})

    def test_no_config_without_host_or_out(self):
        repo = self._repo()
        _, text = self._script(repo, self._proc(repo, {"a": 1}), None)
        self.assertNotIn("--config", text)
        _, text = self._script(repo, self._proc(repo, None), "h1")
        self.assertNotIn("--config", text)

    def test_rejects_hosts_clients_cannot_use(self):
        for host in ("127.0.0.1", "localhost", "0.0.0.0", "::", "a b",
                     "x';rm", "::1"):
            with self.assertRaises(simlib.SimError, msg=host):
                devstack._check_out_host(host)
        devstack._check_out_host("10.220.158.31")
        devstack._check_out_host("drone.example.com")

    def test_shipped_de_servers_announce_the_host(self):
        by = {p["name"]: p["out"] for p in devstack.procs("de_servers")}
        for proc in ("de_server", "de_udp_proxy", "de_storage"):
            self.assertEqual(by[proc]["public_host"], "{host}")
        self.assertEqual(by["de_auth"]["server_ip"], "0.0.0.0")

    def test_help_lists_out_only_where_supported(self):
        text = devstack.render_help("de_servers")
        self.assertIn("usage: ./sim up de_servers", text)
        self.assertIn("--out <host>", text)
        self.assertIn("de_udp_proxy", text)
        self.assertNotIn("--out", devstack.render_help(STACK))

    def test_help_prints_from_the_command_line_with_colors(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = subprocess.run(
            ["script", "-qec", "./sim up de_servers --help", "/dev/null"],
            cwd=root, capture_output=True, text=True).stdout
        self.assertIn("--out", out)
        self.assertIn("\033[32m./sim up de_servers", out)
        self.assertIn("\033[33m", out)

    def test_out_rejected_on_other_kinds(self):
        d = _stacks_dir(NODE_YAML + '    out: {a: 1}\n')
        with mock.patch.object(devstack, "STACKS_DIR", d):
            with self.assertRaises(simlib.SimError):
                devstack._load_stacks()


class T(unittest.TestCase):

    def test_shipped_stack_loads(self):
        self.assertIn(STACK, devstack.stack_names())
        spec = _spec()
        self.assertEqual((spec["provides"] or {}).get("render_web"),
                         PROC)
        p = _proc()
        self.assertEqual(p["kind"], "node")
        self.assertEqual(p["repo_env"], "MAVLINK3DMAP_DIR")
        self.assertTrue(any("vite.js" in s for s in p["script"]))
        self.assertIn("{port}", p["args"])
        self.assertIsInstance(p["port"], int)

    def test_load_stacks_node_requires_script(self):
        d = _stacks_dir(NODE_YAML.replace("    script: [a.js, b.js]\n",
                                          ""))
        with mock.patch.object(devstack, "STACKS_DIR", d):
            with self.assertRaises(simlib.SimError) as cm:
                devstack._load_stacks()
        self.assertIn("script", str(cm.exception))

    def test_load_stacks_node_requires_port(self):
        d = _stacks_dir(NODE_YAML.replace("    port: 8070\n", ""))
        with mock.patch.object(devstack, "STACKS_DIR", d):
            with self.assertRaises(simlib.SimError) as cm:
                devstack._load_stacks()
        self.assertIn("port", str(cm.exception))

    def test_load_stacks_rejects_node_fields_on_exec(self):
        d = _stacks_dir("""\
name: s0
procs:
  - name: x
    kind: exec
    repo: /nonexistent
    bin: bin/x
    script: a.js
""")
        with mock.patch.object(devstack, "STACKS_DIR", d):
            with self.assertRaises(simlib.SimError) as cm:
                devstack._load_stacks()
        self.assertIn("script", str(cm.exception))

    def test_load_stacks_provides_unknown_proc(self):
        d = _stacks_dir("""\
name: s0
provides: {render_web: ghost}
procs:
  - name: web
    kind: node
    repo: /nonexistent
    script: a.js
    port: 8070
""")
        with mock.patch.object(devstack, "STACKS_DIR", d):
            with self.assertRaises(simlib.SimError) as cm:
                devstack._load_stacks()
        self.assertIn("ghost", str(cm.exception))

    def test_node_launch_script(self):
        tmp = tempfile.mkdtemp()
        repo = os.path.join(tmp, "repo")
        os.makedirs(os.path.join(repo, "frontend", "node_modules",
                                 "vite", "bin"))
        open(os.path.join(repo, "frontend", "node_modules", "vite",
                          "bin", "vite.js"), "w").close()
        p = dict(_proc(), repo=repo, repo_env=None)
        with mock.patch.object(devstack, "_stack_dir",
                               return_value=os.path.join(tmp, "stack")):
            path = devstack._launch_script(STACK, p)
        body = open(path).read()
        self.assertIn("cd '%s'" % os.path.join(repo, "frontend"), body)
        self.assertIn("--require", body)
        # the marker file name carries --<stack> for _pid_is_stack
        self.assertIn(f"marker--{STACK}.cjs", body)
        self.assertIn(f"--{STACK}", f"marker--{STACK}.cjs")
        self.assertIn(os.path.join(repo, "frontend", "node_modules",
                                   "vite", "bin", "vite.js"), body)
        self.assertIn("--port %d" % p["port"], body)
        self.assertIn("--strictPort", body)
        self.assertTrue(os.path.isfile(
            os.path.join(tmp, "stack", f"marker--{STACK}.cjs")))

    def test_port_and_port_env(self):
        p = _proc()
        self.assertEqual(devstack._port(STACK, p), p["port"])
        with mock.patch.dict(os.environ,
                             {p["port_env"]: "9099"}):
            self.assertEqual(devstack._port(STACK, p), 9099)

    def test_provided_port_only_when_alive(self):
        p = _proc()
        with mock.patch.object(devstack, "pids", return_value={}):
            self.assertIsNone(devstack.provided_port("render_web"))
        with mock.patch.object(devstack, "pids",
                               return_value={PROC: 12345}):
            self.assertEqual(devstack.provided_port("render_web"),
                             p["port"])

    def test_open_url_substitutes_port(self):
        url = devstack.open_url(STACK)
        self.assertIsNotNone(url)
        self.assertIn(":%d" % _proc()["port"], url)


if __name__ == "__main__":
    unittest.main(verbosity=2)
