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
