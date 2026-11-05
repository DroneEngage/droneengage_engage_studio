#!/usr/bin/env python3
"""prelaunch_install.py - the install stage: test what ships.

A dev build can pass while the package fails. This installs the real release
on the real hosts with the real scripts, proves that each service runs and
that local config survives, and records the installed versions. It never
installs on a `live` target.

The commands come from the target's `install:` block, because each host has
its own scripts:

    install:
      previous_version: 1.1.0        # optional: test the upgrade path
      hosts:
        unit:
          ssh: pi@rpi5.local
          password_env: PRELAUNCH_PI_PASSWORD   # optional; uses sshpass
          update_cmd: bash ~/scripts/updates/sh_deploy_modules.sh {version} all
          versions_cmd: bash ~/scripts/updates/sh_list_versions.sh
          keep_files: [~/drone_engage/de_comm/de_comm.config.module.json]
          services: {de_comm: de_comm}    # name: pgrep -f pattern
        server: {...}

`{version}` is the --version of the run. A host with `local: true` runs the
commands here, for the self test. An empty update_cmd gives SKIP, so a host
with no known install path cannot look like a pass.

Run the self test: python3 engine/prelaunch_install.py
"""

import os
import shlex
import subprocess
import sys
import tempfile
import unittest

# the server processes each airgap install must leave running
DEFAULT_SERVER_SERVICES = {
    "de_auth": "andruavauth", "de_comm_server": "andruavmainserver",
    "de_storage": "de_storage_server", "de_udp": "droneengage-udp-proxy",
    "coturn": "turnserver",
}
CMD_TIMEOUT_S = 900


def run_on(host, cmd, ctx=None, timeout=CMD_TIMEOUT_S):
    """Run a shell command on a host. -> (exit code, output)."""
    env = None
    if host.get("local"):
        argv = ["bash", "-c", cmd]
    else:
        argv = ["ssh", "-o", "ConnectTimeout=10",
                "-o", "StrictHostKeyChecking=accept-new"]
        if host.get("key"):
            argv += ["-i", os.path.expanduser(host["key"])]
        if host.get("password_env"):
            secret = ctx.secret(host["password_env"])
            env = dict(os.environ, SSHPASS=secret)
            argv = ["sshpass", "-e"] + argv
        else:
            argv += ["-o", "BatchMode=yes"]
        argv += [host["ssh"], cmd]
    try:
        r = subprocess.run(argv, env=env, capture_output=True, text=True,
                           timeout=timeout)
    except subprocess.TimeoutExpired:
        return 124, f"no result after {timeout}s"
    except OSError as e:
        return 127, str(e)
    return r.returncode, (r.stdout + r.stderr).strip()


def _hosts(ctx):
    return ((ctx.target.get("install") or {}).get("hosts")) or {}


def _hash_files(host, files, ctx):
    """path -> sha256 line or MISSING."""
    out = {}
    for f in files or []:
        q = f if f.startswith("~") else shlex.quote(f)
        rc, text = run_on(host, f"sha256sum {q}", ctx, timeout=60)
        out[f] = text.split()[0] if rc == 0 and text else "MISSING"
    return out


def _install_one(name, host, version, ctx):
    """Install on one host. -> a fault text or None."""
    cmd = (host.get("update_cmd") or "").replace("{version}", version)
    before = _hash_files(host, host.get("keep_files"), ctx)
    rc, text = run_on(host, cmd, ctx)
    if rc != 0:
        tail = text[-300:].replace("\n", " ")
        return f"{name}: `{cmd}` exit {rc}: {tail}"
    after = _hash_files(host, host.get("keep_files"), ctx)
    changed = [f for f in before if before[f] != after[f]]
    if changed:
        return f"{name}: the install changed local config: {', '.join(changed)}"
    return None


def release(ctx):
    if ctx.role == "live":
        return "FAIL", "the install stage never runs on a live target"
    hosts = _hosts(ctx)
    if not hosts:
        return "SKIP", f"target {ctx.name} has no install.hosts"
    missing = [n for n, h in hosts.items() if not h.get("update_cmd")]
    if missing:
        return "SKIP", (f"no update_cmd for {', '.join(missing)}: the real "
                        f"install path for this host is not defined")
    previous = (ctx.target.get("install") or {}).get("previous_version")
    steps = ([previous] if previous else []) + [ctx.version]
    for version in steps:
        for name, host in hosts.items():
            fault = _install_one(name, host, version, ctx)
            if fault:
                return "FAIL", f"installing {version}: {fault}"
    got = []
    for name, host in hosts.items():
        if not host.get("versions_cmd"):
            continue
        rc, text = run_on(host, host["versions_cmd"], ctx, timeout=120)
        if rc != 0:
            return "FAIL", f"{name}: the versions command exit {rc}"
        ctx.versions[f"installed.{name}"] = " ".join(text.split())[:400]
        got.append(name)
        if host.get("expect_version") and ctx.version not in text:
            return "FAIL", (f"{name}: version {ctx.version} is not in the "
                            f"versions output after the install")
    how = f"upgrade {previous} to {ctx.version}" if previous else ctx.version
    return "PASS", f"installed {how} on {', '.join(hosts)}"


def services(ctx):
    hosts = _hosts(ctx)
    if not hosts:
        return "SKIP", f"target {ctx.name} has no install.hosts"
    faults, ok = [], []
    for name, host in hosts.items():
        svc = host.get("services")
        if svc is None and name == "server":
            svc = DEFAULT_SERVER_SERVICES
        for label, pattern in (svc or {}).items():
            q = shlex.quote(f"[{pattern[0]}]{pattern[1:]}")
            rc, _ = run_on(host, f"pgrep -f {q} >/dev/null", ctx, timeout=30)
            if rc == 0:
                ok.append(label)
            else:
                faults.append(f"{label} is not running on {name}")
    if faults:
        return "FAIL", "; ".join(faults)
    if not ok:
        return "SKIP", "no services are declared"
    return "PASS", "running: " + ", ".join(ok)


# ---- self test ------------------------------------------------------

class _Ctx:
    def __init__(self, target, version="2.0.0"):
        self.target, self.version = target, version
        self.name = target.get("name", "t")
        self.role = target.get("role", "airgap")
        self.versions = {}


class SelfTest(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.TemporaryDirectory()
        self.addCleanup(self.d.cleanup)
        self.log = os.path.join(self.d.name, "log")
        self.cfg = os.path.join(self.d.name, "server.config.local")
        with open(self.cfg, "w") as fh:
            fh.write("{}")

    def host(self, **over):
        h = {"local": True, "update_cmd": f"echo {{version}} >> {self.log}",
             "versions_cmd": f"tail -1 {self.log}", "keep_files": [self.cfg],
             "expect_version": True, "services": {}}
        h.update(over)
        return h

    def test_install_records_the_version(self):
        c = _Ctx({"install": {"hosts": {"unit": self.host()}}})
        st, d = release(c)
        self.assertEqual(st, "PASS", d)
        self.assertIn("2.0.0", c.versions["installed.unit"])

    def test_upgrade_installs_the_previous_version_first(self):
        c = _Ctx({"install": {"previous_version": "1.0.0",
                              "hosts": {"unit": self.host()}}})
        st, d = release(c)
        self.assertEqual(st, "PASS", d)
        self.assertEqual(open(self.log).read().split(), ["1.0.0", "2.0.0"])
        self.assertIn("upgrade", d)

    def test_failed_install_names_host_and_command(self):
        c = _Ctx({"install": {"hosts": {"server": self.host(
            update_cmd="echo boom; exit 3")}}})
        st, d = release(c)
        self.assertEqual(st, "FAIL")
        self.assertIn("server", d)
        self.assertIn("boom", d)

    def test_install_that_changes_local_config_fails(self):
        c = _Ctx({"install": {"hosts": {"server": self.host(
            update_cmd=f"echo changed > {self.cfg}")}}})
        st, d = release(c)
        self.assertEqual(st, "FAIL")
        self.assertIn("local config", d)

    def test_no_install_path_is_skip(self):
        c = _Ctx({"install": {"hosts": {"server": self.host(update_cmd="")}}})
        st, d = release(c)
        self.assertEqual(st, "SKIP")
        self.assertIn("server", d)

    def test_never_on_live(self):
        c = _Ctx({"role": "live", "install": {"hosts": {"u": self.host()}}})
        self.assertEqual(release(c)[0], "FAIL")

    def test_wrong_version_after_install_fails(self):
        c = _Ctx({"install": {"hosts": {"unit": self.host(
            versions_cmd="echo 1.9.9")}}})
        self.assertEqual(release(c)[0], "FAIL")

    def test_stopped_service_is_named(self):
        c = _Ctx({"install": {"hosts": {"server": self.host(services={
            "shell": "pytho", "de_udp": "no-such-process-xyz"})}}})
        st, d = services(c)
        self.assertEqual(st, "FAIL")
        self.assertIn("de_udp is not running on server", d)
        self.assertNotIn("shell", d)


if __name__ == "__main__":
    unittest.main()
