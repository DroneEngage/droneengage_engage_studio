"""prelaunch_promote_test.py - two real local slots. Each slot has a small
auth server that reads server_port from its own config file and answers with
its slot's comm port and its accounts, the same contract as de_auth. The
script changes the same files and runs the same kinds of command as on the
machines. The real Rust servers are not used here."""

import contextlib
import io
import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.request

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import prelaunch_promote as P  # noqa: E402

FAKE_AUTH = r'''
import http.server, json, re, sys
cfg, comm, accounts = sys.argv[1:4]
port = int(re.search(r'"server_port"\s*:\s*"?(\d+)', open(cfg).read()).group(1))
class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        ok = self.path.strip("/") in json.load(open(accounts))
        body = json.dumps({"comm": int(comm), "ok": ok}).encode()
        self.send_response(200); self.end_headers(); self.wfile.write(body)
http.server.HTTPServer(("127.0.0.1", port), H).serve_forever()
'''


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def login(port, account="a1"):
    end = time.time() + 8
    while True:
        try:
            with urllib.request.urlopen(
                    f"http://127.0.0.1:{port}/{account}", timeout=1) as r:
                return json.loads(r.read())
        except OSError:
            if time.time() > end:
                raise
            time.sleep(0.1)


class TwoSlots(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.addCleanup(self._clean)
        self.pub, self.test = free_port(), free_port()
        self.results = os.path.join(self.d, "results")
        self.script = os.path.join(self.d, "fake_auth.py")
        with open(self.script, "w") as fh:
            fh.write(FAKE_AUTH)
        slots = {}
        for name, comm, port in (("blue", 9001, self.pub),
                                 ("green", 9002, self.test)):
            d = os.path.join(self.d, name)
            os.mkdir(d)
            cfg = os.path.join(d, "server.config.local")
            with open(cfg, "w") as fh:
                fh.write(f'{{ // note\n "server_port" : {port},\n'
                         f' "other_port": 5 }}\n')
            web = os.path.join(d, "config.json")
            with open(web, "w") as fh:
                fh.write(f'{{"CONST_PROD_MODE_PORT": "{port}"}}\n')
            acc = os.path.join(d, "accounts.json")
            with open(acc, "w") as fh:
                json.dump(["a1"], fh)
            pid = os.path.join(d, "pid")
            py = shlex_quote(sys.executable)
            slots[name] = {
                "auth": {"machine": "a", "config": cfg,
                         "stop": f"kill $(cat {pid}) 2>/dev/null; rm -f {pid}; "
                                 f"true",
                         "start": f"nohup {py} {self.script} {cfg} {comm} {acc}"
                                  f" >/dev/null 2>&1 & echo $! > {pid}"},
                "webclient": {"machine": "a", "config": web},
                "comm": {"machine": "b",
                         "restart": f"echo restarted >> {d}/comm.log"},
            }
        other = {"blue": "green", "green": "blue"}
        for name in slots:
            slots[name]["data_sync"] = [{
                "machine": "a",
                "cmd": f"cp {self.d}/{name}/accounts.json "
                       f"{self.d}/{other[name]}/accounts.json"}]
        self.live_file = os.path.join(self.d, "live_slot")
        with open(self.live_file, "w") as fh:
            fh.write("blue\n")
        self.plan = os.path.join(self.d, "plan.yaml")
        with open(self.plan, "w") as fh:
            yaml.safe_dump({
                "public_port": self.pub, "test_port": self.test,
                "report_target": "mirror", "live_slot_file": self.live_file,
                "machines": {"a": {"local": True}, "b": {"local": True}},
                "slots": slots}, fh)
        self.slots = slots
        for s in slots.values():
            subprocess.run(s["auth"]["start"], shell=True, check=True)
        self.assertEqual(login(self.pub)["comm"], 9001)

    def _clean(self):
        for name in ("blue", "green"):
            subprocess.run(f"kill $(cat {self.d}/{name}/pid) 2>/dev/null; true",
                           shell=True)
        shutil.rmtree(self.d, ignore_errors=True)

    def report(self, verdict="GO", version="1.2", **extra):
        d = os.path.join(self.results, "prelaunch", version, "mirror")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "report.json"), "w") as fh:
            json.dump(dict(verdict=verdict, **extra), fh)

    def run_tool(self, *args):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            rc = P.main(["--plan", self.plan, *args],
                        results_dir=self.results)
        return rc, out.getvalue() + err.getvalue()

    def live(self):
        return open(self.live_file).read().strip()

    def snapshot(self):
        out = {}
        for name in ("blue", "green"):
            for f in ("server.config.local", "config.json", "accounts.json"):
                out[name, f] = open(f"{self.d}/{name}/{f}").read()
        return out

    def test_dry_run_prints_steps_and_changes_nothing(self):
        before = self.snapshot()
        rc, out = self.run_tool("--version", "1.2")
        self.assertEqual(rc, 0)
        self.assertIn("DRY RUN", out)
        self.assertIn(f"green auth server_port = {self.pub}", out)
        self.assertIn("CONST_PROD_MODE_PORT", out)
        self.assertEqual(self.snapshot(), before)
        self.assertEqual(self.live(), "blue")

    def test_stops_with_no_report(self):
        rc, out = self.run_tool("--version", "1.2", "--message-sent",
                                "--execute")
        self.assertEqual(rc, 1)
        self.assertIn("no pre-launch report", out)
        self.assertEqual(self.live(), "blue")

    def test_stops_on_no_go_and_override_allows(self):
        self.report("NO-GO")
        rc, out = self.run_tool("--version", "1.2", "--message-sent",
                                "--execute")
        self.assertEqual(rc, 1)
        self.assertIn("NO-GO", out)
        self.report("NO-GO", override={"reason": "known"})
        self.assertEqual(self.run_tool("--version", "1.2", "--message-sent",
                                       "--execute")[0], 0)

    def test_stops_when_the_message_was_not_sent(self):
        self.report()
        rc, out = self.run_tool("--version", "1.2", "--execute")
        self.assertEqual(rc, 1)
        self.assertIn("global message", out)
        self.assertEqual(self.live(), "blue")

    def test_armed_units_are_shown_and_do_not_stop(self):
        self.report()
        plan = yaml.safe_load(open(self.plan))
        plan["armed_cmd"] = "echo 3"
        yaml.safe_dump(plan, open(self.plan, "w"))
        rc, out = self.run_tool("--version", "1.2", "--message-sent",
                                "--execute")
        self.assertEqual(rc, 0, out)
        self.assertIn("armed units now: 3", out)

    def test_swap_new_login_reaches_new_comm_on_the_same_port(self):
        self.report()
        rc, out = self.run_tool("--version", "1.2", "--message-sent",
                                "--execute")
        self.assertEqual(rc, 0, out)
        self.assertEqual(login(self.pub)["comm"], 9002)
        self.assertEqual(login(self.test)["comm"], 9001)
        self.assertEqual(self.live(), "green")

    def test_old_comm_restart_is_part_of_the_swap(self):
        self.report()
        self.run_tool("--version", "1.2", "--message-sent", "--execute")
        self.assertIn("restarted", open(f"{self.d}/blue/comm.log").read())
        self.assertFalse(os.path.exists(f"{self.d}/green/comm.log"))

    def test_account_made_before_the_swap_logs_in_after(self):
        self.report()
        acc = f"{self.d}/blue/accounts.json"
        json.dump(["a1", "new-user"], open(acc, "w"))
        self.assertFalse(login(self.test, "new-user")["ok"])
        self.run_tool("--version", "1.2", "--message-sent", "--execute")
        self.assertTrue(login(self.pub, "new-user")["ok"])

    def test_rollback_restores_the_first_state(self):
        self.report()
        self.run_tool("--version", "1.2", "--message-sent", "--execute")
        rc, out = self.run_tool("--rollback", "--execute")
        self.assertEqual(rc, 0, out)
        self.assertEqual(login(self.pub)["comm"], 9001)
        self.assertEqual(login(self.test)["comm"], 9002)
        self.assertEqual(self.live(), "blue")

    def test_webclient_port_moves_and_other_config_stays(self):
        self.report()
        self.run_tool("--version", "1.2", "--message-sent", "--execute")
        self.assertIn(f'"{self.pub}"',
                      open(f"{self.d}/green/config.json").read())
        cfg = open(f"{self.d}/green/server.config.local").read()
        self.assertIn(f'"server_port" : {self.pub}', cfg)
        self.assertIn('"other_port": 5', cfg)
        self.assertIn("// note", cfg)

    def test_missing_live_file_stops(self):
        os.remove(self.live_file)
        rc, out = self.run_tool("--version", "1.2")
        self.assertEqual(rc, 1)
        self.assertIn("live-slot file", out)

    def test_failed_step_stops_and_keeps_the_live_file(self):
        self.report()
        plan = yaml.safe_load(open(self.plan))
        plan["slots"]["blue"]["comm"]["restart"] = "echo nope; exit 7"
        yaml.safe_dump(plan, open(self.plan, "w"))
        rc, out = self.run_tool("--version", "1.2", "--message-sent",
                                "--execute")
        self.assertEqual(rc, 1)
        self.assertIn("exit 7", out)
        self.assertEqual(self.live(), "blue")


def shlex_quote(s):
    import shlex
    return shlex.quote(s)


if __name__ == "__main__":
    unittest.main()
