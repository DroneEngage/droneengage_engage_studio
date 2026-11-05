"""prelaunch/browser/cdp.py - run a real headless Chrome through the DevTools
protocol.

This needs only Chrome or Chromium and the websockets package, which the sim
already uses. Playwright is not used, because it needs a separate browser
download, and that does not fit an offline airgap Pi.
"""

import json
import os
import shutil
import subprocess
import tempfile
import time
import urllib.request

from websockets.sync.client import connect

HERE = os.path.dirname(os.path.abspath(__file__))
CHROME_NAMES = ("google-chrome", "chromium", "chromium-browser", "chrome")


def find_chrome(explicit=None):
    if explicit:
        return explicit if os.path.exists(explicit) else None
    for name in CHROME_NAMES:
        p = shutil.which(name)
        if p:
            return p
    return None


class BrowserError(Exception):
    pass


class Browser:
    def __init__(self, chrome, options=None):
        self.chrome = chrome
        self.options = options or {}
        self.proc = None
        self.ws = None
        self.next_id = 0
        self.dir = tempfile.mkdtemp(prefix="prelaunch-chrome-")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

    def start(self):
        self.proc = subprocess.Popen(
            [self.chrome, "--headless=new", "--no-first-run",
             "--no-default-browser-check", "--disable-gpu",
             "--ignore-certificate-errors", "--remote-debugging-port=0",
             "--autoplay-policy=no-user-gesture-required",
             "--disable-features=WebRtcHideLocalIpsWithMdns",
             "--window-size=1600,1000", f"--user-data-dir={self.dir}",
             "about:blank"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        port_file = os.path.join(self.dir, "DevToolsActivePort")
        end = time.time() + 30
        while time.time() < end and not os.path.exists(port_file):
            if self.proc.poll() is not None:
                raise BrowserError("Chrome exited at start")
            time.sleep(0.1)
        if not os.path.exists(port_file):
            raise BrowserError("Chrome gave no DevTools port in 30 seconds")
        time.sleep(0.2)
        with open(port_file) as fh:
            port = int(fh.readline())
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/json/list",
                                    timeout=10) as r:
            page = [t for t in json.loads(r.read()) if t["type"] == "page"][0]
        self.ws = connect(page["webSocketDebuggerUrl"], max_size=None,
                          proxy=None) if _takes_proxy() else \
            connect(page["webSocketDebuggerUrl"], max_size=None)
        self.call("Page.enable")
        self.call("Runtime.enable")
        self.add_init_script("window.__plOptions = "
                             + json.dumps(self.options) + ";")
        with open(os.path.join(HERE, "rtc_hook.js")) as fh:
            self.add_init_script(fh.read())

    def stop(self):
        try:
            if self.ws:
                self.ws.close()
        except Exception:
            pass
        if self.proc:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        shutil.rmtree(self.dir, ignore_errors=True)

    def call(self, method, params=None, timeout=30):
        self.next_id += 1
        my = self.next_id
        self.ws.send(json.dumps({"id": my, "method": method,
                                 "params": params or {}}))
        end = time.time() + timeout
        while time.time() < end:
            raw = self.ws.recv(timeout=max(0.1, end - time.time()))
            msg = json.loads(raw)
            if msg.get("id") == my:
                if "error" in msg:
                    raise BrowserError(f"{method}: {msg['error']}")
                return msg.get("result", {})
        raise BrowserError(f"{method}: no answer in {timeout} seconds")

    def add_init_script(self, source):
        self.call("Page.addScriptToEvaluateOnNewDocument", {"source": source})

    def navigate(self, url):
        self.call("Page.navigate", {"url": url})

    def evaluate(self, expr, timeout=30):
        r = self.call("Runtime.evaluate",
                      {"expression": expr, "awaitPromise": True,
                       "returnByValue": True, "timeout": timeout * 1000},
                      timeout=timeout + 5)
        if r.get("exceptionDetails"):
            d = r["exceptionDetails"]
            text = (d.get("exception") or {}).get("description") or d.get("text")
            raise BrowserError(f"page script failed: {text}")
        return (r.get("result") or {}).get("value")

    def wait_for(self, expr, seconds, what):
        """Poll a page expression until it is true."""
        end = time.time() + seconds
        while time.time() < end:
            try:
                if self.evaluate(expr):
                    return True
            except BrowserError:
                pass
            time.sleep(0.5)
        raise BrowserError(f"{what}: not seen in {int(seconds)} seconds")


def _takes_proxy():
    import inspect
    return "proxy" in inspect.signature(connect).parameters
