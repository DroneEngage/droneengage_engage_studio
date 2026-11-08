"""prelaunch_checks/video.py - stage 3: video frames really arrive.

A real headless Chrome logs in to the deployed webclient, finds the test
unit, opens its camera, and reads getStats. A check passes only when
framesDecoded grows. An SDP offer proves nothing about media.

Each camera runs twice: normal, and relay-only (iceTransportPolicy=relay).
On one LAN a direct path always wins, so a broken TURN stays hidden. The
relay-only run is the real ICE test. The force comes from a script the check
injects into its own browser, so the webclient needs no test setting and
production users cannot get it.

Target keys: services.webclient.url, test_unit.name, and optional
video.chrome, video.iceServers (a negative control: replace the page's ICE
list).
"""

import json
import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_ROOT, "prelaunch", "browser"))

FRAMES_MIN = 100
WATCH_S = 30.0
ICE_WAIT_S = 45.0
LOGIN_WAIT_S = 45.0

LOGIN_JS = """
(() => {
  const set = (id, v) => {
    const e = document.getElementById(id);
    if (!e) return false;
    const proto = Object.getOwnPropertyDescriptor(
        HTMLInputElement.prototype, 'value');
    proto.set.call(e, v);
    e.dispatchEvent(new Event('input', {bubbles: true}));
    e.dispatchEvent(new Event('change', {bubbles: true}));
    return true;
  };
  if (!set('txtLoginName', %(user)s) || !set('txtAccessCode', %(code)s))
    return false;
  const b = document.getElementById('btnConnect');
  if (!b) return false;
  b.click();
  return true;
})()
"""

OPEN_CAMERA_JS = """
(() => {
  const name = %(unit)s;
  const icons = [...document.querySelectorAll('i.bi-camera-video,'
                                              + 'i.bi-camera-video-fill')];
  for (const i of icons) {
    let n = i;
    for (let up = 0; up < 8 && n; up++, n = n.parentElement)
      if (n.textContent && n.textContent.includes(name)) { i.click(); return true; }
  }
  return false;
})()
"""

STREAM_BUTTON_JS = """
(() => {
  const b = [...document.querySelectorAll('button')]
      .find(x => x.textContent.trim() === 'stream' && !x.disabled);
  if (!b) return false;
  b.click();
  return true;
})()
"""


def judge(samples, relay_only, frames_min=FRAMES_MIN):
    """(status, detail) from __plSample results over the watch window."""
    if not samples or not samples[-1].get("pcs"):
        return "FAIL", ("the webclient made no peer connection, so the video "
                        "request did not reach WebRTC")
    last, first = samples[-1], samples[0]
    frames = last.get("frames", 0)
    grew = frames > first.get("frames", 0)
    ice = ",".join(last.get("ice") or []) or "none"
    if frames < frames_min or not grew:
        servers = last.get("iceServers")
        hint = ""
        if relay_only and not servers:
            hint = " The ICE list is empty, so a relay-only call cannot work."
        elif relay_only:
            hint = (" In a relay-only run this means ICE failed: check the TURN "
                    "server, its credentials and its relay range.")
        elif "failed" in ice or "checking" in ice:
            hint = " ICE did not connect: check the ICE servers and the firewall."
        errs = last.get("iceErrors") or []
        if errs:
            hint += " ICE errors: " + "; ".join(errs[:4])
        return "FAIL", (f"{frames} frames decoded, and {frames_min} that grow "
                        f"are needed. ICE state {ice}, {servers} ICE servers."
                        f"{hint}")
    if relay_only and last.get("candidate") != "relay":
        return "FAIL", (f"{frames} frames arrived on a {last.get('candidate')} "
                        f"path, not on the relay. ICE was not forced")
    return "PASS", (f"{frames} frames decoded and growing, on a "
                    f"{last.get('candidate')} candidate")


def watch(br, seconds=WATCH_S):
    """Wait for ICE, then sample for `seconds`."""
    end = time.time() + ICE_WAIT_S
    while time.time() < end:
        s = br.evaluate("window.__plSample && window.__plSample()")
        if s and s.get("frames", 0) > 0:
            break
        time.sleep(1.0)
    samples = []
    end = time.time() + seconds
    while time.time() < end:
        samples.append(br.evaluate("window.__plSample()"))
        time.sleep(2.0)
    return samples


def _camera_run(ctx, relay_only):
    import cdp
    web = ctx.service("webclient")
    unit = (ctx.target.get("test_unit") or {}).get("name")
    if not unit:
        return "SKIP", f"target {ctx.name} has no test_unit.name"
    vcfg = ctx.target.get("video") or {}
    chrome = cdp.find_chrome(vcfg.get("chrome"))
    if not chrome:
        return "SKIP", "no Chrome or Chromium on this host"
    user, code = ctx.account()
    options = {"forceRelay": bool(relay_only)}
    if "iceServers" in vcfg:
        options["iceServers"] = vcfg["iceServers"]
    try:
        with cdp.Browser(chrome, options) as br:
            br.navigate(web["url"])
            br.wait_for("!!document.getElementById('txtLoginName')",
                        LOGIN_WAIT_S, "the webclient login form")
            ok = br.evaluate(LOGIN_JS % {"user": json.dumps(user),
                                         "code": json.dumps(code)})
            if not ok:
                return "FAIL", "the webclient login form has changed"
            sel = OPEN_CAMERA_JS % {"unit": json.dumps(unit)}
            br.wait_for(sel, LOGIN_WAIT_S,
                        f"the video control of unit {unit}")
            br.wait_for(STREAM_BUTTON_JS, 20, "the stream button of the camera")
            return judge(watch(br), relay_only)
    except cdp.BrowserError as e:
        return "FAIL", str(e)


def camera_normal(ctx):
    return _camera_run(ctx, False)


def camera_relay(ctx):
    return _camera_run(ctx, True)
