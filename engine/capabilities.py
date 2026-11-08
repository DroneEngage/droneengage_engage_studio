#!/usr/bin/env python3
"""capabilities.py - what this host can run (SL-0215).

One probe per name in simlib.CAPABILITIES - the list a yaml's
`requires:` draws from. probe() returns {name: {"ok", "reason"}}; a
probe never hangs (every subprocess call carries a few-second timeout)
and never raises - a probe error reports as ok=False.

    ./sim doctor --caps             # the table
    ./sim doctor --caps --json      # the map

A `requires:` entry the runner cannot satisfy SKIPs the test with the
probe's reason (SL-0216) - missing hardware is not a failure.
"""

import glob
import os
import shutil
import subprocess
import sys

import simlib

# every subprocess a probe spawns is bounded by this (seconds) - the
# file/device checks themselves cannot block
PROBE_TIMEOUT = 4


def _run(argv, timeout=PROBE_TIMEOUT):
    """Bounded subprocess -> (returncode, stdout+stderr). rc -1 on
    spawn failure or timeout."""
    try:
        r = subprocess.run(argv, capture_output=True, text=True,
                           timeout=timeout)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except (OSError, subprocess.SubprocessError) as e:
        return -1, str(e)


def _video_indices():
    out = []
    for p in glob.glob("/dev/video*"):
        tail = os.path.basename(p)[5:]
        if tail.isdigit():
            out.append(int(tail))
    return sorted(out)


def probe_sitl():
    """At least one built SITL vehicle binary under ARDUPILOT_DIR."""
    for veh, spec in simlib.FC_KINDS.items():
        b = os.path.join(simlib.ARDUPILOT_DIR,
                         "build", "sitl", "bin", spec["bin"])
        if os.path.isfile(b):
            return True, f"{veh}: {b}"
    return False, (f"no SITL binary under {simlib.ARDUPILOT_DIR}"
                   "/build/sitl/bin - build one (`./waf copter`) or "
                   "set ARDUPILOT_DIR in sim.local.env")


def probe_camera():
    """A /dev/videoN that advertises CAPTURE (a real camera, or a fed
    exclusive_caps loopback)."""
    idxs = _video_indices()
    if not idxs:
        return False, ("no /dev/video* devices - attach a camera or "
                       "./sim vcam <sim> for v4l2loopback nodes")
    good = [i for i in idxs if simlib.video_capture_capable(i)]
    if good:
        return True, ", ".join(f"/dev/video{i}" for i in good)
    return False, ("/dev/video* present but none advertises CAPTURE "
                   "(exclusive_caps loopbacks need a producer - "
                   "camera_feed: or scripts/video_clock.sh)")


def probe_v4l2loopback():
    """A v4l2loopback virtual video device exists."""
    loops = [i for i in _video_indices() if simlib.video_is_loopback(i)]
    if loops:
        return True, ", ".join(f"/dev/video{i}" for i in loops)
    return False, ("no v4l2loopback devices - ./sim vcam <sim> creates "
                   "them, or sudo tools/setup_v4l2loopback.sh for the "
                   "named-label pool")


def probe_android_adb():
    """adb present and at least one device in state `device`."""
    adb = shutil.which("adb")
    if not adb:
        return False, ("adb not on PATH - install the Android "
                       "platform-tools")
    rc, out = _run([adb, "devices"])
    if rc != 0:
        return False, f"`adb devices` failed: {out.strip()[:120]}"
    live = [ln.split()[0] for ln in out.splitlines()[1:]
            if ln.strip().endswith("\tdevice")]
    if live:
        return True, ", ".join(live)
    return False, ("no live adb device (a device shows 'unauthorized'"
                   "/'offline' - accept the prompt or replug)")


def probe_rpi_target(target=None):
    """An SSH-reachable lab target: --target user@host / SIM_TARGET in
    sim.local.env."""
    t = target or simlib.setting("SIM_TARGET")
    if not t:
        return False, ("no target - pass --target user@host or set "
                       "SIM_TARGET in sim.local.env")
    ssh = shutil.which("ssh")
    if not ssh:
        return False, "ssh not on PATH"
    rc, out = _run([ssh, "-o", "BatchMode=yes",
                    "-o", f"ConnectTimeout={PROBE_TIMEOUT - 1}",
                    t, "true"], timeout=PROBE_TIMEOUT + 1)
    if rc == 0:
        return True, f"{t} reachable over SSH"
    return False, (f"{t} not reachable over SSH "
                   f"({out.strip()[:120] or 'timeout'}) - needs "
                   "key-based login (BatchMode)")


def probe_mavlink3dmap():
    """MAVLINK3DMAP_DIR points at a usable checkout (backend bridge +
    vite)."""
    m3d = simlib.MAVLINK3DMAP_DIR
    bridge = os.path.join(m3d, "backend", "src", "udp2websocket.js")
    vite = next((p for p in (
            os.path.join(m3d, "node_modules", "vite", "bin", "vite.js"),
            os.path.join(m3d, "frontend", "node_modules", "vite",
                         "bin", "vite.js"))
        if os.path.isfile(p)), None)
    if os.path.isfile(bridge) and vite:
        mism = simlib.m3d_version_mismatch(m3d)
        if mism is None:
            return True, f"{m3d} ({simlib.m3d_version(m3d)})"
        return False, (f"{m3d} - {mism} - checkout the pinned release "
                       "or set SIM_M3D_VERSION in sim.local.env")
    return False, (f"{m3d} incomplete (needs the backend bridge + an "
                   "installed vite) - check out Mavlink3DMap, npm "
                   "install, set MAVLINK3DMAP_DIR in sim.local.env")


def probe_real_servers():
    """The spawned Rust comm server binary is built (transport: real)."""
    b = simlib.server_bin(simlib.SERVER_REPO,
                          simlib.SERVER_BINS["de_server"])
    if os.path.isfile(b) and os.access(b, os.X_OK):
        return True, b
    return False, (f"{b} not built - cd {simlib.SERVER_REPO} && "
                   "cargo build --release")


def probe_esp32():
    """An ESP32 board's USB-serial port exists and is openable."""
    import esp32 as _esp32
    port = _esp32.default_port()
    if port and _esp32.port_usable(port):
        return True, os.path.realpath(port)
    return False, ("no usable ESP32 USB-serial port - attach the board "
                   "(dialout group for rw) or set SIM_ESP32_PORT in "
                   "sim.local.env")


PROBES = {
    "sitl": probe_sitl,
    "camera": probe_camera,
    "v4l2loopback": probe_v4l2loopback,
    "android_adb": probe_android_adb,
    "rpi_target": probe_rpi_target,
    "mavlink3dmap": probe_mavlink3dmap,
    "real_servers": probe_real_servers,
    "esp32": probe_esp32,
}


def probe(only=None, target=None):
    """-> {capability: {"ok": bool, "reason": str}}. `only` limits to a
    subset; `target` feeds the rpi_target probe (CLI --target wins over
    SIM_TARGET). Unknown names report ok=False - the loader rejects
    them long before, so this is only a guard."""
    out = {}
    for name in (only or simlib.CAPABILITIES):
        fn = PROBES.get(name)
        if fn is None:
            out[name] = {"ok": False, "reason": "no probe"}
            continue
        try:
            ok, reason = (fn(target) if name == "rpi_target"
                          else fn())
        except Exception as e:
            ok, reason = False, f"probe error: {e}"
        out[name] = {"ok": bool(ok), "reason": reason}
    return out


def missing(requires, target=None):
    """A requires: list -> [(capability, reason)] for every entry whose
    probe fails. Empty = the host can run the file."""
    caps = probe(only=requires, target=target)
    return [(n, caps[n]["reason"]) for n in requires
            if not caps.get(n, {}).get("ok")]


def main(argv):
    """`python3 engine/capabilities.py [--target user@host]` - the table."""
    target = None
    a = list(argv)
    if "--target" in a:
        i = a.index("--target")
        if i + 1 >= len(a):
            sys.exit("--target needs user@host")
        target = a[i + 1]
        del a[i:i + 2]
    if a:
        sys.exit("usage: capabilities.py [--target user@host]")
    caps = probe(target=target)
    w = max(len(n) for n in caps)
    for name, r in caps.items():
        tag = simlib.col_green("ok     ") if r["ok"] \
            else simlib.col_yellow("missing")
        print(f"{tag} {name:{w}s}  {r['reason']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
