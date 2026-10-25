"""adb-connected Android phone as a sim unit (SL-0095 / megaplan A9).

A unit entry gains ``android: {serial, remote_modules}`` — the listed DE
modules run on the phone inside the Andruav app (per-module services
:de_comm / :de_mavlink from AN-0034); SITL and the server stack stay on
the dev host exactly as today (docs/HIL.md scope rule).

Covers: device probe, clock-offset measurement, config push to the app's
de_override dir, service START/STOP, a mirror proc that pulls the module
journals and logcat into the unit's local dir so every engine reader
works unchanged, and ./sim doctor rows.
"""

import os
import re
import subprocess
import time

import simlib

APP_PKG = "ardupilot.andruav"
OVERRIDE_DIR = f"/sdcard/Android/data/{APP_PKG}/files/de_override"
DEVICE_FILES = "files"  # under run-as, the app's private files dir

# service component + process suffix per remote module
MODULE_SERVICE = {
    "de_comm": "ap.andruav.denative.DeCommService",
    "de_mavlink": "ap.andruav.denative.DeMavlinkService",
}
MODULE_PROC = {
    "de_comm": f"{APP_PKG}:de_comm",
    "de_mavlink": f"{APP_PKG}:de_mavlink",
}
ACTION_PREFIX = "ap.andruav.denative"


def serial_of(u):
    """Unit's device serial — spec key, else ANDROID_SERIAL setting."""
    spec = u.get("android") or {}
    return spec.get("serial") or simlib.setting("ANDROID_SERIAL")


def host_ip(u=None):
    """The host address the phone can reach. ANDROID_HOST_IP wins; else
    probe the phone's own LAN IP and take the host address on its subnet
    (USB-tether rndis hands out a different /24 address each time)."""
    ip = simlib.setting("ANDROID_HOST_IP")
    if ip:
        return ip
    if u is None:
        return None
    dev = device_ip(u)
    if not dev:
        return None
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr", "show"],
                             capture_output=True, text=True).stdout
    except OSError:
        return None
    net = dev.rsplit(".", 1)[0] + "."
    for line in out.splitlines():
        for tok in line.split():
            if "/" in tok and tok.split("/")[0].startswith(net):
                return tok.split("/")[0]
    return None


def remote_modules(u):
    spec = u.get("android") or {}
    return list(spec.get("remote_modules") or [])


def is_remote(u, proc):
    return proc in remote_modules(u)


def _adb(u, *args, timeout=30, binary=False):
    cmd = ["adb"]
    serial = serial_of(u)
    if serial:
        cmd += ["-s", serial]
    cmd += list(args)
    return subprocess.run(cmd, capture_output=True,
                          text=not binary, timeout=timeout)


def adb_ok(u):
    r = _adb(u, "get-state")
    return r.returncode == 0 and r.stdout.strip() == "device"


def app_installed(u):
    r = _adb(u, "shell", "pm", "path", APP_PKG)
    return r.returncode == 0 and APP_PKG in r.stdout


def device_ip(u):
    """The phone's LAN IPv4. ANDROID_DEVICE_IP wins; else prefer the
    interface on the same subnet as the host IP (rndis/wlan), then any
    non-loopback - cellular rmnet IPs are unreachable for s2s."""
    env_ip = simlib.setting("ANDROID_DEVICE_IP")
    if env_ip:
        return env_ip
    r = _adb(u, "shell", "ip", "-f", "inet", "addr")
    if r.returncode != 0:
        return None
    ips = [ln.split()[1].split("/")[0] for ln in r.stdout.splitlines()
           if ln.strip().startswith("inet ") and "127.0.0.1" not in ln]
    hip = simlib.setting("ANDROID_HOST_IP")
    if hip:
        net = hip.rsplit(".", 1)[0] + "."
        for ip in ips:
            if ip.startswith(net):
                return ip
    return ips[0] if ips else None


def measure_clock_offset_ms(u, samples=5):
    """device_unix_ms - host_unix_ms, min-RTT sample (HIL.md Q4)."""
    best = None
    for _ in range(samples):
        t0 = time.time_ns()
        r = _adb(u, "shell", "date", "+%s%N", timeout=10)
        t1 = time.time_ns()
        if r.returncode != 0 or not r.stdout.strip().isdigit():
            continue
        dev_ms = int(r.stdout.strip()) // 1_000_000
        host_ms = (t0 + t1) // 2_000_000  # midpoint, ms
        rtt = t1 - t0
        if best is None or rtt < best[1]:
            best = (dev_ms - host_ms, rtt)
    return best[0] if best else 0


def push_module_config(u, mod, module_dir):
    """Push the generated config set into de_override/<mod>/ on the phone.

    The app overlays de_override/<mod>/* onto the module workDir at START;
    file names there must match the asset names (*.config.module.json).
    """
    cfg = os.path.join(module_dir, f"{mod}.config.json")
    local = os.path.join(module_dir, f"{mod}.local")
    dest = f"{OVERRIDE_DIR}/{mod}"
    r = _adb(u, "shell", "mkdir", "-p", dest)
    if r.returncode != 0:
        raise RuntimeError(f"adb mkdir {dest} failed: {r.stderr.strip()}")
    if os.path.exists(cfg):
        # host-absolute state paths mean nothing on the phone - the
        # module workdir is files/<mod>/, so make them relative
        import json as _json
        cfg_obj = _json.loads(re.sub(r"//[^\n]*", "", open(cfg).read()))
        for k in ("de_journal_dir", "de_plan_file", "de_package_file",
                  "de_run_file"):
            if k in cfg_obj:
                cfg_obj[k] = os.path.basename(cfg_obj[k])
        if "de_journal_dir" in cfg_obj:
            cfg_obj["de_journal_dir"] = "journal/"
        import tempfile
        fd, tmp = tempfile.mkstemp(suffix=".json")
        with os.fdopen(fd, "w") as fh:
            _json.dump(cfg_obj, fh, indent=1)
        r = _adb(u, "push", tmp, f"{dest}/{mod}.config.module.json")
        os.unlink(tmp)
        if r.returncode != 0:
            raise RuntimeError(f"adb push {mod} config failed: {r.stderr.strip()}")
    if os.path.exists(local):
        r = _adb(u, "push", local, f"{dest}/{mod}.local")
        if r.returncode != 0:
            raise RuntimeError(f"adb push {mod} local failed: {r.stderr.strip()}")


def start_module(u, mod):
    r = _adb(u, "shell", "am", "start-foreground-service", "-n",
             f"{APP_PKG}/{MODULE_SERVICE[mod]}", "-a", f"{ACTION_PREFIX}.START")
    if r.returncode != 0:
        raise RuntimeError(f"am START {mod} failed: {r.stderr.strip() or r.stdout.strip()}")


def stop_module(u, mod):
    _adb(u, "shell", "am", "startservice", "-n",
         f"{APP_PKG}/{MODULE_SERVICE[mod]}", "-a", f"{ACTION_PREFIX}.STOP")


def module_alive(u, mod):
    r = _adb(u, "shell", "pidof", MODULE_PROC[mod])
    return r.returncode == 0 and r.stdout.strip().isdigit()


def wait_module_down(u, mod, timeout_s=8):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if not module_alive(u, mod):
            return True
        time.sleep(0.5)
    return False


# ---------------------------------------------------------------- mirror


def _tar_pull(u, mod, unit_dir):
    """Snapshot the module's device files into <unit_dir>/<mod>/device/;
    a remote de_comm's journal also lands in <unit_dir>/journal/ where
    every engine journal reader looks for it."""
    dest = os.path.join(unit_dir, mod, "device")
    os.makedirs(dest, exist_ok=True)
    r = _adb(u, "exec-out", "run-as", APP_PKG,
             "tar", "-cf", "-", "-C", DEVICE_FILES, mod,
             timeout=60, binary=True)
    if not (r.returncode == 0 and r.stdout):
        return False
    out = subprocess.run(["tar", "-xf", "-", "-C", dest],
                         input=r.stdout, capture_output=True)
    if mod == "de_comm":
        # device files/de_comm/journal/*.log -> unit_dir/journal/*.log
        r2 = _adb(u, "exec-out", "run-as", APP_PKG,
                  "tar", "-cf", "-", "-C", f"{DEVICE_FILES}/de_comm",
                  "journal", timeout=60, binary=True)
        if r2.returncode == 0 and r2.stdout:
            subprocess.run(["tar", "-xf", "-", "-C", unit_dir],
                           input=r2.stdout, capture_output=True)
    return out.returncode == 0


def mirror_loop(u, unit_dir, interval_s=2.0):
    """`python3 engine/android.py mirror <sim> <unit>` body — runs forever.

    Polls each remote module's device files (journals) and streams its
    logcat tag into logs/<mod>.log so the local readers stay unchanged.
    """
    sim_name = u["sim"]
    del sim_name  # the run dir is already resolved by the caller

    log_procs = {}
    log_files = {}

    def _spawn_logcat(mod, history):
        """adb logcat follower for one module tag. `history` replays the
        whole tag buffer (first spawn - catches the startup banner);
        respawns use -T 1 so old lines don't duplicate."""
        log_path = os.path.join(unit_dir, "logs", f"{mod}.log")
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        lf = log_files.get(mod) or open(log_path, "ab", buffering=0)
        log_files[mod] = lf
        cmd = ["adb"]
        serial = serial_of(u)
        if serial:
            cmd += ["-s", serial]
        cmd += ["logcat", "-s", mod] if history \
            else ["logcat", "-s", mod, "-T", "1"]
        log_procs[mod] = subprocess.Popen(cmd, stdout=lf,
                                          stderr=subprocess.DEVNULL)

    for mod in remote_modules(u):
        _spawn_logcat(mod, history=True)

    try:
        while True:
            for mod in remote_modules(u):
                p = log_procs.get(mod)
                if p is None or p.poll() is not None:
                    _spawn_logcat(mod, history=False)
                try:
                    _tar_pull(u, mod, unit_dir)
                except Exception:
                    pass
            time.sleep(interval_s)
    finally:
        for p in log_procs.values():
            try:
                p.kill()
            except Exception:
                pass
        for lf in log_files.values():
            try:
                lf.close()
            except Exception:
                pass


def spawn_mirror(u, run_dir):
    """Detach `engine/android.py mirror` for this unit; returns the Popen."""
    engine_dir = os.path.dirname(os.path.abspath(__file__))
    unit_dir = simlib.unit_dir(u)
    log_path = os.path.join(unit_dir, "logs", "android_mirror.log")
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    lf = open(log_path, "ab")
    # embed RUN_DIR in argv so pid_is_ours / stray sweep recognise it
    return subprocess.Popen(
        ["python3", os.path.join(engine_dir, "android.py"), "mirror",
         u["sim"], u["name"], run_dir],
        stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)


# ---------------------------------------------------------------- doctor


def doctor_rows(d, required=False, probe=False):
    """./sim doctor rows for Android units (mirrors esp32.doctor_rows)."""
    adb = simlib.setting("ANDROID_SERIAL")
    host = host_ip()
    adb_bin = shutil_which("adb")
    if not adb_bin:
        (d.missing if required else d.optional)(
            "adb", "adb not on PATH",
            "install Android platform-tools")
        return
    d.ok("adb", adb_bin)
    d.check("ANDROID_HOST_IP", bool(host), host or "",
            "set ANDROID_HOST_IP in sim.local.env to the dev host LAN IP",
            required=required)
    if not adb:
        (d.missing if required else d.optional)(
            "ANDROID_SERIAL", "not set",
            "set ANDROID_SERIAL in sim.local.env")
        return
    r = subprocess.run(["adb", "-s", adb, "get-state"],
                       capture_output=True, text=True, timeout=10)
    if not (r.returncode == 0 and r.stdout.strip() == "device"):
        (d.missing if required else d.optional)(
            f"device {adb}", "not connected",
            "adb connect / check cable + usb debugging")
        return
    d.ok(f"device {adb}", "connected")
    if probe:
        r = subprocess.run(["adb", "-s", adb, "shell", "pm", "path", APP_PKG],
                           capture_output=True, text=True, timeout=10)
        d.check("andruav app", APP_PKG in r.stdout,
                "installed", "install the debug APK on the device",
                required=required)


def shutil_which(cmd):
    for p in os.environ.get("PATH", "").split(os.pathsep):
        c = os.path.join(p, cmd)
        if os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    return None


def main(argv):
    if len(argv) >= 4 and argv[0] == "mirror":
        _, sim_name, unit_name, run_dir = argv[:4]
        m = simlib.load_manifest(sim_name)
        u = next(x for x in m["units"] if x["name"] == unit_name)
        mirror_loop(u, simlib.unit_dir(u))
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    import sys
    sys.exit(main(sys.argv[1:]))
