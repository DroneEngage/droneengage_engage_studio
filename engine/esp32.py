"""./sim esp32 <list|build|upload|check> [firmware] [--port DEV] [--baud N]
[--no-check] - the ESP32 HIL firmwares under tools/hil/.

A firmware is tools/hil/<name>/<name>.ino (+ an optional check_*.py that
verifies the flashed board from the host over --device). Default
firmware: esp32_fake_fc - the fake ArduCopter used to prove a phone's
USB (OTG) FCB link without a real flight controller.

  list                 firmwares found under tools/hil/
  build  [fw]          compile into var/esp32/<fw>/ (arduino-cli, else
                       an Arduino 1.8.x arduino-builder)
  upload [fw]          build, flash the merged image over the board's
                       USB-serial, then run the firmware's check script
                       (--no-check skips it)
  check  [fw]          only the host-side check against the flashed board

Optional feature: nothing else in the sim needs it. `./sim doctor`
lists its pieces as OPTIONAL; `./sim doctor --esp32` makes them required
and also probes the attached board with esptool.

Machine keys (sim.local.env): ARDUINO_CLI, SIM_ARDUINO_DIR, SIM_ARDUINO15,
SIM_ESP32_FQBN, SIM_ESP32_PORT, ESPTOOL, SIM_MAVLINK_C_LIB.
"""

import glob
import os
import re
import shutil
import stat
import subprocess
import sys

import simlib

HIL_DIR = os.path.join(simlib.SIM_ROOT, "tools", "hil")
BUILD_ROOT = os.path.join(simlib.VAR_DIR, "esp32")
DEFAULT_FW = "esp32_fake_fc"
ARDUINO15 = simlib.setting("SIM_ARDUINO15", os.path.expanduser("~/.arduino15"))
FQBN = simlib.setting("SIM_ESP32_FQBN", "esp32:esp32:esp32")
MAVLINK_C_LIB = simlib.setting(
    "SIM_MAVLINK_C_LIB",
    os.path.join(simlib.repo_dir("DE_MAVLINK_DIR", "droneengage_mavlink"),
                 "c_library_v2"))
# USB-serial bridge chips ESP32 dev boards ship with (by-id names)
_BRIDGE_RE = re.compile(r"CP210|CH34|CH910|FTDI|FT232|USB_Serial|UART",
                        re.I)


def _newest(paths):
    """The path whose basename sorts highest as a version (1.10 > 1.9)."""
    def key(p):
        return [int(x) if x.isdigit() else x
                for x in re.split(r"(\d+)", os.path.basename(p))]
    return max(paths, key=key) if paths else None


def firmwares():
    """{name: dir} for every tools/hil/<name>/<name>.ino."""
    out = {}
    for d in sorted(glob.glob(os.path.join(HIL_DIR, "*"))):
        name = os.path.basename(d)
        if os.path.isfile(os.path.join(d, name + ".ino")):
            out[name] = d
    return out


def arduino_cli():
    p = simlib.setting("ARDUINO_CLI") or shutil.which("arduino-cli")
    return p if p and os.path.isfile(p) else None


def arduino_builder_dir():
    """An extracted Arduino 1.8.x IDE (it ships arduino-builder):
    SIM_ARDUINO_DIR, else var/tools/arduino-1.* or ~/arduino-1.*."""
    cands = [simlib.setting("SIM_ARDUINO_DIR")] if \
        simlib.setting("SIM_ARDUINO_DIR") else \
        (glob.glob(os.path.join(simlib.VAR_DIR, "tools", "arduino-1.*"))
         + glob.glob(os.path.expanduser("~/arduino-1.*")))
    cands = [c for c in cands
             if os.path.isfile(os.path.join(c, "arduino-builder"))]
    return _newest(cands)


def toolchain():
    """("arduino-cli", path) | ("arduino-builder", ide dir) | None."""
    cli = arduino_cli()
    if cli:
        return ("arduino-cli", cli)
    ide = arduino_builder_dir()
    return ("arduino-builder", ide) if ide else None


def esp32_core():
    """The newest installed esp32 Arduino core dir."""
    return _newest(glob.glob(os.path.join(
        ARDUINO15, "packages", "esp32", "hardware", "esp32", "*")))


def esptool_cmd():
    """argv prefix that runs esptool: ESPTOOL, the core's bundled
    esptool.py, esptool on PATH, then `python -m esptool`."""
    p = simlib.setting("ESPTOOL")
    if not p:
        p = _newest(glob.glob(os.path.join(
            ARDUINO15, "packages", "esp32", "tools", "esptool_py", "*",
            "esptool.py")))
    if p and os.path.isfile(p):
        return [sys.executable, p] if p.endswith(".py") else [p]
    for name in ("esptool", "esptool.py"):
        w = shutil.which(name)
        if w:
            return [w]
    try:
        import esptool  # noqa: F401
        return [sys.executable, "-m", "esptool"]
    except ImportError:
        return None


def default_port():
    """SIM_ESP32_PORT, else the first by-id USB-serial bridge, else
    /dev/ttyUSB0 when it exists."""
    p = simlib.setting("SIM_ESP32_PORT")
    if p:
        return p
    for byid in sorted(glob.glob("/dev/serial/by-id/*")):
        if _BRIDGE_RE.search(os.path.basename(byid)):
            return byid
    return "/dev/ttyUSB0" if os.path.exists("/dev/ttyUSB0") else None


def port_usable(dev):
    try:
        return stat.S_ISCHR(os.stat(dev).st_mode) and \
            os.access(dev, os.R_OK | os.W_OK)
    except (OSError, TypeError):
        return False


def probe_chip(port):
    """esptool chip_id over `port` (resets the board). (ok, detail)."""
    cmd = esptool_cmd()
    if not cmd:
        return False, "no esptool"
    try:
        r = subprocess.run(cmd + ["--port", port, "chip_id"],
                           capture_output=True, text=True, timeout=40)
    except subprocess.TimeoutExpired:
        return False, "esptool timed out"
    m = re.search(r"Chip is (.+)", r.stdout)
    if r.returncode == 0 and m:
        return True, m.group(1).strip()
    tail = (r.stdout + r.stderr).strip().splitlines()
    return False, tail[-1] if tail else f"esptool exit {r.returncode}"


def _fw_dir(name):
    fws = firmwares()
    if name not in fws:
        raise simlib.SimError(
            f"esp32: no firmware {name!r} under {HIL_DIR} "
            f"(have: {', '.join(fws) or 'none'})")
    return fws[name]


def build(name):
    """Compile `name`; returns the merged flash image path."""
    src = _fw_dir(name)
    tc = toolchain()
    if not tc:
        raise simlib.SimError(
            "esp32: no arduino-cli and no Arduino 1.8.x IDE - see "
            "./sim doctor --esp32")
    if not os.path.isdir(os.path.join(MAVLINK_C_LIB, "ardupilotmega")):
        raise simlib.SimError(
            f"esp32: mavlink C headers not found at {MAVLINK_C_LIB} - "
            "set SIM_MAVLINK_C_LIB or DE_MAVLINK_DIR in sim.local.env")
    out = os.path.join(BUILD_ROOT, name)
    bpath, cache = os.path.join(out, "build"), os.path.join(out, "cache")
    os.makedirs(bpath, exist_ok=True)
    os.makedirs(cache, exist_ok=True)
    defines = f"build.defines=-I{MAVLINK_C_LIB} -Wno-address-of-packed-member"
    ino = os.path.join(src, name + ".ino")
    if tc[0] == "arduino-cli":
        cmd = [tc[1], "compile", "--fqbn", FQBN, "--build-path", bpath,
               "--build-property", defines, src]
    else:
        ide = tc[1]
        cmd = [os.path.join(ide, "arduino-builder"), "-compile",
               "-hardware", os.path.join(ide, "hardware"),
               "-hardware", os.path.join(ARDUINO15, "packages"),
               "-tools", os.path.join(ide, "tools-builder"),
               "-tools", os.path.join(ARDUINO15, "packages"),
               "-built-in-libraries", os.path.join(ide, "libraries"),
               "-fqbn", FQBN, "-build-path", bpath, "-build-cache", cache,
               "-prefs", defines, ino]
    print(simlib.col_dim(f"esp32: building {name} with {tc[0]} ({FQBN})"))
    r = subprocess.run(cmd, capture_output=True, text=True)
    noise = ("Using ", "Detecting ", "Generating ", "Compiling core",
             "Compiling libraries", "Linking everything")
    for ln in (r.stdout + r.stderr).splitlines():
        if r.returncode != 0 or not ln.startswith(noise):
            print("  " + ln)
    if r.returncode != 0:
        raise simlib.SimError(f"esp32: build of {name} failed")
    merged = os.path.join(bpath, name + ".ino.merged.bin")
    if not os.path.isfile(merged):
        raise simlib.SimError(
            f"esp32: {merged} not produced - needs the esp32 core >= 3.0 "
            "(it writes the merged flash image)")
    return merged


def flash(image, port, baud):
    cmd = esptool_cmd()
    if not cmd:
        raise simlib.SimError("esp32: esptool not found - see "
                              "./sim doctor --esp32")
    print(simlib.col_dim(f"esp32: flashing {os.path.basename(image)} "
                         f"to {port} @ {baud}"))
    r = subprocess.run(cmd + ["--port", port, "--baud", str(baud),
                              "--before", "default_reset",
                              "--after", "hard_reset",
                              "write_flash", "0x0", image],
                       capture_output=True, text=True)
    out = (r.stdout + r.stderr).strip().splitlines()
    keep = [ln for ln in out if ln.startswith(
        ("Chip is", "Wrote ", "Hash of data", "A fatal", "Failed"))
        or "error" in ln.lower()]
    for ln in keep or out[-5:]:
        print("  " + ln)
    if r.returncode != 0:
        raise simlib.SimError(
            f"esp32: flashing failed on {port} - is the port held by a "
            "running sim's host_serial bridge or a serial monitor?")


def check(name, port):
    """Run the firmware's check_*.py against `port`; its exit code."""
    scripts = sorted(glob.glob(os.path.join(_fw_dir(name), "check_*.py")))
    if not scripts:
        print(simlib.col_yellow(f"esp32: {name} has no check_*.py - "
                                "nothing to verify"))
        return 0
    import time
    time.sleep(2)   # the board boots after the flash's hard reset
    sys.stdout.flush()
    rc = subprocess.call([sys.executable, scripts[0], "--device", port])
    print((simlib.col_green("esp32: check PASS") if rc == 0
           else simlib.col_red(f"esp32: check FAIL (exit {rc})")))
    return rc


def doctor_rows(d, required=False, probe=False):
    """The ESP32 HIL rows for ./sim doctor. Optional unless `required`
    (doctor --esp32); `probe` also talks to the board via esptool."""
    tc = toolchain()
    d.check("esp32 toolchain", tc, f"{tc[0]} {tc[1]}" if tc else
            "no arduino-cli, no Arduino 1.8.x IDE",
            "install arduino-cli (or extract arduino-1.8.x-linux64.tar.xz "
            "into var/tools/) or set ARDUINO_CLI / SIM_ARDUINO_DIR in "
            "sim.local.env", required)
    core = esp32_core()
    d.check("esp32 core", core, core or f"none under {ARDUINO15}",
            "install the esp32 core >= 3.0 (Boards Manager or "
            "`arduino-cli core install esp32:esp32`)", required)
    et = esptool_cmd()
    d.check("esptool", et, " ".join(et[-1:]) if et else "not found",
            "comes with the esp32 core, or pip3 install esptool "
            "(or set ESPTOOL)", required)
    mav = os.path.isdir(os.path.join(MAVLINK_C_LIB, "ardupilotmega"))
    d.check("esp32 mavlink headers", mav, MAVLINK_C_LIB,
            "check out droneengage_mavlink (c_library_v2) or set "
            "SIM_MAVLINK_C_LIB in sim.local.env", required)
    port = default_port()
    usable = port_usable(port)
    d.check("esp32 board port", usable,
            (os.path.realpath(port) + (f" ({os.path.basename(port)})"
                                       if "by-id" in port else ""))
            if port else "no USB-serial board attached",
            "attach the ESP32 over USB, join the `dialout` group, or set "
            "SIM_ESP32_PORT in sim.local.env", required)
    if probe and usable:
        ok, detail = probe_chip(port)
        d.check("esp32 chip", ok, detail,
                "hold BOOT while it connects, or free the port (a sim "
                "bridge / serial monitor holding it)", required)


def main(argv):
    a = list(argv)
    opts = {"port": None, "baud": "921600"}
    no_check = "--no-check" in a
    a = [x for x in a if x != "--no-check"]
    rest = []
    i = 0
    while i < len(a):
        if a[i] in ("--port", "--baud") and i + 1 < len(a):
            opts[a[i][2:]] = a[i + 1]
            i += 2
            continue
        if a[i].startswith("-"):
            sys.exit(__doc__.strip() + f"\n\nunknown flag {a[i]}")
        rest.append(a[i])
        i += 1
    if not rest or rest[0] not in ("list", "build", "upload", "check"):
        print(__doc__.strip())
        return 1
    sub, fw = rest[0], (rest[1] if len(rest) > 1 else DEFAULT_FW)
    if sub == "list":
        for name, d in firmwares().items():
            chk = glob.glob(os.path.join(d, "check_*.py"))
            print(f"{name:20s} {os.path.relpath(d, simlib.SIM_ROOT)}"
                  + (f"  (check: {os.path.basename(chk[0])})" if chk else ""))
        return 0
    if sub == "build":
        print(build(fw))
        return 0
    port = opts["port"] or default_port()
    if not port:
        raise simlib.SimError("esp32: no board port found - attach the "
                              "ESP32 or pass --port / set SIM_ESP32_PORT")
    if not port_usable(port):
        raise simlib.SimError(f"esp32: {port} is not a read/write serial "
                              "device (dialout group?)")
    if sub == "upload":
        flash(build(fw), port, opts["baud"])
        print(simlib.col_green(f"esp32: {fw} flashed to {port}"))
        if no_check:
            return 0
    return check(fw, port)
