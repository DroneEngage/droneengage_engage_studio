"""./sim doctor [sim.yaml] [--json] - is this machine ready to run a sim?

One line per item:

  OK       <item>  <detail>
  MISSING  <item>  <detail> - fix: <one-line fix>
  OPTIONAL <item>  <detail> - fix: <one-line fix>

With no sim file the whole default path is checked. With a sim file
only what that sim needs is required - the rest still prints as
OPTIONAL, so `./sim doctor sims/quadcopter.yaml` does not fail on the
camera repo it never uses. Mavlink3DMap is always OPTIONAL (the sims
run without it; render modes degrade, never block). The ESP32 HIL rows
(./sim esp32) are OPTIONAL too unless --esp32 is given, which makes them
required and probes the attached board with esptool (resets it). Exit 1
on any MISSING.
"""

import importlib
import json
import os
import shutil
import socket
import subprocess
import sys

import simlib


def _which(cmd_or_path):
    """An executable by PATH name or by path; None when absent."""
    if os.sep in cmd_or_path:
        return cmd_or_path if os.path.isfile(cmd_or_path) else None
    return shutil.which(cmd_or_path)


class Doctor:
    """Collects {item, status, detail, fix} rows; `fail` counts the
    required misses that decide the exit code."""

    def __init__(self):
        self.rows = []

    def ok(self, item, detail=""):
        self.rows.append({"item": item, "status": "ok",
                          "detail": detail, "fix": ""})

    def missing(self, item, detail, fix):
        self.rows.append({"item": item, "status": "missing",
                          "detail": detail, "fix": fix})

    def optional(self, item, detail, fix=""):
        self.rows.append({"item": item, "status": "optional",
                          "detail": detail, "fix": fix})

    def check(self, item, present, detail, fix, required=True):
        """present truthy -> ok; else missing (required) or optional."""
        if present:
            self.ok(item, detail)
        elif required:
            self.missing(item, detail, fix)
        else:
            self.optional(item, detail, fix)

    @property
    def failed(self):
        return any(r["status"] == "missing" for r in self.rows)


def _module_check(d, name, spec, required, used_by=None):
    """One line for a catalog module: repo dir, then its resolved bin.
    `used_by` names the sim that requires it (None = the no-sim full
    check, where a repo that is not even checked out is informational)."""
    repo_dir = simlib.module_dir(spec)
    envs = ", ".join(k for k in
                     (spec.get("dir_env"), spec.get("bin_env")) if k)
    label = f"module {name}"
    if not os.path.isdir(repo_dir):
        # a repo that is not checked out can only be REQUIRED by a sim
        # that uses the module - elsewhere it is noise
        d.check(label, False,
                f"repo {spec['repo']} not found at {repo_dir}",
                f"clone {spec['repo']} under $DE_CODE or set "
                f"{spec.get('dir_env', '<dir_env>')} in sim.local.env",
                required)
        return
    try:
        b = simlib.module_bin(name, spec)
    except Exception as e:
        d.check(label, False, f"cannot resolve bin ({e})",
                f"check {envs or spec['repo'] + ' layout'} in "
                "sim.local.env", required)
        return
    d.check(label, os.path.isfile(b), b,
            f"build {name} in {repo_dir}"
            + (f" or set {envs} in sim.local.env" if envs else ""),
            required)


def _server_repo_check(d, label, repo, bin_name, required):
    """The spawned Rust servers: <repo>/target/release/<bin> must be a
    built executable."""
    b = simlib.server_bin(repo, bin_name)
    ok = os.path.isfile(b) and os.access(b, os.X_OK)
    if _which("cargo"):
        fix = f"cd {repo} && cargo build --release"
    else:
        fix = ("install rustup (https://rustup.rs), then cd " + repo +
               " && cargo build --release")
    d.check(label, ok, b, fix, required)


def gather(sim=None, esp32=False):
    """All check rows for this machine (+ `sim`'s extra needs when
    given). sim is a resolved simlib.load_sim() dict or None. esp32
    makes the ESP32 HIL rows required and probes the board."""
    d = Doctor()
    catalog = simlib.load_catalog()

    # ---- always: the machine itself
    d.check("sim.local.env", os.path.isfile(simlib.LOCAL_ENV),
            simlib.LOCAL_ENV,
            "cp sim.local.env.example sim.local.env and edit the paths")
    d.check("DE_CODE", os.path.isdir(simlib.DE_CODE), simlib.DE_CODE,
            "set DE_CODE in sim.local.env to your checkouts root")
    for mod in ("websockets", "pymavlink", "yaml"):
        try:
            importlib.import_module(mod)
            d.ok(f"python {mod}")
        except ImportError:
            pkg = "pyyaml" if mod == "yaml" else mod
            d.missing(f"python {mod}", "import failed",
                      f"pip3 install {pkg}")

    # ---- what the (optional) sim needs
    srv_type = (sim["server"]["type"] if sim else "real")
    real_stack = srv_type == "real"
    auth_on = bool(sim and sim["server"].get("auth"))
    render = (sim.get("render") if sim else {}) or {}
    render_mode = render.get("mode")
    render_active = bool(render_mode in ("viewer", "cameras", "physics",
                                         "full")
                         and not (render.get("headless")
                                  and render_mode != "physics"))

    # node: the render farm (vite/mav_bridge) and the webclient stack
    # run it; the spawned servers are Rust binaries. A real-stack sim
    # with no render never needs it
    node_needed = (sim is None) or render_active
    node = _which(simlib.NODE)
    d.check("node", node, node or simlib.NODE,
            "install node (nvm) or set NODE in sim.local.env",
            node_needed)

    # openssl: the fake server generates its certs with it
    d.check("openssl", _which("openssl"), "",
            "apt install openssl",
            sim is None or srv_type == "fake")

    # de_comm: every sim needs it
    d.check("de_comm", os.path.isfile(simlib.DE_COMM_BIN),
            simlib.DE_COMM_BIN,
            "build de_comm (droneengage_communication ./build.sh) or set "
            "DE_COMM_BIN in sim.local.env")

    # spawned server repos: the Rust release binaries
    if sim is None or real_stack:
        _server_repo_check(d, "server repo", simlib.SERVER_REPO,
                           simlib.SERVER_BINS["de_server"],
                           real_stack or sim is None)
        if sim is None or auth_on:
            _server_repo_check(d, "auth repo", simlib.AUTH_REPO,
                               simlib.SERVER_BINS["de_auth"],
                               sim is None or auth_on)
        if sim is None or sim["server"].get("udp_relay"):
            _server_repo_check(d, "udp relay repo",
                               simlib.UDP_RELAY_REPO,
                               simlib.SERVER_BINS["udp_relay"], True)
    else:
        # external: the user's own stack - report reachability, never
        # fail (the user starts it when running the sim)
        srv = sim["server"]
        host, port = (srv["auth_ip"], srv["auth_port"]) \
            if auth_on else (srv["ip"], srv["port"])
        up = False
        try:
            socket.create_connection((host, port), 2).close()
            up = True
        except OSError:
            pass
        d.check(f"external server {host}:{port}", up,
                "reachable" if up else "not reachable (start your own "
                "stack when you run this sim)", "", False)

    # SITL vehicles: only the ones the sim flies (all three for the
    # no-sim full check)
    vehicles = (sorted({u["fc"]["vehicle"] for u in sim["units"]
                        if u["fc"]})
                if sim else sorted(simlib.FC_KINDS))
    if sim is not None and not vehicles:
        d.optional("sitl", "no fc units in this sim")
    for veh in vehicles:
        b = os.path.join(simlib.ARDUPILOT_DIR,
                         "build", "sitl", "bin",
                         simlib.FC_KINDS[veh]["bin"])
        d.check(f"sitl {veh}", os.path.isfile(b), b,
                f"cd {simlib.ARDUPILOT_DIR} && ./waf {veh} "
                "(or set ARDUPILOT_DIR in sim.local.env)")

    # catalog modules: used -> required; unused -> optional. No sim:
    # required only when the repo is checked out (a repo you do not
    # have is not a readiness gap for the default fleet)
    used = set()
    if sim:
        for u in sim["units"]:
            used.update(simlib.module_base(m, catalog)
                        for m in u["modules"] if m != "de_comm")
    for name in sorted(catalog):
        spec = catalog[name]
        if sim:
            _module_check(d, name, spec, name in used)
        else:
            _module_check(d, name, spec,
                          os.path.isdir(simlib.module_dir(spec)))

    # cameras: render-fed units need their v4l2loopback labels; a plain
    # de_camera unit needs a V4L2 device (or its pinned /dev/videoN)
    if sim:
        labels = simlib.v4l2_label_map()
        for u in sim["units"]:
            if u.get("cam_labels"):
                need = list(u["cam_labels"])
                if "de_yolo_ai" not in u["modules"]:
                    need = need[:1]
                missing = [l for l in need if l not in labels]
                n_cam = max(1, sum(1 for x in sim["units"]
                                   if x.get("cam_labels")))
                d.check(f"v4l2loopback {u['name']}", not missing,
                        ", ".join(need),
                        f"sudo tools/setup_v4l2loopback.sh {n_cam} "
                        "or set SIM_V4L2_SRC_LABELS/SIM_V4L2_AI_LABELS "
                        "in sim.local.env")
            elif "de_camera" in u["modules"]:
                cam = ((u["config"].get("de_camera") or {})
                       .get("camera") or {})
                start = cam.get("camera_start_index")
                end = cam.get("camera_end_index", start)
                if isinstance(start, int):
                    devs = [f"/dev/video{n}"
                            for n in range(start, (end or start) + 1)]
                    missing = [v for v in devs if not os.path.exists(v)]
                    d.check(f"v4l2 {u['name']}", not missing,
                            ", ".join(devs),
                            "create the devices (camera repo "
                            "scripts/create_virtual_videos.sh) or "
                            "attach a camera")
                else:
                    d.check(f"v4l2 {u['name']}", bool(labels),
                            ", ".join(sorted(labels.values())) or
                            "no /dev/video* found",
                            "attach a camera or run the camera repo's "
                            "create_virtual_videos.sh")

    # libfaketime: `faketime: true` units and the clock_step fault
    # (SL-0014) - required only by a sim with a faketime unit
    ft = simlib.faketime_lib()
    d.check("libfaketime", ft, ft or "not found",
            "apt install libfaketime, or set SIM_FAKETIME_LIB in "
            "sim.local.env",
            bool(sim and any(u.get("faketime") for u in sim["units"])))

    # socat: the serial/udp fcb_link bridge (SL-0102 pty, SL-0104
    # host_serial, SL-0106 udp) - required only by a sim whose units
    # use those fcb_link kinds (bt_spp runs engine/bt_spp_bridge.py
    # instead)
    d.check("socat", _which("socat"), "",
            "apt install socat",
            bool(sim and any(
                u.get("fc") and (u["fc"].get("fcb_link") or {})
                .get("kind") in ("pty", "host_serial", "udp")
                for u in sim["units"])))

    # bt_spp fcb_link (SL-0105): an RFCOMM server needs a controller
    # that is UP, a python with AF_BLUETOOTH, and the sdptool compat
    # socket for the SPP record a phone's SDP lookup needs
    uses_spp = bool(sim and any(
        u.get("fc") and (u["fc"].get("fcb_link") or {})
        .get("kind") == "bt_spp" for u in sim["units"]))
    adapters = simlib.bt_adapters() if uses_spp else []
    ups = [a["name"] for a in adapters if a["up"]]
    d.check("bluetooth adapter",
            bool(ups),
            ("up: " + ", ".join(ups)) if ups else
            (", ".join(a["name"] for a in adapters) + " (not UP)"
             if adapters else "no controller"),
            "attach/enable a Bluetooth controller: "
            "`bluetoothctl power on` (or hciconfig hciN up)",
            uses_spp)
    if uses_spp:
        probe = "import socket,sys;sys.exit(" \
            "0 if hasattr(socket,'AF_BLUETOOTH') else 1)"
        btpy = None
        for cand in (sys.executable, "/usr/bin/python3.14",
                     "/usr/bin/python3.13", "/usr/bin/python3.12",
                     "/usr/bin/python3.11", "/usr/bin/python3"):
            if cand == "python3" or not os.path.isfile(cand):
                continue
            try:
                if subprocess.call([cand, "-c", probe],
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL) == 0:
                    btpy = cand
                    break
            except OSError:
                continue
        d.check("python AF_BLUETOOTH", btpy, btpy or "none found",
                "install a distro python3 (BlueZ sockets need "
                "AF_BLUETOOTH; conda builds often lack it)", True)
        # BlueZ >= 5 only opens the legacy sdpd socket /var/run/sdp
        # when bluetoothd runs with --compat; sdptool writes the SPP
        # record through it
        sdp_sock = os.path.exists("/var/run/sdp")
        d.check("sdptool compat", sdp_sock,
                "/var/run/sdp " + ("present" if sdp_sock else "absent"),
                "BlueZ has no sdptool socket: add --compat to "
                "bluetoothd's ExecStart (systemctl edit bluetooth, or "
                "edit /lib/systemd/system/bluetooth.service), then "
                "systemctl daemon-reload && systemctl restart "
                "bluetooth - without it the SPP record never reaches "
                "the phone's SDP lookup (the channel still serves)",
                True)

    # host_serial fcb_link (SL-0104): the bridged USB-serial adapter
    # must exist and be openable - the dialout group hint is the
    # usual miss
    import stat as _stat
    for u in (sim or {}).get("units", []):
        fl = (u.get("fc") or {}).get("fcb_link") or {}
        if fl.get("kind") != "host_serial":
            continue
        dev = str(fl.get("device") or "")
        usable = False
        try:
            usable = _stat.S_ISCHR(os.stat(dev).st_mode) and \
                os.access(dev, os.R_OK | os.W_OK)
        except OSError:
            pass
        d.check(f"fcb_link device {u['name']}", usable,
                dev or "(no device configured)",
                "attach the USB-serial adapter, and join the `dialout` "
                "group (or udev-rule the device) for rw access")

    # DE-0050: camera units' configured /dev/videoN - missing nodes
    # fail `sim up` outright; existing-but-not-capture ones (unfed
    # exclusive_caps loopback) are a warning
    cam_idx = sorted({i for u in (sim or {}).get("units", [])
                      for i in simlib.camera_video_indices(u)})
    for i in cam_idx:
        dev = f"/dev/video{i}"
        if not os.path.exists(dev):
            d.check(f"camera device {dev}", False, "missing",
                    f"./sim vcam <sim> creates it (sudo), or fix the "
                    "unit's camera_start_index/camera_end_index")
        elif not simlib.video_capture_capable(i):
            d.check(f"camera device {dev}", False,
                    "exists but no CAPTURE (no producer)",
                    "stream it - camera_feed: in the sim or "
                    "scripts/video_clock.sh", required=False)
        else:
            d.ok(f"camera device {dev}",
                 "loopback" if simlib.video_is_loopback(i) else "real")

    # render deps: the frame farm's chromium + ffmpeg (cameras|full);
    # Mavlink3DMap itself is always optional (never a failure)
    needs_farm = bool(sim and render_mode in ("cameras", "full")
                      and not render.get("headless"))
    for tool, env, pkg in (("chromium", simlib.CHROMIUM_BIN,
                            "google-chrome / chromium"),
                           ("ffmpeg", simlib.FFMPEG_BIN, "ffmpeg")):
        p = _which(env)
        d.check(tool, p, p or env,
                f"install {pkg} or set its key in sim.local.env",
                needs_farm)
    m3d = simlib.MAVLINK3DMAP_DIR
    bridge = os.path.join(m3d, "backend", "src", "udp2websocket.js")
    vite = next((p for p in (
            os.path.join(m3d, "node_modules", "vite", "bin", "vite.js"),
            os.path.join(m3d, "frontend", "node_modules", "vite",
                         "bin", "vite.js"))
        if os.path.isfile(p)), None)
    m3d_ok = os.path.isfile(bridge) and bool(vite)
    if m3d_ok:
        mism = simlib.m3d_version_mismatch(m3d)
        if mism is None:
            d.ok("Mavlink3DMap",
                 f"{m3d} ({simlib.m3d_version(m3d)})")
        else:
            d.optional("Mavlink3DMap", f"{m3d} - {mism}",
                       "checkout the pinned release or set "
                       "SIM_M3D_VERSION in sim.local.env")
    else:
        d.optional("Mavlink3DMap",
                   f"{m3d} incomplete (render modes only)",
                   "check out Mavlink3DMap + npm install, set "
                   "MAVLINK3DMAP_DIR in sim.local.env")

    # ESP32 HIL firmwares (./sim esp32): optional gear - required and
    # probed only with --esp32
    import esp32 as _esp32
    _esp32.doctor_rows(d, required=esp32, probe=esp32)

    # Android units (SL-0095): required+probed when the loaded sim
    # actually declares an android: unit, else optional
    import android as _android
    needs_android = bool(sim and
                         any(u.get("android") for u in sim["units"]))
    _android.doctor_rows(d, required=needs_android, probe=needs_android)

    # ports: the sim's own set, else the default server pair
    if sim:
        held = simlib.ports_held(simlib.used_ports(sim)
                                 - simlib.remote_bound_ports(sim))
        d.check("ports", not held,
                f"{len(simlib.used_ports(sim))} ports",
                f"ports {held} already bound - ./sim status, down the "
                "holder, or --slot-base")
    else:
        held = simlib.ports_held({simlib.SERVER_PORT,
                                  simlib.SERVER_PORT + 1})
        d.check("ports", not held,
                f"default server {simlib.SERVER_PORT}/"
                f"{simlib.SERVER_PORT + 1}",
                f"ports {held} already bound - ./sim status or down "
                "the holder")
    return d


def _load_target(path):
    """A sims/ file, or a test/scenario - then the fleet it runs (so
    `./sim doctor tests/x.yaml` answers "can this machine run x?")."""
    raw = simlib.load_yaml(path) or {}
    import scenario
    if isinstance(raw, dict) and not scenario.is_sim_spec(raw):
        # probe_ports like `./sim run`: the run picks a free slot, so
        # the ports row reports that slot, not the default one
        return simlib.load_sim(scenario.load_scenario(path)["sim"])
    return simlib.load_sim(path)


def main(argv):
    a = list(argv)
    json_out = "--json" in a
    esp32 = "--esp32" in a
    a = [x for x in a if x not in ("--json", "--esp32")]
    files = [x for x in a if not x.startswith("-")]
    bad = [x for x in a if x.startswith("-")]
    if bad or len(files) > 1:
        sys.exit("usage: ./sim doctor [sim.yaml|test.yaml] [--json] "
                 "[--esp32]"
                 + (f" - unknown {' '.join(bad)}" if bad else ""))
    sim, load_err = None, None
    if files:
        try:
            sim = _load_target(files[0])
        except simlib.SimError as e:
            # e.g. a clock_step test on a machine without libfaketime -
            # report it as a row (keeps --json parseable), then check
            # the machine itself
            load_err = str(e)
    d = gather(sim, esp32=esp32)
    if load_err:
        d.missing(files[0], load_err, "see the message - ./sim validate "
                  f"{files[0]} shows the same problem")
    if json_out:
        print(json.dumps(d.rows, indent=1))
    else:
        tags = {"ok": ("OK", simlib.col_green),
                "missing": ("MISSING", simlib.col_red),
                "optional": ("OPTIONAL", simlib.col_yellow)}
        for r in d.rows:
            tag, c = tags[r["status"]]
            line = f"{c(tag.ljust(8))} {r['item']}"
            if r["detail"]:
                line += f"  {r['detail']}"
            if r["fix"]:
                line += f" - fix: {r['fix']}"
            print(line)
    return 1 if d.failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
