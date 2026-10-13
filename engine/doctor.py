"""./sim doctor [sim.yaml] [--json] - is this machine ready to run a sim?

One line per item:

  OK       <item>  <detail>
  MISSING  <item>  <detail> - fix: <one-line fix>
  OPTIONAL <item>  <detail> - fix: <one-line fix>

With no sim file the whole default path is checked. With a sim file
only what that sim needs is required - the rest still prints as
OPTIONAL, so `./sim doctor sims/quadcopter.yaml` does not fail on the
camera repo it never uses. Mavlink3DMap is always OPTIONAL (the sims
run without it; render modes degrade, never block). Exit 1 on any
MISSING.
"""

import importlib
import json
import os
import shutil
import socket
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


def _server_repo_check(d, label, repo, required):
    """The spawned node servers: src/server.js + an npm-installed tree."""
    srv_js = os.path.join(repo, "src", "server.js")
    d.check(label, os.path.isfile(srv_js), srv_js,
            f"check out the repo at {repo} or point its *_DIR key in "
            "sim.local.env", required)
    if os.path.isfile(srv_js):
        nm = os.path.join(repo, "node_modules")
        d.check(f"{label} npm install", os.path.isdir(nm), nm,
                f"cd {repo} && npm install", required)


def gather(sim=None):
    """All check rows for this machine (+ `sim`'s extra needs when
    given). sim is a resolved simlib.load_sim() dict or None."""
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

    # node: spawned servers and the render farm run it; a sim with an
    # external server and no render does not need it
    node_needed = (sim is None) or real_stack or render_active
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

    # spawned server repos
    if sim is None or real_stack:
        _server_repo_check(d, "server repo", simlib.SERVER_REPO,
                           real_stack or sim is None)
        if sim is None or auth_on:
            _server_repo_check(d, "auth repo", simlib.AUTH_REPO,
                               sim is None or auth_on)
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
            used.update(m for m in u["modules"] if m != "de_comm")
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
        d.ok("Mavlink3DMap", m3d)
    else:
        d.optional("Mavlink3DMap",
                   f"{m3d} incomplete (render modes only)",
                   "check out Mavlink3DMap + npm install, set "
                   "MAVLINK3DMAP_DIR in sim.local.env")

    # ports: the sim's own set, else the default server pair
    if sim:
        held = simlib.ports_held(simlib.used_ports(sim))
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
    a = [x for x in a if x != "--json"]
    files = [x for x in a if not x.startswith("-")]
    bad = [x for x in a if x.startswith("-")]
    if bad or len(files) > 1:
        sys.exit("usage: ./sim doctor [sim.yaml|test.yaml] [--json]"
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
    d = gather(sim)
    if load_err:
        d.missing(files[0], load_err, "see the message - ./sim validate "
                  f"{files[0]} shows the same problem")
    if json_out:
        print(json.dumps(d.rows, indent=1))
    else:
        for r in d.rows:
            tag = {"ok": "OK", "missing": "MISSING",
                   "optional": "OPTIONAL"}[r["status"]]
            line = f"{tag:8s} {r['item']}"
            if r["detail"]:
                line += f"  {r['detail']}"
            if r["fix"]:
                line += f" - fix: {r['fix']}"
            print(line)
    return 1 if d.failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
