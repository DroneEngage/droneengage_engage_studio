"""prelaunch_checks/andruav.py - stage 3: Andruav on a real phone.

The Andruav app connects to the target server like a unit. These checks reuse
the same browser viewer and the same relay probe as de_camera, on the phone
unit. With no phone attached the result is SKIP, and a P0 SKIP is a no-go
unless the human signs the checklist item with --signoff.

Target key: test_phone: {name: <unit name>, party: <party id, optional>,
adb_serial: <optional>}.
"""

import copy
import json
import shutil
import subprocess

from prelaunch_checks import config as cfg
from prelaunch_checks import relay as relay_probe
from prelaunch_checks import video


def phones_attached(adb="adb"):
    """Serials adb reports as ready. An 'unauthorized' phone does not count."""
    if not shutil.which(adb):
        return []
    try:
        out = subprocess.run([adb, "devices"], capture_output=True, text=True,
                             timeout=15).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    rows = [line.split() for line in out.splitlines()[1:]]
    return [r[0] for r in rows if len(r) >= 2 and r[1] == "device"]


def _phone(ctx):
    """(phone config, None) or (None, the reason to skip)."""
    phone = ctx.target.get("test_phone") or {}
    if not phone.get("name"):
        return None, f"target {ctx.name} has no test_phone.name"
    attached = phones_attached()
    if not attached:
        return None, "no phone is attached over adb"
    serial = phone.get("adb_serial")
    if serial and serial not in attached:
        return None, f"phone {serial} is not attached (have {attached})"
    return phone, None


def _for_phone(ctx, phone):
    """A copy of ctx whose test unit is the phone. The target stays as it is."""
    other = copy.copy(ctx)
    other.target = copy.copy(ctx.target)
    other.target["test_unit"] = {"name": phone["name"],
                                 "party": phone.get("party")}
    return other


def video_normal(ctx):
    phone, why = _phone(ctx)
    if not phone:
        return "SKIP", why
    return video.camera_normal(_for_phone(ctx, phone))


def video_relay(ctx):
    phone, why = _phone(ctx)
    if not phone:
        return "SKIP", why
    return video.camera_relay(_for_phone(ctx, phone))


def relay(ctx):
    phone, why = _phone(ctx)
    if not phone:
        return "SKIP", why
    return relay_probe.mavlink_both_ways(_for_phone(ctx, phone))


# where the app keeps an ICE list that overrides its built-in one (DE-0083)
PHONE_ICE_FILE = "/sdcard/Android/data/ardupilot.andruav/files/de_override/ice_servers.json"


def phone_ice(serial=None, path=PHONE_ICE_FILE, adb="adb"):
    """(entries, None) from the ICE file on the phone, (None, None) when the
    file is absent (the app then uses its built-in list), or (None, error)."""
    cmd = [adb] + (["-s", serial] if serial else []) + ["shell", "cat", path]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as e:
        return None, f"adb could not read the phone ICE file: {e}"
    if r.returncode != 0 or "No such file" in (r.stdout + r.stderr):
        return None, None
    try:
        data = json.loads(cfg.strip_json_comments(r.stdout))
    except ValueError as e:
        return None, f"the phone ICE file {path} is not valid JSON: {e}"
    if isinstance(data, dict):
        data = data.get("iceServers")
    return (data or []), None


def stale_ice(ctx):
    """FAIL when the ICE list set on the phone points at a host the target
    cannot reach, or is empty."""
    phone, why = _phone(ctx)
    if not phone:
        return "SKIP", why
    entries, err = phone_ice(phone.get("adb_serial") or None,
                             phone.get("ice_file") or PHONE_ICE_FILE)
    if err:
        return "FAIL", f"ICE on the phone: {err}"
    if entries is None:
        return "PASS", ("the phone has no ICE override, so it uses the "
                        "built-in list that config.ice checks")
    if not entries:
        return "FAIL", ("the ICE list on the phone is empty, so a call "
                        "across a NAT has no STUN or TURN server")
    allowed = cfg._allowed_hosts(ctx)
    stale = []
    for e in entries:
        for url in cfg._entry_urls(e):
            host = cfg.ice_host(url)
            if allowed is not None and host not in allowed:
                stale.append(f"{url} (target {ctx.name} cannot reach {host})")
    if stale:
        return "FAIL", "stale ICE setting on the phone: " + "; ".join(stale)
    return "PASS", f"the phone ICE list has {len(entries)} entries, all reachable"
