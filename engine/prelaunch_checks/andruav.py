"""prelaunch_checks/andruav.py - stage 3: Andruav on a real phone.

The Andruav app connects to the target server like a unit. These checks reuse
the same browser viewer and the same relay probe as de_camera, on the phone
unit. With no phone attached the result is SKIP, and a P0 SKIP is a no-go
unless the human signs the checklist item with --signoff.

Target key: test_phone: {name: <unit name>, party: <party id, optional>,
adb_serial: <optional>}.
"""

import copy
import shutil
import subprocess

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
