"""prelaunch_checks/slots.py - checks that exist only for a production mirror.

The mirror shares the machines with live, and each slot keeps its ports for
all time. So the ports must not collide, the mirror must be a closed set
(mirror auth to mirror comm), and the test unit must differ from a production
unit only in auth_port, or the test does not prove what ships.

Target keys: other_slot: {ports: [...], relay_range: [a, b]}, and
test_unit: {config: <path>, production_config: <path>}.
"""

import json
import os

from prelaunch_checks import config, reach


def _slot_ports(ctx):
    """Every fixed port of this slot, as {port: 'service.key'}."""
    out = {}
    for name, s in (ctx.target.get("services") or {}).items():
        for k, v in (s or {}).items():
            if k.endswith("port") and isinstance(v, int):
                out[v] = f"{name}.{k}"
    return out


def ports(ctx):
    other = ctx.target.get("other_slot")
    if not other:
        return "SKIP", f"target {ctx.name} has no other_slot"
    mine = _slot_ports(ctx)
    faults = [f"port {p} is {mine[p]} here and is used by the other slot"
              for p in other.get("ports") or [] if p in mine]
    if not other.get("ports"):
        return "SKIP", "other_slot.ports is empty, so nothing was compared"
    lo_hi = other.get("relay_range")
    if lo_hi:
        faults += [f"port {p} ({mine[p]}) is inside the other slot's relay "
                   f"range {lo_hi[0]}-{lo_hi[1]}"
                   for p in mine if lo_hi[0] <= p <= lo_hi[1]]
    st, d = config.port_overlap(ctx)
    if st == "FAIL":
        faults.append(d)
    if faults:
        return "FAIL", "; ".join(faults)
    return "PASS", (f"{len(mine)} ports of this slot are clear of the "
                    f"{len(other['ports'])} ports of the other slot")


def comm_auth(ctx):
    """Log in on the mirror's auth port. The comm server it announces must be
    the mirror's comm, or a test passes through the live servers."""
    srv = reach._srv(ctx)
    want = ctx.service("comm")
    try:
        host, port, _ = reach._gcs_login(srv)
    except Exception as e:
        return "FAIL", f"the test account cannot log in on the mirror: {e}"
    live = (ctx.target.get("other_slot") or {}).get("ports") or []
    if int(port) != int(want["port"]):
        which = "the live comm server" if int(port) in live else "another port"
        return "FAIL", (f"mirror auth sends the client to port {port}, which "
                        f"is {which}. The mirror comm is on {want['port']}")
    return "PASS", f"mirror auth announces mirror comm {host}:{port}"


def _flat(data):
    return {k: json.dumps(v, sort_keys=True) for k, v in data.items()}


def unit_config(ctx):
    """The test unit's config may differ from production only in auth_port."""
    tu = ctx.target.get("test_unit") or {}
    a, b = tu.get("config"), tu.get("production_config")
    if not a or not b:
        return "SKIP", ("target has no test_unit.config and "
                        "test_unit.production_config to compare")
    for p in (a, b):
        if not os.path.isfile(os.path.expanduser(p)):
            return "FAIL", f"{p} does not exist"
    da, _, ea = config._load_json_merged([a])
    db, _, eb = config._load_json_merged([b])
    if ea or eb:
        return "FAIL", ea or eb
    fa, fb = _flat(da), _flat(db)
    diff = sorted(k for k in set(fa) | set(fb) if fa.get(k) != fb.get(k))
    extra = [k for k in diff if k != "auth_port"]
    if extra:
        return "FAIL", (f"the test unit differs from production in "
                        f"{', '.join(extra)}, and only auth_port may differ")
    return "PASS", ("differs from production only in auth_port" if diff
                    else "same as production; auth_port is not changed")
