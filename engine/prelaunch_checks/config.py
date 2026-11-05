"""prelaunch_checks/config.py - stage 0: lint the release config.

Runs in seconds with no network. ICE settings live in four places and
nothing else compares them: the webclient config, the de_camera config,
the Andruav built-in list and coturn. A fault in any of them stops video in
the field while every sim test stays green.

The files come from the target's `config:` map. Each value is one path or
a list of paths merged in order (server.config, then server.config.local):

    config:
      webclient: [public/config.json, public/config.local.json]
      de_camera: de_camera.config.module.json
      andruav: .../PnSignalingParams.java
      coturn: /etc/turnserver.conf
      comm: [server.config, server.config.local]
      relay: [server.config, server.config.local]
      storage: [server.config, server.config.local]

Optional target keys: ice_allowed_hosts (hosts this target can reach; an
airgap target always enforces it), other_slot.relay_range (the other
production slot) and services.turn.relay_range (when the coturn file holds
install-script placeholders).

A probe returns (status, detail). No detail ever holds a credential: ICE
entries are named by their url only.
"""

import json
import os
import re
from urllib.parse import urlsplit

LOOPBACK = re.compile(r"^(127\.|localhost$|::1$|0\.0\.0\.0$)")
_ANDRUAV_ENTRY = re.compile(
    r'IceServer\(\s*"([^"]+)"(?:\s*,\s*"([^"]*)"\s*,\s*"([^"]*)")?\s*\)')


def strip_json_comments(text):
    """JSON with // comments and trailing commas, as the DE configs are.
    Text inside strings stays as it is ("turn://" or a url)."""
    out, i, n, in_str = [], 0, len(text), False
    while i < n:
        ch = text[i]
        if in_str:
            out.append(ch)
            if ch == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 1
            elif ch == '"':
                in_str = False
        elif ch == '"':
            in_str = True
            out.append(ch)
        elif text.startswith("//", i):
            while i < n and text[i] != "\n":
                i += 1
            continue
        elif text.startswith("/*", i):
            end = text.find("*/", i + 2)
            i = n if end < 0 else end + 2
            continue
        else:
            out.append(ch)
        i += 1
    return re.sub(r",(\s*[}\]])", r"\1", "".join(out))


def _paths(ctx, key):
    """The target's config paths for `key`, or None when not declared."""
    raw = (ctx.target.get("config") or {}).get(key)
    if not raw:
        return None
    return [raw] if isinstance(raw, str) else list(raw)


def _load_json_merged(paths):
    """Merge the files that exist, in order. Returns (data, files, error).
    The first path must exist; a later missing path is a normal absent
    .local file."""
    data, used = {}, []
    for i, p in enumerate(paths):
        p = os.path.expanduser(p)
        if not os.path.isfile(p):
            if i == 0:
                return None, used, f"{p} does not exist"
            continue
        try:
            with open(p, errors="replace") as fh:
                data.update(json.loads(strip_json_comments(fh.read())))
        except ValueError as e:
            return None, used, f"{p} is not valid JSON: {e}"
        used.append(p)
    return data, used, None


def parse_coturn(text):
    """-> {users: {name: password}, min, max, auth_secret}. Install-script
    ### placeholders count as not set."""
    out = {"users": {}, "min": None, "max": None, "auth_secret": False}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, _, val = line.partition("=")
        if key == "user" and ":" in val and "#" not in val:
            name, _, pwd = val.partition(":")
            out["users"][name] = pwd
        elif key in ("min-port", "max-port") and val.isdigit():
            out["min" if key == "min-port" else "max"] = int(val)
        elif key in ("use-auth-secret", "static-auth-secret"):
            out["auth_secret"] = True
    return out


def _read_coturn(ctx):
    """(parsed, path, error); parsed is None when not declared."""
    paths = _paths(ctx, "coturn")
    if not paths:
        return None, None, None
    p = os.path.expanduser(paths[0])
    if not os.path.isfile(p):
        return None, p, f"coturn config {p} does not exist"
    with open(p, errors="replace") as fh:
        return parse_coturn(fh.read()), p, None


def parse_andruav_ice(text):
    return [dict(urls=m.group(1), username=m.group(2), password=m.group(3))
            for m in _ANDRUAV_ENTRY.finditer(text)]


def ice_host(url):
    """stun:h:p, turn:h:p?transport=tcp, turns:h -> h"""
    rest = url.split(":", 1)[1] if ":" in url else url
    rest = rest.split("?", 1)[0]
    return urlsplit("//" + rest).hostname or rest


def _entry_urls(entry):
    u = entry.get("urls") or entry.get("url") or []
    return [u] if isinstance(u, str) else list(u)


def _ice_lists(ctx):
    """[(label, field, entries, error)] for each list the target declares."""
    out = []
    p = _paths(ctx, "webclient")
    if p:
        data, used, err = _load_json_merged(p)
        where = ", ".join(used) or p[0]
        out.append((f"webclient ({where})", "CONST_ICE_SERVERS",
                    (data or {}).get("CONST_ICE_SERVERS"), err))
    p = _paths(ctx, "de_camera")
    if p:
        data, used, err = _load_json_merged(p)
        where = ", ".join(used) or p[0]
        out.append((f"de_camera ({where})", "iceServers",
                    (data or {}).get("iceServers"), err))
    p = _paths(ctx, "andruav")
    if p:
        fp = os.path.expanduser(p[0])
        if os.path.isfile(fp):
            with open(fp, errors="replace") as fh:
                out.append((f"andruav ({fp})", "defaultIceServers",
                            parse_andruav_ice(fh.read()), None))
        else:
            out.append((f"andruav ({fp})", "defaultIceServers", None,
                        f"{fp} does not exist"))
    return out


def _allowed_hosts(ctx):
    """Hosts this target can reach, or None when there is no rule."""
    allowed = list(ctx.target.get("ice_allowed_hosts") or [])
    turn = (ctx.target.get("services") or {}).get("turn") or {}
    if turn.get("host"):
        allowed.append(turn["host"])
    if allowed or ctx.role == "airgap":
        return set(allowed)
    return None


def ice(ctx):
    lists = _ice_lists(ctx)
    if not lists or not _paths(ctx, "webclient"):
        return "SKIP", (f"target {ctx.name} has no config.webclient: the ICE "
                        f"lists were not read")
    coturn, coturn_path, cerr = _read_coturn(ctx)
    allowed = _allowed_hosts(ctx)
    faults, seen = [], 0
    if cerr:
        faults.append(cerr)
    for label, field, entries, err in lists:
        if err:
            faults.append(f"{label}: {err}")
            continue
        if not entries:
            faults.append(f"{label}: {field} is empty, so a peer has no STUN "
                          f"or TURN server and video fails across a NAT")
            continue
        for e in entries:
            for url in _entry_urls(e):
                seen += 1
                host = ice_host(url)
                if allowed is not None and host not in allowed:
                    faults.append(f"{label}: {field} has {url}, and target "
                                  f"{ctx.name} cannot reach {host}")
                if not url.startswith("turn"):
                    continue
                for v in (e.get("password"), e.get("credential")):
                    if v:
                        ctx.secrets.add(v)
                faults += _coturn_faults(label, field, url, e, coturn,
                                         coturn_path, allowed, host)
    if faults:
        return "FAIL", "; ".join(faults)
    return "PASS", (f"{len(lists)} ICE lists, {seen} urls"
                    + ("; coturn not declared, TURN users not compared"
                       if coturn is None else ""))


def _coturn_faults(label, field, url, entry, coturn, coturn_path, allowed,
                   host):
    """Compare one TURN entry with the coturn users. Only for an entry
    that points at this target's own coturn."""
    if coturn is None or coturn["auth_secret"] or not coturn["users"]:
        return []
    if allowed is not None and host not in allowed:
        return []
    user = entry.get("username")
    pwd = entry.get("password") or entry.get("credential")
    if not user:
        return [f"{label}: {field} has {url} with no username"]
    if user not in coturn["users"]:
        return [f"{label}: {field} has {url} with a user that is not in "
                f"{coturn_path}"]
    if pwd != coturn["users"][user]:
        return [f"{label}: {field} has {url} with a password that differs "
                f"from {coturn_path}"]
    return []


def public_host(ctx):
    checked, faults = 0, []
    for key in ("comm", "storage", "relay"):
        paths = _paths(ctx, key)
        if not paths:
            continue
        data, used, err = _load_json_merged(paths)
        if err:
            faults.append(f"{key}: {err}")
            continue
        val = data.get("public_host")
        where = ", ".join(used)
        checked += 1
        if not val:
            faults.append(f"{key} ({where}): public_host is not set")
            continue
        svc = (ctx.target.get("services") or {}).get(key) or {}
        want = svc.get("public_host") or svc.get("host")
        if LOOPBACK.match(str(val)) and ctx.role != "local":
            faults.append(f"{key} ({where}): public_host is {val}, so a "
                          f"remote unit cannot reach it")
        elif want and str(val) != str(want):
            faults.append(f"{key} ({where}): public_host is {val}, and the "
                          f"target says {want}. The address may be stale")
    if not checked and not faults:
        return "SKIP", f"target {ctx.name} has no config.comm/storage/relay"
    if faults:
        return "FAIL", "; ".join(faults)
    return "PASS", f"{checked} public_host values match the target"


def udp_relay(ctx):
    paths = _paths(ctx, "comm")
    if not paths:
        return "SKIP", f"target {ctx.name} has no config.comm"
    data, used, err = _load_json_merged(paths)
    if err:
        return "FAIL", f"comm: {err}"
    where = ", ".join(used)
    entries = data.get("udp_relay")
    if not entries:
        return "FAIL", (f"comm ({where}): udp_relay is empty, so this server "
                        f"gives no UDP proxy")
    bad = [e for e in entries if not (e.get("host") and e.get("port"))]
    if bad:
        return "FAIL", f"comm ({where}): a udp_relay entry has no host or port"
    relay = (ctx.target.get("services") or {}).get("relay") or {}
    want = relay.get("control_port")
    if want and all(int(e["port"]) != int(want) for e in entries):
        return "FAIL", (f"comm ({where}): no udp_relay entry uses control port "
                        f"{want} of the target")
    return "PASS", f"{len(entries)} udp_relay entries"


def _relay_range(ctx):
    paths = _paths(ctx, "relay")
    if paths:
        data, _, err = _load_json_merged(paths)
        if not err and data.get("udp_port_range"):
            return data["udp_port_range"]
    relay = (ctx.target.get("services") or {}).get("relay") or {}
    return relay.get("udp_range")


def _turn_range(ctx):
    coturn, _, _ = _read_coturn(ctx)
    if coturn and coturn["min"] and coturn["max"]:
        return [coturn["min"], coturn["max"]]
    turn = (ctx.target.get("services") or {}).get("turn") or {}
    return turn.get("relay_range")


def port_overlap(ctx):
    ranges = []
    for label, r in (("coturn relay range", _turn_range(ctx)),
                     ("UDP relay range", _relay_range(ctx)),
                     ("other slot UDP relay range",
                      (ctx.target.get("other_slot") or {}).get("relay_range"))):
        if r:
            ranges.append((label, int(r[0]), int(r[1])))
    if len(ranges) < 2:
        return "SKIP", (f"target {ctx.name} declares {len(ranges)} port range; "
                        f"two are needed to compare")
    faults = []
    for i, (la, a0, a1) in enumerate(ranges):
        for lb, b0, b1 in ranges[i + 1:]:
            if a0 <= b1 and b0 <= a1:
                faults.append(f"{la} {a0}-{a1} overlaps {lb} {b0}-{b1}")
    if faults:
        return "FAIL", "; ".join(faults)
    return "PASS", ", ".join(f"{la} {a}-{b}" for la, a, b in ranges)
