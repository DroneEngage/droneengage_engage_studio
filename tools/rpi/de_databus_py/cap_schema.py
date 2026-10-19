"""
Phase-3 capability advert dialect ("de.cap/1") - validator subset +
advert hash, mirroring de_common/de_databus/cap_schema.cpp.

Normative doc: Tasks/mission_planner/Phase-3-Tasks/CAPABILITY_SCHEMA.md
"""

import hashlib
import json
import re

CAP_SCHEMA_ID = "de.cap/1"
CAP_ADVERT_MAX_BYTES = 16 * 1024
CAP_NAME_MAX_LEN = 32

CAP_SCALAR_TYPES = {
    "bool", "int", "number", "string", "enum", "geo", "array", "object"
}

_CAP_NAME_RE = re.compile(r"^[a-z][a-z0-9_]*$")


def cap_advert_hash(adverts):
    """ch = first 16 hex chars of SHA-256 over the advert strings exactly
    as sent, joined with '\n' in the order sent."""
    joined = "\n".join(adverts)
    return hashlib.sha256(joined.encode("utf-8")).hexdigest()[:16]


def cap_is_valid_name(name):
    return isinstance(name, str) and len(name) <= CAP_NAME_MAX_LEN \
        and _CAP_NAME_RE.match(name) is not None


def _type_list(param):
    """'type' may be a single type name or a list of scalar type names."""
    t = param.get("type")
    if isinstance(t, str):
        return [t]
    if isinstance(t, list) and t and all(isinstance(e, str) for e in t):
        return list(t)
    return None


def _value_matches(value, type_name):
    if type_name == "bool":
        return isinstance(value, bool)
    if type_name == "int":
        return isinstance(value, int) and not isinstance(value, bool)
    if type_name == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if type_name == "string":
        return isinstance(value, str)
    if type_name == "geo":
        return isinstance(value, list) and len(value) == 3 \
            and all(_value_matches(v, "number") for v in value)
    if type_name == "enum":
        return isinstance(value, (str, int, float, bool))
    if type_name == "array":
        return isinstance(value, list)
    if type_name == "object":
        return isinstance(value, dict)
    return False


def _check_value(param, value, path, errors, depth=0):
    types = _type_list(param)
    if not types:
        errors.append(f"{path}: schema has no usable 'type'")
        return
    # limits belong to the type the value matched (["string","int"] + int)
    t = next((c for c in types if _value_matches(value, c)), None)
    if t is None:
        errors.append(f"{path}: expected {'|'.join(types)}")
        return

    if t in ("int", "number"):
        if "min" in param and value < param["min"]:
            errors.append(f"{path}: must be >= {param['min']}")
        if "max" in param and value > param["max"]:
            errors.append(f"{path}: must be <= {param['max']}")
    if t == "enum" and value not in param.get("values", []):
        errors.append(f"{path}: not one of the allowed values")
    if t == "array" and "items" in param and depth < 4:
        for i, item in enumerate(value):
            _check_value(param["items"], item, f"{path}[{i}]", errors, depth + 1)
    if t == "object" and "fields" in param and depth < 4:
        for fname, fparam in param["fields"].items():
            if fname in value:
                _check_value(fparam, value[fname], f"{path}.{fname}", errors, depth + 1)
            elif not fparam.get("optional") and "default" not in fparam:
                errors.append(f"{path}.{fname}: missing required field")


def cap_check_params(params_schema, with_, filled):
    """Check invoke params against an action's "params" schema.
    Returns a list of error strings; `filled` gets `with_` + defaults."""
    errors = []
    filled.clear()
    if isinstance(with_, dict):
        filled.update(with_)

    if params_schema is None:
        return errors
    if not isinstance(params_schema, dict):
        errors.append("params schema is not an object")
        return errors
    if with_ is not None and not isinstance(with_, dict):
        errors.append("with: expected an object")

    if isinstance(with_, dict):
        for key in with_:
            if key not in params_schema:
                errors.append(f"{key}: unknown param")

    for name, param in params_schema.items():
        if isinstance(with_, dict) and name in with_:
            _check_value(param, with_[name], name, errors)
        elif "default" in param:
            filled[name] = param["default"]
        elif not param.get("optional"):
            errors.append(f"{name}: missing required param")

    return errors


def _validate_param(param, path, depth=0):
    if depth > 4:
        return f"{path}: param nesting too deep"
    if not isinstance(param, dict):
        return f"{path}: param must be an object"
    types = _type_list(param)
    if not types:
        return f"{path}: missing or invalid 'type'"
    for t in types:
        if t not in CAP_SCALAR_TYPES:
            return f"{path}: unknown type '{t}'"
    if len(types) > 1:
        for t in types:
            if t in ("enum", "geo", "array", "object"):
                return f"{path}: type list allows scalar types only"
    t = types[0]
    if t == "enum" and not param.get("values"):
        return f"{path}: enum requires a non-empty 'values' array"
    if t == "array" and "items" in param:
        r = _validate_param(param["items"], path + "[]", depth + 1)
        if r:
            return r
    if t == "object" and "fields" in param:
        if not isinstance(param["fields"], dict):
            return f"{path}: 'fields' must be an object"
        for fname, fparam in param["fields"].items():
            if not cap_is_valid_name(fname):
                return f"{path}: bad field name '{fname}'"
            r = _validate_param(fparam, f"{path}.{fname}", depth + 1)
            if r:
                return r
    if "min" in param and not isinstance(param["min"], (int, float)):
        return f"{path}: 'min' must be a number"
    if "max" in param and not isinstance(param["max"], (int, float)):
        return f"{path}: 'max' must be a number"
    if ("min" in param and "max" in param
            and isinstance(param["min"], (int, float))
            and isinstance(param["max"], (int, float))
            and param["min"] > param["max"]):
        return f"{path}: 'min' > 'max'"
    if "default" in param:
        errs = cap_check_params({path: param}, {path: param["default"]}, {})
        if errs:
            return f"{path}: bad default ({errs[0]})"
    return None


def _validate_param_map(map_, path):
    if not isinstance(map_, dict):
        return f"{path} must be an object"
    for name, param in map_.items():
        if not cap_is_valid_name(name):
            return f"{path}: bad name '{name}'"
        r = _validate_param(param, f"{path}.{name}")
        if r:
            return r
    return None


def cap_validate_advert(advert):
    """Returns "" on success or an error string."""
    if not isinstance(advert, dict):
        return "advert must be an object"
    if advert.get("schema") != CAP_SCHEMA_ID:
        return f"missing or wrong 'schema' (expected {CAP_SCHEMA_ID})"
    if not cap_is_valid_name(advert.get("ns")):
        return "missing or bad 'ns'"
    if not isinstance(advert.get("module"), str):
        return "missing 'module'"
    if not isinstance(advert.get("ver"), str):
        return "missing 'ver'"

    actions = advert.get("actions")
    if actions is not None:
        if not isinstance(actions, dict):
            return "'actions' must be an object"
        for name, act in actions.items():
            path = f"actions.{name}"
            if not cap_is_valid_name(name):
                return f"{path}: bad action name"
            if not isinstance(act, dict):
                return f"{path} must be an object"
            if "desc" in act and not isinstance(act["desc"], str):
                return f"{path}: 'desc' must be a string"
            if "timeout_s" in act and not (
                    isinstance(act["timeout_s"], (int, float))
                    and act["timeout_s"] > 0):
                return f"{path}: 'timeout_s' must be a positive number"
            if "deprecated_alias" in act and not isinstance(
                    act["deprecated_alias"], str):
                return f"{path}: 'deprecated_alias' must be a string"
            # TASK-PF-01: "claim" narrows a sensor-lane claim from the
            # whole namespace to this action only ("action"); absent or
            # "ns" keeps the namespace-wide behaviour
            if "claim" in act and act["claim"] not in ("ns", "action"):
                return f"{path}: 'claim' must be \"ns\" or \"action\""
            if "params" in act:
                r = _validate_param_map(act["params"], f"{path}.params")
                if r:
                    return r

    events = advert.get("events")
    if events is not None:
        if not isinstance(events, dict):
            return "'events' must be an object"
        for name, ev in events.items():
            path = f"events.{name}"
            if not cap_is_valid_name(name):
                return f"{path}: bad event name"
            if not isinstance(ev, dict):
                return f"{path} must be an object"
            if "payload" in ev:
                r = _validate_param_map(ev["payload"], f"{path}.payload")
                if r:
                    return r

    if "state" in advert:
        r = _validate_param_map(advert["state"], "state")
        if r:
            return r

    # TASK-PF-02: the event surfaces of dynamic per-resource names
    # "<ns>.<key>.<verb>". Both keys are additive: pre-PF-02 validators
    # ignore unknown top-level keys, so a PF-02 advert still validates
    # (and is forwarded verbatim) by an old de_comm.

    resources = advert.get("resources")
    if resources is not None:
        if not isinstance(resources, dict):
            return "'resources' must be an object"
        for rname, cls in resources.items():
            if not cap_is_valid_name(rname):
                return f"resources: bad resource name '{rname}'"
            if not isinstance(cls, str) or not cap_is_valid_name(cls):
                return f"resources.{rname}: bad class name"

    patterns = advert.get("event_patterns")
    if patterns is not None:
        if not isinstance(patterns, dict):
            return "'event_patterns' must be an object"
        for key, pat in patterns.items():
            path = f"event_patterns.{key}"
            # keys are exactly "*.<verb>" with <verb> a cap name
            if (not isinstance(key, str) or len(key) < 3
                    or key[0] != "*" or key[1] != "."
                    or not cap_is_valid_name(key[2:])):
                return f"{path}: pattern keys must be '*.<verb>'"
            if not isinstance(pat, dict):
                return f"{path} must be an object"
            if "kind" in pat and pat["kind"] not in ("fact", "command"):
                return f"{path}: 'kind' must be 'fact' or 'command'"
            if "res" in pat:
                res = pat["res"]
                if not isinstance(res, list):
                    return f"{path}: 'res' must be an array"
                for cls in res:
                    if not isinstance(cls, str) or not cap_is_valid_name(cls):
                        return f"{path}: 'res' entries must be class names"
            if "desc" in pat and not isinstance(pat["desc"], str):
                return f"{path}: 'desc' must be a string"
            if "payload" in pat:
                r = _validate_param_map(pat["payload"], f"{path}.payload")
                if r:
                    return r

    return ""


def cap_match_event_pattern(advert, key, verb, kind):
    """Match a dynamic per-resource name against the advert's
    "event_patterns" + "resources" (TASK-PF-02). A name
    "<ns>.<key>.<verb>" matches when the advert has a "*.<verb>" pattern
    of the given kind ("fact" or "command") and - when "resources" is
    present - it lists `key` with a class allowed by the pattern's "res"
    (absent "res" = every class; absent "resources" = key unchecked).
    Pure: no validation; the caller validates the advert first.
    Returns the pattern dict, or None."""
    if not isinstance(advert, dict):
        return None
    patterns = advert.get("event_patterns")
    if not isinstance(patterns, dict):
        return None
    pat = patterns.get("*." + verb)
    if not isinstance(pat, dict):
        return None
    if pat.get("kind", "fact") != kind:
        return None

    # the resource check applies only when the advert lists resources:
    # without it the key cannot be class-checked and the pattern wins
    resources = advert.get("resources")
    if resources is not None:
        if not isinstance(resources, dict):
            return None
        cls = resources.get(key)
        if not isinstance(cls, str):
            return None
        res = pat.get("res")
        if isinstance(res, list) and cls not in res:
            return None

    return pat


def cap_find_action(advert, act):
    """action `act` of an advert, by name or by its "deprecated_alias"."""
    actions = advert.get("actions", {}) if isinstance(advert, dict) else {}
    if act in actions:
        return actions[act]
    for a in actions.values():
        if isinstance(a, dict) and a.get("deprecated_alias") == act:
            return a
    return None


def cap_canonical_action(advert, act):
    """current name of action `act` (itself or the action it is an alias of), or None."""
    actions = advert.get("actions", {}) if isinstance(advert, dict) else {}
    if act in actions:
        return act
    for name, a in actions.items():
        if isinstance(a, dict) and a.get("deprecated_alias") == act:
            return name
    return None


def cap_normalize_adverts(adverts):
    """Accept advert strings or dicts; returns (strings, parsed list).
    Dicts are serialized canonically (sorted keys, compact) and that
    serialization is what is hashed and sent."""
    strings = []
    parsed = []
    for a in adverts:
        s = a if isinstance(a, str) else json.dumps(a, separators=(",", ":"), sort_keys=True)
        strings.append(s)
        parsed.append(json.loads(s))
    return strings, parsed
