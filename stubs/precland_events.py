#!/usr/bin/env python3
"""precland_events.py - the Python port of de_precland's pure event
half (PL-08): precland_events.hpp's CPreclandEventModel (locked/lost
pair hysteresis, stabilized timer, degraded rate limit, faulted edge,
enabled/target facts), the PL-03 command helpers (plParseCommand,
plEventPayload, plValidateSetTarget) and the nonce dedupe ring - same
names, same fact ordering, same defaults. The golden contract
src/precland/test/golden/precland_events.jsonl replays through this
file unchanged (stubs/test_precland.py), so stub and module cannot
drift.
"""

import json

# -------------------------------------------------------------------
# wire codes (de_databus/messages.hpp) - mirrored verbatim

PRECLAND_STATUS_DISABLED = 0
PRECLAND_STATUS_SEARCHING = 1
PRECLAND_STATUS_LOCKED = 2
PRECLAND_STATUS_DEGRADED = 3   # detecting, but gated (RMSE/stale)
PRECLAND_STATUS_ERROR = 4      # no camera / no camera_registry.json / bad layout

PRECLAND_REASON_NOMINAL = 0
PRECLAND_REASON_NO_TARGET = 1
PRECLAND_REASON_STALE = 2
PRECLAND_REASON_MIN_TAGS = 3
PRECLAND_REASON_RMSE = 4
PRECLAND_REASON_ERROR = 5

PRECLAND_ACTION_DISABLE = 0
PRECLAND_ACTION_ENABLE = 1
PRECLAND_ACTION_SET_TARGET = 2      # uses field b (target_num)
PRECLAND_ACTION_SELFTEST = 3
PRECLAND_ACTION_CALIBRATE = 4
PRECLAND_ACTION_CALIBRATE_CANCEL = 5

_STATUS_NAMES = ["disabled", "searching", "locked", "degraded", "error"]
_REASON_NAMES = ["nominal", "no_target", "stale", "min_tags", "rmse",
                 "error"]
# PL_CAUSE names minus "loop": enable/disable/setTargetNum writers are
# only ever called with legacy/invoke/event/config causes
_CAUSE_NAMES = {0: "loop", 1: "legacy", 2: "invoke", 3: "event",
                4: "config"}
CAUSE_LOOP = 0
CAUSE_LEGACY = 1
CAUSE_INVOKE = 2
CAUSE_EVENT = 3
CAUSE_CONFIG = 4


def precland_status_name(state):
    if 0 <= state < len(_STATUS_NAMES):
        return _STATUS_NAMES[state]
    return "unknown"


def precland_reason_name(reason):
    if 0 <= reason < len(_REASON_NAMES):
        return _REASON_NAMES[reason]
    return "unknown"


def pl_cause_name(cause):
    return _CAUSE_NAMES.get(cause, "unknown")


def cause_from_name(name):
    """The golden replay's string cause -> the enum the model stores."""
    for code, nm in _CAUSE_NAMES.items():
        if nm == name:
            return code
    return CAUSE_LOOP


# -------------------------------------------------------------------
# the top-level "events" config object (PL_EVENTS_CONFIG)

class EventsConfig:
    """All keys optional; lockpair_us is the frozen Phase-3 locked/lost
    pair hysteresis and is not configurable."""

    def __init__(self, enabled=True, commands=True, stabilized_s=2.0,
                 min_interval_us=2000000, lockpair_us=2000000):
        self.enabled = enabled
        self.commands = commands
        self.stabilized_s = stabilized_s
        self.min_interval_us = min_interval_us
        self.lockpair_us = lockpair_us


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _is_integer(v):
    """nlohmann is_number_integer(): true for signed/unsigned ints,
    never for bools or floats."""
    return isinstance(v, int) and not isinstance(v, bool)


def events_config_from_json(config):
    """Read the top-level "events" object; every key optional and falls
    back to the EventsConfig default. min_interval_ms is stored in
    microseconds."""
    cfg = EventsConfig()
    events = (config or {}).get("events")
    if isinstance(events, dict):
        if isinstance(events.get("enabled"), bool):
            cfg.enabled = events["enabled"]
        if isinstance(events.get("commands"), bool):
            cfg.commands = events["commands"]
        if _is_number(events.get("stabilized_s")):
            cfg.stabilized_s = float(events["stabilized_s"])
        if _is_number(events.get("min_interval_ms")):
            cfg.min_interval_us = int(events["min_interval_ms"] * 1000.0)
    return cfg


class PreclandEventModel:
    """Edge-only fact rules for the precision_landing namespace. Fed
    from the setState choke point (on_transition), the enabled/target
    writers and the per-frame on_tick; returns the facts to emit as
    (verb, payload) pairs. "enabled == False" makes every entry point
    return an empty list; the lock bookkeeping still runs so a re-enable
    mid-lock cannot emit an orphaned lost."""

    def __init__(self, config=None):
        self._config = config or EventsConfig()
        self._last_lockpair_us = 0       # last EMITTED locked stamp
        self._lock_event_suppressed = False  # this lock's locked was
        # suppressed -> suppress its lost
        self._lock_held_since_us = 0     # lock start for held_s /
        # the stabilized timer
        self._stabilized_fired = False
        self._last_degraded_us = 0

    def set_config(self, config):
        self._config = config

    def on_transition(self, prev_state, new_state, reason, now_us,
                      range_m=0.0, rmse_px=0.0, tags=0, target=0,
                      fault=""):
        """One state transition. locked fires on entering LOCKED only
        outside the pair hysteresis - otherwise the lock is marked
        suppressed and its lost is suppressed too (frozen Phase-3
        behaviour). lost fires on leaving LOCKED only when the matching
        locked was emitted. degraded is rate limited to one per
        min_interval_us. faulted fires on every edge into ERROR, reason
        = the fault tag the caller passed."""
        facts = []

        leaving_lock = (prev_state == PRECLAND_STATUS_LOCKED
                        and new_state != PRECLAND_STATUS_LOCKED)
        entering_lock = (new_state == PRECLAND_STATUS_LOCKED
                         and prev_state != PRECLAND_STATUS_LOCKED)

        if leaving_lock:
            held_s = ((now_us - self._lock_held_since_us) / 1000000.0
                      if self._lock_held_since_us > 0 else 0.0)
            if self._config.enabled and not self._lock_event_suppressed:
                facts.append(("lost",
                              {"reason": precland_reason_name(reason),
                               "held_s": held_s}))
            self._lock_event_suppressed = False
            self._lock_held_since_us = 0
            self._stabilized_fired = False

        if entering_lock:
            self._lock_held_since_us = now_us
            self._stabilized_fired = False
            if self._config.enabled and \
                    now_us - self._last_lockpair_us >= \
                    self._config.lockpair_us:
                self._last_lockpair_us = now_us
                self._lock_event_suppressed = False
                facts.append(("locked",
                              {"range_m": range_m, "target": target,
                               "tags": tags, "rmse_px": rmse_px}))
            else:
                # also covers the disabled case: a lock whose locked
                # never went out must not emit its lost either
                self._lock_event_suppressed = True

        if self._config.enabled \
                and new_state == PRECLAND_STATUS_DEGRADED \
                and prev_state != PRECLAND_STATUS_DEGRADED:
            if now_us - self._last_degraded_us >= \
                    self._config.min_interval_us:
                self._last_degraded_us = now_us
                facts.append(("degraded",
                              {"reason": precland_reason_name(reason)}))

        if self._config.enabled \
                and new_state == PRECLAND_STATUS_ERROR \
                and prev_state != PRECLAND_STATUS_ERROR:
            facts.append(("faulted",
                          {"reason": fault if fault else "error"}))

        return facts

    def on_enabled_changed(self, was, now, cause, _now_us=0):
        """m_enabled edge: a real change only, src = the cause name."""
        facts = []
        if was == now or not self._config.enabled:
            return facts
        facts.append(("enabled" if now else "disabled",
                      {"src": pl_cause_name(cause)}))
        return facts

    def on_target_changed(self, was, now, cause):
        """m_target_num edge: a real change only."""
        facts = []
        if was == now or not self._config.enabled:
            return facts
        facts.append(("target_changed",
                      {"target": now, "src": pl_cause_name(cause)}))
        return facts

    def on_tick(self, state, now_us, range_m, rmse_px, target):
        """Per-processed-frame tick: fires stabilized once per lock when
        the current lock has lasted stabilized_s. Not tied to the pair
        hysteresis - a suppressed locked still reports a stable lock."""
        facts = []

        if state != PRECLAND_STATUS_LOCKED:
            return facts
        if self._lock_held_since_us == 0:
            # a lock the model never saw begin counts from the first tick
            self._lock_held_since_us = now_us
            self._stabilized_fired = False
        if self._config.enabled and not self._stabilized_fired \
                and now_us - self._lock_held_since_us >= \
                int(self._config.stabilized_s * 1000000.0):
            self._stabilized_fired = True
            held_s = (now_us - self._lock_held_since_us) / 1000000.0
            facts.append(("stabilized",
                          {"range_m": range_m, "target": target,
                           "held_s": held_s, "rmse_px": rmse_px}))
        return facts


# -------------------------------------------------------------------
# PL-03 command helpers

def pl_parse_command(d):
    """Parse a "precision_landing.<verb>" command name; the verb must be
    one of enable/disable/set_target. Facts (locked, lost, ...), other
    namespaces and extra segments are rejected. -> verb or None."""
    prefix = "precision_landing."
    if not isinstance(d, str) or not d.startswith(prefix):
        return None
    verb = d[len(prefix):]
    if verb in ("enable", "disable", "set_target"):
        return verb
    return None


def pl_event_payload(cmd):
    """Pick the command payload of a Sync_EventFire message: "ed"
    (event data) first, then "m" (the legacy payload slot), then an
    empty object. A non-object value is skipped, not returned."""
    if isinstance(cmd.get("ed"), dict):
        return cmd["ed"]
    if isinstance(cmd.get("m"), dict):
        return cmd["m"]
    return {}


def pl_validate_set_target(payload, allowed_targets):
    """Validate a precision_landing.set_target payload: "target" must be
    a non-negative integer and, when allowed_targets is a non-empty
    array (config target.targets), one of its values. -> (target_num,
    why): on accept why is "" and target_num carries the value; on
    reject target_num is -1 and why the log-once reason."""
    target = payload.get("target")
    if not _is_integer(target) or target < 0:
        return -1, "target must be a non-negative integer"

    if isinstance(allowed_targets, list) and allowed_targets:
        listed = any(_is_integer(entry) and entry == target
                     for entry in allowed_targets)
        if not listed:
            return -1, "target not in target.targets"

    return target, ""


class NonceRing:
    """Bounded nonce dedupe ring: seen() checks membership, add()
    inserts and evicts the oldest entry past the capacity (64)."""

    def __init__(self, capacity=64):
        self._capacity = capacity
        self._set = set()
        self._order = []

    def seen(self, nonce):
        return nonce in self._set

    def add(self, nonce):
        if not nonce or nonce in self._set:
            return
        self._set.add(nonce)
        self._order.append(nonce)
        while len(self._order) > self._capacity:
            self._set.remove(self._order.pop(0))


def facts_jsonable(facts):
    """[(verb, payload)] -> the golden row's [{verb, payload}] form."""
    return [{"verb": v, "payload": p} for v, p in facts]


def facts_dump(facts):
    return "[" + ", ".join(f"{v} {json.dumps(p)}" for v, p in facts) + "]"
