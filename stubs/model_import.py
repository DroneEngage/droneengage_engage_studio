#!/usr/bin/env python3
"""model_import.py - import the stub model code with no databus checkout.

The stubs are databus modules: stub_base imports the de_databus python
client (messages.py ids, CModule, CFacade_Base) at import time. The
--selftest model path never opens the databus socket - self.module
stays None - so only those imports need the checkout. This installs a
shim carrying the same message ids the real client exports, so the
pure tests (stubs/test_golden.py, engine/test_world_obstacles.py) can
replay the models in-process on any machine, databus or not.
"""

import os
import sys
import types

_STUB_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_STUB_DIR)

# the de_databus python client's constants (messages.py) - the shim
# carries the same values, so a selftest log matches the live path
_IDS = {
    "ANDRUAV_PROTOCOL_MESSAGE_TYPE": "mt",
    "ANDRUAV_PROTOCOL_MESSAGE_CMD": "ms",
    "TYPE_AndruavMessage_ID": 1004,
    "TYPE_AndruavModule_Location_Info": 9102,
    "TYPE_AndruavMessage_AI_Recognition_ACTION": 1076,
    "TYPE_AndruavMessage_AI_Recognition_STATUS": 1077,
    "TYPE_AndruavMessage_AI_Recognition_TargetLocation": 1078,
}

_STUBS = ("stub_base", "stub_ai_box", "stub_sdr", "stub_payload")


def import_stubs():
    """-> {name: module} for the stub model modules, with the databus
    shims installed first. Idempotent per process."""
    for name, attrs in (
            ("messages", _IDS),
            ("de_module", {"CModule": type("CModule", (), {})}),
            ("de_facade_base", {"CFacade_Base": type("CFacade_Base", (), {})})):
        if name not in sys.modules:
            mod = types.ModuleType(name)
            for key, val in attrs.items():
                setattr(mod, key, val)
            sys.modules[name] = mod
    if _STUB_DIR not in sys.path:
        sys.path.insert(0, _STUB_DIR)
    if _ROOT not in sys.path:
        sys.path.insert(0, _ROOT)
    return {name: __import__(name) for name in _STUBS}
