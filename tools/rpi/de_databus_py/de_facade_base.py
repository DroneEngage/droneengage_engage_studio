import json

try:
    from .messages import *
    from .colors import *
except ImportError:
    from messages import *
    from colors import *


class CFacade_Base:
    def __init__(self, m_module):
        self.m_module = m_module

    def requestID(self, target_party_id):
        message = {
            "C": TYPE_AndruavMessage_ID
        }

        self.m_module.sendJMSG(target_party_id, message, TYPE_AndruavMessage_RemoteExecute, True)

    def sendErrorMessage(self, target_party_id, error_number, info_type, notification_type, description):
        """
        EN: error number "not currently processed".
        IT: info type indicate what component is reporting the error.
        NT: severity and compliant with ardupilot.
        DS: description message.
        """
        message = {
            "EN": error_number,
            "IT": info_type,
            "NT": notification_type,
            "DS": description
        }

        self.m_module.sendJMSG(target_party_id, message, TYPE_AndruavMessage_Error, False)

        print(f"\n{SUCCESS_CONSOLE_BOLD_TEXT} -- sendErrorMessage {NORMAL_CONSOLE_TEXT}{description}")

    def API_sendConfigTemplate(self, target_party_id, module_key, json_file_content_json, reply):
        message = {
            "a": CONFIG_STATUS_FETCH_CONFIG_TEMPLATE,
            "b": json_file_content_json,
            "k": module_key,
            "R": reply
        }

        self.m_module.sendJMSG(target_party_id, message, TYPE_AndruavMessage_CONFIG_STATUS, False)

    # Phase-3 capability advert helpers - pass-throughs to CModule.

    def set_capabilities(self, adverts):
        """Register de.cap/1 advert(s); sets the ID "ch" hash and the
        MODULE_CAPABILITIES / CAPABILITY_INVOKE filter entries."""
        return self.m_module.set_capabilities(adverts)

    def on_invoke(self, handler):
        """handler(ns, act, params) -> (data, err). Idempotent by invoke id."""
        self.m_module.on_invoke(handler)

    def publish_state(self, ns, changed, full=False):
        """MODULE_STATE 6545: changed keys (or a full snapshot)."""
        self.m_module.publish_state(ns, changed, full)

    def fire_event(self, ns, ev, payload):
        """Sync_EventFire d="<ns>.<ev>", ed=payload, fresh nonce."""
        self.m_module.fire_event(ns, ev, payload)

class MyFacade(CFacade_Base):
    def __init__(self, m_module):
        super().__init__(m_module)

    # Add your custom methods here
    def my_custom_method(self, arg1, arg2):
        # Implement your custom method here
        pass


# Backward compatibility alias
FacadeBase = CFacade_Base
CMyFacade = MyFacade
