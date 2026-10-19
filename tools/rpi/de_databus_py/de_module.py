

import json
import time
import threading
from collections import OrderedDict
from enum import Enum

try:
    from .colors import *
    from .messages import *
    from .udpClient import *
    from . import cap_schema
except ImportError:
    from colors import *
    from messages import *
    from udpClient import *
    import cap_schema


import json


MODULE_FEATURE_RECEIVING_TELEMETRY      = "R"
MODULE_FEATURE_SENDING_TELEMETRY        = "T"
MODULE_FEATURE_CAPTURE_IMAGE            = "C"
MODULE_FEATURE_CAPTURE_VIDEO            = "V"
MODULE_FEATURE_GPIO                     = "G"
MODULE_FEATURE_AI_RECOGNITION           = "A"
MODULE_FEATURE_TRACKING                 = "K"
MODULE_FEATURE_P2P                      = "P"


MODULE_CLASS_COMM                       = "comm"
MODULE_CLASS_FCB                        = "fcb"
MODULE_CLASS_VIDEO                      = "camera"
MODULE_CLASS_P2P                        = "p2p"
MODULE_CLASS_GENERIC                    = "gen"
MODULE_CLASS_GPIO                       = "gpio"
MODULE_CLASS_A_RECOGNITION              = "ai_rec"
MODULE_CLASS_TRACKING                   = "trk"
MODULE_CLASS_VIEWLINK                   = "vlk"

HARDWARE_TYPE_UNDEFINED = 0
HARDWARE_TYPE_CPU = 1

DEFAULT_UDP_DATABUS_PACKET_SIZE = 8192
SAFE_REMOTE_UDP_PAYLOAD = 1472

class CModule(object):

    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(CModule, cls).__new__(cls)
        return cls._instance
    
    def __init__(self):
        self.m_module_class = ""
        self.m_module_id = ""
        self.m_module_key = ""
        self.m_module_version = ""
        self.m_message_filter = {}
        self.cUDPClient = None
        self.m_party_id = ""
        self.m_group_id = ""
        self.m_OnReceive = None
        self.m_stdinValues = {}
        self.m_FirstReceived = False
        self.m_module_features = []  # Initialize the list of module features
        self.m_hardware_serial = ""
        self.m_hardware_serial_type = 0
        self.m_instance_time_stamp = time.time()
        self.m_lock = threading.RLock()
        # Phase-3 capability helper state
        self.m_capability_adverts = []      # raw strings, as sent
        self.m_capability_parsed = []       # parsed, same order
        self.m_capability_hash = ""
        self.m_on_invoke = None
        self.m_capabilities_sent = False
        self.m_invoke_results = OrderedDict()  # id -> result, last 64 kept
        self.m_invoke_inflight = set()
        self.m_cap_lock = threading.Lock()
        self.m_event_counter = 0

    def init(self, target_ip, broadcasts_port, host, listening_port, chunk_size,):
        # chunk_size <= 0: auto-detect like the C++ CModule - loopback uses
        # the full default, other targets are capped at
        # SAFE_REMOTE_UDP_PAYLOAD to avoid IP fragmentation.
        if chunk_size <= 0:
            if target_ip in ("127.0.0.1", "::1", "localhost"):
                chunk_size = DEFAULT_UDP_DATABUS_PACKET_SIZE
            else:
                chunk_size = SAFE_REMOTE_UDP_PAYLOAD
            print(INFO_CONSOLE_TEXT + "Auto chunk size: " + str(chunk_size)
                  + " (target " + target_ip + ")" + NORMAL_CONSOLE_TEXT)

        # UDP Server
        self.cUDPClient = CUDPClient()
        self.cUDPClient.init(target_ip, broadcasts_port, host, listening_port, chunk_size, self.onReceive)
        self.createJSONID(True)
        self.cUDPClient.start()
        return True

    def uninit(self):
        self.cUDPClient.stop()
        return True

    def defineModule(self, module_class, module_id, module_key, module_version, message_filter):
        self.m_module_class = module_class
        self.m_module_id = module_id
        self.m_module_key = module_key
        self.m_module_version = module_version
        self.m_message_filter = message_filter

    def add_module_features(self, feature):
        self.m_module_features.append(feature)
    
    def set_hardware(self, hardware_serial, hardware_serial_type):
        self.m_hardware_serial = hardware_serial
        self.m_hardware_serial_type = hardware_serial_type
    
    def sendMSG(self, msg, length):
        self.cUDPClient.sendMSG(msg, length)
    
    def send_sys_msg(self, jmsg, andruav_message_id):
        full_message = {
            ANDRUAV_PROTOCOL_TARGET_ID: ANDRUAV_PROTOCOL_SENDER_COMM_SERVER,
            INTERMODULE_ROUTING_TYPE: CMD_COMM_SYSTEM,
            ANDRUAV_PROTOCOL_MESSAGE_TYPE: andruav_message_id,
            ANDRUAV_PROTOCOL_MESSAGE_CMD: jmsg
        }
        msg = json.dumps(full_message)
        self.sendMSG(msg.encode(), len(msg))

    def sendJMSG(self, targetPartyID, jmsg, andruav_message_id, internal_message):
        with self.m_lock:
            fullMessage = {}

            msg_routing_type = CMD_COMM_GROUP
            if internal_message:
                msg_routing_type = CMD_TYPE_INTERMODULE
            elif targetPartyID:
                msg_routing_type = CMD_COMM_INDIVIDUAL

            fullMessage[INTERMODULE_MODULE_KEY] = self.m_module_key
            fullMessage[ANDRUAV_PROTOCOL_TARGET_ID] = targetPartyID
            fullMessage[INTERMODULE_ROUTING_TYPE] = msg_routing_type
            fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE] = andruav_message_id
            fullMessage[ANDRUAV_PROTOCOL_MESSAGE_CMD] = jmsg

            msg = json.dumps(fullMessage)
            print(f"sendJMSG: {msg}")
            self.sendMSG(msg.encode(), len(msg))

    def sendBMSG(self, targetPartyID, bmsg, bmsg_length, andruav_message_id, internal_message, message_cmd):
        with self.m_lock:
            fullMessage = {}

            msg_routing_type = CMD_COMM_GROUP
            if internal_message:
                msg_routing_type = CMD_TYPE_INTERMODULE
            elif targetPartyID:
                msg_routing_type = CMD_COMM_INDIVIDUAL

            fullMessage[INTERMODULE_MODULE_KEY] = self.m_module_key
            fullMessage[ANDRUAV_PROTOCOL_TARGET_ID] = targetPartyID
            fullMessage[INTERMODULE_ROUTING_TYPE] = msg_routing_type
            fullMessage[ANDRUAV_PROTOCOL_MESSAGE_TYPE] = andruav_message_id
            fullMessage[ANDRUAV_PROTOCOL_MESSAGE_CMD] = message_cmd

            json_msg = json.dumps(fullMessage)

            msg = json_msg.encode('utf-8')
            msg += b'\0'
            if bmsg_length:
                msg += bmsg

            self.sendMSG(msg, len(msg))

    def sendMREMSG(self, command_type):
        with self.m_lock:
            json_msg = {
                INTERMODULE_MODULE_KEY: self.m_module_key,
                INTERMODULE_ROUTING_TYPE: CMD_TYPE_INTERMODULE,
                ANDRUAV_PROTOCOL_MESSAGE_TYPE: TYPE_AndruavModule_RemoteExecute,
                ANDRUAV_PROTOCOL_MESSAGE_CMD: {"C": command_type}
            }

            msg = json.dumps(json_msg)
            self.sendMSG(msg.encode(), len(msg))

    def forwardMSG(self, message, datalength):
        self.sendMSG(message, datalength)

    def appendExtraField(self, name, ms):
        """Extra fields merged into the module ID message by createJSONID.
        Set before init()/createJSONID, or call createJSONID again."""
        self.m_stdinValues[name] = ms

    def onReceive(self, message, len):
        """
            This function is called from udpClient when a complete message has been received.
        """
        # print(f"RX MSG: :len {len}:{message}")

        try:
            # The JSON header is always null-terminated; any binary payload (not
            # currently used by this module) would follow the null terminator.
            null_index = message.find(b'\x00')
            json_part = message[:null_index] if null_index != -1 else message
            jMsg = json.loads(json_part)

            # print(f"RX MSG: jMsg{json.dumps(jMsg)}")

            if ANDRUAV_PROTOCOL_MESSAGE_TYPE not in jMsg:
                return
            if INTERMODULE_ROUTING_TYPE not in jMsg:
                return

            if jMsg[INTERMODULE_ROUTING_TYPE] == CMD_TYPE_INTERMODULE:
                """
                    CMD_TYPE_INTERMODULE messages section.
                    These messages are sent by other modules to be consumed only 
                    by modules and not to be sent outside DroneEngage unit.
                """                    
                if ANDRUAV_PROTOCOL_MESSAGE_CMD not in jMsg:
                    return

                cmd = jMsg[ANDRUAV_PROTOCOL_MESSAGE_CMD]

                message_type = jMsg[ANDRUAV_PROTOCOL_MESSAGE_TYPE]
                if message_type == TYPE_AndruavModule_ID:
                    if JSON_INTERMODULE_PARTY_RECORD not in cmd:
                        return
                    moduleID = cmd[JSON_INTERMODULE_PARTY_RECORD]

                    if ANDRUAV_PROTOCOL_SENDER not in moduleID:
                        return
                    if ANDRUAV_PROTOCOL_GROUP_ID not in moduleID:
                        return

                    self.m_party_id = moduleID[ANDRUAV_PROTOCOL_SENDER]
                    self.m_group_id = moduleID[ANDRUAV_PROTOCOL_GROUP_ID]

                    if not self.m_FirstReceived:
                        print(SUCCESS_CONSOLE_BOLD_TEXT + " ** Communicator Server Found: " + SUCCESS_CONSOLE_TEXT +  "m_party_id(" + INFO_CONSOLE_TEXT + str(self.m_party_id) + SUCCESS_CONSOLE_TEXT + ") m_group_id(" + INFO_CONSOLE_TEXT + str(self.m_group_id) + SUCCESS_CONSOLE_TEXT + ")" + NORMAL_CONSOLE_TEXT)
                        self.createJSONID(False)
                        self.m_FirstReceived = True
                        # Phase-3: publish the advert once on registration
                        if not self.m_capabilities_sent and self.m_capability_adverts:
                            self.m_capabilities_sent = True
                            self._send_capabilities()

                    if self.m_OnReceive:
                        self.m_OnReceive(message, len, jMsg)

                    return

                elif message_type == TYPE_AndruavMessage_MODULE_CAPABILITIES:
                    # de_comm asking for the advert (P3-01). Helper-owned:
                    # not forwarded to the module's m_OnReceive.
                    if cmd.get("r"):
                        self._send_capabilities()
                    return

                elif message_type == TYPE_AndruavMessage_CAPABILITY_INVOKE:
                    self._handle_invoke(cmd)
                    return

                elif message_type == TYPE_AndruavMessage_DUMMY:
                    print(f" TYPE_AndruavMessage_DUMMY {message}")

            if self.m_OnReceive:
                self.m_OnReceive(message, len, jMsg)

        except Exception as e:
            print(f"ERROR:{e}")

    
    def createJSONID(self, reSend):
        """
        Create TYPE_AndruavModule_ID - JSON message 
        This message is essential to identify module to de-Communicator.
        """
        
        json_msg = {}
        
        json_msg[INTERMODULE_ROUTING_TYPE] = CMD_TYPE_INTERMODULE
        json_msg[ANDRUAV_PROTOCOL_MESSAGE_TYPE] = TYPE_AndruavModule_ID
        ms = {}
        
        ms[JSON_INTERMODULE_MODULE_ID] = self.m_module_id
        ms[JSON_INTERMODULE_MODULE_CLASS] = self.m_module_class
        ms[JSON_INTERMODULE_MODULE_MESSAGES_LIST] = self.m_message_filter
        ms[JSON_INTERMODULE_MODULE_FEATURES] = self.m_module_features
        ms[JSON_INTERMODULE_MODULE_KEY] = self.m_module_key
        # Only include hardware fields when a hardware serial has been set
        # (via set_hardware). The C++ communicator enters its license-check
        # branch when JSON_INTERMODULE_HARDWARE_ID is present, and expects
        # JSON_INTERMODULE_HARDWARE_TYPE to be an int. Sending empty strings
        # here causes json.exception.type_error.302 on the C++ side.
        if self.m_hardware_serial:
            ms[JSON_INTERMODULE_HARDWARE_ID] = self.m_hardware_serial
            ms[JSON_INTERMODULE_HARDWARE_TYPE] = self.m_hardware_serial_type
        ms[JSON_INTERMODULE_VERSION] = self.m_module_version
        ms[JSON_INTERMODULE_RESEND] = reSend
        ms[JSON_INTERMODULE_TIMESTAMP_INSTANCE] = self.m_instance_time_stamp
        
        for key, value in self.m_stdinValues.items():
            ms[key] = value
        
        json_msg[ANDRUAV_PROTOCOL_MESSAGE_CMD] = ms
        
        #print(json.dumps(json_msg, indent=4))
        
        ## Store the message into cUDPClient
        self.cUDPClient.setJsonId(json.dumps(json_msg))


    # ------------------------------------------------------------------
    # Phase-3 capability advert helpers (de.cap/1)
    # ------------------------------------------------------------------

    def _cap_add_to_filter(self, msg_id):
        if not isinstance(self.m_message_filter, list):
            self.m_message_filter = []
        if msg_id not in self.m_message_filter:
            self.m_message_filter.append(msg_id)

    def set_capabilities(self, adverts):
        """Register this module's de.cap/1 advert(s) - one per namespace.
        Validates each advert, derives the "ch" hash carried in the module
        ID message, and subscribes MODULE_CAPABILITIES / CAPABILITY_INVOKE.
        Returns False (+ log) when any advert fails validation."""
        strings, parsed = cap_schema.cap_normalize_adverts(adverts)
        for s, a in zip(strings, parsed):
            if len(s.encode("utf-8")) > cap_schema.CAP_ADVERT_MAX_BYTES:
                print(f"set_capabilities: advert exceeds {cap_schema.CAP_ADVERT_MAX_BYTES} bytes")
                return False
            err = cap_schema.cap_validate_advert(a)
            if err:
                print(f"set_capabilities: invalid advert: {err}")
                return False

        with self.m_cap_lock:
            self.m_capability_adverts = strings
            self.m_capability_parsed = parsed
            self.m_capability_hash = cap_schema.cap_advert_hash(strings)

        # the ID message only carries the hash; the full adverts go on 6542
        self.appendExtraField("ch", self.m_capability_hash)

        self._cap_add_to_filter(TYPE_AndruavMessage_MODULE_CAPABILITIES)
        self._cap_add_to_filter(TYPE_AndruavMessage_CAPABILITY_INVOKE)

        if self.cUDPClient is not None:
            self.createJSONID(False)

        # registered already (party known)? send the advert now - the normal
        # path sends it once when the first ID reply arrives
        if self.m_party_id and not self.m_capabilities_sent:
            self.m_capabilities_sent = True
            self._send_capabilities()

        return True

    def on_invoke(self, handler):
        """Register the action handler: handler(ns, act, params) ->
        (data, err). Params arrive validated (defaults filled). Results are
        idempotent: a repeated invoke id replays the cached result."""
        with self.m_cap_lock:
            self.m_on_invoke = handler

    def publish_state(self, ns, changed, full=False):
        """Publish changed state keys (MODULE_STATE 6545, intermodule)."""
        msg = {"ns": ns, "s": changed}
        if full:
            msg["full"] = True
        self.sendJMSG("", msg, TYPE_AndruavMessage_MODULE_STATE, True)

    def fire_event(self, ns, ev, payload):
        """Fire "<ns>.<ev>" as Sync_EventFire d, ed=payload, fresh nonce n,
        no scope - intermodule only. Returns the nonce sent (modules that
        consume their own namespace's events record it in a self-echo
        ring - playbook 4.3)."""
        self.m_event_counter += 1
        nonce = f"{int(time.time() * 1000)}-{self.m_event_counter}"
        msg = {"d": f"{ns}.{ev}", "ed": payload, "n": nonce}
        self.sendJMSG("", msg, TYPE_AndruavMessage_Sync_EventFire, True)
        return nonce

    def _send_capabilities(self):
        msg = {"caps": list(self.m_capability_adverts), "ch": self.m_capability_hash}
        self.sendJMSG("", msg, TYPE_AndruavMessage_MODULE_CAPABILITIES, True)

    def _send_invoke_result(self, result):
        self.sendJMSG("", result, TYPE_AndruavMessage_CAPABILITY_RESULT, True)

    def _cache_invoke_result(self, id_, result):
        with self.m_cap_lock:
            self.m_invoke_results[id_] = result
            self.m_invoke_results.move_to_end(id_)
            while len(self.m_invoke_results) > 64:
                self.m_invoke_results.popitem(last=False)

    def _handle_invoke(self, cmd):
        """CAPABILITY_INVOKE {id, ns, act, p, dl}. Idempotent by "id"."""
        id_ = cmd.get("id", "")
        ns = cmd.get("ns", "")
        act = cmd.get("act", "")
        with_ = cmd.get("p") if isinstance(cmd.get("p"), dict) else {}
        dl = cmd.get("dl", 10)
        if not isinstance(dl, (int, float)) or dl <= 0:
            dl = 10

        if not id_:
            return

        with self.m_cap_lock:
            if id_ in self.m_invoke_results:
                # idempotent replay - never run the handler twice
                self._send_invoke_result(self.m_invoke_results[id_])
                return
            if id_ in self.m_invoke_inflight:
                return
            self.m_invoke_inflight.add(id_)

        def fail(err):
            with self.m_cap_lock:
                self.m_invoke_inflight.discard(id_)
            result = {"id": id_, "ok": False, "err": err}
            self._cache_invoke_result(id_, result)
            self._send_invoke_result(result)

        # resolve "<ns>.<act>" in the adverts
        action = None
        with self.m_cap_lock:
            for advert in self.m_capability_parsed:
                if advert.get("ns") == ns:
                    action = cap_schema.cap_find_action(advert, act)
                    if action is not None:
                        act = cap_schema.cap_canonical_action(advert, act)  # alias -> current name
                    break
            handler = self.m_on_invoke

        if action is None:
            fail(f"unknown action {ns}.{act}")
            return

        filled = {}
        errors = cap_schema.cap_check_params(action.get("params", {}), with_, filled)
        if errors:
            fail("invalid params: " + "; ".join(errors))
            return

        if handler is None:
            fail("no invoke handler")
            return

        def run_handler(holder):
            try:
                data, err = handler(ns, act, filled)
                res = {"ok": not err}
                if err:
                    res["err"] = err
                else:
                    res["data"] = data
            except Exception as e:
                res = {"ok": False, "err": f"handler exception: {e}"}
            holder["res"] = res

        def manager():
            holder = {}
            worker = threading.Thread(target=run_handler, args=(holder,), daemon=True)
            worker.start()
            worker.join(dl)
            if worker.is_alive():
                # deadline passed - the handler keeps running; its late
                # result is dropped and logged
                with self.m_cap_lock:
                    self.m_invoke_inflight.discard(id_)
                result = {"id": id_, "ok": False, "err": "timeout"}
                self._cache_invoke_result(id_, result)
                self._send_invoke_result(result)
                worker.join()
                print(f"capability invoke {id_} ({ns}.{act}) finished after its deadline - result dropped")
            else:
                result = holder.get("res", {"ok": False, "err": "handler failed"})
                result["id"] = id_
                with self.m_cap_lock:
                    self.m_invoke_inflight.discard(id_)
                self._cache_invoke_result(id_, result)
                self._send_invoke_result(result)

        threading.Thread(target=manager, daemon=True).start()
