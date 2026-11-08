// esp32_fake_fc: a fake ArduCopter an ESP32 serves on three links at once:
//   - USB-serial (UART0 behind the board's CP2102/CH340) - the OTG link
//   - UART2 pins  (RX=GPIO16 TX=GPIO17, 115200 8N1) - wire to an RPi or a
//     USB-serial adapter
//   - Bluetooth Classic SPP ("DE-FakeFC") - the BT telemetry-radio link
// Lets a phone running Andruav prove its USB (OTG) and Bluetooth FCB links
// without a real flight controller: heartbeats + a parked position stream
// go out on every link; SET_MODE / DO_SET_MODE / ARM arriving on any link
// are accepted and acked on all of them, and the new mode/armed state shows
// up in the next HEARTBEAT.
//
// Build: needs droneengage_mavlink/c_library_v2 on the include path and a
// partition scheme that fits Bluedroid (see the fqbn file / README.md).
// Baud must match the app's USB baud selector.

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "ardupilotmega/mavlink.h"

#define FC_BAUD      115200
#define FC_SYSID     1
#define FC_COMPID    MAV_COMP_ID_AUTOPILOT1
#define LED_PIN      2
#define UART2_RX_PIN 16
#define UART2_TX_PIN 17
#define BT_NAME      "DE-FakeFC"

// parked at a fixed spot (lat/lon in degE7, alt in mm)
static const int32_t HOME_LAT = 300444000;
static const int32_t HOME_LON = 312357000;
static const int32_t HOME_ALT = 50000;

struct Param { const char *id; float value; };
static const Param PARAMS[] = {
  {"SYSID_THISMAV", FC_SYSID},
  {"FRAME_CLASS",   1},
  {"FRAME_TYPE",    1},
  {"FS_THR_ENABLE", 0},
};
static const uint16_t PARAM_COUNT = sizeof(PARAMS) / sizeof(PARAMS[0]);

static uint32_t custom_mode = 0;   // COPTER_MODE_STABILIZE
static bool     armed = false;
static float    yaw = 0;

static uint8_t  gcs_sysid = 255;
static uint8_t  gcs_compid = MAV_COMP_ID_MISSIONPLANNER;
static uint32_t rx_packets = 0;

static BluetoothSerial SerialBT;

static const char *mode_name(uint32_t m) {
  switch (m) {
    case COPTER_MODE_STABILIZE: return "STABILIZE";
    case COPTER_MODE_ACRO:      return "ACRO";
    case COPTER_MODE_ALT_HOLD:  return "ALT_HOLD";
    case COPTER_MODE_AUTO:      return "AUTO";
    case COPTER_MODE_GUIDED:    return "GUIDED";
    case COPTER_MODE_LOITER:    return "LOITER";
    case COPTER_MODE_RTL:       return "RTL";
    case COPTER_MODE_CIRCLE:    return "CIRCLE";
    case COPTER_MODE_LAND:      return "LAND";
    case COPTER_MODE_BRAKE:     return "BRAKE";
    case COPTER_MODE_SMART_RTL: return "SMART_RTL";
    default:                    return nullptr;
  }
}

static void send_msg(const mavlink_message_t &msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
  Serial2.write(buf, len);
  if (SerialBT.hasClient()) SerialBT.write(buf, len);
}

// drain one link's bytes into the shared packet handler; each link keeps
// its own mavlink_status_t so interleaved links don't confuse the parser
static void read_link(Stream &link, mavlink_channel_t channel,
                      mavlink_status_t *status) {
  mavlink_message_t msg;
  while (link.available())
    if (mavlink_parse_char(channel, (uint8_t)link.read(), &msg, status))
      handle_message(msg);
}

static void send_heartbeat() {
  mavlink_message_t msg;
  uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED;
  if (custom_mode == COPTER_MODE_GUIDED || custom_mode == COPTER_MODE_AUTO)
    base |= MAV_MODE_FLAG_GUIDED_ENABLED;
  if (armed) base |= MAV_MODE_FLAG_SAFETY_ARMED;
  mavlink_msg_heartbeat_pack(FC_SYSID, FC_COMPID, &msg, MAV_TYPE_QUADROTOR,
                             MAV_AUTOPILOT_ARDUPILOTMEGA, base, custom_mode,
                             armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
  send_msg(msg);
}

static void send_statustext(uint8_t severity, const char *text) {
  mavlink_message_t msg;
  char buf[50] = {0};
  strncpy(buf, text, sizeof(buf));
  mavlink_msg_statustext_pack(FC_SYSID, FC_COMPID, &msg, severity, buf, 0, 0);
  send_msg(msg);
}

static void send_ack(uint16_t command, uint8_t result) {
  mavlink_message_t msg;
  mavlink_msg_command_ack_pack(FC_SYSID, FC_COMPID, &msg, command, result,
                               0, 0, gcs_sysid, gcs_compid);
  send_msg(msg);
}

static void send_param(uint16_t index) {
  if (index >= PARAM_COUNT) return;
  mavlink_message_t msg;
  mavlink_msg_param_value_pack(FC_SYSID, FC_COMPID, &msg, PARAMS[index].id,
                               PARAMS[index].value, MAV_PARAM_TYPE_REAL32,
                               PARAM_COUNT, index);
  send_msg(msg);
}

static void send_telemetry_1hz() {
  mavlink_message_t msg;
  mavlink_msg_sys_status_pack(FC_SYSID, FC_COMPID, &msg, 0, 0, 0, 500,
                              12600, -1, 95, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  send_msg(msg);
  mavlink_msg_gps_raw_int_pack(FC_SYSID, FC_COMPID, &msg, micros(),
                               GPS_FIX_TYPE_3D_FIX, HOME_LAT, HOME_LON, HOME_ALT,
                               90, 120, 0, UINT16_MAX, 12, 0, 0, 0, 0, 0, 0);
  send_msg(msg);
}

static void send_telemetry_5hz() {
  mavlink_message_t msg;
  yaw += 0.01f;
  if (yaw > PI) yaw -= 2 * PI;
  mavlink_msg_attitude_pack(FC_SYSID, FC_COMPID, &msg, millis(), 0, 0, yaw, 0, 0, 0);
  send_msg(msg);
  int16_t hdg = (int16_t)((yaw < 0 ? yaw + 2 * PI : yaw) * 18000.0f / PI);
  mavlink_msg_global_position_int_pack(FC_SYSID, FC_COMPID, &msg, millis(),
                                       HOME_LAT, HOME_LON, HOME_ALT, 0,
                                       0, 0, 0, hdg);
  send_msg(msg);
}

static void set_mode(uint32_t mode, uint16_t ack_command) {
  const char *name = mode_name(mode);
  if (name == nullptr) {
    send_ack(ack_command, MAV_RESULT_DENIED);
    send_statustext(MAV_SEVERITY_WARNING, "Fake FC: mode change denied");
    return;
  }
  custom_mode = mode;
  send_ack(ack_command, MAV_RESULT_ACCEPTED);
  send_heartbeat();
  char text[50];
  snprintf(text, sizeof(text), "Fake FC: mode %s", name);
  send_statustext(MAV_SEVERITY_INFO, text);
}

static void handle_command_long(const mavlink_command_long_t &cmd) {
  switch (cmd.command) {
    case MAV_CMD_DO_SET_MODE:
      set_mode((uint32_t)cmd.param2, cmd.command);
      break;
    case MAV_CMD_COMPONENT_ARM_DISARM:
      armed = cmd.param1 > 0.5f;
      send_ack(cmd.command, MAV_RESULT_ACCEPTED);
      send_heartbeat();
      send_statustext(MAV_SEVERITY_INFO, armed ? "Fake FC: armed" : "Fake FC: disarmed");
      break;
    case MAV_CMD_REQUEST_MESSAGE:
    case MAV_CMD_SET_MESSAGE_INTERVAL:
    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
      send_ack(cmd.command, MAV_RESULT_ACCEPTED);
      break;
    default:
      send_ack(cmd.command, MAV_RESULT_UNSUPPORTED);
      break;
  }
}

static void handle_message(const mavlink_message_t &msg) {
  rx_packets++;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    gcs_sysid = msg.sysid;
    gcs_compid = msg.compid;
    return;
  }

  switch (msg.msgid) {
    case MAVLINK_MSG_ID_SET_MODE: {
      mavlink_set_mode_t m;
      mavlink_msg_set_mode_decode(&msg, &m);
      if (m.target_system != FC_SYSID && m.target_system != 0) return;
      // ArduPilot acks SET_MODE with the message id as the command
      set_mode(m.custom_mode, MAVLINK_MSG_ID_SET_MODE);
      break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG: {
      mavlink_command_long_t cmd;
      mavlink_msg_command_long_decode(&msg, &cmd);
      if (cmd.target_system != FC_SYSID && cmd.target_system != 0) return;
      handle_command_long(cmd);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
      for (uint16_t i = 0; i < PARAM_COUNT; i++) send_param(i);
      break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
      mavlink_param_request_read_t r;
      mavlink_msg_param_request_read_decode(&msg, &r);
      if (r.param_index >= 0) {
        send_param(r.param_index);
      } else {
        for (uint16_t i = 0; i < PARAM_COUNT; i++)
          if (strncmp(PARAMS[i].id, r.param_id, 16) == 0) send_param(i);
      }
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      mavlink_mission_request_list_t r;
      mavlink_msg_mission_request_list_decode(&msg, &r);
      mavlink_message_t out;
      mavlink_msg_mission_count_pack(FC_SYSID, FC_COMPID, &out, msg.sysid,
                                     msg.compid, 0, r.mission_type);
      send_msg(out);
      break;
    }
    default:
      break;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(FC_BAUD);
  Serial2.begin(FC_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  SerialBT.enableSSP(false, false);  // Just Works pairing - no PIN prompt
  // auto-accept the SSP confirm step - headless, nothing to display; the
  // lib refuses pairing otherwise (Android shows a confirm dialog and
  // fails)
  SerialBT.onConfirmRequest([](uint32_t) { SerialBT.confirmReply(true); });
  SerialBT.begin(BT_NAME, false, true);   // SPP slave, BLE left off
  // the Arduino lib hard-codes a "computer" class of device - report as
  // uncategorized, like HC-05-style serial radios
  esp_bt_cod_t cod;
  cod.major = 0b11111;
  cod.minor = 0;
  cod.service = 0;
  esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD);
  // Bluedroid keeps link keys in NVS, so bonds survive a reset. A full
  // merged-image flash (sim esp32 upload) erases NVS: forget DE-FakeFC on
  // the phone/PC after a reflash, or SSP auth fails on every connect.
  delay(200);
  send_statustext(MAV_SEVERITY_INFO, "Fake FC: ESP32 ArduCopter ready");
}

void loop() {
  static uint32_t last_1hz = 0, last_5hz = 0;
  static mavlink_status_t rx_status[3];

  read_link(Serial,   MAVLINK_COMM_0, &rx_status[0]);   // USB-serial
  read_link(Serial2,  MAVLINK_COMM_1, &rx_status[1]);   // GPIO16/17
  read_link(SerialBT, MAVLINK_COMM_2, &rx_status[2]);   // Bluetooth SPP

  uint32_t now = millis();
  if (now - last_1hz >= 1000) {
    last_1hz = now;
    send_heartbeat();
    send_telemetry_1hz();
  }
  if (now - last_5hz >= 200) {
    last_5hz = now;
    send_telemetry_5hz();
  }
}
