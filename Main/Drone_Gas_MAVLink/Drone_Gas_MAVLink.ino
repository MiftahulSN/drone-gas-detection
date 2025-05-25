#include "Arduino.h"
#include "mavlink.h"

/*
  MAVLink Documentation 
  https://mavlink.io/en/
*/

#define MAVLINK_VERSION (1)

// ESP32 Serial MAVLink Connection
#define TXD_PIN (17)
#define RXD_PIN (16)
#define BAUD_RATE 57600

// ENUM MAVLink Properties
#define PIXHAWK_SYS_ID  1
#define PIXHAWK_COMP_ID 1
#define ESP_SYS_ID      2
#define ESP_COMP_ID     158

// ENUM Sequence Mission Plan
#define FIRST_MISSION 1
#define LAST_MISSION 4
#define NAN_MISSION 0

// ENUM Mission Command
#define MAV_CMD_DO_PAUSE_CONTINUE 193

// ENUM Mav Command
#define STOP 0
#define START 1
#define PAUSE 0
#define CONTINUE 1

// ENUM Gas State
#define INLET 1
#define OUTLET 0

// MQ2 Gas Sensor Params Configuration
const uint8_t MQ2_pin  = 26; // pcb_2mq: 26
const uint8_t MQ2_rl   = 20;
const float MQ2_ro = 21.5;
const float MQ2_m  = -0.46612;
const float MQ2_b  = 1.28400;

// MQ5 Gas Sensor Params Configuration
const uint8_t MQ5_pin  = 27; // pcb_2mq: 25
const uint8_t MQ5_rl   = 20;
const float MQ5_ro = 76.93;
const float MQ5_m  = -0.39949;
const float MQ5_b  = 0.76076;

// Pinout Variables
const uint8_t led_pin = 2;
const uint8_t fan_pin = 4; // pcb_2mq: 35

// Timer Variables
unsigned long start_sampling = 0;
unsigned long sampling_time  = 7000;
unsigned long start_cleaning = 0;
unsigned long cleaning_time  = 3000;
unsigned long start_request = 0;
unsigned long request_timeout  = 500;
unsigned long start_cmd = 0;
unsigned long cmd_time = 250;
unsigned long last_stream = 0;
unsigned long stream_frequency = 10;
unsigned long last_message = 0;
unsigned long message_frequency = 1000;
unsigned long last_command = 0;
unsigned long command_frequency = 500;
unsigned long last_heartbeat = 0;
unsigned long heartbeat_frequency = 1000;

// Global Variables
uint8_t mission_sequence = 0;

struct GPS_coordinate {
  float lat;
  float lon;
};

GPS_coordinate gps;

struct PPM_gas {
  float MQ2;
  float MQ5;
};

PPM_gas ppm;

HardwareSerial SerialMAV(2);

bool mission_sampling(uint8_t current_mission);
bool request_mission_reached(uint8_t* mission_sequence);
uint8_t request_sequence_mission(uint8_t* mission_sequence);
float get_analog(uint8_t MQ_sensor);
float get_ppm(uint8_t MQ_sensor, uint8_t rl, float ro, float m, float b);
void loop_debug();
void stream_heartbeat();
void heartbeat_interval();
void stream_mav_cmd_land();
void gas_status(uint8_t fan_state);
void request_all_data(uint8_t state);
void sampling_interval(uint8_t sequence);
void stream_mav_mission_cmd(uint8_t state);
void stream_nval_msg(const char* name, float value);
void stream_text_msg(const char* text, uint8_t severity = MAV_SEVERITY_DEBUG);
void messages_interval(uint8_t sequence, uint8_t severity = MAV_SEVERITY_DEBUG);
void messages_interval(const char* text, uint8_t severity = MAV_SEVERITY_DEBUG);
void command_interval(uint8_t state, unsigned long frequency = command_frequency);
GPS_coordinate request_gps_coordinate();

void setup() {
  // Serial Monitor Setup
  Serial.begin(115200);

  // Serial MAVLink Setup
  SerialMAV.begin(BAUD_RATE, SERIAL_8N1, RXD_PIN, TXD_PIN);

  // Pinout Setup
  pinMode(MQ2_pin, INPUT);
  pinMode(MQ5_pin, INPUT);
  pinMode(fan_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);

  // Analog Setup
  analogReadResolution(12);
  adcAttachPin(MQ2_pin);
  adcAttachPin(MQ5_pin);
  analogSetPinAttenuation(MQ2_pin, ADC_11db);
  analogSetPinAttenuation(MQ5_pin, ADC_11db);

  request_all_data(START);
}

void loop() {
  //--- HEARTBEAT ---//
  heartbeat_interval();
  //--- HEARTBEAT ---//

  //--- DEBUG ---//
  // loop_debug();
  //--- DEBUG ---//

  //--- MAIN LOOP ---//
  if (request_mission_reached(&mission_sequence)) {
    if (mission_sampling(mission_sequence)) {
      start_sampling = millis();
      while (millis() - start_sampling <= sampling_time) {
        gas_status(INLET);
        command_interval(PAUSE);
        sampling_interval(mission_sequence);
        messages_interval("SAMPLING", MAV_SEVERITY_WARNING);
        heartbeat_interval();
      }
      start_cleaning = millis();
      while (millis() - start_cleaning <= cleaning_time) {
        gas_status(OUTLET);
        messages_interval("CLEANING", MAV_SEVERITY_WARNING);
        heartbeat_interval();
      }
    }
    start_cmd = millis();
    bool msg_debug = false;
    while (millis() - start_cmd <= cmd_time) {
      command_interval(CONTINUE, 50);
      heartbeat_interval();
      if (!msg_debug) {
        char buffer[20]; 
        sprintf(buffer, "sequence: %d", mission_sequence); 
        stream_text_msg(buffer);
        msg_debug = true;
      }
    }
  }
  //--- MAIN LOOP ---//
}

/*
  DEBUG Function
*/
void loop_debug() {
  const uint8_t target_wp = 3;
  static uint8_t seq = 0;
  static bool paused = false;
  seq = request_sequence_mission(&seq);
  messages_interval(seq);

  if (seq == target_wp && !paused) {
    paused = true;
    start_sampling = millis();
    while (millis() - start_sampling <= sampling_time) {
      gas_status(INLET);
      command_interval(PAUSE);
      sampling_interval(mission_sequence);
      messages_interval("SAMPLING", MAV_SEVERITY_WARNING);
      heartbeat_interval();
    }
    start_cleaning = millis();
    while (millis() - start_cleaning <= cleaning_time) {
      gas_status(OUTLET);
      messages_interval("CLEANING", MAV_SEVERITY_WARNING);
      heartbeat_interval();
    }
    start_cmd = millis();
    while (millis() - start_cmd <= cmd_time) {
      command_interval(CONTINUE, 50);
      heartbeat_interval();
    }
  }
}

/*
  Protector Sampling Sequence
*/
bool mission_sampling(uint8_t current_mission) {
  if (current_mission != NAN_MISSION && 
      current_mission != FIRST_MISSION && 
      current_mission != LAST_MISSION) return true;
  else return false;
}

/*
  Fan System for Gas Behaviour
*/
void gas_status(uint8_t fan_state) {
  digitalWrite(fan_pin, fan_state);
  digitalWrite(led_pin, fan_state);
}

/*
  Read Analog from MQ Sensor
*/
float get_analog(uint8_t MQ_sensor) {
  uint32_t mV = analogReadMilliVolts(MQ_sensor);
  float adc = (mV * 4095) / 3300;
  adc = roundf(adc * 10.0) / 10.0;
  return adc;
}

/*
  Read PPM from MQ Sensor
*/
float get_ppm(uint8_t MQ_sensor, uint8_t rl, float ro, float m, float b) {
  float raw = get_analog(MQ_sensor);
  float v_rl = raw * (3.3 / 4095.0);
  if (v_rl < 0.01) v_rl = 0.01;
  float rs = ((3.3 * rl) / v_rl) - rl;
  float ratio = rs / ro;
  float ppm_log = (log10(ratio) - b) / m;
  float ppm_val = pow(10, ppm_log);
  return ppm_val;
}

/*
  Stream HeartBeat via MAVLink to Pixhawk
*/
void stream_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(
    ESP_SYS_ID,     
    ESP_COMP_ID, 
    &msg,
    MAV_TYPE_QUADROTOR,     
    MAV_AUTOPILOT_PIXHAWK, 
    MAV_MODE_MANUAL_ARMED, 
    0, 
    MAV_STATE_ACTIVE
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len); 
}

/*
  Stream Status Text Message via MAVLink to Pixhawk
*/
void stream_text_msg(const char* text, uint8_t severity) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_statustext_pack(
    ESP_SYS_ID,           
    ESP_COMP_ID,
    &msg,
    severity,
    text
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

/*
  Stream MQ Gas Sensor via MAVLink to Pixhawk
*/
void stream_nval_msg(const char* name, float value) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_named_value_float_pack(
    ESP_SYS_ID,         // system ID        (1: Become one with pixhawk sys_ID)
    ESP_COMP_ID,        // component ID     (158: Generic autopilot peripheral comp_ID)
    &msg,               // mavlink message
    millis(),           // timeboot ms
    name,               // variable name
    value               // floating point value
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

/*
  Stream Mission Land Command via MAVLink to Pixhawk
*/
void stream_mav_cmd_land() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    ESP_SYS_ID, ESP_COMP_ID,            // system ID, component ID
    &msg,                               // mavlink messages
    PIXHAWK_SYS_ID, PIXHAWK_COMP_ID,    // target system ID, target component ID
    MAV_CMD_NAV_LAND,                   // mavlink command
    0,                                  // confirmation
    0, 0, 0, NAN, NAN, NAN, NAN         // command parameters
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

/*
  Stream Mission Pause and Continue Command via MAVLink to Pixhawk
*/
void stream_mav_mission_cmd(uint8_t state) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    ESP_SYS_ID, ESP_COMP_ID,            // system ID, component ID
    &msg,                               // mavlink messages
    PIXHAWK_SYS_ID, PIXHAWK_COMP_ID,    // target system ID, target component ID
    MAV_CMD_DO_PAUSE_CONTINUE,          // mavlink command
    0,                                  // confirmation
    state, 0, 0, 0, 0, 0, 0             // command parameters
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

/*
  Request All Pixhawk Data from Pixhawk via MAVLink
*/
void request_all_data(uint8_t state) {
  mavlink_message_t msg;
  mavlink_status_t status;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_request_data_stream_pack(
    ESP_SYS_ID, ESP_COMP_ID,
    &msg,
    PIXHAWK_SYS_ID, PIXHAWK_COMP_ID,
    MAV_DATA_STREAM_ALL,
    10,
    state
  );

  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  SerialMAV.write(buffer, len); 
}

/*
  Request Current Sequence of Mission Plan from Pixhawk via MAVLink
*/
uint8_t request_sequence_mission(uint8_t* mission_sequence) {
  // request_all_data(START);
  mavlink_message_t msg;
  mavlink_status_t status;

  start_request = millis();
  while (millis() - start_request < request_timeout) {
    while (SerialMAV.available()) {
      uint8_t c = SerialMAV.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
          mavlink_mission_current_t mission;
          mavlink_msg_mission_current_decode(&msg, &mission);
          *mission_sequence =  mission.seq;
          return *mission_sequence;
        }
      }
    }
  }
  return *mission_sequence;
}

/*
  Request Waypoint Reached Messages from Pixhawk via MAVLink
*/
bool request_mission_reached(uint8_t* mission_sequence) {
  // request_all_data(START);
  mavlink_message_t msg;
  mavlink_status_t status;

  start_request = millis();
  while (millis() - start_request < request_timeout) {
    while (SerialMAV.available()) {
      uint8_t c = SerialMAV.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_REACHED) {
          mavlink_mission_item_reached_t reached;
          mavlink_msg_mission_item_reached_decode(&msg, &reached);
          *mission_sequence = reached.seq;
          return true;
        }
      }
    }
  }
 return false; 
}

/*
  Request GPS Coordinate from Pixhawk via MAVLink
*/ 
GPS_coordinate request_gps_coordinate() {
  // request_all_data(START);
  static GPS_coordinate coord = {0.0, 0.0};
  mavlink_message_t msg;
  mavlink_status_t status;

  start_request = millis();
  while (millis() - start_request < request_timeout) {
    while (SerialMAV.available()) {
      uint8_t c = SerialMAV.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
          mavlink_gps_raw_int_t gps;
          mavlink_msg_gps_raw_int_decode(&msg, &gps);
          coord.lat = gps.lat / 1e7;
          coord.lon = gps.lon / 1e7;
          return coord;
        }
      }
    }
  }
  return coord;
}

/*
  Interval Frequency of Heartbeat Stream
*/
void heartbeat_interval() {
  if (millis() - last_heartbeat >= heartbeat_frequency) {
    stream_heartbeat();
    last_heartbeat = millis();
  }
}

/*
  Interval Frequency of Mavlink Command Stream
*/
void command_interval(uint8_t state, unsigned long frequency) {
  if (millis() - last_command >= frequency) {
    stream_mav_mission_cmd(state);
    last_command = millis();
  }
}

/*
  Interval Frequency of Sequence Messages Stream
*/
void messages_interval(uint8_t sequence, uint8_t severity) {
  if (millis() - last_message >= message_frequency) {
    char buffer[20]; 
    sprintf(buffer, "sequence: %d", sequence); 
    stream_text_msg(buffer, severity);
    last_message = millis();
  }
}

/*
  Interval Frequency of Text Messages Stream
*/
void messages_interval(const char* text, uint8_t severity) {
  if (millis() - last_message >= message_frequency) {
    stream_text_msg(text, severity);
    last_message = millis();
  }
}

/*
  Interval Frequency of Gas Sampling Stream
*/
void sampling_interval(uint8_t sequence) {
  ppm.MQ2 = get_ppm(MQ2_pin, MQ2_rl, MQ2_ro, MQ2_m, MQ2_b);
  ppm.MQ5 = get_ppm(MQ5_pin, MQ5_rl, MQ5_ro, MQ5_m, MQ5_b);
  if (millis() - last_stream >= stream_frequency) {
    char name_MQ2[10];
    char name_MQ5[10];
    snprintf(name_MQ2, sizeof(name_MQ2), "MQ2-%d", sequence);
    snprintf(name_MQ5, sizeof(name_MQ5), "MQ5-%d", sequence);

    stream_nval_msg(name_MQ2, ppm.MQ2);
    stream_nval_msg(name_MQ5, ppm.MQ5);

    last_stream = millis();
  }
}
