#include "Arduino.h"
#include "mavlink.h"

#define MAVLINK_VERSION (1)

// ESP32 Serial MAVLink Connection
#define TXD_PIN (17)
#define RXD_PIN (16)
#define BAUD_RATE 57600

// ENUM Mission Planning [redundant]
#define MISSION_TAKEOFF 22
#define MISSION_DELAY   93
#define MISSION_LAND    21
#define MISSION_WP      16

// ENUM Sequence Mission Plan
#define FIRST_MISSION 1
#define LAST_MISSION 10

// ENUM Mission Command [empty in library]
#define MAV_CMD_DO_PAUSE_CONTINUE 193

// ENUM Mav Command
#define PAUSE 0
#define CONTINUE 1

// ENUM Gas State
#define INLET 1
#define OUTLET 0

// MQ2 Gas Sensor Params Configuration
const int MQ2_pin  = 26;
const int MQ2_rl   = 20;
const float MQ2_ro = 21.5;
const float MQ2_m  = -0.46612;
const float MQ2_b  = 1.28400;

// MQ5 Gas Sensor Params Configuration
const int MQ5_pin  = 25;
const int MQ5_rl   = 20;
const float MQ5_ro = 76.93;
const float MQ5_m  = -0.39949;
const float MQ5_b  = 0.76076;

const int fan_pin = 35;
unsigned long last_mill = 0;
unsigned long frequency = 100;

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

bool is_sampling();
void gas_state(int fan_state);
float get_analog(int MQ_sensor);
float get_ppm(int MQ_sensor, int rl, float ro, float m, float b);
void stream_heartbeat();
void stream_nval_msg(const char* name, float value);
void stream_mav_command(int param_1) ;
int request_sequence_mission();
GPS_coordinate request_gps_coordinate();
void sampling_and_stream(int seq); 

// DEBUG
bool paused = false;

void setup() {
  // Serial Monitor Setup
  Serial.begin(115200);

  // Serial MAVLink Setup
  SerialMAV.begin(BAUD_RATE, SERIAL_8N1, RXD_PIN, TXD_PIN);

  // Pinout Setup
  pinMode(MQ2_pin, INPUT);
  pinMode(MQ5_pin, INPUT);
  pinMode(fan_pin, OUTPUT);

  // Analog Setup
  analogReadResolution(12);
  adcAttachPin(MQ2_pin);
  adcAttachPin(MQ5_pin);
  analogSetPinAttenuation(MQ2_pin, ADC_11db);
  analogSetPinAttenuation(MQ5_pin, ADC_11db);

  // MAVLink Heartbeat Initialization
  stream_heartbeat();
}

void loop() {
  // --- MAIN LOOP --- //
  // if (is_sampling()) {
      // executed code 
  // }
  
  // --- TEST FLIGHT (PAUSE) --- //
  // attr plan : takeoff (seq1) -> wp_1 (seq2) -> wp_2 (seq3) -> land (seq4)
  // debug behav : wp_2 -> delay 12s (esp intterupt command) -> land

  const int wp_3 = 4; 
  if ((request_sequence_mission() == wp_3) && !paused) {
    paused = true;
    unsigned long start_pause = millis();
    unsigned long pause_time = 10000;
    while (millis() - start_pause <= pause_time) {
      stream_mav_command(PAUSE);
      request_sequence_mission();
      delay(100);
    }
    unsigned long start_continue = millis();
    unsigned long continue_time = 2000;
    while (millis() - start_continue <= continue_time) {
      stream_mav_command(CONTINUE);
      request_sequence_mission();
      delay(100);
    }
  }

  delay(10);
}

/*
  Protector Sampling Sequence
*/
bool is_sampling() {
  uint8_t current_mission = request_sequence_mission();
  if (current_mission == FIRST_MISSION || current_mission == LAST_MISSION) return false;
  else return true;
}

/*
  Fungsi untuk melakukan millis() pengiriman ppm dan command mission kah (continue or paused)?
*/
// void timer() {

// }

/*
  Fan System for Gas Behaviour
*/
void gas_state(int fan_state) {
  digitalWrite(fan_pin, fan_state);
}

/*
  Read Analog from MQ Sensor
*/
float get_analog(int MQ_sensor) {
  uint32_t mV = analogReadMilliVolts(MQ_sensor);
  float adc = (mV * 4095) / 3300;
  adc = roundf(adc * 10.0) / 10.0;
  return adc;
}

/*
  Read PPM from MQ Sensor
*/
float get_ppm(int MQ_sensor, int rl, float ro, float m, float b) {
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
    1,     
    158, 
    &msg,
    MAV_TYPE_QUADROTOR,     
    MAV_AUTOPILOT_INVALID, 
    MAV_MODE_MANUAL_ARMED, 
    0, 
    MAV_STATE_ACTIVE
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
    1,                  // system ID        (1: Become one with pixhawk sys_ID)
    158,                // component ID     (158: Generic autopilot peripheral comp_ID)
    &msg,               // mavlink message
    millis() / 1000,    // timeboot
    name,               // variable name
    value               // floating point value
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

/*
  Stream Mission State Command via MAVLink to Pixhawk
*/
void stream_mav_command(int param_1) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(
    1, 158,                     // system ID, component ID
    &msg,                       // mavlink messages
    1, 1,                       // target system ID, target component ID
    MAV_CMD_DO_PAUSE_CONTINUE,  // mavlink command
    0,                          // confirmation
    param_1, 0, 0, 0, 0, 0, 0   // command parameters
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

/*
  Request Current Mission Sequence from Pixhawk via MAVLink
*/
int request_sequence_mission() {
  static int last_mission = 0;
  mavlink_message_t msg;
  mavlink_status_t status;

  while (SerialMAV.available()) {
    uint8_t c = SerialMAV.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
        mavlink_mission_current_t mission;
        mavlink_msg_mission_current_decode(&msg, &mission);
        last_mission = mission.seq;
      }
    }
  }
  return last_mission;
}

/*
  Request GPS Lattitude and Longitude from Pixhawk via MAVLink
*/
GPS_coordinate request_gps_coordinate() {
  static GPS_coordinate coord = {0.0, 0.0};
  mavlink_message_t msg;
  mavlink_status_t status;

  while (SerialMAV.available()) {
    uint8_t c = SerialMAV.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        mavlink_gps_raw_int_t gps;
        mavlink_msg_gps_raw_int_decode(&msg, &gps);
        coord.lat = gps.lat / 1e7;
        coord.lon = gps.lon / 1e7;
      }
    }
  }
  return coord;
}

void sampling_and_stream(int seq) {
  gas_state(INLET);
  ppm.MQ2 = get_ppm(MQ2_pin, MQ2_rl, MQ2_ro, MQ2_m, MQ2_b);
  ppm.MQ5 = get_ppm(MQ5_pin, MQ5_rl, MQ5_ro, MQ5_m, MQ5_b);
  if (millis() - last_mill >= frequency) {
    char name_MQ2[10];
    char name_MQ5[10];
    snprintf(name_MQ2, sizeof(name_MQ2), "MQ2-%d", seq);
    snprintf(name_MQ5, sizeof(name_MQ5), "MQ5-%d", seq);

    stream_nval_msg(name_MQ2, ppm.MQ2);
    stream_nval_msg(name_MQ5, ppm.MQ5);

    last_mill = millis();
  }
}