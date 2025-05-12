#include "Arduino.h"
#include "mavlink.h"

#define MAVLINK_VERSION (1)

// ESP32 Serial MAVLink Connection
#define TXD_PIN (17)
#define RXD_PIN (16)
#define BAUD_RATE 57600

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

float MQ2_ppm;
float MQ5_ppm;

const int fan_pin = 35;
bool stop_request = false;
bool request_check = false;
bool gas_system_init = false;

unsigned long last_time_mil = 0;
unsigned long interval_mil = 1000;

HardwareSerial SerialMAV(2);

float get_analog(int MQ_sensor);
float get_ppm(int MQ_sensor, int rl, float ro, float m, float b);
void stream_heartbeat();
void stream_mavlink_message();
void stream_request_rc_channel(bool state); 
void gas_system_initialization();

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
  MQ2_ppm = get_ppm(MQ2_pin, MQ2_rl, MQ2_ro, MQ2_m, MQ2_b);
  MQ5_ppm = get_ppm(MQ5_pin, MQ5_rl, MQ5_ro, MQ5_m, MQ5_b);

  Serial.print("MQ2 PPM: ");
  Serial.println(MQ2_ppm);
  Serial.print("MQ5 PPM: ");
  Serial.println(MQ5_ppm);
  Serial.println("----------\n");
  delay(500);
  
  // if (!gas_system_init) {
  //   gas_system_initialization();
  //   digitalWrite(fan_pin, LOW);
  //   stop_request = false;
  // } 
  // else {
  //   digitalWrite(fan_pin, HIGH);
  //   stream_mavlink_message("PPM_MQ2", MQ2_ppm);
  //   stream_mavlink_message("PPM_MQ5", MQ5_ppm);

  // }

  // if (gas_system_init && !stop_request) {
  //   stream_request_rc_channel(false); 
  //   stop_request = true;
  // }

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
  float PPM = pow(10, ppm_log);
  
  return PPM;
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
void stream_mavlink_message(const char* name, float value) {
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
  Request RC Channel Vakue from Pixhawk via MAVLink
*/
void stream_request_rc_channel(bool state) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  if (state == true) {
    mavlink_msg_request_data_stream_pack(1, 158, &msg, 1, 1,                 
      MAV_DATA_STREAM_RC_CHANNELS, 10, 1
    );
  } 
  else if (state == false){
    mavlink_msg_request_data_stream_pack(1, 158, &msg, 1, 1,                 
      MAV_DATA_STREAM_RC_CHANNELS, 10, 0
    );
  }

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

/*
  Condition to Start Gas System Initialization
*/
void gas_system_initialization() {

  // using millis() to hold spamming request stream
  // if (millis() - last_time_mil > interval_mil) {
  //   stream_request_rc_channel(true);
  //   last_time_mil = millis();
  // }

  // using variables to send request stream for once
  if (!request_check) {
    stream_request_rc_channel(true);
    request_check = true;
  }

  if (SerialMAV.available()) {
    uint8_t c = SerialMAV.read(); 

    mavlink_message_t msg;
    mavlink_status_t status;

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_RAW) {
        mavlink_rc_channels_raw_t rc_channels;
        mavlink_msg_rc_channels_raw_decode(&msg, &rc_channels);

        // Fit the channel that used for switch mode
        uint16_t pwm_value = rc_channels.chan1_raw;
        uint16_t threshold = 1500;

        // This is would be deleted soon
        Serial.print("PWM Value : ");
        Serial.println(pwm_value);

        if (pwm_value < threshold) {
          gas_system_init = false;
        } 
        else if (pwm_value >= threshold) {
          gas_system_init = true;
        }
      }
    }
  }
}