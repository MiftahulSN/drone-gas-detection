#include "Arduino.h"
#include "mavlink.h"

// ESP32 Serial MAVLink Connection
#define TXD_PIN (17)
#define RXD_PIN (16)
#define BAUD_RATE 57600

// PI Constanta
#define PI 3.14159265358979323846

HardwareSerial SerialMAV(2);

unsigned long last_time_mil = 0;
unsigned long interval_mil = 1000;

void stream_heartbeat();
void request_all_data();
void decode_mav_msg();

void setup() {
  Serial.begin(115200);
  SerialMAV.begin(BAUD_RATE, SERIAL_8N1, RXD_PIN, TXD_PIN);

  stream_heartbeat();
}

void loop() {

  if (millis() - last_time_mil > interval_mil) {
    request_all_data();
    last_time_mil = millis();
  }

  decode_mav_msg();
}

/*
  Stream Heartbeat
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
  Request All Data Streams
*/
void request_all_data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_request_data_stream_pack(1, 158, &msg, 1, 1, 
      MAV_DATA_STREAM_ALL, 10, 1
    );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

/*
  Decode Mavlink Messages
*/
void decode_mav_msg() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (SerialMAV.available()) {
    uint8_t c = SerialMAV.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);

        float roll  = attitude.roll  * 180.0 / PI;
        float pitch = attitude.pitch * 180.0 / PI;
        float yaw   = attitude.yaw   * 180.0 / PI;

        Serial.print("Pitch: ");
        Serial.print(pitch);

        Serial.print("° \t\t| Roll: ");
        Serial.print(roll);

        Serial.print("° \t\t| Yaw: ");
        Serial.print(yaw);
        Serial.println("°");
      }
    }
  }
}