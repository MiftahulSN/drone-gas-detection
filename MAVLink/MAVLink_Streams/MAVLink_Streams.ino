#include "Arduino.h"
#include "mavlink.h"

// ESP32 Serial MAVLink Connection
#define TXD_PIN (17)
#define RXD_PIN (16)
#define BAUD_RATE 57600

HardwareSerial SerialMAV(2);

void setup() {
  // Serial Monitor Setup
  Serial.begin(115200);

  // Serial MAVLink Setup
  SerialMAV.begin(BAUD_RATE, SERIAL_8N1, RXD_PIN, TXD_PIN);
}

void loop() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Data yang ingin dikirim (contoh: nilai int 100)
  uint8_t data = 100;
  char message[50];
  snprintf(message, sizeof(message), "data-value: %d", data);

  // Streams 'STATUSTEXT' Message
  mavlink_msg_statustext_pack(
    1,                            // system ID
    200,                          // component ID
    &msg,                         // MAVLink pointer
    MAV_SEVERITY_INFO,            // severity level
    message                       // data message
  );

  // Convert to Buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Streams Message to Pixhawk
  SerialMAV.write(buf, len);

  // Streams Frequency
  delay(1000);
}
