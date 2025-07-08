#define MQ_RL 20
#define MQ_VC 3.3

const int MQ2_pin = 27;
const int MQ5_pin = 26;

float MQ2_clean_air = 9.7;
float MQ5_clean_air = 6.478;
float MQ2_ro;
float MQ5_ro;

void setup() {
  Serial.begin(9600);

  pinMode(MQ2_pin, INPUT);
  pinMode(MQ5_pin, INPUT);

  analogReadResolution(12);
  adcAttachPin(MQ2_pin);
  adcAttachPin(MQ5_pin);
  analogSetPinAttenuation(MQ2_pin, ADC_11db);
  analogSetPinAttenuation(MQ5_pin, ADC_11db);
}

void loop() {
  MQ2_ro = get_ro(MQ2_pin, MQ2_clean_air);
  MQ5_ro = get_ro(MQ5_pin, MQ5_clean_air);

  print_ro("MQ2", MQ2_ro);
  print_ro("MQ5", MQ5_ro);

  // -- DEBUG ANALOG RAW mV -- //

  // float mq2 = get_analog(MQ2_pin);
  // float mq5 = get_analog(MQ5_pin);

  // -- DEBUG ANALOG RAW -- //

  // // float mq2 = analogRead(MQ2_pin);
  // // float mq5 = analogRead(MQ5_pin);

  // -- DEBUG PRINT-OUT ANALOG -- //

  // Serial.print("analog mq2 = ");
  // Serial.println(mq2);
  // Serial.print("analog mq5 = ");
  // Serial.println(mq5);

  Serial.println("-------------\n");

  delay(1000);
}

float get_analog(int MQ_sensor) {
  uint32_t mV = analogReadMilliVolts(MQ_sensor);
  float adc = (mV * 4095) / 3300;
  adc = roundf(adc * 10.0) / 10.0;

  return adc;
}

float get_ro(int MQ_sensor, float clean_air) {
  float raw = get_analog(MQ_sensor);
  float v_out = raw * (MQ_VC / 4095.0);
  float rs = ((MQ_VC * MQ_RL) / v_out) - MQ_RL;
  float ro = rs / clean_air;

  // Serial.print("Vc  = ");
  // Serial.println(MQ_VC);
  // Serial.print("Vrl = ");
  // Serial.println(v_out);
  // Serial.print("Rl  = ");
  // Serial.println(MQ_RL);
  // Serial.print("Rs  = ");
  // Serial.println(rs);
  // Serial.print("Ro  = ");
  // Serial.println(ro);

  return ro;
}

void print_ro(String sensor, float ro_val) {
  Serial.print(sensor);
  Serial.print(" RO value in clean air = ");
  Serial.println(ro_val);
}
