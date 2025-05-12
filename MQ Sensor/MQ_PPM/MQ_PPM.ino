// MQ-2 Config
const int MQ2_pin  = 26;
const int MQ2_rl   = 20;
const float MQ2_ro = 21.5;
const float MQ2_m  = -0.46612;
const float MQ2_b  = 1.28400;
float MQ2_ppm;

// MQ-5 Config
const int MQ5_pin  = 25;
const int MQ5_rl   = 20;
const float MQ5_ro = 76.93;
const float MQ5_m  = -0.39949;
const float MQ5_b  = 0.76076;
float MQ5_ppm;

float get_analog(int MQ_sensor);
float read_ppm(int MQ_sensor, int rl, float ro, float m, float b);
void print_ppm(String sensor, float ppm);

void setup() {
  Serial.begin(115200);

  pinMode(MQ2_pin, INPUT);
  pinMode(MQ5_pin, INPUT);

  analogReadResolution(12);
  adcAttachPin(MQ2_pin);
  adcAttachPin(MQ5_pin);
  analogSetPinAttenuation(MQ2_pin, ADC_11db);
  analogSetPinAttenuation(MQ5_pin, ADC_11db);
}

void loop() {
  MQ2_ppm = read_ppm(MQ2_pin, MQ2_rl, MQ2_ro, MQ2_m, MQ2_b);
  MQ5_ppm = read_ppm(MQ5_pin, MQ5_rl, MQ5_ro, MQ5_m, MQ5_b);

  print_ppm("MQ-2", MQ2_ppm);
  print_ppm("MQ-5", MQ5_ppm);
  Serial.println("--------");

  delay(500);
}

float get_analog(int MQ_sensor) {
  uint32_t mV = analogReadMilliVolts(MQ_sensor);
  float adc = (mV * 4095) / 3300;
  adc = roundf(adc * 10.0) / 10.0;

  return adc;
}

float read_ppm(int MQ_sensor, int rl, float ro, float m, float b) {
  float raw = get_analog(MQ_sensor);
  float v_rl = raw * (3.3 / 4095.0);
  if (v_rl < 0.01) v_rl = 0.01;
  float rs = ((3.3 * rl) / v_rl) - rl;
  float ratio = rs / ro;
  float ppm_log = (log10(ratio) - b) / m;
  float PPM = pow(10, ppm_log);
  
  return PPM;
}

void print_ppm(String sensor, float ppm) {
  Serial.print(sensor);
  Serial.print(" PPM = ");
  Serial.println(ppm);
}