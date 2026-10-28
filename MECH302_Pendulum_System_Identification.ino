// MECH 302 - Pendulum system identification (free decay)
// Encoder A = D10, B = D11. 1200 decoded counts per revolution.
// Samples at 100 Hz; prints CSV: time_s,angle_deg,angle_rad,omega_rad_s,alpha_rad_s2,count
// Note: Uses polling (no external interrupt dependency). Keep loop free of delays.

const uint8_t ENC_A = 10;
const uint8_t ENC_B = 11;
const float COUNTS_PER_REV = 1200.0f; // verify by manually rotating exactly 360 degrees
const unsigned long SAMPLE_US = 10000UL;
const float TWO_PI_F = 6.28318530718f;

long encoderCount = 0;
uint8_t previousAB = 0;
unsigned long lastSampleUs = 0;
unsigned long startUs = 0;
float lastAngle = 0.0f;
float lastOmega = 0.0f;
float filteredOmega = 0.0f;
float filteredAlpha = 0.0f;
const float FILTER = 0.25f; // smoothing factor: 0 < FILTER <= 1
bool firstSample = true;

void pollEncoder() {
  uint8_t a = digitalRead(ENC_A);
  uint8_t b = digitalRead(ENC_B);
  uint8_t currentAB = (a << 1) | b;
  // Quadrature transition table, rejects invalid two-bit transitions.
  static const int8_t transition[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
  };
  encoderCount += transition[(previousAB << 2) | currentAB];
  previousAB = currentAB;
}

void setup() {
  pinMode(ENC_A, INPUT); // use INPUT_PULLUP only if compatible with encoder output
  pinMode(ENC_B, INPUT);
  Serial.begin(115200);
  previousAB = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  startUs = micros();
  lastSampleUs = startUs;
  Serial.println(F("time_s,angle_deg,angle_rad,omega_rad_s,alpha_rad_s2,count"));
}

void loop() {
  pollEncoder();
  unsigned long now = micros();
  unsigned long elapsed = now - lastSampleUs;
  if (elapsed < SAMPLE_US) return;
  lastSampleUs = now;
  float dt = elapsed * 1.0e-6f;
  float angle = (float)encoderCount * TWO_PI_F / COUNTS_PER_REV;
  float omegaRaw = (angle - lastAngle) / dt;
  if (firstSample) {
    filteredOmega = 0.0f;
    filteredAlpha = 0.0f;
    firstSample = false;
  } else {
    filteredOmega += FILTER * (omegaRaw - filteredOmega);
    float alphaRaw = (filteredOmega - lastOmega) / dt;
    filteredAlpha += FILTER * (alphaRaw - filteredAlpha);
  }
  lastOmega = filteredOmega;
  lastAngle = angle;
  Serial.print((now - startUs) * 1.0e-6f, 4);
  Serial.print(','); Serial.print(angle * 180.0f / 3.14159265359f, 3);
  Serial.print(','); Serial.print(angle, 5);
  Serial.print(','); Serial.print(filteredOmega, 5);
  Serial.print(','); Serial.print(filteredAlpha, 5);
  Serial.print(','); Serial.println(encoderCount);
}
