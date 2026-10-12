#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Manual serial-controlled cart/arm test firmware.
// DRV8825: STEP=9, DIR=7. Pendulum encoder A=10, B=11.
// Motor scale: 1600 commanded steps/rev. Encoder scale: 1200 counts/rev.
// Serial: 115200 baud, newline terminated commands. No re-upload to change settings.
// Commands: HELP, VEL <rpm>, ACCEL <rpm_per_second>, DIST <signed_steps>,
//           ZERO, START, STOP, HALT, STATUS, STREAM ON|OFF
// DIST is a relative move in motor steps. Configure values, then START.
// START begins a move; STOP decelerates to a stop; HALT cuts commanded motion now.
// This sketch provides commanded movement and measurements; it does not balance
// or automatically hold the pendulum upright.

const uint8_t STEP_PIN = 9;
const uint8_t DIR_PIN = 7;
const uint8_t ENC_A = 10;
const uint8_t ENC_B = 11;

const float MOTOR_STEPS_PER_REV = 1600.0f;
const float ENCODER_COUNTS_PER_REV = 1200.0f;
const float MAX_ALLOWED_RPM = 265.0f;
const float MAX_ALLOWED_ACCEL_RPM_S = 4250.0f;
const unsigned long SAMPLE_PERIOD_MS = 20; // 50 Hz telemetry; motor.run() remains frequent
const uint8_t SERIAL_LINE_MAX = 80;

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
volatile long encoderCount = 0;
volatile uint8_t previousEncoderState = 0;

float requestedRPM = 60.0f;
float requestedAccelRPM_S = 500.0f;
long requestedDistanceSteps = 1600;
long angleZeroCount = 0;
bool motionActive = false;
bool streamEnabled = true;
unsigned long lastSampleMs = 0;
char inputLine[SERIAL_LINE_MAX];
uint8_t inputLength = 0;

void encoderISR() {
  uint8_t state = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  // Quadrature transition table. Invalid two-bit transitions contribute zero.
  static const int8_t change[16] = {
     0, 1,-1, 0,
    -1, 0, 0, 1,
     1, 0, 0,-1,
     0,-1, 1, 0
  };
  encoderCount += change[(previousEncoderState << 2) | state];
  previousEncoderState = state;
}

long readEncoderCount() {
  noInterrupts();
  long value = encoderCount;
  interrupts();
  return value;
}

float rpmToStepsPerSecond(float rpm) {
  return rpm * MOTOR_STEPS_PER_REV / 60.0f;
}

float rpmPerSecondToStepsPerSecondSquared(float rpmPerSecond) {
  return rpmPerSecond * MOTOR_STEPS_PER_REV / 60.0f;
}

void printHelp() {
  Serial.println(F("#COMMANDS: HELP | VEL rpm | ACCEL rpm_per_s | DIST signed_steps | ZERO | START | STOP | HALT | STATUS | STREAM ON|OFF"));
  Serial.println(F("#DIST is relative steps; set values, then START. STOP decelerates; HALT stops commanded pulses now."));
  Serial.println(F("#DATA,time_ms,encoder_count,angle_deg,cart_steps,cart_velocity_steps_s,target_steps,requested_rpm,requested_accel_rpm_s"));
}

void printStatus() {
  Serial.print(F("#STATUS,moving=")); Serial.print(motionActive ? 1 : 0);
  Serial.print(F(",rpm=")); Serial.print(requestedRPM, 2);
  Serial.print(F(",accel_rpm_s=")); Serial.print(requestedAccelRPM_S, 2);
  Serial.print(F(",distance_steps=")); Serial.print(requestedDistanceSteps);
  Serial.print(F(",cart_steps=")); Serial.print(motor.currentPosition());
  Serial.print(F(",remaining_steps=")); Serial.println(motor.distanceToGo());
}

void reportError(const __FlashStringHelper *message) {
  Serial.print(F("#ERROR,")); Serial.println(message);
}

void processCommand(char *line) {
  // Trim leading spaces and normalize command name only.
  while (*line == ' ' || *line == '\t') ++line;
  for (char *p = line; *p; ++p) if (*p >= 'a' && *p <= 'z') *p -= ('a' - 'A');
  if (!*line) return;

  if (!strcmp(line, "HELP")) { printHelp(); return; }
  if (!strcmp(line, "STATUS")) { printStatus(); return; }
  if (!strcmp(line, "ZERO")) {
    if (motionActive) { reportError(F("STOP_BEFORE_ZERO")); return; }
    motor.setCurrentPosition(0);
    angleZeroCount = readEncoderCount();
    Serial.println(F("#OK,ZEROED_CART_AND_ENCODER_REFERENCE"));
    return;
  }
  if (!strcmp(line, "START")) {
    if (motionActive) { reportError(F("ALREADY_MOVING")); return; }
    if (requestedDistanceSteps == 0) { reportError(F("DISTANCE_IS_ZERO")); return; }
    motor.setMaxSpeed(rpmToStepsPerSecond(requestedRPM));
    motor.setAcceleration(rpmPerSecondToStepsPerSecondSquared(requestedAccelRPM_S));
    motor.move(requestedDistanceSteps); // relative to current cart position
    motionActive = true;
    Serial.println(F("#EVENT,START"));
    return;
  }
  if (!strcmp(line, "STOP")) {
    if (motionActive) {
      motor.stop(); // controlled deceleration using current acceleration setting
      Serial.println(F("#EVENT,STOPPING"));
    } else Serial.println(F("#EVENT,ALREADY_STOPPED"));
    return;
  }
  if (!strcmp(line, "HALT")) {
    motor.setSpeed(0.0f);
    motor.moveTo(motor.currentPosition());
    motionActive = false;
    Serial.println(F("#EVENT,HALTED"));
    return;
  }
  if (!strncmp(line, "STREAM ", 7)) {
    const char *arg = line + 7;
    if (!strcmp(arg, "ON")) { streamEnabled = true; Serial.println(F("#OK,STREAM_ON")); }
    else if (!strcmp(arg, "OFF")) { streamEnabled = false; Serial.println(F("#OK,STREAM_OFF")); }
    else reportError(F("USE_STREAM_ON_OR_OFF"));
    return;
  }

  float f;
  long integer;
  char extra;
  if (sscanf(line, "VEL %f %c", &f, &extra) == 1) {
    if (!(f > 0.0f && f <= MAX_ALLOWED_RPM)) { reportError(F("VEL_RANGE_0_TO_265_RPM")); return; }
    requestedRPM = f;
    Serial.print(F("#OK,VEL,")); Serial.println(requestedRPM, 2);
    return;
  }
  if (sscanf(line, "ACCEL %f %c", &f, &extra) == 1) {
    if (!(f > 0.0f && f <= MAX_ALLOWED_ACCEL_RPM_S)) { reportError(F("ACCEL_RANGE_0_TO_4250_RPM_S")); return; }
    requestedAccelRPM_S = f;
    Serial.print(F("#OK,ACCEL,")); Serial.println(requestedAccelRPM_S, 2);
    return;
  }
  if (sscanf(line, "DIST %ld %c", &integer, &extra) == 1) {
    if (integer == 0) { reportError(F("DIST_MUST_NOT_BE_ZERO")); return; }
    requestedDistanceSteps = integer;
    Serial.print(F("#OK,DIST,")); Serial.println(requestedDistanceSteps);
    return;
  }
  reportError(F("UNKNOWN_COMMAND_SEND_HELP"));
}

void readSerialCommands() {
  // Read a few characters each pass; never wait for a complete line while stepping.
  for (uint8_t n = 0; n < 16 && Serial.available(); ++n) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      inputLine[inputLength] = '\0';
      processCommand(inputLine);
      inputLength = 0;
    } else if (inputLength < SERIAL_LINE_MAX - 1) {
      inputLine[inputLength++] = c;
    } else {
      inputLength = 0;
      reportError(F("COMMAND_TOO_LONG"));
    }
  }
}

void emitTelemetry() {
  unsigned long now = millis();
  if (!streamEnabled || now - lastSampleMs < SAMPLE_PERIOD_MS) return;
  lastSampleMs = now;
  long counts = readEncoderCount();
  float angle = (counts - angleZeroCount) * (360.0f / ENCODER_COUNTS_PER_REV);
  char row[120];
  int length = snprintf(row, sizeof(row), "%lu,%ld,%.3f,%ld,%.2f,%ld,%.2f,%.2f\n",
      now, counts, angle, motor.currentPosition(), motor.speed(), motor.targetPosition(),
      requestedRPM, requestedAccelRPM_S);
  // If host isn't reading, drop this sample rather than blocking motor pulses.
  if (length > 0 && length < (int)sizeof(row) && Serial.availableForWrite() >= length)
    Serial.write((const uint8_t *)row, length);
}

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  previousEncoderState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  int irqA = digitalPinToInterrupt(ENC_A);
  int irqB = digitalPinToInterrupt(ENC_B);
  if (irqA != NOT_AN_INTERRUPT && irqB != NOT_AN_INTERRUPT) {
    attachInterrupt(irqA, encoderISR, CHANGE);
    attachInterrupt(irqB, encoderISR, CHANGE);
  }
  motor.setMaxSpeed(rpmToStepsPerSecond(requestedRPM));
  motor.setAcceleration(rpmPerSecondToStepsPerSecondSquared(requestedAccelRPM_S));
  Serial.println(F("#READY,MANUAL_CART_CONTROL,115200"));
  printHelp();
}

void loop() {
  readSerialCommands();
  if (motionActive) {
    motor.run();
    if (motor.distanceToGo() == 0) {
      motionActive = false;
      Serial.println(F("#EVENT,MOVE_COMPLETE"));
    }
  }
  emitTelemetry();
}
