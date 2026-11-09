
#include <AccelStepper.h>
#include <math.h>

// ============================================================
// 1. MOTOR CONFIGURATION
// ============================================================

#define STEP_PIN 9
#define DIR_PIN 7

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const float STEPS_PER_REV = 1600.0;
const float STEPS_PER_MM = 43.24;

const float MAX_SPEED_RPM = 265.0;
const float ACCELERATION_RPM_S = 4250.0;

const float MAX_SPEED_STEPS =
  MAX_SPEED_RPM * STEPS_PER_REV / 60.0;

const float MAX_ACCEL_STEPS =
  ACCELERATION_RPM_S * STEPS_PER_REV / 60.0;


// ============================================================
// 2. ENCODER CONFIGURATION
// ============================================================

#define ENCODER_A 10
#define ENCODER_B 11

const float ENCODER_COUNTS_PER_REV = 1200.0;
const float ENCODER_SIGN = 1.0;

volatile long encoderCount = 0;
int previousEncoderState = 0;

float getTheta() {
  return ENCODER_SIGN * encoderCount *
         (2.0 * PI / ENCODER_COUNTS_PER_REV);
}

void updateEncoder() {

  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);

  int currentState = (a << 1) | b;

  int transition =
    (previousEncoderState << 2) | currentState;

  switch (transition) {

    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoderCount++;
      break;

    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoderCount--;
      break;

    default:
      break;
  }

  previousEncoderState = currentState;
}


// ============================================================
// 3. PENDULUM MODEL PARAMETERS
// ============================================================

const float NATURAL_FREQUENCY = 1.36;
const float OMEGA_N = 2.0 * PI * NATURAL_FREQUENCY;

const float ZETA = 0.02119;

const float NATURAL_DAMPING =
  2.0 * ZETA * OMEGA_N;

const float K =
  (OMEGA_N * OMEGA_N) / 9.81;


// ============================================================
// 4. CONTROL SETTINGS
// ============================================================

const float CONTROL_DT = 0.01;
const unsigned long LOG_INTERVAL = 20;

const unsigned long FREE_SWING_TIME = 700;

// Cart limits
const float MAX_CART_DISPLACEMENT = 0.070;
const float MAX_CART_VELOCITY = 0.1634;
const float MAX_CART_ACCELERATION = 2.621;

// Acceleration filter
float filteredAcceleration = 0.0;
const float ACCEL_FILTER = 0.15;

// Angular velocity filter
const float VELOCITY_FILTER = 0.25;


// ============================================================
// 5. TRIAL VARIABLES
// ============================================================

float cartPosition = 0.0;
float cartVelocity = 0.0;

float previousTheta = 0.0;
float filteredThetaDot = 0.0;

long feedbackStartSteps = 0;

unsigned long trialStartTime = 0;
unsigned long lastControlTime = 0;
unsigned long lastLogTime = 0;


// ============================================================
// 6. DATA LOGGING
// ============================================================

void logData(
  String stage,
  float thetaDot,
  float targetMM
) {

  float timeSeconds =
    (millis() - trialStartTime) / 1000.0;

  float cartMM =
    motor.currentPosition() / STEPS_PER_MM;

  Serial.print("DATA,");
  Serial.print(timeSeconds, 4);
  Serial.print(",");
  Serial.print(stage);
  Serial.print(",");
  Serial.print(cartMM, 4);
  Serial.print(",");
  Serial.print(getTheta(), 6);
  Serial.print(",");
  Serial.print(thetaDot, 6);
  Serial.print(",");
  Serial.println(targetMM, 4);
}


// ============================================================
// 7. CART EXCITATION
// ============================================================

void moveCartToMM(float targetMM) {

  long targetSteps =
    lround(targetMM * STEPS_PER_MM);

  motor.moveTo(targetSteps);

  unsigned long lastLog = millis();

  while (motor.distanceToGo() != 0) {

    motor.run();
    updateEncoder();

    // Log at 50 Hz, not on every motor-loop iteration
    if (millis() - lastLog >= LOG_INTERVAL) {

      logData("EXCITATION", 0.0, targetMM);

      lastLog = millis();
    }
  }
}


// ============================================================
// 8. ANGULAR VELOCITY ESTIMATION
// ============================================================

void updateAngularVelocity(float dt) {

  float theta = getTheta();

  float difference = theta - previousTheta;

  // Correct for encoder angle wrapping
  if (difference > PI) {
    difference -= 2.0 * PI;
  }

  if (difference < -PI) {
    difference += 2.0 * PI;
  }

  float rawThetaDot = difference / dt;

  filteredThetaDot += VELOCITY_FILTER *
    (rawThetaDot - filteredThetaDot);

  previousTheta = theta;
}


// ============================================================
// 9. RUN TRIAL
// ============================================================

void runTrial(
  String mode,
  float excitationMM,
  float Kp,
  float Kd,
  float durationS
) {

  // Validate mode
  if (
    mode != "CONTROL" &&
    mode != "QUASH" &&
    mode != "MAINTAIN" &&
    mode != "FREQUENCY"
  ) {
    Serial.println("ERROR,INVALID_MODE");
    return;
  }

  // Validate excitation and duration
  if (excitationMM < 0 || excitationMM > 40) {
    Serial.println("ERROR,EXCITATION_RANGE");
    return;
  }

  if (durationS <= 0 || durationS > 20) {
    Serial.println("ERROR,DURATION_RANGE");
    return;
  }

  // Reset controller variables
  cartPosition = 0.0;
  cartVelocity = 0.0;

  filteredAcceleration = 0.0;
  filteredThetaDot = 0.0;

  trialStartTime = millis();
  lastLogTime = millis();

  Serial.println("BEGIN");


  // ----------------------------------------------------------
  // STEP 1: INITIAL CART EXCITATION
  // ----------------------------------------------------------

  moveCartToMM(excitationMM);


  // ----------------------------------------------------------
  // STEP 2: FREE-SWING PERIOD
  // ----------------------------------------------------------

  previousTheta = getTheta();
  filteredThetaDot = 0.0;

  unsigned long freeStart = millis();
  unsigned long lastFreeUpdate = micros();

  lastLogTime = millis();

  while (millis() - freeStart < FREE_SWING_TIME) {

    updateEncoder();

    unsigned long now = micros();

    float dt =
      (now - lastFreeUpdate) / 1000000.0;

    if (dt >= CONTROL_DT) {

      updateAngularVelocity(dt);

      lastFreeUpdate = now;
    }

    if (millis() - lastLogTime >= LOG_INTERVAL) {

      logData("FREE", filteredThetaDot, excitationMM);

      lastLogTime = millis();
    }
  }


  // ----------------------------------------------------------
  // STEP 3: INITIALIZE FEEDBACK
  // ----------------------------------------------------------

  feedbackStartSteps = motor.currentPosition();

  cartPosition = 0.0;
  cartVelocity = 0.0;

  filteredAcceleration = 0.0;

  previousTheta = getTheta();
  filteredThetaDot = 0.0;

  unsigned long feedbackStartTime = millis();

  lastControlTime = micros();
  lastLogTime = millis();

  Serial.println("CONTROL_START");


  // ----------------------------------------------------------
  // STEP 4: FEEDBACK CONTROL
  // ----------------------------------------------------------

  while (
    millis() - feedbackStartTime <
    (unsigned long)(durationS * 1000.0)
  ) {

    updateEncoder();

    // Keep stepper motor running
    motor.run();

    unsigned long now = micros();

    float dt =
      (now - lastControlTime) / 1000000.0;

    if (dt >= CONTROL_DT) {

      lastControlTime = now;

      updateAngularVelocity(dt);

      float theta = getTheta();

      float commandAcceleration = 0.0;


      // ------------------------------------------------------
      // CALCULATE CONTROL ACCELERATION
      // ------------------------------------------------------

      if (mode != "CONTROL") {

        commandAcceleration =
          ((Kp - OMEGA_N * OMEGA_N) / K) * theta
          +
          ((Kd - NATURAL_DAMPING) / K) *
          filteredThetaDot;
      }


      // ------------------------------------------------------
      // LIMIT ACCELERATION
      // ------------------------------------------------------

      commandAcceleration = constrain(
        commandAcceleration,
        -MAX_CART_ACCELERATION,
        MAX_CART_ACCELERATION
      );


      // ------------------------------------------------------
      // FILTER ACCELERATION
      // ------------------------------------------------------

      filteredAcceleration += ACCEL_FILTER *
        (commandAcceleration - filteredAcceleration);


      // ------------------------------------------------------
      // INTEGRATE ACCELERATION INTO CART MOTION
      // ------------------------------------------------------

      if (mode != "CONTROL") {

        cartVelocity += filteredAcceleration * dt;

        cartVelocity = constrain(
          cartVelocity,
          -MAX_CART_VELOCITY,
          MAX_CART_VELOCITY
        );

        cartPosition += cartVelocity * dt;

        // Stop integrating into the track limits
        if (cartPosition >= MAX_CART_DISPLACEMENT) {
          cartPosition = MAX_CART_DISPLACEMENT;
          cartVelocity = 0.0;
        }

        if (cartPosition <= -MAX_CART_DISPLACEMENT) {
          cartPosition = -MAX_CART_DISPLACEMENT;
          cartVelocity = 0.0;
        }

        long targetSteps =
          feedbackStartSteps +
          lround(cartPosition * 1000.0 * STEPS_PER_MM);

        motor.moveTo(targetSteps);
      }


      // ------------------------------------------------------
      // LOG DATA
      // ------------------------------------------------------

      if (millis() - lastLogTime >= LOG_INTERVAL) {

        logData(
          "CONTROL",
          filteredThetaDot,
          cartPosition * 1000.0
        );

        lastLogTime = millis();
      }
    }
  }


  // ----------------------------------------------------------
  // STEP 5: RETURN CART TO ZERO
  // ----------------------------------------------------------

  Serial.println("END");

  motor.moveTo(0);

  while (motor.distanceToGo() != 0) {

    motor.run();
    updateEncoder();
  }

  Serial.println("RETURNED");
}


// ============================================================
// 10. SERIAL COMMAND HANDLING
// ============================================================

void handleCommand(String command) {

  command.trim();

  if (command == "PING") {

    Serial.println("READY");
    return;
  }

  if (command == "ZERO") {

    encoderCount = 0;

    previousEncoderState =
      (digitalRead(ENCODER_A) << 1) |
       digitalRead(ENCODER_B);

    Serial.println("OK,ZERO");
    return;
  }

  if (command.startsWith("TRIAL,")) {

    int c1 = command.indexOf(',');
    int c2 = command.indexOf(',', c1 + 1);
    int c3 = command.indexOf(',', c2 + 1);
    int c4 = command.indexOf(',', c3 + 1);
    int c5 = command.indexOf(',', c4 + 1);

    if (
      c1 < 0 || c2 < 0 || c3 < 0 ||
      c4 < 0 || c5 < 0
    ) {
      Serial.println("ERROR,INVALID_COMMAND");
      return;
    }

    String mode =
      command.substring(c1 + 1, c2);

    float excitationMM =
      command.substring(c2 + 1, c3).toFloat();

    float Kp =
      command.substring(c3 + 1, c4).toFloat();

    float Kd =
      command.substring(c4 + 1, c5).toFloat();

    float durationS =
      command.substring(c5 + 1).toFloat();

    runTrial(
      mode,
      excitationMM,
      Kp,
      Kd,
      durationS
    );

    return;
  }

  Serial.println("ERROR,UNKNOWN_COMMAND");
}


// ============================================================
// 11. SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  previousEncoderState =
    (digitalRead(ENCODER_A) << 1) |
     digitalRead(ENCODER_B);

  motor.setMaxSpeed(MAX_SPEED_STEPS);
  motor.setAcceleration(MAX_ACCEL_STEPS);

  Serial.println("READY");
}


// ============================================================
// 12. MAIN LOOP
// ============================================================

void loop() {

  if (Serial.available()) {

    String command =
      Serial.readStringUntil('\n');

    handleCommand(command);
  }
}
