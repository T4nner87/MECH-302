#include <AccelStepper.h>

// ============================================================
// CART-PENDULUM AUTO-SWEEP FIRMWARE
// ============================================================
// Upload ONCE. After that, everything (distance, speed, accel,
// measurement window) is set from Python over serial, so no
// recompiling between tests.
//
// Based on cart_pendulum_no_interrupts.ino:
//   - same motor setup / AccelStepper moveTo()+run() movement
//   - same polled (no-interrupt) quadrature encoder decoding
//
// START CONDITION: cart at one end of the track (this is x = 0)
// and the pendulum hanging still. The cart only ever moves
// from 0 -> +D -> back to 0.
//
// SERIAL PROTOCOL (115200 baud, newline-terminated lines)
//   PING                      -> READY
//   RUN,<steps>               -> RESULT,<steps>,<peak_pos_deg>,
//                                <peak_neg_deg>,<amp_deg>,<settled>
//   SET,<NAME>,<value>        -> OK,<NAME>,<value>
//       VMAX_RPM, ACCEL_RPM_S, WINDOW_MS,
//       RETURN_RPM, RETURN_ACCEL_RPM_S
//   ZERO                      -> OK,ZERO   (re-define current
//                                cart position + pendulum as 0)
//   Errors are reported as    -> ERR,<reason>
// ============================================================


// -------------------- PINS --------------------
const int STEP_PIN = 9;
const int DIR_PIN  = 7;

const int POLE_ENC_A = 10;
const int POLE_ENC_B = 11;


// -------------------- HARDWARE CONSTANTS --------------------
const float STEPS_PER_REV = 1600.0;          // motor
const float ENCODER_COUNTS_PER_REV = 2400.0; // pendulum encoder
const float DEGREES_PER_COUNT = 360.0 / ENCODER_COUNTS_PER_REV;


// -------------------- SAFETY --------------------
// Longest move the firmware will accept. Set this to the number
// of motor steps that is safely available from the start end of
// the track. 8000 matches the move used in your earlier sketches.
const long MAX_TRAVEL_STEPS = 10810;


// -------------------- SETTLE DETECTION --------------------
// After returning to start, the pendulum is "settled" when its
// encoder count varies by <= SETTLE_BAND_COUNTS over a full
// SETTLE_WINDOW_MS window.
const unsigned long SETTLE_WINDOW_MS  = 1000;
const long          SETTLE_BAND_COUNTS = 4;
const unsigned long SETTLE_TIMEOUT_MS = 90000;


// -------------------- RUNTIME PARAMETERS (set via SET,...) --------------------
float vmaxRpm          = 265.0;   // test move: max speed
float accelRpmS        = 4250.0;  // test move: acceleration
float returnRpm        = 265.0;    // gentle return move: max speed
float returnAccelRpmS  = 4250.0;    // gentle return move: acceleration
unsigned long windowMs = 2000;    // how long to watch the pendulum after the stop
                                  // (make this >= ~1 pendulum period)


// ============================================================

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

long encoderCount = 0;
uint8_t oldEncoderState = 0;


// ============================================================
// CONVERSIONS
// ============================================================

float rpmToSteps(float rpm)
{
    return (rpm * STEPS_PER_REV) / 60.0;
}

void applyMotorParams(float speedRpm, float accelRpmPerS)
{
    motor.setMaxSpeed(rpmToSteps(speedRpm));
    motor.setAcceleration(rpmToSteps(accelRpmPerS));
}


// ============================================================
// POLLED ENCODER (no interrupts, so motor pulses aren't disturbed)
// ============================================================

void updateEncoder()
{
    uint8_t a = digitalRead(POLE_ENC_A);
    uint8_t b = digitalRead(POLE_ENC_B);
    uint8_t newState = (a << 1) | b;

    if (newState == oldEncoderState)
        return;

    if ((oldEncoderState == 0 && newState == 1) ||
        (oldEncoderState == 1 && newState == 3) ||
        (oldEncoderState == 3 && newState == 2) ||
        (oldEncoderState == 2 && newState == 0))
    {
        encoderCount++;
    }
    else if ((oldEncoderState == 0 && newState == 2) ||
             (oldEncoderState == 2 && newState == 3) ||
             (oldEncoderState == 3 && newState == 1) ||
             (oldEncoderState == 1 && newState == 0))
    {
        encoderCount--;
    }

    oldEncoderState = newState;
}


// ============================================================
// MOVE CART (blocking; encoder polled between motor.run() calls)
// ============================================================

void moveCart(long targetPosition)
{
    motor.moveTo(targetPosition);

    while (motor.distanceToGo() != 0)
    {
        motor.run();
        updateEncoder();
    }
}


// ============================================================
// WAIT FOR PENDULUM TO COME TO REST
// Returns true if it settled, false if it timed out.
// ============================================================

bool waitForSettle()
{
    unsigned long overallStart = millis();

    while (millis() - overallStart < SETTLE_TIMEOUT_MS)
    {
        long minCount = encoderCount;
        long maxCount = encoderCount;
        unsigned long windowStart = millis();

        while (millis() - windowStart < SETTLE_WINDOW_MS)
        {
            updateEncoder();

            long c = encoderCount;
            if (c < minCount) minCount = c;
            if (c > maxCount) maxCount = c;
        }

        if ((maxCount - minCount) <= SETTLE_BAND_COUNTS)
            return true;
    }

    return false;
}


// ============================================================
// ONE FULL TRIAL
//   1. Move cart 0 -> steps with the test speed/accel
//   2. Watch pendulum for windowMs after the stop, record peak angles
//   3. Return to 0 gently
//   4. Wait for pendulum to settle, then re-zero the encoder
//   5. Send one RESULT line
// ============================================================

void runTrial(long steps)
{
    // ---- 1. test move ----
    applyMotorParams(vmaxRpm, accelRpmS);
    moveCart(steps);

    // ---- 2. measurement window (starts the instant the cart stops) ----
    long maxPositiveCount = encoderCount;
    long maxNegativeCount = encoderCount;

    unsigned long startTime = millis();

    while (millis() - startTime < windowMs)
    {
        updateEncoder();

        long c = encoderCount;
        if (c > maxPositiveCount) maxPositiveCount = c;
        if (c < maxNegativeCount) maxNegativeCount = c;
    }

    long maxAbsoluteCount = max(maxPositiveCount, -maxNegativeCount);

    // ---- 3. gentle return ----
    applyMotorParams(returnRpm, returnAccelRpmS);
    moveCart(0);

    // ---- 4. settle, then re-zero so encoder drift can't accumulate ----
    bool settled = waitForSettle();

    if (settled)
        encoderCount = 0;

    // ---- 5. report ----
    Serial.print("RESULT,");
    Serial.print(steps);
    Serial.print(",");
    Serial.print(maxPositiveCount * DEGREES_PER_COUNT, 3);
    Serial.print(",");
    Serial.print(maxNegativeCount * DEGREES_PER_COUNT, 3);
    Serial.print(",");
    Serial.print(maxAbsoluteCount * DEGREES_PER_COUNT, 3);
    Serial.print(",");
    Serial.println(settled ? 1 : 0);
}


// ============================================================
// COMMAND HANDLING
// ============================================================

void handleSet(const String &line)
{
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);

    if (c1 < 0 || c2 < 0)
    {
        Serial.println("ERR,SET needs SET,<NAME>,<value>");
        return;
    }

    String key = line.substring(c1 + 1, c2);
    key.trim();
    key.toUpperCase();
    float value = line.substring(c2 + 1).toFloat();

    if (value <= 0)
    {
        Serial.println("ERR,value must be > 0");
        return;
    }

    if      (key == "VMAX_RPM")           vmaxRpm = value;
    else if (key == "ACCEL_RPM_S")        accelRpmS = value;
    else if (key == "WINDOW_MS")          windowMs = (unsigned long)value;
    else if (key == "RETURN_RPM")         returnRpm = value;
    else if (key == "RETURN_ACCEL_RPM_S") returnAccelRpmS = value;
    else
    {
        Serial.print("ERR,unknown parameter ");
        Serial.println(key);
        return;
    }

    Serial.print("OK,");
    Serial.print(key);
    Serial.print(",");
    Serial.println(value, 3);
}

void handleCommand(String line)
{
    line.trim();

    if (line.length() == 0)
        return;

    String upper = line;
    upper.toUpperCase();

    if (upper == "PING")
    {
        Serial.println("READY");
    }
    else if (upper == "ZERO")
    {
        motor.setCurrentPosition(0);
        encoderCount = 0;
        Serial.println("OK,ZERO");
    }
    else if (upper.startsWith("SET,"))
    {
        handleSet(line);
    }
    else if (upper.startsWith("RUN,"))
    {
        long steps = line.substring(4).toInt();

        if (steps <= 0)
        {
            Serial.println("ERR,steps must be > 0");
        }
        else if (steps > MAX_TRAVEL_STEPS)
        {
            Serial.print("ERR,steps exceeds MAX_TRAVEL_STEPS (");
            Serial.print(MAX_TRAVEL_STEPS);
            Serial.println(")");
        }
        else
        {
            runTrial(steps);
        }
    }
    else
    {
        Serial.print("ERR,unknown command ");
        Serial.println(line);
    }
}


// ============================================================
// SETUP / LOOP
// ============================================================

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(50);

    applyMotorParams(vmaxRpm, accelRpmS);
    motor.setCurrentPosition(0);
    motor.setMinPulseWidth(3);

    pinMode(POLE_ENC_A, INPUT_PULLUP);
    pinMode(POLE_ENC_B, INPUT_PULLUP);

    oldEncoderState =
        (digitalRead(POLE_ENC_A) << 1) |
         digitalRead(POLE_ENC_B);

    // Pendulum must be hanging still here: this is encoder = 0.
    encoderCount = 0;

    Serial.println("READY");
}

void loop()
{
    // Keep tracking the pendulum even while idle.
    updateEncoder();

    if (Serial.available())
    {
        String line = Serial.readStringUntil('\n');
        handleCommand(line);
    }
}
