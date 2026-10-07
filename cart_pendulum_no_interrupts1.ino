#include <AccelStepper.h>

// =====================================================
// EXACT MOTOR SETTINGS FROM WORKING MOVEMENT CODE
// =====================================================

const int STEP_PIN = 9;
const int DIR_PIN  = 7;

const float STEPS_PER_REV = 1600.0;

const float MAX_SPEED_RPM = 265.0;
const float ACCELERATION_RPM_S = 4250.0;
const long DISTANCE_STEPS = 7277.292;

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// =====================================================
// PENDULUM ENCODER
// =====================================================

const int POLE_ENC_A = 10;
const int POLE_ENC_B = 11;

const float ENCODER_COUNTS_PER_REV = 1200.0;
const float DEGREES_PER_COUNT = 360.0 / ENCODER_COUNTS_PER_REV;

const unsigned long SAMPLE_INTERVAL_US = 5000; // 200 Hz
const int PERIODS_TO_AVERAGE = 5;
const long CROSSING_HYSTERESIS_COUNTS = 3;
const unsigned long MEASUREMENT_TIMEOUT_MS = 15000;

long encoderCount = 0;
uint8_t oldEncoderState = 0;

// =====================================================
// MOTOR CONVERSIONS - SAME AS WORKING CODE
// =====================================================

float rpmToSteps(float rpm)
{
    return (rpm * STEPS_PER_REV) / 60.0;
}

float rpmPerSecondToSteps(float rpmPerSecond)
{
    return (rpmPerSecond * STEPS_PER_REV) / 60.0;
}

// =====================================================
// POLL ENCODER
//
// IMPORTANT:
// No attachInterrupt() is used here.
// This prevents the encoder ISR from interfering with the
// AccelStepper motor pulse timing.
// =====================================================

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

// =====================================================
// MOVE CART
//
// This preserves the working AccelStepper movement.
// Encoder is simply polled between motor.run() calls.
// =====================================================

void moveCart(long targetPosition)
{
    motor.moveTo(targetPosition);

    while (motor.distanceToGo() != 0)
    {
        motor.run();
        updateEncoder();
    }
}

// =====================================================
// MEASURE PENDULUM
// =====================================================

void measurePendulum(const char *label)
{
    Serial.println();
    Serial.println("========================================");
    Serial.print("MEASUREMENT: ");
    Serial.println(label);
    Serial.println("========================================");

    // Use the hanging-zero established in setup.
    // encoderCount has been continuously tracked through the move.
    long previousRelativeCount = encoderCount;

    long maxPositiveCount = encoderCount;
    long maxNegativeCount = encoderCount;
    long maxAbsoluteCount = labs(encoderCount);

    bool wentNegative = false;

    unsigned long lastPositiveCrossingTime = 0;
    unsigned long lastSampleTime = micros();
    unsigned long startTime = millis();

    float periodSum = 0.0;
    int periodCount = 0;

    while (periodCount < PERIODS_TO_AVERAGE &&
           millis() - startTime < MEASUREMENT_TIMEOUT_MS)
    {
        // Poll as fast as possible so quadrature transitions are not missed.
        updateEncoder();

        unsigned long now = micros();

        // Analyze pendulum at 200 Hz.
        if (now - lastSampleTime >= SAMPLE_INTERVAL_US)
        {
            lastSampleTime += SAMPLE_INTERVAL_US;

            long relativeCount = encoderCount;

            // -------------------------------
            // AMPLITUDE
            // -------------------------------

            if (relativeCount > maxPositiveCount)
                maxPositiveCount = relativeCount;

            if (relativeCount < maxNegativeCount)
                maxNegativeCount = relativeCount;

            if (labs(relativeCount) > maxAbsoluteCount)
                maxAbsoluteCount = labs(relativeCount);

            // -------------------------------
            // FREQUENCY
            // -------------------------------

            if (relativeCount <= -CROSSING_HYSTERESIS_COUNTS)
            {
                wentNegative = true;
            }

            if (wentNegative &&
                previousRelativeCount < 0 &&
                relativeCount >= 0)
            {
                unsigned long crossingTime = now;

                if (lastPositiveCrossingTime != 0)
                {
                    float period =
                        (crossingTime - lastPositiveCrossingTime)
                        / 1000000.0;

                    if (period > 0.2 && period < 5.0)
                    {
                        periodCount++;
                        periodSum += period;

                        Serial.print("Oscillation ");
                        Serial.print(periodCount);
                        Serial.print(": Period = ");
                        Serial.print(period, 4);
                        Serial.print(" s | Frequency = ");
                        Serial.print(1.0 / period, 4);
                        Serial.println(" Hz");
                    }
                }

                lastPositiveCrossingTime = crossingTime;
                wentNegative = false;
            }

            previousRelativeCount = relativeCount;
        }
    }

    float positiveAngle =
        maxPositiveCount * DEGREES_PER_COUNT;

    float negativeAngle =
        maxNegativeCount * DEGREES_PER_COUNT;

    float amplitude =
        maxAbsoluteCount * DEGREES_PER_COUNT;

    float peakToPeak =
        (maxPositiveCount - maxNegativeCount)
        * DEGREES_PER_COUNT;

    Serial.println();
    Serial.println("------------- FINAL RESULT -------------");

    Serial.print("Maximum positive angle: ");
    Serial.print(positiveAngle, 3);
    Serial.println(" deg");

    Serial.print("Maximum negative angle: ");
    Serial.print(negativeAngle, 3);
    Serial.println(" deg");

    Serial.print("Amplitude: ");
    Serial.print(amplitude, 3);
    Serial.println(" deg");

    Serial.print("Peak-to-peak: ");
    Serial.print(peakToPeak, 3);
    Serial.println(" deg");

    if (periodCount > 0)
    {
        float averagePeriod = periodSum / periodCount;
        float averageFrequency = 1.0 / averagePeriod;

        Serial.print("Average period: ");
        Serial.print(averagePeriod, 4);
        Serial.println(" s");

        Serial.print("Average frequency: ");
        Serial.print(averageFrequency, 4);
        Serial.println(" Hz");
    }
    else
    {
        Serial.println("Frequency: not enough oscillations detected.");
    }

    Serial.println("========================================");
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    // EXACT motor setup from working movement sketch
    float maxSpeed = rpmToSteps(MAX_SPEED_RPM);
    float acceleration =
        rpmPerSecondToSteps(ACCELERATION_RPM_S);

    motor.setMaxSpeed(maxSpeed);
    motor.setAcceleration(acceleration);

    // Encoder pins
    pinMode(POLE_ENC_A, INPUT_PULLUP);
    pinMode(POLE_ENC_B, INPUT_PULLUP);

    oldEncoderState =
        (digitalRead(POLE_ENC_A) << 1) |
         digitalRead(POLE_ENC_B);

    // Pendulum must be hanging straight down here.
    // We define that physical position as encoderCount = 0.
    encoderCount = 0;

    Serial.println();
    Serial.println("================================");
    Serial.println("MOTOR + PENDULUM TEST");
    Serial.println("================================");

    Serial.print("Maximum speed: ");
    Serial.print(MAX_SPEED_RPM);
    Serial.println(" RPM");

    Serial.print("Maximum speed: ");
    Serial.print(maxSpeed);
    Serial.println(" steps/s");

    Serial.print("Acceleration: ");
    Serial.print(ACCELERATION_RPM_S);
    Serial.println(" RPM/s");

    Serial.print("Acceleration: ");
    Serial.print(acceleration);
    Serial.println(" steps/s^2");

    Serial.print("Distance: ");
    Serial.print(DISTANCE_STEPS);
    Serial.println(" steps");

    Serial.println();
    Serial.println("IMPORTANT:");
    Serial.println("Start with pendulum hanging straight down.");
    Serial.println("Starting in 3 seconds...");
    delay(3000);
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
    // ================================================
    // MOVE FORWARD
    // ================================================

    Serial.println();
    Serial.println("Moving FORWARD...");

    moveCart(DISTANCE_STEPS);

    Serial.println("Forward movement complete.");

    // Measure the oscillation caused by the movement.
    measurePendulum("AFTER FORWARD MOVE");

    delay(1000);

    // ================================================
    // MOVE BACKWARD
    // ================================================

    Serial.println();
    Serial.println("Moving BACKWARD...");

    moveCart(0);

    Serial.println("Backward movement complete.");

    measurePendulum("AFTER BACKWARD MOVE");

    delay(1000);
}
