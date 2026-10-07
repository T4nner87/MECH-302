#include <AccelStepper.h>

// =====================================================
// MOTOR SETTINGS
// =====================================================
const int STEP_PIN = 9;
const int DIR_PIN  = 7;

// Motor: 1600 commanded motor steps = 1 motor revolution
const float MOTOR_STEPS_PER_REV = 1600.0;

const float MAX_SPEED_RPM = 265.0;
const float ACCELERATION_RPM_S = 4250.0;
const long DISTANCE_STEPS = 8000;

// =====================================================
// PENDULUM ENCODER SETTINGS
// =====================================================
const int POLE_ENC_A = 10;
const int POLE_ENC_B = 11;

// Encoder: measured 1200 counts = 360 degrees
const float ENCODER_COUNTS_PER_REV = 1200.0;
const float DEGREES_PER_COUNT = 360.0 / ENCODER_COUNTS_PER_REV;

// Required hardware sampling rate: at least 200 Hz
const unsigned long SAMPLE_INTERVAL_US = 5000;  // 5 ms = 200 Hz

// Number of complete periods to average
const int PERIODS_TO_AVERAGE = 5;

// Crossing hysteresis prevents encoder noise near equilibrium
const long CROSSING_HYSTERESIS_COUNTS = 3;

// =====================================================

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

volatile long encoderCount = 0;

long equilibriumCount = 0;
long previousRelativeCount = 0;

unsigned long lastSampleTime = 0;
unsigned long lastPositiveCrossingTime = 0;

float periodSum = 0.0;
int periodCount = 0;

bool wentNegative = false;
bool testComplete = false;


// =====================================================
// MOTOR CONVERSIONS
// =====================================================

float rpmToSteps(float rpm)
{
    return (rpm * MOTOR_STEPS_PER_REV) / 60.0;
}

float rpmPerSecondToSteps(float rpmPerSecond)
{
    return (rpmPerSecond * MOTOR_STEPS_PER_REV) / 60.0;
}


// =====================================================
// ENCODER INTERRUPT
// Quadrature decoding using both A and B channels
// =====================================================

void readEncoder()
{
    static uint8_t oldState = 0;

    uint8_t a = digitalRead(POLE_ENC_A);
    uint8_t b = digitalRead(POLE_ENC_B);
    uint8_t newState = (a << 1) | b;

    // Valid quadrature transitions
    if ((oldState == 0 && newState == 1) ||
        (oldState == 1 && newState == 3) ||
        (oldState == 3 && newState == 2) ||
        (oldState == 2 && newState == 0))
    {
        encoderCount++;
    }
    else if ((oldState == 0 && newState == 2) ||
             (oldState == 2 && newState == 3) ||
             (oldState == 3 && newState == 1) ||
             (oldState == 1 && newState == 0))
    {
        encoderCount--;
    }

    oldState = newState;
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);
    delay(1500);

    // Motor setup
    motor.setMaxSpeed(rpmToSteps(MAX_SPEED_RPM));
    motor.setAcceleration(rpmPerSecondToSteps(ACCELERATION_RPM_S));

    // Keep cart stationary during natural-frequency measurement
    motor.setCurrentPosition(0);

    // Encoder setup
    pinMode(POLE_ENC_A, INPUT_PULLUP);
    pinMode(POLE_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(POLE_ENC_A), readEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(POLE_ENC_B), readEncoder, CHANGE);

    Serial.println();
    Serial.println("========================================");
    Serial.println("PENDULUM NATURAL FREQUENCY TEST");
    Serial.println("========================================");
    Serial.println("Motor and pendulum encoder are separate:");
    Serial.print("Motor steps/rev: ");
    Serial.println(MOTOR_STEPS_PER_REV, 0);
    Serial.print("Encoder counts/rev: ");
    Serial.println(ENCODER_COUNTS_PER_REV, 0);

    Serial.println();
    Serial.println("Sampling interval: 5.000 ms");
    Serial.println("Sampling rate: 200.0 Hz");

    Serial.println();
    Serial.println("INSTRUCTIONS:");
    Serial.println("1. Leave the pendulum hanging straight down and still.");
    Serial.println("2. Wait for the equilibrium calibration below.");
    Serial.println("3. Pull the pendulum to one side and release it.");
    Serial.println("4. Do NOT push it after release.");
    Serial.println("5. The code will calculate 5 periods and average them.");
    Serial.println();

    // Give user time to leave pendulum hanging at equilibrium
    Serial.println("Calibrating equilibrium in 3 seconds...");
    delay(3000);

    noInterrupts();
    equilibriumCount = encoderCount;
    interrupts();

    previousRelativeCount = 0;

    Serial.print("Equilibrium encoder count = ");
    Serial.println(equilibriumCount);
    Serial.println();
    Serial.println("NOW DISPLACE THE PENDULUM AND RELEASE IT.");
    Serial.println("Waiting for oscillations...");
    Serial.println();

    lastSampleTime = micros();
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
    if (testComplete)
    {
        return;
    }

    unsigned long now = micros();

    if (now - lastSampleTime >= SAMPLE_INTERVAL_US)
    {
        lastSampleTime += SAMPLE_INTERVAL_US;

        long countCopy;

        noInterrupts();
        countCopy = encoderCount;
        interrupts();

        long relativeCount = countCopy - equilibriumCount;
        float angleDegrees = relativeCount * DEGREES_PER_COUNT;

        // Once the pendulum has clearly gone to the negative side,
        // arm the detector for the next positive-going equilibrium crossing.
        if (relativeCount <= -CROSSING_HYSTERESIS_COUNTS)
        {
            wentNegative = true;
        }

        // Detect an equilibrium crossing from negative to positive.
        // Same-direction crossings are one full period apart.
        if (wentNegative &&
            previousRelativeCount < 0 &&
            relativeCount >= 0)
        {
            unsigned long crossingTime = now;

            if (lastPositiveCrossingTime != 0)
            {
                float period =
                    (crossingTime - lastPositiveCrossingTime) / 1000000.0;

                // Ignore obviously invalid detections
                if (period > 0.2 && period < 5.0)
                {
                    periodCount++;
                    periodSum += period;

                    float frequency = 1.0 / period;

                    Serial.print("Oscillation ");
                    Serial.print(periodCount);
                    Serial.print(": Period = ");
                    Serial.print(period, 4);
                    Serial.print(" s, Frequency = ");
                    Serial.print(frequency, 4);
                    Serial.println(" Hz");
                }
            }

            lastPositiveCrossingTime = crossingTime;
            wentNegative = false;

            // Once enough periods have been measured, print final result.
            if (periodCount >= PERIODS_TO_AVERAGE)
            {
                float averagePeriod = periodSum / periodCount;
                float naturalFrequency = 1.0 / averagePeriod;

                Serial.println();
                Serial.println("========================================");
                Serial.println("FINAL RESULTS");
                Serial.println("========================================");

                Serial.print("Average natural period: ");
                Serial.print(averagePeriod, 4);
                Serial.println(" s");

                Serial.print("Average natural period: ");
                Serial.print(averagePeriod * 1000.0, 1);
                Serial.println(" ms");

                Serial.print("Natural frequency: ");
                Serial.print(naturalFrequency, 4);
                Serial.println(" Hz");

                Serial.println();
                Serial.println("Sampling rate: 200.0 Hz");
                Serial.println("Sampling interval: 5.000 ms");
                Serial.println("========================================");

                testComplete = true;
            }
        }

        previousRelativeCount = relativeCount;

        // Uncomment these lines if you want live angle data:
        /*
        Serial.print(now / 1000000.0, 4);
        Serial.print(",");
        Serial.println(angleDegrees, 3);
        */
    }
}
