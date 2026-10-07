'''cpp
#include <AccelStepper.h>

const int STEP_PIN = 9;
const int DIR_PIN  = 7;

const float STEPS_PER_REV = 1600.0;

// =====================================================
// USER SETTINGS
// =====================================================

// Maximum motor speed
const float MAX_SPEED_RPM = 265.0;

// MANUALLY SET ACCELERATION HERE
// Units: RPM per second
const float ACCELERATION_RPM_S = 4250.0;

// Distance to travel in each direction
const long DISTANCE_STEPS = 8000;

// =====================================================

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);


// Convert RPM to steps/second
float rpmToSteps(float rpm)
{
    return (rpm * STEPS_PER_REV) / 60.0;
}


// Convert RPM/s to steps/s^2
float rpmPerSecondToSteps(float rpmPerSecond)
{
    return (rpmPerSecond * STEPS_PER_REV) / 60.0;
}


void setup()
{
    Serial.begin(115200);

    // Convert maximum speed
    float maxSpeed =
        rpmToSteps(MAX_SPEED_RPM);

    // Convert acceleration
    float acceleration =
        rpmPerSecondToSteps(ACCELERATION_RPM_S);

    // Set motor parameters
    motor.setMaxSpeed(maxSpeed);
    motor.setAcceleration(acceleration);

    // Print settings
    Serial.println();
    Serial.println("================================");
    Serial.println("MOTOR ACCELERATION TEST");
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
    Serial.println("Starting motor...");
}


void loop()
{
    // ================================================
    // MOVE FORWARD
    // ================================================

    Serial.println("Moving FORWARD...");

    motor.moveTo(DISTANCE_STEPS);

    while (motor.distanceToGo() != 0)
    {
        motor.run();
    }

    Serial.println("Forward movement complete.");

    delay(1000);


    // ================================================
    // MOVE BACKWARD
    // ================================================

    Serial.println("Moving BACKWARD...");

    motor.moveTo(0);

    while (motor.distanceToGo() != 0)
    {
        motor.run();
    }

    Serial.println("Backward movement complete.");

    delay(1000);
}
'''
