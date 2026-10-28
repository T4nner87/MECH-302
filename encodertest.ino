
const uint8_t ENCODER_A = 10;
const uint8_t ENCODER_B = 11;

// Example: 600 pulses/revolution, counting both A-channel edges
const float COUNTS_PER_REV = 1200.0;
const unsigned long SAMPLE_TIME_MS = 10;

long encoderCount = 0;
int previousA;

long previousCount = 0;
float previousOmega = 0.0;

void setup()
{
    Serial.begin(115200);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    previousA = digitalRead(ENCODER_A);
}

void loop()
{
    // Read encoder
    int currentA = digitalRead(ENCODER_A);

    if (currentA != previousA)
    {
        int currentB = digitalRead(ENCODER_B);

        if (currentA == currentB)
            encoderCount++;
        else
            encoderCount--;

        previousA = currentA;
    }

    // Calculate motion at regular intervals
    static unsigned long lastPrint = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastPrint >= SAMPLE_TIME_MS)
    {
        float dt = (currentTime - lastPrint) / 1000.0;
        lastPrint = currentTime;

        // Angular position (radians)
        float theta = encoderCount * (2.0 * PI / COUNTS_PER_REV);

        // Angular velocity (rad/s)
        float omega = (encoderCount - previousCount)
                      * (2.0 * PI / COUNTS_PER_REV) / dt;

        // Angular acceleration (rad/s^2)
        float alpha = (omega - previousOmega) / dt;

        // Print results
        Serial.print("Count = ");
        Serial.print(encoderCount);

        Serial.print(" | Angle = ");
        Serial.print(theta, 4);

        Serial.print(" rad | Angular Velocity = ");
        Serial.print(omega, 4);

        Serial.print(" rad/s | Angular Acceleration = ");
        Serial.print(alpha, 4);

        Serial.println(" rad/s^2");

        // Save values for next calculation
        previousCount = encoderCount;
        previousOmega = omega;
    }
}
