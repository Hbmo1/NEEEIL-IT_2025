#include "esp32-hal-ledc.h"
#include "motor_control.h"

// ESC pin assignments
const int escA = 2;
const int escB = 4;
const int escC = 13;
const int escD = 15;

// PWM settings
const int pwmFreq = 50;
const int pwmResolution = 16;
const float MAX_THROTTLE = 100;

// Duty cycle values for ESC control
const int dutyMin = 3277;
const int dutyMax = 6554;

// Maps throttle percentage (0-100) to PWM duty cycle
void setThrottle(float throttlePercentA, float throttlePercentB, float throttlePercentC, float throttlePercentD) {
    int dutyA = map(throttlePercentA, 0, 100, dutyMin, dutyMax);
    int dutyB = map(throttlePercentB, 0, 100, dutyMin, dutyMax);
    int dutyC = map(throttlePercentC, 0, 100, dutyMin, dutyMax);
    int dutyD = map(throttlePercentD, 0, 100, dutyMin, dutyMax);
    
    ledcWrite(escA, dutyA);
    ledcWrite(escB, dutyB);
    ledcWrite(escC, dutyC);
    ledcWrite(escD, dutyD);
}

// Setup function for motors
void setupMotors() {
    Serial.begin(115200);
    Serial.println("Initializing ESC PWM control...");

    ledcAttach(escA, pwmFreq, pwmResolution);
    ledcAttach(escB, pwmFreq, pwmResolution);
    ledcAttach(escC, pwmFreq, pwmResolution);
    ledcAttach(escD, pwmFreq, pwmResolution);
}

void calibrateArmMotors() {
    Serial.println("Step 1: Max throttle for 10 seconds");
    setThrottle(100, 100, 100, 100);
    delay(10000);

    Serial.println("Step 2: Zero throttle for 8 seconds");
    setThrottle(0, 0, 0, 0); // Zero throttle (min pulse, ~1000µs)
    delay(8000);

    Serial.println("ESC should now be calibrated and armed.");
}

void armMotors() {
    setThrottle(10, 10, 10, 10);
    delay(200);

    // Waiting for the initiation sequence
    Serial.println("Step 1: Zero throttle for 4 seconds (waiting for startup beeps)...");
    setThrottle(0, 0, 0, 0); // Zero throttle (min pulse, ~1000µs)
    delay(4000);

    Serial.println("ESC should now be armed.");
}

void rampMotors(float throttle) {
    Serial.println("Ramping up...");
    for (float t = 0; t <= throttle; t += 0.2) {
      setThrottle(t, t, t, t);
      delay(10);
    }
}