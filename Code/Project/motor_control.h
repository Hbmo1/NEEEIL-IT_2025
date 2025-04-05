#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// ESC configuration
extern const int escA, escB, escC, escD;
extern const int pwmFreq, pwmResolution;
extern const float MAX_THROTTLE;
extern const int dutyMin, dutyMax;

// Function declarations
void setThrottle(float throttlePercentA, float throttlePercentB, float throttlePercentC, float throttlePercentD);
void setupMotors();
void calibrateArmMotors();
void armMotors();
void rampMotors(float throttle);

#endif