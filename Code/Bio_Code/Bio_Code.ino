#include "FastIMU.h"  // Library to read IMU data
#include <Wire.h>
#include <math.h>
#include "MadgwickAHRS.h" // filter library

#include <SPI.h>  // For Lora communication
#include <LoRa.h>

#include <motor_control.h> // A library I created to control the motors

Madgwick filter;  // Creation of madgwick filter

#define IMU_ADDRESS 0x68    // The I2C address of the IMU
MPU6500 IMU;               // Change to the name of any supported IMU!

calData calib = { 0 };  // Calibration data
AccelData accelData;    // Sensor data
GyroData gyroData;

unsigned long previousMillis = 0;   // To get the time between the readings
unsigned long currentMillis = 0;
float dt = 0.01;  // 100Hz

float roll = 0, pitch = 0, yaw = 0;
float a_roll = 0, a_pitch = 0;

float biasGyro_X = 0, biasGyro_Y = 0, biasGyro_Z = 0;
float initialRoll = 0, initialPitch = 0, initialYaw = 0;

float newBaseThrottle = 0, baseThrottle = 0;

bool armed = false;
// ---------- PID -----------
// gains
float Kp = 1.5, Ki = 0.5, Kd = 1, Kc = 0.5;
// PID variables
float error_roll = 0, previous_error_roll = 0, integral_roll = 0;
float error_pitch = 0, previous_error_pitch = 0, integral_pitch = 0;

// Define a limit for the integral term
const float integralLimit = 2.0 * 0.01; // 10% of max output

void computePID(float desired_roll, float desired_pitch) {
    // Get the current orientation from the Madgwick filter
    float roll = filter.getRoll() - initialRoll;
    float pitch = filter.getPitch() - initialPitch;

    // Compute errors
    error_roll = desired_roll - roll;
    error_pitch = desired_pitch - pitch;

    // Integral (accumulated error) with anti-windup
    integral_roll += (previous_error_roll + error_roll)* dt / 2;
    integral_pitch += (previous_error_pitch + error_pitch)* dt / 2;

    // Constrain the integral term to prevent windup
    integral_roll = constrain(integral_roll, -integralLimit, integralLimit);
    integral_pitch = constrain(integral_pitch, -integralLimit, integralLimit);

    // if (abs(error_roll) == 0) integral_roll = 0;
    // if (abs(error_pitch) == 0) integral_pitch = 0;

    // Derivative (change in error)
    float derivative_roll = (error_roll - previous_error_roll)/dt;
    float derivative_pitch = (error_pitch - previous_error_pitch)/dt;

    // Compute PID outputs
    float pid_roll = (Kp * error_roll + Ki * integral_roll + Kd * derivative_roll) * Kc;
    float pid_pitch = (Kp * error_pitch + Ki * integral_pitch + Kd * derivative_pitch) * Kc;

    // Save previous errors
    previous_error_roll = error_roll;
    previous_error_pitch = error_pitch;

    // Apply PID output to motor speeds (you need to map these values correctly)
    adjustMotorSpeeds(pid_roll, pid_pitch);
}

void adjustMotorSpeeds(float pid_roll, float pid_pitch) {
    // float baseThrottle = 95; // Mid-point throttle value, 10% of 20% max

    // Adjust motor speeds based on PID outputs
    float motorA = baseThrottle + pid_pitch + pid_roll;  // Front Right
    float motorB = baseThrottle + pid_pitch - pid_roll;  // Back Right
    float motorC = baseThrottle - pid_pitch - pid_roll;  // Back Left
    float motorD = baseThrottle - pid_pitch + pid_roll;  // Front Left
    
    // Ensure motor values are within safe limits
    float minThrottle = 0, maxThrottle = 100;
    motorA = constrain(motorA, minThrottle, maxThrottle);
    motorB = constrain(motorB, minThrottle, maxThrottle);
    motorC = constrain(motorC, minThrottle, maxThrottle);
    motorD = constrain(motorD, minThrottle, maxThrottle);

    // Send the calculated speed to ESCs
    if (armed) {
      setThrottle(motorA, motorB, motorC, motorD);
    }
    Serial.print("mA:");        // Print filtered values 
    Serial.print(motorA);
    Serial.print("\tmB:");
    Serial.print(motorB);
    Serial.print("\tmC:");
    Serial.print(motorC);
    Serial.print("\tmD:");
    Serial.println(motorD);
}

void calibrateGyro() {
  int countCal = 0;
  float sumGyro_X = 0, sumGyro_Y = 0, sumGyro_Z = 0;

  unsigned long timerCal = millis();
  const int calibrationTime = 10000; // Collect data for 10 seconds

  Serial.println("Calibrating Gyro... Keep IMU still.");

  while ((millis() - timerCal) < calibrationTime) {
    currentMillis = millis();  
    if (currentMillis - previousMillis >= 20) {  // Ensure stable sampling at 50Hz
      IMU.update();
      IMU.getGyro(&gyroData);

      sumGyro_X += gyroData.gyroX;
      sumGyro_Y += gyroData.gyroY;
      sumGyro_Z += gyroData.gyroZ;
      
      countCal++;

      Serial.print(countCal); Serial.print("\t");
      Serial.print(sumGyro_X); Serial.print("\t");
      Serial.print(sumGyro_Y); Serial.print("\t");
      Serial.println(sumGyro_Z); Serial.print("\t");

      previousMillis = currentMillis;
    }
  }

  // Compute the average bias
  biasGyro_X = sumGyro_X / countCal;
  biasGyro_Y = sumGyro_Y / countCal;
  biasGyro_Z = sumGyro_Z / countCal;

  Serial.println("Gyro Calibration Complete!");
  Serial.print("Bias X: "); Serial.println(biasGyro_X);
  Serial.print("Bias Y: "); Serial.println(biasGyro_Y);
  Serial.print("Bias Z: "); Serial.println(biasGyro_Z);
}

void setup() {
  Wire.begin();
  Wire.setClock(400000); //400khz clock
  Serial.begin(115200);

  setupMotors();

  Serial.println("Starting LoRa...");

  // Set the correct pins for the T-Beam v1.1
  LoRa.setPins(18, 14, 26);
  LoRa.setSpreadingFactor(7);  // Fastest SF
  LoRa.setSignalBandwidth(250E3);  // 250 kHz (Allowed in Europe)
  LoRa.setCodingRate4(5);  // 4/5 coding rate
  LoRa.enableCrc();  // Enable CRC for error checking

  if (!LoRa.begin(868E6)) {     // Check LoRa connection
    Serial.println("Starting LoRa failed!");
    while (true) {;}
  }
  Serial.println("SUCCESS");

  int err = IMU.init(calib, IMU_ADDRESS);   // Initializing IMU
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {;}
  }
  
  IMU.calibrateAccelGyro(&calib); // Library calibration
  calibrateGyro();                // My implementation

  //err = IMU.setGyroRange(500);      // USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       // THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
 
  if (err != 0) {
    Serial.print("IMU Error Setting range: ");
    Serial.println(err);
    while (true) {;}
  }

  filter.begin(1/dt);  // Set the expected IMU update rate, ~100Hz

  Serial.println("Calibrating starting orientation...");

  for (int i = 0; i < 1000; i++) {  
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    float gx = (gyroData.gyroX - biasGyro_X) * (PI / 180.0);
    float gy = (gyroData.gyroY - biasGyro_Y) * (PI / 180.0);
    float gz = (gyroData.gyroZ - biasGyro_Z) * (PI / 180.0);
    
    filter.updateIMU(gx, gy, gz, accelData.accelX, accelData.accelY, accelData.accelZ);
    delay(10);
  }

  // Store initial offsets
  initialRoll = filter.getRoll();
  initialPitch = filter.getPitch();
  initialYaw = filter.getYaw();
  Serial.println("Calibration complete.");
}

void loop() {  
  currentMillis = millis();
  if (currentMillis - previousMillis >= dt / 1000) { // Run at ~100Hz

    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    // Esta parte está estranha...
    float gx = (gyroData.gyroX - biasGyro_X) * 0.8;  // Gyro data with the bias compensation-----------------------------------------------------
    float gy = (gyroData.gyroY - biasGyro_Y) * (PI / 180.0) * 1.5;  //converted from DPS to rad/s.
    float gz = (gyroData.gyroZ - biasGyro_Z) * (PI / 180.0) * 1.5;
    
    filter.updateIMU(gx, gy, gz, accelData.accelX, accelData.accelY, accelData.accelZ);

    float roll = filter.getRoll() - initialRoll;      // Remove inicial random rotation------------------------------------------
    float pitch = filter.getPitch() - initialPitch;
    float yaw = filter.getYaw() - initialYaw;

    Serial.print("Roll:");        // Print filtered values 
    Serial.print(roll);
    Serial.print("\tPitch:");
    Serial.print(pitch);
    Serial.print("\tYaw:");
    Serial.print(yaw); Serial.print("\t");
    Serial.print(Kp); Serial.print("\t");
    Serial.print(Ki); Serial.print("\t");
    Serial.print(Kd); Serial.print("\t");
    Serial.println(Kc);
  
    computePID(0,0);
    previousMillis = currentMillis;
  }

  // Try to parse LoRa packet, used for FailSafe
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Read packet
    while (LoRa.available()) {
      String received = LoRa.readString();
      if(received == "STOP"){
        Serial.println("--FAILSAFE ACTIVE--\nStopping all motors.");
        while(1);
      }
      if(received == "ARM" && !armed){
        armMotors();
        rampMotors(baseThrottle);
        armed = true;
      }
      if(received == "CALIBRATE" && !armed){
        calibrateArmMotors();
        rampMotors(baseThrottle);
        armed = true;
      }
      else if (received.indexOf("kp") >= 0) {
        Kp = received.substring(2).toFloat();
      }
      else if (received.indexOf("ki") >= 0) {
        Ki = received.substring(2).toFloat();
      }
      else if (received.indexOf("kd") >= 0) {
        Kd = received.substring(2).toFloat();
      }
      else if (received.indexOf("kc") >= 0) {
        Kc = received.substring(2).toFloat();
      }
      else if (received.indexOf("bt") >= 0) {
        if (armed) {
          newBaseThrottle = received.substring(2).toFloat();
        }
      }
      Serial.println(received);
    }
    Serial.println();
  }

  if (newBaseThrottle > baseThrottle){
    baseThrottle += 1;
  }
  else if (newBaseThrottle < baseThrottle){
    baseThrottle -= 1;
  }
}