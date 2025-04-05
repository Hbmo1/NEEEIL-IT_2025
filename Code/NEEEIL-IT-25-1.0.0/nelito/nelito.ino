#include "nelito.h"
#include "pio_encoder.h"
#include "quadrature.pio.h"

float v_linear = 10000;
float w = 0;

float K_motor_dir = 0.951; // ez set at 10000

float Kp = 1.5, Ki = 0.5, Kd = 1; // Kc = 0.5

float PWM_init = 0; // descobrir!!!

// void computePID(float desired_w) {
  
//   float w =  
// }

// float getCurrentSpeed()

void setup() {
  Serial.begin(115200);

  motor_init();

  for (float t = 0; t <= v_linear; t += 10) {
      set_motor(t * K_motor_dir, t);
      Serial.println(t);
      delay(10);
    }
  set_motor(v_linear * K_motor_dir, v_linear);
}

void loop() {  
  //currentMillis = millis();
  //if (currentMillis - previousMillis >= 10) { // Run at ~100Hz
  //}
}

//amarelo ; vermelho