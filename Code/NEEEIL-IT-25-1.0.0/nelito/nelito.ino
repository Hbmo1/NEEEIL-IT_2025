#include "nelito.h"
#include "pio_encoder.h"
#include "quadrature.pio.h"

float v_linear = 10000;
float v_angular = 0;

float K_motor2 = 1.2; // ez set at 10000

void setup() {
  Serial.begin(115200);

  motor_init();

  for (float t = 0; t <= v_linear; t += 50) {
      set_motor(t * K_motor2, t);
      Serial.print(t);
      delay(10);
    }
  set_motor(v_linear * K_motor2, v_linear);
}

void loop() {  
  //currentMillis = millis();
  //if (currentMillis - previousMillis >= 10) { // Run at ~100Hz
  //}
}