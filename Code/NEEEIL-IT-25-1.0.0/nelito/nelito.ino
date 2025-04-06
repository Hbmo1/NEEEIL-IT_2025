#include "nelito.h"
#include "pio_encoder.h"
#include "quadrature.pio.h"

float v_linear = 20000;
float Vmax = 20000;

float w = 0;

float Kp = 4300, Kd = 600, Ki = 0, Ks = 0.951, Kc = 0.52;

float PWM_init = 0; // descobrir!!!

float linha = 0;
const int IR1 = 10;
const int IR2 = 8;
const int IR3 = 7;
const int IR4 = 6;
const int IR5 = 5;

bool sensorArray[5] = {1, 1, 0, 1, 1}; // init with middle sensor on

long int currentMillis = 0;
long int previousMillisLoop = 0;
long int previousMillisMean = 0;

float current_line = 0;
float current_w = 0;

float prev_line = 0;
float prev_weight = 0;
float prev_active_pins = 1;
float four_prev_weights[4] = {0};
float four_prev_active_pins[4] = {1, 1, 1, 1};

float v_motor_a = 0;
float v_motor_b = 0;

float pondered_weight = 0;

void fifo_push(float *fifo, float new_value, int size) {
    for (int i = size - 1; i > 0; i--) {
        fifo[i] = fifo[i - 1];
    }
    fifo[0] = new_value;
}


float compute_line() {
  sense_read(sensorArray);
  Serial.print(sensorArray[0]);
  Serial.print(sensorArray[1]);
  Serial.print(sensorArray[2]);
  Serial.print(sensorArray[3]);
  Serial.println(sensorArray[4]);

  float total_weight = 0;
  float active_pins = 0;

  if (!sensorArray[0]) {
    total_weight -= 2;
    active_pins++;

  }

  if (!sensorArray[1]) {
    total_weight -= 1;
    active_pins++;
  }

  if (!sensorArray[2]) {
    total_weight -= 0;
    active_pins++;
  }

  if (!sensorArray[3]) {
    total_weight += 1;
    active_pins++;
  }

  if (!sensorArray[4]) {
    total_weight += 2;
    active_pins++;
  }

  if (active_pins == 0) { 
    prev_active_pins = active_pins;
    if (prev_weight > 0) { 
      return 2;                       // strafe as far right as you can
    } else if (prev_weight < 0) { 
      return -2;                      // strafe as far left as you can
    } else {            
      return 0;
    }
  }

  if (active_pins == 5) {
    if (prev_weight == 0.5 && prev_active_pins == 4) {
      prev_active_pins = active_pins;
      return 2;
    } else if (prev_weight == -0.5 && prev_active_pins == 4) {
      prev_active_pins = active_pins;
      return -2;
    }
    if (prev_weight > 0) {            
      prev_active_pins = active_pins;
      return -2;                      // strafe as far left as you can (opp direction)
    } else if (prev_weight < 0) {     
      prev_active_pins = active_pins;
      return 2;                       // strafe as far right as you can (opp direction)
    } else {            
      prev_active_pins = active_pins;
      return 0;
    }
  }   

  Serial.println(total_weight / active_pins);
  prev_weight = total_weight / active_pins;
  prev_active_pins = active_pins;

  // if (active_pins == 0) { 
  //   pondered_weight = (total_weight + four_prev_weights[0] + four_prev_weights[1] + four_prev_weights[2] + four_prev_weights[3]) / (active_pins + four_prev_active_pins[0] + four_prev_active_pins[1] + four_prev_active_pins[2] + four_prev_active_pins[3]);
  //   // return prev_weight;
  //   if (pondered_weight > 0) {            // 
  //     return 2;                      // strafe as far left as you can (opp direction)
  //   } else if (pondered_weight < 0) {     //
  //     return -2;                       // strafe as far right as you can (opp direction)
  //   } else {            
  //     return 0;
  //   }
  // }


  // if (active_pins == 5) { 

  //   pondered_weight = - (total_weight + four_prev_weights[0] + four_prev_weights[1] + four_prev_weights[2] + four_prev_weights[3]) / (active_pins + four_prev_active_pins[0] + four_prev_active_pins[1] + four_prev_active_pins[2] + four_prev_active_pins[3]);
  //   // return prev_weight;
  //   if (pondered_weight > 0) {           
  //     return -2;                      // strafe as far left as you can (opp direction)
  //   } else if (pondered_weight < 0) {    
  //     return 2;                       // strafe as far right as you can (opp direction)
  //   } else {            
  //     return 0;
  //   }
  // }

  // if (currentMillis - previousMillisMean >= 200) {
  //   fifo_push(four_prev_weights, total_weight, 4);
  //   fifo_push(four_prev_active_pins, active_pins, 4);
  //   previousMillisMean = currentMillis;
  // }
 
  return total_weight / active_pins;
}

float compute_PID(float line) {
  float w = Kp * line + Kd * (line - prev_line)*200 + Ki * (line + prev_line)/(2 * 200);
  prev_line = line;
  return w;
}

void setup() {
  Serial.begin(115200);

  motor_init();
  sense_init(IR1, IR2, IR3, IR4, IR5);
  analogWrite(12,75); //shhhhhh
  for (float t = 5000; t <= v_linear; t += 250) {   // Initialize motor
      set_motor(t * Ks, t);
      Serial.println(t);
      delay(10);
    }

}


void loop() {  
  currentMillis = millis();
  if (currentMillis - previousMillisLoop >= 10) { // Run at ~100Hz
    current_line = compute_line();
    current_w = compute_PID(current_line);
    v_linear = Vmax * cos(Kc * current_w);
    v_motor_a = v_linear * Ks - current_w;
    v_motor_b = v_linear + current_w;

    if (v_motor_a <= 0 && v_motor_a >= -8700 * Ks) {
      v_motor_a = -8700 * Ks;
    } else if (v_motor_a >= 0 && v_motor_a <= 8700 * Ks) {
      v_motor_a = 8700 * Ks;
    }
    if (v_motor_b <= 0 && v_motor_b >= -8700) {
      v_motor_b = -8700;
    } else if (v_motor_b >= 0 && v_motor_b <= 8700) {
      v_motor_b = 8700;
    }
    set_motor(v_motor_a, v_motor_b);  // Straight line with current_w changes to adapt to line
    previousMillisLoop = currentMillis;
  }
}

// GPIOs: 5,6,7,8,10