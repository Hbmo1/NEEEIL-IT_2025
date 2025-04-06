#include "nelito.h"
#include "pio_encoder.h"
#include "quadrature.pio.h"


float v_linear = 18000;
float Vmax = 18000;

float w = 0;

float Kp = 4050, Kd = 455, Ki = 60, Ks = 0.951, Kc = 0.00011; // Kp_antes = 4000, Kd = 450 -> 0.0011 <= Kc <= 0.0033 

float PWM_init = 9200; 
float linha = 0;

bool enable_delay = 0;

const float DT = 10;
const int IR1 = 10;
const int IR2 = 8;
const int IR3 = 7;
const int IR4 = 6;
const int IR5 = 5;

const int shortcutPin = 11;

bool sensorArray[5] = {1, 1, 0, 1, 1}; // init with middle sensor on

long int currentMillis = 0;
long int previousMillis = 0;
long int previousMillisShortcut = 0;  // Use to avoid detecting the same shortcut immediately after using it
long int foundLineMillis = -100000;    // Init at -10000 to enable identification immediately after start

float current_line = 0;
float current_w = 0;

float prev_line = 0;

float v_motor_a = 0;
float v_motor_b = 0;

float prev_weight = 0;
float prev_active_pins  = 1;

bool allSensorsOFF() {
  for (int i = 0; i < 5; i++) {
    if (sensorArray[i] != true) { // or sensorArray[i] != 1
      return false;
    }
  }
  return true;
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

  // if (!sensorArray[0] && !sensorArray[1] && sensorArray[2] && !sensorArray[3] && sensorArray[4]) { // ON ON OFF ON OFF
  //   enable_delay = 1;
  //   return 5.5;   // Very sharp-- curve right
  // }

  // // if (!sensorArray[0] && !sensorArray[1] && sensorArray[2] && sensorArray[3] && !sensorArray[4]) { // ON ON OFF OFF ON
  // //   enable_delay = 1;
  // //   return 4;     // Sharp curve right
  // // }

  // if (sensorArray[0] && !sensorArray[1] && !sensorArray[2] && sensorArray[3] && !sensorArray[4]) { // OFF ON ON OFF ON
  //   enable_delay = 1;
  //   return 6;     // Very sharp curve right
  // }

  // if (!sensorArray[0] && !sensorArray[1] && !sensorArray[2] && sensorArray[3] && !sensorArray[4]) { // ON ON ON OFF ON 
  //   enable_delay = 1;
  //   return 6.5;   // Very sharp++ curve right
  // } 

  // // if (sensorArray[0] && !sensorArray[1] && !sensorArray[2] && !sensorArray[3] && !sensorArray[4]) { // OFF ON ON ON ON 
  // //   enable_delay = 1;
  // //   return 6.5;   // Very sharp++ curve right
  // // }   

  // if (sensorArray[0] && !sensorArray[1] && sensorArray[2] && !sensorArray[3] && !sensorArray[4]) { // OFF ON OFF ON ON
  //   enable_delay = 1;
  //   return -5.5;  // Very sharp-- curve left
  // }

  // // if (!sensorArray[0] && sensorArray[1] && sensorArray[2] && !sensorArray[3] && !sensorArray[4]) { // ON OFF OFF ON ON
  // //   enable_delay = 1;
  // //   return -5;    // Sharp curve left
  // // }

  // if (!sensorArray[0] && sensorArray[1] && !sensorArray[2] && !sensorArray[3] && sensorArray[4]) { // ON OFF ON ON OFF
  //   enable_delay = 1;
  //   return -6;    // Very sharp curve left 
  // }

  // if (!sensorArray[0] && sensorArray[1] && !sensorArray[2] && !sensorArray[3] && !sensorArray[4]) { // ON OFF ON ON ON
  //   enable_delay = 1;
  //   return -6.5;    // Very sharp++ curve left 
  // }

  // // if (!sensorArray[0] && !sensorArray[1] && !sensorArray[2] && !sensorArray[3] && sensorArray[4]) { // ON ON ON ON OFF
  // //   enable_delay = 1;
  // //   return -6.5;    // Very sharp++ curve left 
  // // }

  // if (sensorArray[0] && !sensorArray[1] && !sensorArray[2] && sensorArray[3] && !sensorArray[4]) { // ON OFF ON ON PFF
  //   enable_delay = 1;
  //   return 6; 
  // }

  if (!sensorArray[0]) {
    total_weight -= 5;
    active_pins++;
  }

  if (!sensorArray[1]) {
    total_weight -= 1.5;
    active_pins++;
  }

  if (!sensorArray[2]) {
    total_weight -= 0;
    active_pins++;
  }

  if (!sensorArray[3]) {
    total_weight += 1.5;
    active_pins++;
  }

  if (!sensorArray[4]) {
    total_weight += 5;
    active_pins++;
  }

  if (active_pins == 0 || active_pins == 5) { 
    // return prev_weight;
    if (prev_weight > 0) {            // 
      return 6.2;                       // strafe as far right as you can
    } else if (prev_weight < 0) {     //
      return -6.2;                      // strafe as far left as you can
    } else {            
      return 0;
    }
  }

  // Serial.println(total_weight / active_pins);
  if (total_weight / active_pins != 0) {
    prev_weight = total_weight / active_pins;
  }
  
  return total_weight / active_pins;

  // if (sensorArray[0] == 0 && sensorArray[1] == 1 && sensorArray[2] == 1 && sensorArray[3] == 1 && sensorArray[4] == 0) {          // 11 -> [0,1,1,1,0]  
  //   return 0    // straight ahead captain!
  // } else if (sensorArray[0] == 0 && sensorArray[1] == 1 && sensorArray[2] == 1 && sensorArray[3] == 0 && sensorArray[4] == 0) {   // 10 -> [0,1,1,0,0]
  //   return -0.25   // slight turn left
  // } else if (sensorArray[0] == 0 && sensorArray[1] == 0 && sensorArray[2] == 1 && sensorArray[3] == 1 && sensorArray[4] == 0) {   // 13 -> [0,0,1,1,0]
  //   return 
  // }
}

float compute_PID(float line) {


  if (abs(line) == 0) Ki = 0;

  float w = Kp * line + Kd * (line - prev_line)*100 + constrain(Ki * (line + prev_line)/(DT*2), -2000, 2000);
  prev_line = line;

  return w;

}

void setup() {
  Serial.begin(115200);

  pinMode(shortcutPin, INPUT);

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
  if (currentMillis - previousMillis >= DT) { // Run at ~100Hz
    current_line = compute_line();
    current_w = compute_PID(current_line);
    v_linear = Vmax * cos(Kc * current_w);
    Serial.print(cos(Kc*current_w));
    // v_motor_b = v_linear + current_w;
    // v_motor_a = v_linear * Ks - current_w;
    if (current_w >= 0) {
      v_motor_b = v_linear + current_w;
      v_motor_a = v_linear * Ks;
    } else {
      v_motor_b = v_linear;
      v_motor_a = v_linear * Ks - current_w;
    }  

    if (v_motor_a <= 0 && v_motor_a >= -PWM_init) {
      v_motor_a = -PWM_init;
    } else if (v_motor_a >= 0 && v_motor_a <= PWM_init) {
      v_motor_a = PWM_init;
    }
    if (v_motor_b <= 0 && v_motor_b >= -PWM_init) {
      v_motor_b = -PWM_init;
    } else if (v_motor_b >= 0 && v_motor_b <= PWM_init) {
      v_motor_b = PWM_init;
    }
    set_motor(v_motor_a, v_motor_b);  // Straight line with current_w changes to adapt to line
    previousMillis = currentMillis;
  }
  
  if (!digitalRead(shortcutPin) && currentMillis - foundLineMillis >= 8000) {
    delay(60);
    set_motor(17000, 5000);
    while (!allSensorsOFF()) {
      sense_read(sensorArray);
    }
    delay(10);
    while (allSensorsOFF()){
      sense_read(sensorArray);
      foundLineMillis = millis();
    }
  }

  // if (enable_delay) {
  //   delay(250);       // Delay if edge cases found
  //   enable_delay = 0;
  // }
  // for (float t = 0; t <= v_linear; t += 5) {
  //     set_motor(t * Ks, t);
  //     Serial.println(t);
  //     delay(10);
  //   }
  // set_motor(v_linear * Ks, v_linear);

  // delay (3000);

  // for (float t = v_linear; t >= 0; t -= 5) {
  //     set_motor(t * Ks, t);
  //     Serial.println(t);
  //     delay(10);
  //   }
  // set_motor(0, 0);

  // delay (3000);
}

//amarelo ; vermelho

// GPIOs: 5,6,7,8,10