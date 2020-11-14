#include "configs.h"
#include "simulator.h"
#include <math.h>
#include <string.h>
//  /home/gnl/.platformio/penv/bin/platformio device monitor

int counter = 0;
int pwm_pos = 0;
float gain = 0;

float v_i = 0;
float t_i = 0;


int serial_read_pwm ();
float get_lux ();
float calc_gain ();
float get_voltage();

Simulator* sim;

void setup() {
  Serial.begin(115200);
  TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  pinMode(LED_PIN, OUTPUT);
  gain = calc_gain();
  Serial.print("Gain [lux/dc]: ");
  Serial.println(gain);
  sim = new Simulator(gain);
}


void loop() {
  unsigned long startTime = micros();

  
  if(counter++ == 99) {
    counter = 0;
    pwm_pos++;
    if(pwm_pos==sizeof(pwm_array)/sizeof(*pwm_array))
      pwm_pos = 0;
    analogWrite(LED_PIN, pwm_array[pwm_pos]);
  }
  unsigned long t = micros();
  //float lux = sim->calc_LUX_at_LDR(pwm_array[pwm_pos]);
  
  Serial.print(t);
  Serial.print(", ");
  Serial.print(pwm_array[pwm_pos]);
  Serial.print(", ");
  Serial.print(float(analogRead(LUX_PIN)*5)/1023);
  //Serial.print(", ");
  //Serial.println(sim->calc_LDR_voltage(lux, v_i, t_i, t));
  
  //Serial.println(get_lux());

  unsigned long endTime = micros();
  unsigned long elapsedTime = endTime - startTime;
  if(elapsedTime > sampInterval)
      Serial.println("ERROR: Sampling period was exceeded!");

  delayMicroseconds(sampInterval - elapsedTime);
}

//Gets the voltage value from the analog input and returns the lux value
float get_lux () {
  float V_AD;
  
  float R_lux;
  V_AD = float(analogRead(LUX_PIN)*VCC)/1023;
  R_lux = (R1/V_AD)*VCC - R1;
  
  return pow(10, (log10(R_lux)-b)/m);
}

float get_voltage() {
  return float(analogRead(LUX_PIN)*VCC)/1023;
}

float calc_gain () {
  int max_pwm = 255;
  analogWrite(LED_PIN, max_pwm);
  delay(400);
  float max_lux = get_lux();
  Serial.println();
  Serial.print("GAIN");
  Serial.println(max_lux/max_pwm);
  analogWrite(LED_PIN, 0);
  delay(400);
  return max_lux/max_pwm;
}