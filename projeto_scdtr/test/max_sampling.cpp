#include "configs.h"
//#include "simulator.h"
#include <math.h>
#include <string.h>

const short VECTOR_SIZE = 210;
//  /home/gnl/.platformio/penv/bin/platformio device monitor

//int counter = 0;
//int pwm_pos = 0;
//float gain = 0;
//int read_lux = 0;
//float t_i = 0;
//float v_i = 0;
//Simulator* sim;

unsigned long saved_time[VECTOR_SIZE];
short saved_pwm[VECTOR_SIZE];
int saved_analog_read[VECTOR_SIZE];
long loop_counter = 0;
int pwm = 0;

//int serial_read_lux (int *ref);
float get_lux ();
float calc_gain ();
float get_voltage();



void setup() {
  Serial.begin(115200);
  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  pinMode(LED_PIN, OUTPUT);
  delay(1000);
  //gain = calc_gain();
  //Serial.print("Gain [lux/dc]: ");
  //Serial.println(gain);
  //sim = new Simulator(gain);
}


void loop() {
  loop_counter++;
  if(loop_counter == 3) {
    pwm = 130;
    analogWrite(LED_PIN, pwm);
  }

  if (loop_counter < VECTOR_SIZE){
    saved_time[loop_counter] = micros();
    saved_pwm[loop_counter] = pwm;
    saved_analog_read[loop_counter] = analogRead(LUX_PIN);
    //float(analogRead(LUX_PIN)*5)/1023;
  }
  else if (loop_counter < 2*VECTOR_SIZE){
    Serial.print(saved_time[loop_counter - VECTOR_SIZE]);
    Serial.print(", ");
    Serial.print(saved_pwm[loop_counter - VECTOR_SIZE]);
    Serial.print(", ");
    Serial.println(saved_analog_read[loop_counter - VECTOR_SIZE]);
  }
  else {
    return;
  }
  
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
  analogWrite(LED_PIN, 0);
  delay(400);
  return max_lux/max_pwm;
}
