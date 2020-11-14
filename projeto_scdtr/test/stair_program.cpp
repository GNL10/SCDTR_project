#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "configs.h"


int serial_read_pwm ();
float get_lux ();


void setup() {
  pinMode(LED_PIN, OUTPUT);
  TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  Serial.begin(115200);
}

int duty_cycle = 0;
int increment = 5;
int loop_counter = 0;

void loop() {
  
  duty_cycle += increment;
  analogWrite(LED_PIN, duty_cycle);
  if(duty_cycle%255 == 0)
    increment = -increment;

  loop_counter++;
  Serial.print(loop_counter);
  Serial.print(", ");

  Serial.print(duty_cycle);
  Serial.print(", ");


  delay(300);
  Serial.print(float(analogRead(LUX_PIN)*5)/1023);
  Serial.print(", ");
  Serial.println(get_lux());
}

// reads from serial the pwm value
// returns -1 if number incorrect or not finished
// else returns pwm value after enter is pressed
int serial_read_pwm () {
  byte read_byte = 0; 
  while (Serial.available() > 0) {
    read_byte = Serial.read();
    if(read_byte == 10) { // 10 -> new line
      if(duty_cycle > 255){
        Serial.println("ERROR: Inserted number is over 255");
        duty_cycle = 0;
      }
      else{
        int aux = duty_cycle;
        duty_cycle = 0;
        return aux;
      }
    }
    else if (read_byte != 13) { //13 -> carriage return
      if(read_byte - 48 >= 0 && read_byte - 48 <= 9){ // makes sure that a number was inserted
        duty_cycle *= 10;
        duty_cycle += read_byte - 48; // -48 converts it to an int
      }
      else
        Serial.println("ERROR: Character is not a digit");
    }
  }
  return -1;
}

//Gets the voltage value from the analog input and returns the lux value
float get_lux () {
  float V_AD = float(analogRead(LUX_PIN)*5)/1023;;
  float R_lux = (R1/V_AD)*VCC - R1; 
  return pow(10, (log10(R_lux)-b)/m);
}