#ifndef _CONFIGS_H_
#define _CONFIGS_H_

#include <Arduino.h>

#define SERIAL_INPUT_SIZE 15

const int LED_PIN = 3;
const int LUX_PIN = A0;
const int VCC = 5;
const int R1 = 10000;
const float C1 = 0.0000022; //0.000001; // [uF]
const float m = -0.75;//-0.835;//-0.650515;
const float b = 5.7;//5.8; //4.778;
const float TETA = 0.00005; //50us
const float K1 = 10;//5; 
const float K2 = 100;
const float error_margin = 0.02;

const int pwm_array[] = {0, 15, 0, 30, 0, 45, 0, 60, 0, 75, 0, 90, 0, 105, 0, 120, 0, 135, 0, 150, 0, 165, 0, 180, 0, 195, 0, 210, 0, 225, 0, 240, 0, 255};
const unsigned long sampInterval = 10000; //microseconds 100Hz


#endif