#ifndef _CONFIGS_H_
#define _CONFIGS_H_

#include <Arduino.h>

#define SERIAL_INPUT_SIZE 15

const int LED_PIN = 3;
const int LUX_PIN = A0;
const int VCC = 5;
const int R1 = 10000;
const float C1 = 0.0000022; //0.000001; // [uF]
const float m = -0.793; //-0.650515;
const float b = 5.1;    //4.778;
const float TETA = 0.00005; //50us
const float K1 = 0.2;  
const float K2 = 0.1;
const float error_margin = 0.20;




#endif