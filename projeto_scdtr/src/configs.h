#ifndef _CONFIGS_H_
#define _CONFIGS_H_

#include <Arduino.h>

#define SERIAL_INPUT_SIZE 15

const int LED_PIN = 3;  // Pin connected to LED
const int LUX_PIN = A0; // Pin connected to LDR
const int VCC = 5;      // VCC voltage
const int R1 = 10000;   // R1 resistance
const float C1 = 0.0000015; //0.000001; // Capacitor capacitance[uF]
const float m = -0.793; //-0.650515;
const float b = 5.1;    //4.778;
const float TETA = 0.00005; //50us
const float K1 = 2.2;//0.2;  // Proportional gain
const float K2 = 0.29;//0.1;  // Integrator gain
const float error_margin = 0.5;//0.20;    // Margin of error to be ignored




#endif