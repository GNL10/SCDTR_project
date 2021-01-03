#ifndef _CONFIGS_H_
#define _CONFIGS_H_

#include <Arduino.h>

#define SERIAL_INPUT_SIZE 15
#define ID_MAX_NUM 64 // uint8 max 255


const unsigned long sampInterval = 10000; //microseconds 100Hz
const int LED_PIN = 3;  // Pin connected to LED
const int LUX_PIN = A0; // Pin connected to LDR
const int VCC = 5;      // VCC voltage
const int R1 = 10000;   // R1 resistance
const float TETA = 0.00005; //50us
const float K1 = 1.6;//2.2;//0.2;  // Proportional gain
const float K2 = 0.5;//0.5;//0.1;  // Integrator gain
const float error_margin = 0.5;//0.20;    // Margin of error to be ignored


/* SERIAL COMMANDS */
#define SPACE ' '
#define CMD_GET 'g'
#define CMD_ILLUM 'l'
#define CMD_DUTY_CYCLE 'd'
#define CMD_OCCUPANCY 'o'
#define CMD_OCCUPIED_ILLUM 'O'
#define CMD_UNOCCUPIED_ILLUM 'U'
#define CMD_ILLUM_LB 'L' // lower bound
#define CMD_EXT_ILLUM 'x'
#define CMD_CONTROL_REF 'r'
#define CMD_ENERGY_COST 'c'
#define CMD_POWER_CONSUPTION 'p'
#define CMD_TOTAL 'T'
#define CMD_TIME 't'
#define CMD_ACCUM_ENERGY 'e'
#define CMD_VISIBILITY_ERR 'v'
#define CMD_FLICKER_ERR 'f'
#define CMD_RESET 'r'
#define CMD_ACK 'A'


#endif