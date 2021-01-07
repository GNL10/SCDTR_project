#ifndef _CONFIGS_H_
#define _CONFIGS_H_

#include <Arduino.h>

#define SERIAL_INPUT_SIZE 15
#define ID_MAX_NUM 3 // uint8 max 255


const unsigned long sampInterval = 10000; //microseconds 100Hz
const int LED_PIN = 3;  // Pin connected to LED
const int LUX_PIN = A0; // Pin connected to LDR
const int VCC = 5;      // VCC voltage
const int R1 = 10000;   // R1 resistance
const float TETA = 0.00005; //50us
const float K1 = 1.6;//2.2;//0.2;  // Proportional gain
const float K2 = 0.5;//0.5;//0.1;  // Integrator gain
const float error_margin = 0.5;//0.20;    // Margin of error to be ignored

// FRAME ID 11bits: ID_TO 2bits + ID_FROM 2bits + CMD 7bits 
#define TO_SHIFT 9
#define FROM_SHIFT 7
#define ID_MASK 0b11 //2bits per id
#define CMD_MASK 0b1111111 // 7 bits for the code

#define CAN_BROADCAST_ID (uint8_t) 3

// 7 BITS FOR THESE CAN IDS!
// The 7th bit (msb) can only be 1 if the message is of type can_d_element, and the lesser significant bits
// will indicate the position to which the element refers to
#define CAN_NEW_ID (uint8_t) 0x00
#define CAN_ACK (uint8_t) 0x01
#define CAN_SYNC (uint8_t) 0x02
#define CAN_GET_ILLUM (uint8_t) 0x03
#define CAN_GET_DUTY_CYCLE (uint8_t) 0x04
#define CAN_GET_OCCUPANCY (uint8_t) 0x05
#define CAN_GET_OCCUPIED_ILLUM (uint8_t) 0x06
#define CAN_GET_UNOCCUPIED_ILLUM (uint8_t) 0x07
#define CAN_GET_ILLUM_LB (uint8_t) 0x08 // lower bound
#define CAN_GET_EXT_ILLUM (uint8_t) 0x09
#define CAN_GET_CONTROL_REF (uint8_t) 0x0A
#define CAN_GET_ENERGY_COST (uint8_t) 0x0B
#define CAN_GET_POWER_CONSUMPTION (uint8_t) 0x0C
#define CAN_GET_POWER_CONSUMPTION_TOTAL (uint8_t) 0x0D
#define CAN_GET_TIME (uint8_t) 0x0E
#define CAN_GET_ACCUM_ENERGY (uint8_t) 0x0F
#define CAN_GET_VISIBILITY_ERR (uint8_t) 0x10
#define CAN_GET_VISIBILITY_ERR_TOTAL (uint8_t) 0x11
#define CAN_GET_FLICKER_ERR (uint8_t) 0x12
#define CAN_GET_FLICKER_ERR_TOTAL (uint8_t) 0x13
#define CAN_MEAS_RESIDUAL_LUX (uint8_t) 0x14
#define CAN_MEAS_LUX (uint8_t) 0x15
#define CAN_LIGHT_ON (uint8_t) 0x16
#define CAN_LIGHT_OFF (uint8_t) 0x17
#define CAN_CALIB_COMPLETE (uint8_t) 0x18
#define CAN_SET_OCCUPANCY (uint8_t) 0x19
#define CAN_SET_OCCUPIED_LUX (uint8_t) 0x1A
#define CAN_SET_UNOCCUPIED_LUX (uint8_t) 0x1B
#define CAN_D_ELEMENT (uint8_t) 0x40 //0100 0000 command to send element of local solution
#define CAN_D_ELEMENT_MASK (uint8_t) 0x3F


/* COMMANDS */
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
#define CMD_ACK 'k' //todo delete
#define CMD_SYNC 's' //todo delete


#define SERIAL_ACK "ack"
#define SERIAL_ERR "err"


#endif