#ifndef _CAN_BUS_COMMS_H_
#define _CAN_BUS_COMMS_H_

#include "can_frame_stream.h"
#include "utils.h"

#define TX_BUF_FULL_ERR "\t\t\t\tMCP2515 TX Buf Full"

#define WAIT_ID_TIME (unsigned long) 2000000
#define  WAIT_ACK_TIME (unsigned long) 100000000

#define CAN_BROADCAST_ID (uint32_t) 0x000007FF
#define CAN_NEW_ID (uint8_t) 1 // message sent that indicates the addition of a new node
#define CAN_SYNC (uint8_t) 's' // message to synchronize the nodes
#define CAN_ACK (uint8_t) 'k' // acknowledge

//union to pack/unpack long ints into bytes
union my_can_msg {
   float value;
   unsigned char bytes[4];
};

typedef union{
    float val;
    char bytes[4];
} float_byte;

void can_bus_send_response(uint8_t id, const char* cmd1, uint8_t own_id, float val);
void can_bus_send_cmd(uint8_t id, const char* cmd1, const char* cmd2, uint8_t own_id);
bool send_msg (uint8_t id, uint8_t my_id, uint8_t code);
bool broadcast (uint8_t id, uint8_t code);
//bool wait_for_acks (uint8_t N_nodes);
bool send_control_msg(uint8_t id, uint8_t my_id, uint8_t code, float u);
void print_msg ();
MCP2515::ERROR write(uint32_t id, uint8_t bytes[], uint8_t n_bytes);
#endif