#ifndef _CAN_BUS_COMMS_H_
#define _CAN_BUS_COMMS_H_

#include "can_frame_stream.h"
#define WAIT_ID_TIME (unsigned long) 2000000//100000000//5000000 // 5 secs
#define CAN_BROADCAST_ID (uint32_t) 0x000007FF
#define CAN_NEW_ID (uint8_t) 1 // message sent that indicates the addition of a new node
#define  WAIT_ACK_TIME (unsigned long) 100000000

//union to pack/unpack long ints into bytes
union my_can_msg {
unsigned long value;
unsigned char bytes[4];
};


/* EXAMPLE NOT TO BE USED!!!!!
union { // anonymous union
      struct { int i, j; }; // anonymous structure
      struct { long k, l; } w;
   };
*/

bool send_id_broadcast (uint8_t id);
bool send_msg (uint8_t id, uint8_t my_id, uint8_t code);
bool broadcast (uint8_t id, uint8_t code);
bool wait_for_acks (uint8_t N_nodes);
void print_msg ();
MCP2515::ERROR write(uint32_t id, uint8_t bytes[], uint8_t n_bytes);
#endif