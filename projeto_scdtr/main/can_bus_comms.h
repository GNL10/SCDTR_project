#ifndef _CAN_BUS_COMMS_H_
#define _CAN_BUS_COMMS_H_

#include "can_frame_stream.h"
#include "utils.h"

#define WAIT_ID_TIME (unsigned long) 10000000
#define CAN_BROADCAST_ID (uint32_t) 0x000007FF
#define CAN_NEW_ID (uint8_t) 1 // message sent that indicates the addition of a new node
#define CAN_SYNC (uint8_t) 's' // message to synchronize the nodes
#define CAN_ACK (uint8_t) 'k' // acknowledge

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
uint8_t analyse_id_broadcast (uint8_t cmd, uint8_t id, Utils *utils);
bool send_sync_broadcast (uint8_t id);
MCP2515::ERROR write(uint32_t id, uint8_t bytes[], uint8_t n_bytes);
#endif