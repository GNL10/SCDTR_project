#ifndef _CAN_BUS_COMMS_H_
#define _CAN_BUS_COMMS_H_

#include "can_frame_stream.h"
#define WAIT_ID_TIME (unsigned long) 5000000 // 5 secs
#define BROADCAST_BIT (uint32_t) 0x00000400	// bit used to know if message is of type broadcast
#define CAN_NEW_ID (uint8_t) 0x15 // message sent that indicates the addition of a new node

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
#endif