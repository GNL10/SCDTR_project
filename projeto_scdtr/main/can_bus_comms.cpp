#include "can_bus_comms.h"
MCP2515 mcp2515(10); //SS pin 10


bool send_id_broadcast (uint8_t id) {
    uint8_t msg[] {CAN_NEW_ID, id};
    if(write(CAN_BROADCAST_ID, msg, 2) != MCP2515::ERROR_OK )
        return false;
    return true;
}

MCP2515::ERROR write(uint32_t id, uint8_t bytes[], uint8_t n_bytes) {
    can_frame frame;
    frame.can_id = id;
    frame.can_dlc = n_bytes;
    for( int i = 0; i < n_bytes; i++ ) //prepare can message
        frame.data[i] = bytes[i];
    
    //send data
    return mcp2515.sendMessage(&frame);
}

/*MCP2515::ERROR write(uint32_t id, uint32_t val, uint8_t n_bytes) {
    can_frame frame;
    frame.can_id = id;
    frame.can_dlc = n_bytes;
    my_can_msg msg;
    msg.value = val; //pack data
    for( int i = 0; i < n_bytes; i++ ) //prepare can message
        frame.data[i] = msg.bytes[i];
    
    //send data
    return mcp2515.sendMessage(&frame);
}*/