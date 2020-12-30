#include "can_bus_comms.h"
MCP2515 mcp2515(10); //SS pin 10

extern can_frame_stream *cf_stream; // defined in main.cpp
extern can_frame frame; // defined in main.cpp

bool send_msg (uint8_t id, uint8_t my_id, uint8_t code) {
    uint8_t msg[] {code, my_id};
    if(write(id, msg, sizeof(msg)) != MCP2515::ERROR_OK )
        return false;
    return true;
}

bool broadcast (uint8_t id, uint8_t code) { //make it general
    uint8_t msg[] {code, id};

    if(write(CAN_BROADCAST_ID, msg, sizeof(msg)) != MCP2515::ERROR_OK )
        return false;
    return true;
}

void print_msg () {
    uint8_t sender_id = frame.data[1];
    Serial.print( "\t\tReceiving : ID : ");
    Serial.print(sender_id);
    Serial.print("\t\tdata : ");
    for (uint8_t i = 0; i < frame.can_dlc; i++)
    {
    if(i == 0)
        Serial.print((char)frame.data[i]);
    else
        Serial.print(frame.data[i]);   
    }
        
    Serial.println();
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