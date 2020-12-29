#include "can_bus_comms.h"
MCP2515 mcp2515(10); //SS pin 10


bool send_id_broadcast (uint8_t id) {
	uint8_t msg[] {CAN_NEW_ID, id};
	if(write(CAN_BROADCAST_ID, msg, 2) != MCP2515::ERROR_OK )
		return false;
	return true;
}

uint8_t analyse_id_broadcast (uint8_t cmd, uint8_t id, Utils *utils){
	Serial.print("\t\tReceiving : cmd : "); Serial.print(cmd);
	Serial.print(" ID : "); Serial.println(id);

	if(cmd != CAN_NEW_ID){
		Serial.println("ERROR: Wrong msg in wait_for_ids.");
		return 3;
	}

	if(utils->find_id(id) == -1){ // if id is not found, then it is a new node
		if (!utils->add_id(id)) { // if id number has been exceeded
			Serial.println("ERROR: ID number exceeded");
			return 2;
		}
		else
			return 1;// if the id was correctly added
	}
	return 0;// if it is not new node, then ignore
}

bool send_sync_broadcast (uint8_t id) {
	uint8_t msg[] {CAN_SYNC, id};
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