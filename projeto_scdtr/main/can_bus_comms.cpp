#include "can_bus_comms.h"
MCP2515 mcp2515(10); //SS pin 10

extern can_frame_stream *cf_stream; // defined in main.cpp
extern can_frame frame; // defined in main.cpp


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

bool send_msg (uint8_t id, uint8_t my_id, uint8_t code) {
    uint8_t msg[] {code, my_id};
    if(write(id, msg, sizeof(msg)) != MCP2515::ERROR_OK )
        return false;
    return true;
}

bool send_control_msg(uint8_t id, uint8_t my_id, uint8_t code, float u){
    my_can_msg m;
    m.value = u;
    uint8_t msg[6];
    msg[0] = code;
    msg[1] = my_id;
    for( int i = 2; i < 6; i++ ) //prepare can message
        msg[i] = m.bytes[i-2];
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

/*bool wait_for_acks (uint8_t N_nodes) {
    unsigned long t_i = micros();
    int ack_ctr = 0;
    bool has_data;

    while((micros()-t_i) < WAIT_ACK_TIME)
    {
      cli(); has_data = cf_stream->get( frame ); sei();
      
      if(has_data && frame.data[0]==107) //'k' in decimal
      {
        ack_ctr++;
        Serial.print("Received one ACK from ");
        Serial.println(frame.data[1]);
        if(ack_ctr == N_nodes - 1)
        {
            Serial.println("Received all ACKs.");
            return true;  
        }
          
      }
    }
    if(ack_ctr != N_nodes - 1)
        return false;
}*/

void print_msg () {

    Serial.print("\t\tReceiving : cmd : "); Serial.print((char)frame.data[0]);
	Serial.print(" ID : "); Serial.print(frame.data[1]);
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