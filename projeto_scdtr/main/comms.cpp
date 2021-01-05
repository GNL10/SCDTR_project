#include "comms.h"
MCP2515 mcp2515(10); //SS pin 10

extern can_frame_stream *cf_stream; // defined in main.cpp
extern can_frame frame; // defined in main.cpp

void comms::forward_can_to_serial(char msg[]){
	float_byte l;
    char cmd = msg[0];
    uint8_t id = msg[2];
    uint8_t idx = 4;
    
    switch (cmd) {
    case CMD_ILLUM:
        for(unsigned long i = 0; i < sizeof(float); i++)
            l.bytes[i] = msg[i+idx];
        comms::serial_respond(CMD_ILLUM, id, l.val);
        break;
    
    case CMD_DUTY_CYCLE:
        comms::serial_respond(CMD_DUTY_CYCLE, id, msg[4]); //duty cycle in %
        break;

    case CMD_OCCUPANCY:
        comms::serial_respond(CMD_OCCUPANCY, id, msg[4]); //occupancy
        break;

    case CMD_OCCUPIED_ILLUM:
        for(unsigned long i = 0; i < sizeof(float); i++)
            l.bytes[i] = msg[i+idx];
        comms::serial_respond(CMD_OCCUPIED_ILLUM, id, l.val); // illuminance at ocuppied state
        break;

    case CMD_UNOCCUPIED_ILLUM:
        for(unsigned long i = 0; i < sizeof(float); i++)
            l.bytes[i] = msg[i+idx];
        comms::serial_respond(CMD_UNOCCUPIED_ILLUM, id, l.val); // illuminance at ocuppied state
        break;
    // missing cases here

    default:
        break;
    }
}

void comms::can_bus_send_response(uint8_t id, char cmd1, uint8_t own_id, float val){
	char can_msg[CAN_MAX_DLEN];
	float_byte f;

    sprintf(can_msg, "%c %c ", cmd1, (char) own_id);
	
	f.val = val;
	for(int i = 0; i < 4; i++)
		can_msg[4+i] = f.bytes[i];

	Serial.print("CANBUS SENDING : ");
	for(int i = 0; i < 8; i++) {
		Serial.print((uint8_t) can_msg[i]);
		Serial.print(SPACE);
	}
	Serial.print("\tto id : ");
	Serial.println(id);
	if (write(id, (uint8_t *) can_msg, 8) != MCP2515::ERROR_OK)  // respond to id who asked
		Serial.println(TX_BUF_FULL_ERR);
}

void comms::can_bus_send_cmd(uint8_t id, char cmd1, char cmd2, uint8_t own_id){
    char msg[CAN_MAX_DLEN];
    sprintf(msg, "%c %c %c", cmd1, cmd2, own_id); // sending id as char!!
    Serial.print("Sending msg : "); Serial.print(msg); 
    Serial.print("\tto node : "); Serial.println(id);
    if (write(id, (uint8_t *) msg, strlen(msg)) != MCP2515::ERROR_OK)  // send its own id
        Serial.println(TX_BUF_FULL_ERR);
}

bool comms::send_msg (uint8_t id, uint8_t my_id, uint8_t code) {
    uint8_t msg[] {code, my_id};
    if(write(id, msg, sizeof(msg)) != MCP2515::ERROR_OK )
        return false;
    return true;
}

bool comms::send_control_msg(uint8_t id, uint8_t my_id, uint8_t code, float u){
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

bool comms::broadcast (uint8_t id, uint8_t code) { //make it general
    uint8_t msg[] {code, id};

    if(write(CAN_BROADCAST_ID, msg, sizeof(msg)) != MCP2515::ERROR_OK )
        return false;
    return true;
}

float comms::get_float(){
    my_can_msg msg;

    for (int i = 2; i < 6; i++)  // prepare can message
        msg.bytes[i - 2] = frame.data[i];

    return msg.value;
}

void comms::print_msg () {

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

MCP2515::ERROR comms::write(uint32_t id, uint8_t bytes[], uint8_t n_bytes) {
	can_frame frame;
	frame.can_id = id;
	frame.can_dlc = n_bytes;
	for( int i = 0; i < n_bytes; i++ ) //prepare can message
		frame.data[i] = bytes[i];
	
	//send data
	return mcp2515.sendMessage(&frame);
}

void comms::serial_respond(char cmd, uint8_t id, float val){
	Serial.print(cmd);
	Serial.print(SPACE);
	Serial.print(id);	
	Serial.print(SPACE);
	Serial.println(val);
}

void comms::serial_respond(char cmd, uint8_t id, int val){
	Serial.print(cmd);
	Serial.print(SPACE);
	Serial.print(id);	
	Serial.print(SPACE);
	Serial.println(val);
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