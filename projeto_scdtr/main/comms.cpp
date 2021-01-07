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

void comms::can_bus_send_val(uint8_t to_id, uint8_t my_id, uint8_t cmd, float val){
	uint8_t can_msg[CAN_MAX_DLEN];
	float_byte f;
    f.val = val;

    for(size_t i = 0; i < sizeof(float); i++)
        can_msg[i] = f.bytes[i];

    if (write(to_id, my_id, cmd, can_msg, sizeof(float)) != MCP2515::ERROR_OK)  // respond to id who asked
		Serial.println(TX_BUF_FULL_ERR);
}

void comms::can_bus_send_val(uint8_t to_id, uint8_t my_id, uint8_t cmd, int val){
	uint8_t can_msg[CAN_MAX_DLEN];
	int_byte f;
    f.val = val;

    for(size_t i = 0; i < sizeof(int); i++)
        can_msg[i] = f.bytes[i];

    if (write(to_id, my_id, cmd, can_msg, sizeof(int)) != MCP2515::ERROR_OK)  // respond to id who asked
		Serial.println(TX_BUF_FULL_ERR);
}

void comms::can_bus_send_val(uint8_t to_id, uint8_t my_id, uint8_t cmd, unsigned long val){
	uint8_t can_msg[CAN_MAX_DLEN];
    u_long_byte f;
    f.val = val;

    for(size_t i = 0; i < sizeof(unsigned long); i++)
        can_msg[i] = f.bytes[i];

    if (write(to_id, my_id, cmd, can_msg, sizeof(unsigned long)) != MCP2515::ERROR_OK)  // respond to id who asked
		Serial.println(TX_BUF_FULL_ERR);
}

void comms::can_bus_send_val(uint8_t to_id, uint8_t my_id, uint8_t cmd, bool val){
	uint8_t can_msg[CAN_MAX_DLEN];
    can_msg[0] = val;

    if (write(to_id, my_id, cmd, can_msg, sizeof(bool)) != MCP2515::ERROR_OK)  // respond to id who asked
		Serial.println(TX_BUF_FULL_ERR);
}

bool comms::send_msg (uint8_t to_id, uint8_t my_id, uint8_t cmd, bool rtr) {
    if(write(to_id, my_id, cmd, rtr) != MCP2515::ERROR_OK )
        return false;
    return true;
}

bool comms::broadcast (uint8_t id, uint8_t cmd) { //make it general
    if(write( CAN_BROADCAST_ID, id, cmd) != MCP2515::ERROR_OK )
        return false;
    return true;
}

float comms::get_float(){
    float_byte msg;

    for (int i = 0; i < 4; i++)  // prepare can message
        msg.bytes[i] = frame.data[i];
    return msg.val;
}

unsigned long comms::get_ulong(){
    u_long_byte msg;

    for (size_t i = 0; i < sizeof(unsigned long); i++)  // prepare can message
        msg.bytes[i] = frame.data[i];
    return msg.val;
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

MCP2515::ERROR comms::write(uint8_t to_id, uint8_t my_id, uint8_t code, uint8_t bytes[], uint8_t n_bytes) {
	can_frame frame;
    uint32_t id = 0;

    id |= (to_id & ID_MASK) << TO_SHIFT;
    id |= (my_id & ID_MASK) << FROM_SHIFT;
    id |= code & CMD_MASK; 
    Serial.print("SENDING MESSAGE WITH ID : ");
    Serial.println(id, BIN);

	frame.can_id = id;
	frame.can_dlc = n_bytes;
	for( int i = 0; i < n_bytes; i++ ) //prepare can message
		frame.data[i] = bytes[i];
	
	//send data
	return mcp2515.sendMessage(&frame);
}

// Send message with no data inside
MCP2515::ERROR comms::write(uint8_t to_id, uint8_t my_id, uint8_t code, bool rtr) {
	can_frame frame;
    uint32_t id = 0;

    id |= (to_id & ID_MASK) << TO_SHIFT;
    id |= (my_id & ID_MASK) << FROM_SHIFT;
    id |= code & CMD_MASK; 
    if (rtr == true)
        id |= CAN_RTR_FLAG;
    Serial.print("SENDING MESSAGE WITH ID : ");
    Serial.println(id, BIN);

	frame.can_id = id;
	frame.can_dlc = 0;
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

void comms::serial_respond(char cmd, uint8_t id, unsigned long val){
	Serial.print(cmd);
	Serial.print(SPACE);
	Serial.print(id);	
	Serial.print(SPACE);
	Serial.println(val);
}

uint8_t comms::get_src_id(uint32_t id) {
    return (id >> FROM_SHIFT) & ID_MASK;
}
uint8_t comms::get_cmd(uint32_t id) {
    return id & CMD_MASK;
}

bool comms::get_bool(can_frame frame){
    return frame.data[0];
}

uint8_t comms::extract_uint8_t(char input[], uint8_t &idx) {
	char l = input[idx];
	uint8_t num = 0; 

	while (l != '\0' && l != ' ' && l != '\r'){
		num *= 10;
		num += (l - '0'); // add one digit at a time
		l = input[++idx];
	}
	idx++; //leaves idx at beginning of next num
	return num;
}

float comms::extract_float(char input[], uint8_t &idx){
    char l = input[idx];
    float num = 0;
    bool point = false;
    float dec = 0.1;
    
    while (l != '\0' && l != ' ' && l != '\r'){
		if(l == '.'){
            point = true;
            l = input[++idx];
            continue;
        }
        if(point == true) {
            num += dec*(l - '0');
            dec *= 0.1; //decrease the decimal by another x10
        } 
        else{
            num *= 10;
		    num += (l - '0'); // add one digit at a time
        }
		l = input[++idx];
	}
	idx++; //leaves idx at beginning of next num
	return num;
}