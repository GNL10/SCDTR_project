#ifndef _COMMS_H_
#define _COMMS_H_

#include "can_frame_stream.h"
#include "utils.h"

#define TX_BUF_FULL_ERR "\t\t\t\tMCP2515 TX Buf Full"

#define WAIT_ID_TIME (unsigned long) 3000000
#define  WAIT_ACK_TIME (unsigned long) 100000000


//union to pack/unpack floats into bytes
typedef union{
    float val;
    char bytes[4];
} float_byte;

namespace comms {
    void forward_can_to_serial(char msg[]);
    //void can_bus_send_response(uint8_t id, char cmd1, uint8_t own_id, float val);
    //void can_bus_send_cmd(uint8_t id, char cmd1, char cmd2, uint8_t own_id);
    //bool send_msg (uint8_t id, uint8_t my_id, uint8_t code);
    bool send_msg (uint8_t to_id, uint8_t my_id, uint8_t cmd);
    bool broadcast (uint8_t id, uint8_t code);
    //bool wait_for_acks (uint8_t N_nodes);
    bool send_control_msg(uint8_t id, uint8_t my_id, uint8_t code, float u);
    void print_msg ();
    MCP2515::ERROR write(uint8_t to_id, uint8_t my_id, uint8_t code, uint8_t bytes[], uint8_t n_bytes);
    MCP2515::ERROR write(uint8_t to_id, uint8_t my_id, uint8_t code);
    void serial_respond(char cmd, uint8_t id, float val);
    void serial_respond(char cmd, uint8_t id, int val);
    uint8_t get_src_id(uint32_t id);
    uint8_t get_cmd(uint32_t id);
    uint8_t extract_uint8_t(char input[], uint8_t &idx);
    float extract_float(char input[], uint8_t &idx);
}
#endif