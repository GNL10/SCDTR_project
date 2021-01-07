#ifndef _COMMS_H_
#define _COMMS_H_

#include "can_frame_stream.h"
#include "utils.h"

#define TX_BUF_FULL_ERR "\t\t\t\tMCP2515 TX Buf Full"

#define WAIT_ID_TIME (unsigned long) 2000000
#define  WAIT_ACK_TIME (unsigned long) 100000000

//union to pack/unpack floats into bytes
typedef union{
    float val;
    char bytes[4];
} float_byte;

//union to pack/unpack ints into bytes
typedef union{
    int val;
    char bytes[2];
} int_byte;

//union to pack/unpack ints into bytes
typedef union{
    unsigned long val;
    char bytes[4];
} u_long_byte;

namespace comms {
    void forward_can_to_serial(char msg[]);
    void can_bus_send_val(uint8_t to_id, uint8_t my_id, uint8_t cmd, float val);
    void can_bus_send_val(uint8_t to_id, uint8_t my_id, uint8_t cmd, int val);
    void can_bus_send_val(uint8_t to_id, uint8_t my_id, uint8_t cmd, unsigned long val);
    void can_bus_send_val(uint8_t to_id, uint8_t my_id, uint8_t cmd, bool val);
    bool send_msg (uint8_t to_id, uint8_t my_id, uint8_t cmd, bool rtr = false);
    bool broadcast (uint8_t id, uint8_t code);
    bool send_control_msg(uint8_t id, uint8_t my_id, uint8_t code, float u);
    void print_msg ();
    MCP2515::ERROR write(uint8_t to_id, uint8_t my_id, uint8_t code, uint8_t bytes[], uint8_t n_bytes);
    MCP2515::ERROR write(uint8_t to_id, uint8_t my_id, uint8_t code, bool rtr = false);
    void serial_respond(char cmd, uint8_t id, float val);
    void serial_respond(char cmd, uint8_t id, int val);
    void serial_respond(char cmd, uint8_t id, unsigned long val);
    uint8_t get_src_id(uint32_t id);
    uint8_t get_cmd(uint32_t id);
    bool get_bool(can_frame frame);
    uint8_t extract_uint8_t(char input[], uint8_t &idx);
    float extract_float(char input[], uint8_t &idx);
    float get_float();
    unsigned long get_ulong();

}
#endif