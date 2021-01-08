#ifndef _UTILS_H_
#define _UTILS_H_

#include <Arduino.h>
#include "configs.h"
#include "can_frame_stream.h"

#define HUB_WAIT_TIME (long) 1000000
#define LED_WAIT_TIME (unsigned long) 5000
#define MEASURE_WAIT_TIME (unsigned long ) 2000

class Utils
{
private:
public:
  uint8_t id_ctr; // number of ids in id_vec (including its own id)
  uint8_t id_vec[ID_MAX_NUM]; // id[0] is node's own id
  uint8_t my_id;
  uint8_t lowest_id;
  float k[];
  float o;
  float C1, m, b;
  bool hub; // true if node is a hub

  uint8_t ack_ctr;
  bool sync_sent;

  char serial_input[SERIAL_INPUT_SIZE+1];
  uint8_t serial_input_index = 0;

  Utils();
  bool add_id(uint8_t new_id);
  int find_id(uint8_t id);
  void load_EEPROM_vars();
  float calc_lux (float V_AD);
	float calc_lux ();
  float get_voltage();
  void calc_gain (uint8_t sender_id);
  void serial_respond(char cmd, uint8_t id, float val);
  void serial_respond(char cmd, uint8_t id, int val);
  void order_ids();

  uint8_t analyse_id_broadcast (uint32_t id);

  void calc_residual_lux ();
  bool serial_read_lux();
  bool isHub();

  bool sync (bool &sync_recvd, bool &ack_recvd, uint8_t sendto_id);
  bool calibrate (bool has_data, can_frame &frame);
  
};
#endif