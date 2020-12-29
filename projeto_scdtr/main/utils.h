#ifndef _UTILS_H_
#define _UTILS_H_

#include <Arduino.h>
#include "configs.h"

#define HUB_WAIT_TIME (long) 3000000

class Utils
{
private:
public:
  uint8_t id_ctr; // number of ids in id_vec (including its own id)
  uint8_t id_vec[ID_MAX_NUM]; // id[0] is node's own id
  uint8_t lowest_id;
  float k[];
  float o = 0;
  float C1, m, b;
  bool hub; // true if node is a hub

  char serial_input[SERIAL_INPUT_SIZE+1];
  int serial_input_index = 0;

  Utils();
  bool add_id(uint8_t new_id);
  int find_id(uint8_t id);
  void load_EEPROM_vars();
  float calc_lux (float V_AD);
	float calc_lux ();
  float get_voltage();
  void calc_gain (uint8_t sender_id);
  void calc_residual_lux ();
  bool serial_read_lux();
  bool isHub();
  
};
#endif