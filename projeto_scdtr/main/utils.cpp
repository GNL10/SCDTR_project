#include "utils.h"
#include <EEPROM.h>

Utils::Utils(){
  id_ctr = 0;
  lowest_id = 255;
  load_EEPROM_vars();
  hub = false;
}

bool Utils::add_id(uint8_t new_id){

  if(id_ctr == ID_MAX_NUM) // if the max number of nodes has been reached
    return false;
  if(new_id < lowest_id)
    lowest_id = new_id; 
  id_vec[id_ctr++] = new_id;
  return true;
}

/**
 * Searches id_vec for a given id
 * @param id ID that is being looked for
 * @returns -1 if not found, else returns the index of said ID
 */ 
int Utils::find_id(uint8_t id) {
  for(uint8_t i = 0; i < id_ctr; i++){
    if(id_vec[i] == id)
      return i;
  }
  return -1;
}

/**
 * Loads the values from the EEPROM to the variables C1, m and b of the program
 */
void Utils::load_EEPROM_vars() {
  int address = 0;
  uint8_t id_tmp = 0;

  EEPROM.get(address, id_tmp);
  add_id(id_tmp);
  address += sizeof(id_tmp);
  EEPROM.get(address, C1);
  address += sizeof(C1);
  EEPROM.get(address, m);
  address += sizeof(m);
  EEPROM.get(address, b);
  address += sizeof(b);
}

/**
 * Calculates the lux value from a voltage value
 * @param V_AD Voltage at LDR
 * @returns Lux value at LDR
 */
float Utils::calc_lux (float V_AD) {
  float R_lux = (R1/V_AD)*VCC - R1; 
  return pow(10, (log10(R_lux)-b)/m);
}

/**
 * Calculates the lux value from the voltage read from the LDR
 * @returns Lux value at LDR
 */
float Utils::calc_lux () {
	float V_AD = get_voltage();
  float R_lux = (R1/V_AD)*VCC - R1; 
  return pow(10, (log10(R_lux)-b)/m);
}

/**
 * Reads the value from LUX_PIN and converts it to volts
 * @returns voltage at LDR
 */
float Utils::get_voltage() {
  return float(analogRead(LUX_PIN)*VCC)/1023;
}

/**
 * Calculates the gain [lux / duty cycle] of the system
 * @returns Gain [lux / duty cycle] to the static_gain global variable
 */
void Utils::calc_gain () {
  int max_pwm = 255;

  analogWrite(LED_PIN, max_pwm);
  delay(500);
  float max_lux = calc_lux(get_voltage());  
  
  int min_pwm = 0;
  analogWrite(LED_PIN, min_pwm);
  delay(500);
  float min_lux = calc_lux(get_voltage());
  static_gain = (max_lux - min_lux) / (max_pwm-min_pwm);
  static_b = min_lux;

  analogWrite(LED_PIN, 0);
  delay(300);
}


/**
 * reads 1 char from serial and concatenates it into the string serial_input
 * @returns true when the enter key is pressed else false
 */
bool Utils::serial_read_lux () {
  if (Serial.available()) {
    char read_byte = Serial.read();
    if(read_byte == 10){ // 10 -> new line
      serial_input[serial_input_index] = '\0';
      return true;
    }
    serial_input[serial_input_index++] = read_byte;
  }
  return false;
}

bool Utils::isHub() {
  Serial.println("Answer by pressing 'z' in the next 5 secs!");
  long start_t = micros();
  while (micros()-start_t < HUB_WAIT_TIME) {
    if (Serial.available()) {
      char read_byte = Serial.read();
      if (read_byte == 'z'){
        hub = true;
        Serial.println("I AM THE HUB!!");
        return true;
      }
    }
    delay(10);
  }
  hub = false;
  return false;
}