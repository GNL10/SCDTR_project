#include "utils.h"
#include <EEPROM.h>

Utils::Utils(){
  id_ctr = 0;
  lowest_id = 255;
  load_EEPROM_vars();
  hub = false;

  ack_ctr = 0;
  sync_sent = false;
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
  Serial.print("Inside id_tmp, C1, m, b: ");
  Serial.print(id_tmp);
  Serial.print(", ");
  Serial.print(C1, 6);
  Serial.print(", ");
  Serial.print(m, 5);
  Serial.print(", ");
  Serial.println(b, 5);
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
  Serial.print("Press 'z' in the next "); Serial.print(HUB_WAIT_TIME/1000000); Serial.println(" secs to become the hub!");
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

void Utils::calc_gain (uint8_t sender_id) {
  int d = 255;

  int l = calc_lux(get_voltage());
  k[find_id(sender_id)] = (l - o)/ d;

  Serial.print("Gain measured: ");
  Serial.println(k[find_id(sender_id)]);
}

void Utils::calc_residual_lux () {
  o = calc_lux(get_voltage()); 

  Serial.print("Residual volt measured: ");
  Serial.println(get_voltage());
  Serial.print("Residual Lux measured: ");
  Serial.println(o); 
}

uint8_t Utils::analyse_id_broadcast (uint8_t cmd, uint8_t id){
	Serial.print("\t\tReceiving : cmd : "); Serial.print(cmd);
	Serial.print(" ID : "); Serial.println(id);

	if(cmd != CAN_NEW_ID){
		Serial.println("ERROR: Wrong msg in wait_for_ids.");
		return 3;
	}

	if(find_id(id) == -1){ // if id is not found, then it is a new node
		if (!add_id(id)) { // if id number has been exceeded
			Serial.println("ERROR: ID number exceeded");
			return 2;
		}
		else
			return 1;// if the id was correctly added
	}
	return 0;// if it is not new node, then ignore
}

bool Utils::sync (bool has_data, can_frame &frame, bool sync_recvd) {
  if(lowest_id == id_vec[0]){ //if this is lowest id
    if(!sync_sent){
      sync_sent = true;
      if(!comms::broadcast(id_vec[0], CAN_SYNC)) // send its own id
        Serial.println(TX_BUF_FULL_ERR);
    }                                            
    else {
      if(has_data && frame.data[0] == CAN_ACK){ // waits until it receives all the acknowledges
        if((++ack_ctr) == id_ctr - 1){
          ack_ctr = 0;
          return true;
        }
      }
    }
  }
  else{ // normal node, waits for CAN_SYNC msg
    if(sync_recvd){
      if(!comms::send_msg(frame.data[1], id_vec[0], CAN_ACK))
        Serial.println(TX_BUF_FULL_ERR);
        return true;
    }
  }
  return false;
}


bool Utils::calibrate (bool has_data, can_frame &frame) {
  if(id_vec[0] == lowest_id) {
    calc_residual_lux();

    if(!comms::broadcast(id_vec[0], 'r')) //broadcast "Measure Residual Lux"+my_id
        Serial.println(TX_BUF_FULL_ERR);

    analogWrite(LED_PIN, 255); //Light on
    delay(LED_WAIT_TIME);

    calc_gain(id_vec[0]);

    if(!comms::broadcast(id_vec[0], 'm')) //broadcast "Measure"+my_id
        Serial.println(TX_BUF_FULL_ERR);
    delay(MEASURE_WAIT_TIME);
    Serial.println("Light off");
    analogWrite(LED_PIN, 0); //Light off

    for(int i = 1; i< id_ctr; i++)
    {     
      if(!comms::send_msg(id_vec[i], id_vec[0], 'l')) //send "Light on"
        Serial.println(TX_BUF_FULL_ERR);
      delay(LED_WAIT_TIME);

      calc_gain(id_vec[i]);

      if(!comms::broadcast(id_vec[i], 'm')) //broadcast "Measure"+id
        Serial.println(TX_BUF_FULL_ERR);
    
      delay(MEASURE_WAIT_TIME);
      if(!comms::send_msg(id_vec[i], id_vec[0], 'o')) //send "Light off"
        Serial.println(TX_BUF_FULL_ERR);
    }

    Serial.println("Broadcast calibration complete");
    if(!comms::broadcast(id_vec[0], 'c')) //broadcast "Calibration Complete"
        Serial.println(TX_BUF_FULL_ERR);

    for (int i=0; i < id_ctr; i++) {
      Serial.print("k");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(k[i]);
    } 
    Serial.println("Calibration complete");
    return true; // calibration is complete
  }

  else{
    if (has_data) 
    {
      comms::print_msg();
      if(frame.data[0] == 114) //'r' in decimal
      {
        Serial.println("Calculating residual lux."); 
        calc_residual_lux();
      }
      if(frame.data[0] == 108) //'l' in decimal
      {
        analogWrite(LED_PIN, 255); //Light on
        Serial.println("Light on.");
      }

      if(frame.data[0] == 111) //'o' in decimal
      {
        analogWrite(LED_PIN, 0); //Light off
        Serial.println("Light off.");
      }
      if(frame.data[0] == 109) //'m' in decimal
      {
        Serial.println("Calculating gain.");
        calc_gain(frame.data[1]);          
      }

      if(frame.data[0] == 99) //'c' in decimal
      {
        for (int i=0; i < id_ctr; i++) {
          Serial.print("k");
          Serial.print(i);
          Serial.print(": ");
          Serial.println(k[i]);
        }
        Serial.println("Calibration complete");
        return true; // calibration is complete
      }
    }
  }
  return false; // calibration is not complete
}