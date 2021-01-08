#include "utils.h"
#include "comms.h"
#include <EEPROM.h>

Utils::Utils(){
  id_ctr = 0;
  lowest_id = 255;
  load_EEPROM_vars();
  hub = false;

  ack_ctr = 0;
  sync_sent = false;
  o = 0;
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

  EEPROM.get(address, this->my_id);
  add_id(this->my_id);
  address += sizeof(this->my_id);
  EEPROM.get(address, C1);
  address += sizeof(C1);
  EEPROM.get(address, m);
  address += sizeof(m);
  EEPROM.get(address, b);
  address += sizeof(b);
  /*Serial.print("Inside id, C1, m, b: ");
  Serial.print(this->my_id);
  Serial.print(", ");
  Serial.print(C1, 7);
  Serial.print(", ");
  Serial.print(m, 5);
  Serial.print(", ");
  Serial.println(b, 5);*/
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
  int d = 100; // in percentage

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

void Utils::order_ids(){
    uint8_t i, j, temp;

    for (i = 0; i < (this->id_ctr - 1); ++i)
    {
        for (j = 0; j < this->id_ctr - 1 - i; ++j )
        {
            if (this->id_vec[j] > this->id_vec[j+1])
            {
                temp = this->id_vec[j+1];
                this->id_vec[j+1] = this->id_vec[j];
                this->id_vec[j] = temp;
            }
        }
    }
}

uint8_t Utils::analyse_id_broadcast (uint32_t id){
	//Serial.print("\t\tReceiving : cmd : "); Serial.print(cmd);
	//Serial.print(" ID : "); Serial.println(id);
    uint8_t code = id & CMD_MASK;
    uint8_t from_id = comms::get_src_id(id);
	if(code != CAN_NEW_ID){
		Serial.println("ERROR: Wrong msg in wait_for_ids.");
		return 3;
	}

	if(find_id(from_id) == -1){ // if id is not found, then it is a new node
		if (!add_id(from_id)) { // if id number has been exceeded
			Serial.println("ERROR: ID number exceeded");
			return 2;
		}
		else{
            Serial.print("ADDED ID : "); Serial.println(from_id);
			return 1;// if the id was correctly added
        }
	}
	return 0;// if it is not new node, then ignore
}

bool Utils::sync (bool &sync_recvd, bool &ack_recvd, uint8_t sendto_id) {
  if(lowest_id == my_id){ //if this is lowest id
    Serial.println("i should be sending a sync");
    if(!sync_sent){
      sync_sent = true;
      Serial.println("Sending a sync");
      if(!comms::broadcast(my_id, CAN_SYNC)) // send its own id
        Serial.println(TX_BUF_FULL_ERR);
    }                                            
    else {
      if(ack_recvd){ // waits until it receives all the acknowledges
        ack_recvd = false;
        if((++ack_ctr) == id_ctr - 1){
          ack_ctr = 0;
          return true;
        }
      }
    }
  }
  else{ // normal node, waits for CAN_SYNC msg
    if(sync_recvd){
      sync_recvd = false; // reset
      if(comms::write(sendto_id, my_id, CAN_ACK)!= MCP2515::ERROR_OK)
        Serial.println(TX_BUF_FULL_ERR);
      return true;
    }
  }
  return false;
}


bool Utils::calibrate (bool has_data, can_frame &frame) {
  if(my_id == lowest_id) {
    calc_residual_lux();

    if(!comms::broadcast(my_id, CAN_MEAS_RESIDUAL_LUX)) //broadcast "Measure Residual Lux"+my_id
        Serial.println(TX_BUF_FULL_ERR);

    delay(LED_WAIT_TIME);
    analogWrite(LED_PIN, 255); //Light on
    delay(LED_WAIT_TIME);

    calc_gain(my_id);

    if(!comms::broadcast(my_id, CAN_MEAS_LUX)) //broadcast "Measure"+my_id
        Serial.println(TX_BUF_FULL_ERR);
    delay(MEASURE_WAIT_TIME);
    Serial.println("Light off");
    analogWrite(LED_PIN, 0); //Light off
    

    for(int i = 1; i< id_ctr; i++)
    {     
      if(!comms::send_msg(id_vec[i], my_id, CAN_LIGHT_ON)) //send "Light on"
        Serial.println(TX_BUF_FULL_ERR);
      delay(LED_WAIT_TIME);

      calc_gain(id_vec[i]);

      if(!comms::broadcast(id_vec[i], CAN_MEAS_LUX)) //broadcast "Measure"+id
        Serial.println(TX_BUF_FULL_ERR);
    
      delay(MEASURE_WAIT_TIME);
      if(!comms::send_msg(id_vec[i], my_id, CAN_LIGHT_OFF)) //send "Light off"
        Serial.println(TX_BUF_FULL_ERR);
    }

    Serial.println("Broadcast calibration complete");
    if(!comms::broadcast(my_id, CAN_CALIB_COMPLETE)) //broadcast "Calibration Complete"
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
      uint8_t cmd = comms::get_cmd(frame.can_id);
      comms::print_msg();
      if(cmd == CAN_MEAS_RESIDUAL_LUX) //'r' in decimal
      {
        Serial.println("Calculating residual lux."); 
        calc_residual_lux();
      }
      if(cmd == CAN_LIGHT_ON) //'l' in decimal
      {
        analogWrite(LED_PIN, 255); //Light on
        Serial.println("Light on.");
      }

      if(cmd == CAN_LIGHT_OFF) //'o' in decimal
      {
        analogWrite(LED_PIN, 0); //Light off
        Serial.println("Light off.");
      }
      if(cmd == CAN_MEAS_LUX) //'m' in decimal
      {
        Serial.println("Calculating gain.");
        calc_gain(comms::get_src_id(frame.can_id));          
      }

      if(cmd == CAN_CALIB_COMPLETE) //'c' in decimal
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