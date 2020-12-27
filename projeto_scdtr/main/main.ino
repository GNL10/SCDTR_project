#include <math.h>
#include <string.h>
#include "configs.h"
#include "simulator.h"
#include "controller.h"
#include "can_frame_stream.h"
#include "can_bus_comms.h"
#include "utils.h"

// can bus comms
can_frame_stream *cf_stream = new can_frame_stream();
MCP2515 mcp2515(10); //SS pin 10

//notification flag for ISR and loop()
volatile bool can_interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;

const unsigned long sampInterval = 10000; //microseconds 100Hz

char serial_input[SERIAL_INPUT_SIZE+1];
int serial_input_index = 0;

float v_i = 0;
unsigned long t_i = 0;

float u_ff = 0;
float x_ref = 0;

bool occupancy = false;
float occupied_lux = 50;
float unoccupied_lux = 20;

Utils *utils;
Simulator *sim;
Controller *ctrl;

volatile bool flag;

enum state : byte {start, send_id_broadcast, wait_for_ids, calibrate, apply_control}; // states of the system
state curr_state = start;

ISR(TIMER1_COMPA_vect);
int serial_read_lux ();
void process_serial_input_command();
MCP2515::ERROR write(uint32_t id, uint32_t val, uint8_t n_bytes);


void setup() {
  Serial.begin(115200);
  utils = new Utils();

  // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR2B = (TCCR2B & B11111000) | B00000001;
  pinMode(LED_PIN, OUTPUT);

  utils->calc_gain();

  sim = new Simulator(utils->m, utils->b, R1, utils->C1, VCC);
  ctrl = new Controller(error_margin, K1, K2, true);

  cli(); //disable interrupts
  TCCR1A = 0; // clear register
  TCCR1B = 0; // clear register
  TCNT1 = 0; //reset counter

  // OCR1A = desired_period/clock_period – 1
  // = clock_freq/desired_freq - 1
  // = (16*10^6Hz / 8) / (100Hz) – 1
  OCR1A = 19999; //must be <65536
  TCCR1B |= (1 << WGM12); //CTC On
  // Set prescaler for 8
  TCCR1B |= (1 << CS11);

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //enable interrupts

	/* CAN BUS SETUP */
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
	mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);

  mcp2515.setFilterMask(MCP2515::MASK0, 0, CAN_SFF_MASK);
  mcp2515.setFilter(MCP2515::RXF0, 0, (uint32_t)utils->id_vec[0]); // own id
  
  mcp2515.setFilterMask(MCP2515::MASK1, 0, BROADCAST_BIT);
  mcp2515.setFilter(MCP2515::RXF2, 0, BROADCAST_BIT);

  mcp2515.setNormalMode();
  // use mcp2515.setLoopbackMode() for local testing
  //mcp2515.setLoopbackMode();

}


void loop() {
  can_frame frame;
  bool has_data;

	switch (curr_state)
	{
	case start:
		curr_state = send_id_broadcast;
		break;
	case send_id_broadcast:
		Serial.print("Printing own id :");
    Serial.println(utils->id_vec[0]);
		if( write(BROADCAST_BIT | utils->id_vec[0], CAN_NEW_ID, sizeof(CAN_NEW_ID)) != MCP2515::ERROR_OK )
			Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
		curr_state = wait_for_ids;
		break;

	case wait_for_ids:
    cli(); has_data = cf_stream->get( frame ); sei();
		if (has_data) {
			uint8_t id = frame.can_id & ~BROADCAST_BIT;
			Serial.print( "\t\tReceiving: ID :");
      Serial.println(id); // remove the broadcast bit
			if (!utils->add_id(id)) {
				Serial.println("ERROR: ID number exceeded");
				curr_state = calibrate;
			}
			
		}
    //if has_data is true
      // yes -> add id to node

		if (micros() > WAIT_ID_TIME) { // move on to calibration
			curr_state = calibrate;
		}
		// else

		// add more arduinos to the network
		// figure out how to add more memory dynamically 
			

		break;
	case calibrate:
		Serial.println("Calibrating.");
		delay(5000);
		break;
	case apply_control:
		if (flag) {
    unsigned long init_t = micros();

    if(serial_read_lux()){ // command is ready to be processed
      process_serial_input_command();
      x_ref = (occupancy == true) ? occupied_lux : unoccupied_lux; // decides x_ref, depending if it is occupied or not
      u_ff = (x_ref == 0) ? 0 : (x_ref - utils->static_b)/utils->static_gain; //if x_ref = 0, u_ff = 0
      t_i = micros();
      v_i = utils->get_voltage();
    }
    
    unsigned long t = micros();
    
    float y_ref = sim->calc_LDR_lux(x_ref, v_i, t_i, t);
    float y = utils->calc_lux();

    int u_sat = ctrl->run_controller(y, y_ref, u_ff);
    analogWrite(LED_PIN, u_sat);

    
    //Serial.print(t);
    //Serial.print(", ");
    //Serial.print(x_ref);
    //Serial.print(", ");
    //Serial.print(y_ref);
    //Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(u_ff);
    Serial.print(", ");
    Serial.print(u_sat);
    Serial.print(", ");
    Serial.print(y_ref - y);
    //Serial.print(", ");
    //Serial.print(get_voltage());
    Serial.println();
    /**/
    unsigned long endTime = micros();
    unsigned long elapsedTime = endTime - init_t;
    if(elapsedTime > sampInterval)
        Serial.println("ERROR: Sampling period was exceeded!");
    flag = 0;
  }
	default:
		break;
	}

	/*
  if(can_interrupt) {
    can_interrupt = false;
    if( mcp2515_overflow ) {
      Serial.println( "\t\t\t\tMCP2516 RX Buf Overflow" );
      mcp2515_overflow = false;
    }

    if( arduino_overflow ) {
      Serial.println( "\t\t\t\tArduino Buffers Overflow" );
      arduino_overflow = false;
    }
    can_frame frame;
    bool has_data;
    cli(); has_data = cf_stream->get( frame ); sei();
    while (has_data) {
      my_can_msg msg;
      for( uint8_t i = 0 ; i < frame.can_dlc ; i++ )
        msg.bytes[ i ] = frame.data[ i ];

      Serial.print( "\t\tReceiving: ID :");
      Serial.print(frame.can_id & !BROADCAST_BIT); // remove the broadcast bit
      Serial.print(" -> ");
      Serial.println(msg.value); // first byte
      cli(); has_data = cf_stream->get(frame); sei();
      //must process the message here!!!!!
      //process_recvd_msg();
    }
  }*/



}


MCP2515::ERROR write(uint32_t id, uint32_t val, uint8_t n_bytes) {
    can_frame frame;
    frame.can_id = id;
    frame.can_dlc = n_bytes;
    my_can_msg msg;
    msg.value = val; //pack data
    for( int i = 0; i < n_bytes; i++ ) //prepare can message
        frame.data[i] = msg.bytes[i];
    
    //send data
    return mcp2515.sendMessage(&frame);
}

void irqHandler()
{
    can_frame frm;
    uint8_t irq = mcp2515.getInterrupts();
    //check messages in buffer 0
    if ( irq & MCP2515::CANINTF_RX0IF ) {
        mcp2515.readMessage( MCP2515::RXB0, & frm );
    if( !cf_stream->put( frm ) ) //no space
        arduino_overflow = true;
    }
    //check messages in buffer 1
    if ( irq & MCP2515::CANINTF_RX1IF ) {
        mcp2515.readMessage( MCP2515::RXB1, & frm);
    if( !cf_stream->put( frm ) ) //no space
        arduino_overflow = true;
    }
    irq = mcp2515.getErrorFlags(); //read EFLG
    if( (irq & MCP2515::EFLG_RX0OVR) | (irq & MCP2515::EFLG_RX1OVR) ) {
        mcp2515_overflow = true;
        mcp2515.clearRXnOVRFlags();
    }
    mcp2515.clearInterrupts();
    can_interrupt = true; //notify loop()
} //end irqHandler()

/**
 * Interruption
 * Activates the flag that is used to run the loop
 */
ISR(TIMER1_COMPA_vect){
  flag = 1; //notify main loop
}




/**
 * reads 1 char from serial and concatenates it into the string serial_input
 * @returns 1 when the enter key is pressed else 0
 */
int serial_read_lux () {
  if (Serial.available()) {
    char read_byte = Serial.read();
    if(read_byte == 10){ // 10 -> new line
      serial_input[serial_input_index] = '\0';
      return 1;
    }
    serial_input[serial_input_index++] = read_byte;
  }
  return 0;
}

/**
 * Processes the command entered in the serial_input variable
 * Changes occupancy and lower bound values according to command
 */
void process_serial_input_command () {
  serial_input_index = 0;
  char* command = strtok(serial_input, " ");

  if (strcmp(command, "o") == 0) { // SET OCCUPANCY
    int val = atoi(strtok(0, " "));
    if (val == 0){
      occupancy = false;// set occupancy state to unoccupied
    }
    else if (val == 1) {
      occupancy = true;// set occupancy state to occupied
    }
  }
  else if (strcmp(command, "O") == 0) { // SET LOWER BOUND FOR OCCUPIED
    occupied_lux = atof(strtok(0, " "));
  }
  else if (strcmp(command, "U") == 0) { // SET LOWER BOUND FOR UNOCCUPIED
    unoccupied_lux = atof(strtok(0, " "));
  }

}
