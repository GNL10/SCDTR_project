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
can_frame frame;

extern MCP2515 mcp2515; // defined in can_bus_comms.cpp

//notification flag for ISR and loop()
volatile bool can_interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;

const unsigned long sampInterval = 10000; //microseconds 100Hz

float v_i = 0;
unsigned long t_i = 0;

float u_ff = 0;
float x_ref = 0;
int step = 0; //initialize time step for control

bool occupancy = false;
float occupied_lux = 50;
float unoccupied_lux = 20;

uint8_t my_id; 
uint8_t master_id;

Utils *utils;
Simulator *sim;
Controller *ctrl;

volatile bool flag;
unsigned long last_state_change{0};
bool wait_event;
unsigned long timeout{5000000};

bool has_data;

bool sync_sent = false;
bool sync_recvd = false;
uint8_t ack_ctr = 0;

enum class State : byte {start, send_id_broadcast, wait_for_ids, sync, calibrate, apply_control, negotiate, end}; // states of the system
State curr_state = State::start;

ISR(TIMER1_COMPA_vect);
void read_events();
void process_serial_input_command (char serial_input[], int &idx);
void irqHandler();
bool negotiate();


void setup() {
  Serial.begin(115200);
  utils = new Utils();
  Serial.print("Node with ID : ");Serial.println(utils->id_vec[0]);

  utils->isHub();
  my_id = utils->id_vec[0];

  // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR2B = (TCCR2B & B11111000) | B00000001;
  pinMode(LED_PIN, OUTPUT);

  //utils->calc_gain();

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
  
  mcp2515.setFilterMask(MCP2515::MASK1, 0, CAN_SFF_MASK);
  mcp2515.setFilter(MCP2515::RXF2, 0, CAN_BROADCAST_ID); // own id
  //TODO if isHub, set mask to accept everything

  mcp2515.setNormalMode();
  // use mcp2515.setLoopbackMode() for local testing
  //mcp2515.setLoopbackMode();

}


void loop() {
  read_events();

  // process events ? like reset and stuff like that

	switch (curr_state)
	{
	case State::start:
		curr_state = State::send_id_broadcast;
		break;
    
	case State::send_id_broadcast:
		Serial.print("Sending broadcast with ID : "); Serial.println(utils->id_vec[0]);
    if(!broadcast(my_id, CAN_NEW_ID)) // send its own id
      Serial.println("\t\t\t\tMCP2515 TX Buf Full");
    
    last_state_change = micros();
    timeout = WAIT_ID_TIME;
    wait_event = false;
		curr_state = State::wait_for_ids;
		break;

	case State::wait_for_ids:
    if(wait_event || sync_recvd){
      curr_state = State::sync;
    }
    else {
      if (has_data) {
        uint8_t res = analyse_id_broadcast(frame.data[0], frame.data[1], utils);
        if(res == 3) // received msg was not of type CAN_NEW_ID
          break;
        else if (res == 2) // max number of nodes has been reached
          curr_state = State::sync; // stop the search and move on to calibration
        else if (res == 1) // if the id was correctly added
          curr_state = State::send_id_broadcast; // resend own id

        Serial.print("Current number of nodes: "); Serial.println(utils->id_ctr);
      }
    }
		break;

  case State::sync:
    if(utils->lowest_id == utils->id_vec[0]){ //if this is lowest id
      if(!sync_sent){
        sync_sent = true;
        if(!broadcast(my_id, CAN_SYNC)) // send its own id
          Serial.println("\t\t\t\tMCP2515 TX Buf Full");
      }
      else {
        if(has_data && frame.data[0] == CAN_ACK){
          if((++ack_ctr) == utils->id_ctr - 1){
            ack_ctr = 0;
            curr_state = State::calibrate;
          }
        }
      }
      
    }
    else{ // normal node, waits for CAN_SYNC msg
      if(sync_recvd){
        if(!send_msg(frame.data[1], my_id, CAN_ACK))
			    Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
          curr_state = State::calibrate;
      }
    }
    break;

	case State::calibrate:

    if(my_id==utils->lowest_id)
    {
      utils->calc_residual_lux();

      if(!broadcast(my_id, 'r')) //broadcast "Measure Residual Lux"+my_id
          Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );

      analogWrite(LED_PIN, 255); //Light on
      delay(LED_WAIT_TIME);

      utils->calc_gain(my_id);

      if(!broadcast(my_id, 'm')) //broadcast "Measure"+my_id
          Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
      delay(MEASURE_WAIT_TIME);
      Serial.println("Light off");
      analogWrite(LED_PIN, 0); //Light off

    for(int i = 1; i<utils->id_ctr; i++)
    {     
      if(!send_msg(utils->id_vec[i], my_id, 'l')) //send "Light on"
			  Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
      delay(LED_WAIT_TIME);

      utils->calc_gain(utils->id_vec[i]);

      if(!broadcast(utils->id_vec[i], 'm')) //broadcast "Measure"+id
			  Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
      
      delay(MEASURE_WAIT_TIME);
      if(!send_msg(utils->id_vec[i], my_id, 'o')) //send "Light off"
			  Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );
    }

    Serial.println("Broadcast calibration complete");
    if(!broadcast(my_id, 'c')) //broadcast "Calibration Complete"
			  Serial.println( "\t\t\t\tMCP2515 TX Buf Full" );

    curr_state = State::apply_control;

  }

  else{
    if (has_data) 
    {
      if(frame.data[0] == 114) //'r' in decimal
      {
        print_msg();
        master_id = frame.data[1];
        Serial.println("Calculating residual lux."); 
        utils->calc_residual_lux();
      }
      if(frame.data[0] == 108) //'l' in decimal
      {
        print_msg();
        analogWrite(LED_PIN, 255); //Light on
        Serial.println("Light on.");
      }

      if(frame.data[0] == 111) //'o' in decimal
      {
        print_msg();
        analogWrite(LED_PIN, 0); //Light off
        Serial.println("Light off.");
      }
      if(frame.data[0] == 109) //'m' in decimal
      {
        print_msg();
        Serial.println("Calculating gain.");
        utils->calc_gain(frame.data[1]);          
      }

      if(frame.data[0] == 99) //'c' in decimal
      {
        print_msg();
        Serial.println("Calibration complete");
        curr_state = State::apply_control;
      }

      for (int i=0; i<utils->id_ctr; i++)
      {
        Serial.print("k");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(utils->k[i]);
      }
    }
  }
  //listening
  //if receives "Measure"
    //measure and save k[id_sender]
    //send ACK
  //else if receives "Light on"
    //light on
    //send ACK
    //measure and save k[own_id]

    break;
	case State::apply_control:
		if (flag) {
      unsigned long init_t = micros();

      if(utils->serial_read_lux()){ // command is ready to be processed
        process_serial_input_command(utils->serial_input, utils->serial_input_index);
        x_ref = (occupancy == true) ? occupied_lux : unoccupied_lux; // decides x_ref, depending if it is occupied or not
        u_ff = (x_ref == 0) ? 0 : (x_ref - utils->o)/utils->k[utils->id_vec[0]]; //if x_ref = 0, u_ff = 0
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
      curr_state = State::negotiate;
    }

    
    break;
  case State::negotiate:
    if(negotiate()){
      curr_state = State::end;
    }
    break;
  case State::end:
    break;
  default: 
    break;
	}

}

bool negotiate(){

  if(my_id == utils->lowest_id && step == 0){
        send_control_msg(utils->id_vec[1], my_id, 'u', 0.5);
        step = 1;
  }
  if(has_data){
    print_msg();
    Serial.println("Calculating u");
    my_can_msg rcv_u;
    for( int i = 2; i < 6; i++ ) //prepare can message
      rcv_u.bytes[i-2] = frame.data[i];
    Serial.print("float received: ");
    Serial.println(rcv_u.value) ; 
    send_control_msg(utils->id_vec[1], my_id, 'u', 0.5);
    return true;
  }
  
  return false;
}



void read_events() {
  if(micros() - last_state_change > timeout)
    wait_event = true;
  cli(); has_data = cf_stream->get(frame); sei();
  if(has_data)
    print_msg();
  sync_recvd = (frame.data[0] == CAN_SYNC) ? true : false;
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
 * Processes the command entered in the serial_input variable
 * Changes occupancy and lower bound values according to command
 */
void process_serial_input_command (char serial_input[], int &idx) {
  idx = 0;
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
