#include <math.h>

#include "comms.h"
#include "can_frame_stream.h"
#include "configs.h"
#include "controller.h"
#include "simulator.h"
#include "utils.h"
#include "consensus.h"


// can bus comms
can_frame_stream *cf_stream = new can_frame_stream();
can_frame frame;

extern MCP2515 mcp2515;  // defined in comms.cpp

// notification flag for ISR and loop()
volatile bool can_interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;

volatile float v_i = 0;
volatile unsigned long t_i = 0;

int u_sat;
volatile float u_ff = 0;
volatile float x_ref = 0;

int step = 0;  // initialize time step for control

volatile bool occupancy = false;
volatile float occupied_lux = 50;
volatile float unoccupied_lux = 20;

Utils *utils;
Simulator *sim;
Controller *ctrl;
Consensus *consensus;

// case 1
int L1 = 40, o1 = 30, L2 = 30, o2 = 0;
// case 2
//int L1 = 80, o1 = 50, L2 = 150, o2 = 50;
// case 3
//int L1 = 80, o1 = 50, L2 = 270, o2 = 50;

// symmetric costs
int cost_1 = 1, cost_2 = 1;
// asymmetric costs
//int cost_1 = 1, cost_2 = 3;

float k11 = 2, k12 = 1;
float k21 = 1, k22 = 2;
float gains1[] {k11, k12};
float gains2[] {k21, k22};


volatile bool flag = true; // flag for control interrupt
unsigned long last_state_change{0};
volatile bool wait_event;
volatile unsigned long timeout;

volatile bool has_data;

bool sync_recvd = false;
bool ack_recvd = false;

unsigned long start_time{0}; // time at which the arduino was turned on, or reset

enum class State : byte {
    start,
    send_id_broadcast,
    wait_for_ids,
    sync,
    calibrate,
    apply_control,
    negotiate,
    end
};  // states of the system
State curr_state = State::start;

ISR(TIMER1_COMPA_vect);

void read_events();
void process_can_bus_cmd (can_frame frame);
void process_serial_input_command(char serial_input[], uint8_t &idx);
void irqHandler();
void control_interrupt_setup();
void can_bus_setup();
bool negotiate();


void setup() {
    Serial.begin(115200);
    utils = new Utils();
    Serial.println();
    Serial.print("######## ID : ");
    Serial.print(utils->my_id);
    Serial.println(" ########");

    utils->isHub();

    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
    TCCR2B = (TCCR2B & B11111000) | B00000001;
    pinMode(LED_PIN, OUTPUT);

    sim = new Simulator(utils->m, utils->b, R1, utils->C1, VCC);
    ctrl = new Controller(error_margin, K1, K2, true);

    control_interrupt_setup();
    can_bus_setup();
}

void loop() {
    read_events();

    // process events ? like reset and stuff like that

    switch (curr_state) {
        case State::start:
            start_time = millis();
            curr_state = State::send_id_broadcast;
            break;

        case State::send_id_broadcast:
            Serial.print("Sending broadcast with ID : ");
            Serial.println(utils->my_id);
            if (!comms::broadcast(utils->my_id, CAN_NEW_ID))  // send its own id
                Serial.println(TX_BUF_FULL_ERR);

            last_state_change = micros();
            timeout = WAIT_ID_TIME;
            wait_event = false;
            curr_state = State::wait_for_ids;
            break;

        case State::wait_for_ids:
            if (wait_event || sync_recvd) {
                utils->order_ids();
                curr_state = State::sync;
            } else {
                if (has_data) {
                    uint8_t res = utils->analyse_id_broadcast(frame.can_id);
                    if (res == 3)  // received msg was not of type CAN_NEW_ID
                        break;
                    else if (res == 2){  // max number of nodes has been reached
                        utils->order_ids();
                        curr_state = State::sync;  // stop the search and move on to calibration
                    }
                    else if (res == 1)  // if the id was correctly added
                        curr_state = State::send_id_broadcast;  // resend own id

                    Serial.print("Current number of nodes: ");
                    Serial.println(utils->id_ctr);
                }
            }
            break;

        case State::sync:
            if (utils->sync(sync_recvd, ack_recvd, comms::get_src_id(frame.can_id))) {
                Serial.println("\n\n######## SYNCHRONIZED ########\n\n");
                curr_state = State::calibrate;
            }
            break;

        case State::calibrate:
            if (utils->calibrate(has_data, frame) == true){ // if calibration is over
                if(utils->my_id == utils->lowest_id)
                    consensus = new Consensus(utils->find_id(utils->my_id), L1, utils->o, utils->k, cost_1, utils->id_ctr);
                else 
                    consensus = new Consensus(utils->find_id(utils->my_id), L2, utils->o, utils->k, cost_2, utils->id_ctr);
                Serial.println("\n\n######## CALIBRATED ########\n\n");
                curr_state = State::negotiate;
            }  
            break;

        case State::apply_control:
            if (flag) {
                Serial.println("IN APPLY CONTROL");
                unsigned long init_t = micros();

                unsigned long t = micros();

                float y_ref = sim->calc_LDR_lux(x_ref, v_i, t_i, t);
                float y = utils->calc_lux();

                u_sat = ctrl->run_controller(y, y_ref, u_ff);
                analogWrite(LED_PIN, u_sat);

                // Serial.print(t); // Serial.print(", "); //Serial.print(x_ref); // Serial.print(", ");// Serial.print(y_ref);// Serial.print(", ");
                
                Serial.print(y);
                Serial.print(", ");
                Serial.print(u_ff);
                Serial.print(", ");
                Serial.print(u_sat);
                Serial.print(", ");
                Serial.print(y_ref - y);
                // Serial.print(", ");// Serial.print(get_voltage());
                Serial.println();
                
                unsigned long endTime = micros();
                unsigned long elapsedTime = endTime - init_t;
                if (elapsedTime > sampInterval)
                    Serial.println("ERROR: Sampling period was exceeded!");
                flag = false;
            }
            break;
        case State::negotiate:
            if (consensus->negotiate(frame, has_data, utils->my_id)) {
                Serial.println("\n\n######## NEGOTIATED ########\n\n");
                if(utils->my_id == utils->lowest_id)
                    x_ref = L1;
                else
                    x_ref = L2;
                
                u_ff = consensus->d_av[utils->my_id]*255/100;
                t_i = micros();
			    v_i = utils->get_voltage();
                curr_state = State::apply_control;
            }
            break;
        case State::end:
            break;
        default:
            break;
    }
}

void read_events() {
    if(utils->hub) {
		if (utils->serial_read_lux()) {  // command is ready to be processed
			process_serial_input_command(utils->serial_input, utils->serial_input_index);
			x_ref = (occupancy == true) ? occupied_lux : unoccupied_lux;  // decides x_ref, depending if it is occupied or not
            //Negotiate
            u_ff = (x_ref == 0) ? 0 : (x_ref - utils->o) / utils->k[utils->my_id];  // if x_ref = 0, u_ff = 0
			t_i = micros();
			v_i = utils->get_voltage();
		}
	}

    if (micros() - last_state_change > timeout) 
        wait_event = true; // timeout
    cli(); has_data = cf_stream->get(frame); sei();

    if( mcp2515_overflow ) {
        Serial.println( "\t\t\t\tMCP2516 RX Buf Overflow" );
        mcp2515_overflow = false;
    }

    if( arduino_overflow ) {
        Serial.println( "\t\t\t\tArduino Buffers Overflow" );
        arduino_overflow = false;
    }

	if(has_data){
        process_can_bus_cmd(frame);
	}
    /*
    if (changed == true) {
        x_ref = (occupancy == true) ? occupied_lux : unoccupied_lux;  // decides x_ref, depending if it is occupied or not
        u_ff = (x_ref == 0) ? 0 : (x_ref - utils->o) / utils->k[utils->my_id];  // if x_ref = 0, u_ff = 0
        t_i = micros();
        v_i = utils->get_voltage();
        //start negotiate
    }*/
}

void irqHandler() {
    can_frame frm;
    uint8_t irq = mcp2515.getInterrupts();
    // check messages in buffer 0
    if (irq & MCP2515::CANINTF_RX0IF) {
        mcp2515.readMessage(MCP2515::RXB0, &frm);
        if (!cf_stream->put(frm))  // no space
            arduino_overflow = true;
    }
    // check messages in buffer 1
    if (irq & MCP2515::CANINTF_RX1IF) {
        mcp2515.readMessage(MCP2515::RXB1, &frm);
        if (!cf_stream->put(frm))  // no space
            arduino_overflow = true;
    }
    irq = mcp2515.getErrorFlags();  // read EFLG
    if ((irq & MCP2515::EFLG_RX0OVR) | (irq & MCP2515::EFLG_RX1OVR)) {
        mcp2515_overflow = true;
        mcp2515.clearRXnOVRFlags();
    }
    mcp2515.clearInterrupts();
    can_interrupt = true;  // notify loop()
}  // end irqHandler()

/**
 * Interruption
 * Activates the flag that is used to run the loop
 */
ISR(TIMER1_COMPA_vect) {
    flag = 1;  // notify main loop
}

void process_rtr_req(uint32_t can_id){
	Serial.println("I RECEIVED A RTR REQUEST !!");
	uint8_t cmd = comms::get_cmd(can_id);
	uint8_t src_id = comms::get_src_id(can_id);

	switch (cmd){
	case CAN_GET_ILLUM: 
		comms::can_bus_send_val(src_id, utils->my_id, CAN_GET_ILLUM, utils->calc_lux());
		break;
	case CAN_GET_DUTY_CYCLE:
		comms::can_bus_send_val(src_id, utils->my_id, CAN_GET_DUTY_CYCLE, (float)u_sat/255*100);
		break;
	case CAN_GET_OCCUPANCY: 
		comms::can_bus_send_val(src_id, utils->my_id, CAN_GET_OCCUPANCY, occupancy);
		break;	
	case CAN_GET_OCCUPIED_ILLUM: 
		comms::can_bus_send_val(src_id, utils->my_id, CAN_GET_OCCUPIED_ILLUM, occupied_lux);
		break;
	case CAN_GET_UNOCCUPIED_ILLUM: 
		comms::can_bus_send_val(src_id, utils->my_id, CAN_GET_UNOCCUPIED_ILLUM, unoccupied_lux);
		break;
	case CAN_GET_ILLUM_LB:  // lower bound
		break;
	case CAN_GET_EXT_ILLUM: 
		break;
	case CAN_GET_CONTROL_REF: 
		break;
	case CAN_GET_ENERGY_COST: 
		break;
	case CAN_GET_POWER_CONSUMPTION: 
		break;
	case CAN_GET_POWER_CONSUMPTION_TOTAL: 
		break;
	case CAN_GET_TIME:
        Serial.print("TIME : "); Serial.println((millis() - start_time)/1000);
        comms::can_bus_send_val(src_id, utils->my_id, CAN_GET_TIME, (millis() - start_time)/1000);
		break;
	case CAN_GET_ACCUM_ENERGY: 
		break;
	case CAN_GET_VISIBILITY_ERR: 
		break;
	case CAN_GET_VISIBILITY_ERR_TOTAL: 
		break;
	case CAN_GET_FLICKER_ERR: 
		break;
	case CAN_GET_FLICKER_ERR_TOTAL:
		break;
	case CAN_SET_OCCUPIED_LUX:

        break;
	default:
		break;
	}
}

void process_can_bus_cmd (can_frame frame){
    Serial.print("RECEIVED NEW MSG :  ID: ");
    Serial.print(frame.can_id, BIN);
    Serial.print("\t CMD :");
    Serial.println(frame.can_id & CMD_MASK);
    
    uint8_t cmd = comms::get_cmd(frame.can_id);
	uint8_t src_id = comms::get_src_id(frame.can_id);
    float aux_float = 0;
    unsigned long aux_ulong = 0;

	if (frame.can_id & CAN_RTR_FLAG)
		process_rtr_req(frame.can_id);

	else {
		if(utils->hub){
            switch (cmd)
            {
            case CAN_GET_ILLUM:
                aux_float = comms::get_float();
                comms::serial_respond(CMD_ILLUM, src_id, aux_float);
                break;
            case CAN_GET_DUTY_CYCLE:
                aux_float = comms::get_float();
                comms::serial_respond(CMD_DUTY_CYCLE, src_id, aux_float);
                break;
            case CAN_GET_OCCUPANCY: 
                aux_float = comms::get_float();
                comms::serial_respond(CMD_OCCUPANCY, src_id, aux_float);
                break;	
            case CAN_GET_OCCUPIED_ILLUM: 
                aux_float = comms::get_float();
                comms::serial_respond(CMD_OCCUPIED_ILLUM, src_id, aux_float);
                break;
            case CAN_GET_UNOCCUPIED_ILLUM: 
                aux_float = comms::get_float();
                comms::serial_respond(CMD_UNOCCUPIED_ILLUM, src_id, aux_float);
                break;
            case CAN_GET_ILLUM_LB:  // lower bound
                aux_float = comms::get_float();
                comms::serial_respond(CMD_ILLUM_LB, src_id, aux_float);
                break;
		    case CAN_GET_EXT_ILLUM: 
                aux_float = comms::get_float();
                comms::serial_respond(CMD_EXT_ILLUM, src_id, aux_float);
			    break;
            case CAN_GET_CONTROL_REF: 
                break;
            case CAN_GET_ENERGY_COST: 
                break;
            case CAN_GET_POWER_CONSUMPTION: 
                break;
            case CAN_GET_POWER_CONSUMPTION_TOTAL: 
                break;
            case CAN_GET_TIME: 
                aux_ulong = comms::get_ulong();
                Serial.print("TIME : "); Serial.println(aux_ulong);
                comms::serial_respond(CMD_TIME, src_id, aux_ulong);
                break;
            case CAN_GET_ACCUM_ENERGY: 
                break;
            case CAN_GET_VISIBILITY_ERR: 
                break;
            case CAN_GET_VISIBILITY_ERR_TOTAL: 
                break;
            case CAN_GET_FLICKER_ERR: 
                break;
            case CAN_GET_FLICKER_ERR_TOTAL:
                break;
            default:
                break;
            }
		}
		
		switch (cmd){
		case CAN_NEW_ID:
			break;
		case CAN_SYNC:
			sync_recvd = true;
			break;
		case CAN_ACK:
			ack_recvd = true;
			break;
		
        case CAN_SET_OCCUPANCY:
            occupancy = comms::get_bool(frame);
            comms::send_msg(src_id, utils->my_id, CAN_ACK);
            Serial.print("Setting occupancy to : ");
            Serial.println(occupancy);
            break;
        case CAN_SET_OCCUPIED_LUX:
            occupied_lux = comms::get_float();
            Serial.print("Setting occupied lux to : ");
            Serial.println(occupied_lux);
            break;
        case CAN_SET_UNOCCUPIED_LUX:
            unoccupied_lux = comms::get_float();
            Serial.print("Setting unoccupied lux to : ");
            Serial.println(unoccupied_lux);
            break;

		default:
			break;
		}
	}    
}


void cmd_get(char serial_input[]) {		//          012345
	char next_cmd = serial_input[2]; 	// example:<g l 12>
	uint8_t idx = 4;
	uint8_t id = comms::extract_uint8_t(serial_input, idx);

	switch (next_cmd) {
	case CMD_ILLUM: //Get current measured illuminance at desk <i>
		if(id == utils->my_id) // if this is the node being asked
            comms::serial_respond(CMD_ILLUM, id, utils->calc_lux());
		else // send the msg to the node
			comms::send_msg(id, utils->my_id, CAN_GET_ILLUM, true);
		break;

	case CMD_DUTY_CYCLE:
		if(id == utils->my_id) // if this is the node being asked
			comms::serial_respond(CMD_DUTY_CYCLE, utils->my_id, (float)u_sat/255*100);
		else
			comms::send_msg(id, utils->my_id, CAN_GET_DUTY_CYCLE, true);
		break;
	
	case CMD_OCCUPANCY:
		if(id == utils->my_id) // if this is the node being asked
			comms::serial_respond(CMD_OCCUPANCY, id, (int)occupancy);
		else
			comms::send_msg(id, utils->my_id, CAN_GET_OCCUPANCY, true);
		break;

	case CMD_OCCUPIED_ILLUM:
        if(id == utils->my_id) // if this is the node being asked
            comms::serial_respond(CMD_OCCUPIED_ILLUM, id, occupied_lux);
        else
            comms::send_msg(id, utils->my_id, CAN_GET_OCCUPIED_ILLUM, true);
		break;
	case CMD_UNOCCUPIED_ILLUM:
        if(id == utils->my_id) // if this is the node being asked
            comms::serial_respond(CMD_UNOCCUPIED_ILLUM, id, unoccupied_lux);
        else
            comms::send_msg(id, utils->my_id, CAN_GET_UNOCCUPIED_ILLUM, true);
		break;
	case CMD_ILLUM_LB:
		break;
	case CMD_EXT_ILLUM:
		break;
	case CMD_CONTROL_REF:
		break;
	case CMD_ENERGY_COST:
		break;
	case CMD_POWER_CONSUPTION:
		break;
	case CMD_TIME:
        if(id == utils->my_id) // if this is the node being asked
            comms::serial_respond(CMD_TIME, id, (millis() - start_time)/1000);
        else
            comms::send_msg(id, utils->my_id, CAN_GET_TIME, true);
		break;
	case CMD_ACCUM_ENERGY:
		break;
	case CMD_VISIBILITY_ERR:
		break;
	case CMD_FLICKER_ERR:
		break;
	default:
		break;
	}
}

/**
 * Processes the command entered in the serial_input variable
 * Changes occupancy and lower bound values according to command
 */
void process_serial_input_command(char serial_input[], uint8_t &idx) {
	char command = serial_input[0]; // first part of command
	uint8_t id, aux_idx = 2;
    float val;

	switch (command) {
	case CMD_GET: // get parameter
		cmd_get(serial_input);
		break;
	
	case CMD_OCCUPANCY: // set occupancy
        id = comms::extract_uint8_t(serial_input, aux_idx);
        if(id == utils->my_id){
            occupancy = (serial_input[aux_idx] == '0') ? false : true;
            Serial.println(SERIAL_ACK);
        }
        else 
            comms::can_bus_send_val(id, utils->my_id, CAN_SET_OCCUPANCY, (serial_input[aux_idx] == '0') ? false : true);
		break;
	
	case CMD_OCCUPIED_ILLUM: // Set lower bound on illuminance for Occupied state at desk <i>
        id = comms::extract_uint8_t(serial_input, aux_idx);
        val = comms::extract_float(serial_input, aux_idx);
        Serial.print("FLOAT VALUE : "); Serial.println(val);
        if(id == utils->my_id){
            occupied_lux = val;
            Serial.println(SERIAL_ACK);
        }
        else
            comms::can_bus_send_val(id, utils->my_id, CAN_SET_OCCUPIED_LUX, val);
		break;
	
	case CMD_UNOCCUPIED_ILLUM: // Set lower bound on illuminance for Unccupied state at desk <i>
        id = comms::extract_uint8_t(serial_input, aux_idx);
        val = comms::extract_float(serial_input, aux_idx);
        Serial.print("FLOAT VALUE : "); Serial.println(val);
        if(id == utils->my_id){
            unoccupied_lux = val;
            Serial.println(SERIAL_ACK);
        }
        else
            comms::can_bus_send_val(id, utils->my_id, CAN_SET_UNOCCUPIED_LUX, val);
		break;

	case CMD_ENERGY_COST: // Set current energy cost at desk <i>
	
		break;
	
	case CMD_RESET: // Restart system
		// go to reset or start state
		break;

	default:
		Serial.println("Invalid command!");
		break;
	}	
	idx = 0;
}


void control_interrupt_setup() {
    cli();       // disable interrupts
    TCCR1A = 0;  // clear register
    TCCR1B = 0;  // clear register
    TCNT1 = 0;   // reset counter

    // OCR1A = desired_period/clock_period – 1
    // = clock_freq/desired_freq - 1
    // = (16*10^6Hz / 8) / (100Hz) – 1
    OCR1A = 19999;           // must be <65536
    TCCR1B |= (1 << WGM12);  // CTC On
    // Set prescaler for 8
    TCCR1B |= (1 << CS11);

    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();  // enable interrupts
}

void can_bus_setup() {
    /* CAN BUS SETUP */
    SPI.begin();
    attachInterrupt(0, irqHandler, FALLING);  // use interrupt at pin 2
    // Must tell SPI lib that ISR for interrupt vector zero will be using SPI
    SPI.usingInterrupt(0);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);

    mcp2515.setFilterMask(MCP2515::MASK0, 0,  CAN_BROADCAST_ID << TO_SHIFT);
    mcp2515.setFilter(MCP2515::RXF0, 0, utils->my_id << TO_SHIFT);  // own id
    Serial.println();

    mcp2515.setFilterMask(MCP2515::MASK1, 0, CAN_BROADCAST_ID << TO_SHIFT);
    mcp2515.setFilter(MCP2515::RXF2, 0, CAN_BROADCAST_ID << TO_SHIFT);  // broadcast id
    // TODO if isHub, set mask to accept everything

    mcp2515.setNormalMode();
    // use mcp2515.setLoopbackMode() for local testing
    // mcp2515.setLoopbackMode();
}