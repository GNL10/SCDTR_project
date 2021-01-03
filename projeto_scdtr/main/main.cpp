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

volatile bool flag = true;
unsigned long last_state_change{0};
volatile bool wait_event;
volatile unsigned long timeout{5000000};

volatile bool has_data;

volatile bool sync_recvd = false;

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
    Serial.print("Node with ID : ");
    Serial.println(utils->my_id);

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
                    uint8_t res = utils->analyse_id_broadcast(frame.data[0],
                                                              frame.data[1]);
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
            if (utils->sync(has_data, frame, sync_recvd)) {
                Serial.println("\n\n######## SYNCHRONIZED ########\n\n");
                curr_state = State::calibrate;
            }
            break;

        case State::calibrate:
            if (utils->calibrate(has_data, frame) == true)  // if calibration is over
                curr_state = State::apply_control;
            break;

        case State::apply_control:
            if (flag) {
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
                /**/
                unsigned long endTime = micros();
                unsigned long elapsedTime = endTime - init_t;
                if (elapsedTime > sampInterval)
                    Serial.println("ERROR: Sampling period was exceeded!");
                flag = false;
                curr_state = State::negotiate;
            }
            break;
        case State::negotiate:
            if (negotiate()) {
                curr_state = State::end;
            }
            break;
        case State::end:
            break;
        default:
            break;
    }
}

bool negotiate() {
    float d[utils->id_ctr];

    if (utils->my_id == utils->lowest_id && step == 0) {
        consensus->iterate(d);

        comms::send_control_msg(utils->id_vec[1], utils->my_id, 'u', 0.5);
        step = 1;
    }
    if (has_data) {
        comms::print_msg();
        Serial.println("Calculating u");
        my_can_msg rcv_u;
        for (int i = 2; i < 6; i++)  // prepare can message
            rcv_u.bytes[i - 2] = frame.data[i];
        Serial.print("float received: ");
        Serial.println(rcv_u.value);
        comms::send_control_msg(utils->id_vec[1], utils->my_id, 'u', 0.5);
        return true;
    }

    return false;
}

void read_events() {
    if(utils->hub) {
		if (utils->serial_read_lux()) {  // command is ready to be processed
			process_serial_input_command(utils->serial_input, utils->serial_input_index);
			x_ref = (occupancy == true) ? occupied_lux : unoccupied_lux;  // decides x_ref, depending if it is occupied or not
			u_ff = (x_ref == 0) ? 0 : (x_ref - utils->o) / utils->k[utils->my_id];  // if x_ref = 0, u_ff = 0
			t_i = micros();
			v_i = utils->get_voltage();
		}
	}

    if (micros() - last_state_change > timeout) wait_event = true;
    cli(); has_data = cf_stream->get(frame); sei();

	if(has_data){
        process_can_bus_cmd(frame);
	}
    if (has_data && frame.data[0] == CAN_SYNC) {
        sync_recvd = true;
    }
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

uint8_t extract_uint8_t(char input[], uint8_t &idx) {
	char l = input[idx];
	uint8_t num = 0; 

	while (l != '\0' && l != ' ' && l != '\r'){
		num *= 10;
		num += (l - '0'); // add one digit at a time
		l = input[++idx];
	}
	idx++; //leaves idx at beginning of next num
	return num;
}

float extract_float(char input[], uint8_t &idx){
    char l = input[idx];
    float num = 0;
    bool point = false;
    float dec = 0.1;
    
    while (l != '\0' && l != ' ' && l != '\r'){
		if(l == '.'){
            point = true;
            l = input[++idx];
            continue;
        }
        if(point == true) {
            num += dec*(l - '0');
            dec *= 0.1; //decrease the decimal by another x10
        } 
        else{
            num *= 10;
		    num += (l - '0'); // add one digit at a time
        }
		l = input[++idx];
	}
	idx++; //leaves idx at beginning of next num
	return num;
}

void can_bus_cmd_get(char msg[]){
	char next_cmd = msg[2];
	uint8_t id = msg[4];

	switch (next_cmd) {
	case CMD_ILLUM:
		comms::can_bus_send_response(id, CMD_ILLUM, utils->my_id, 123.456);//utils->calc_lux());
		break;

	case CMD_DUTY_CYCLE:
		
		break;
	
	case CMD_OCCUPANCY:
		
		break;

	case CMD_OCCUPIED_ILLUM:
		break;
	case CMD_UNOCCUPIED_ILLUM:
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
		break;
	case CMD_ACCUM_ENERGY:
		break;
	case CMD_VISIBILITY_ERR:
		break;
	case CMD_FLICKER_ERR:
		break;
	default:
		Serial.println("Invalid command!");
		break;
	}
}

void process_can_bus_cmd (can_frame frame){
    char msg[CAN_MAX_DLC + 1];
    
    Serial.print("CAN BUS RECEIVING : (byte*) ");
    for(int i = 0; i < frame.can_dlc; i++) {
        Serial.print(frame.data[i]);
        Serial.print(SPACE);
        msg[i] = frame.data[i];
    }
    msg[frame.can_dlc] = '\0';
	Serial.print("\t (char*) ");
    Serial.println(msg);

	char cmd = msg[0];
	char next_cmd;
	int idx;
	uint8_t id;
	float_byte l;

	if(utils->hub) // if node is hub
		comms::forward_can_to_serial(msg);
    else{
        switch (cmd) {
        case CMD_GET:
            
            break;
        default:
            break;
        }
    }
}



void cmd_get(char serial_input[]) {		//          012345
	char next_cmd = serial_input[2]; 	// example:<g l 12>
	uint8_t idx = 4;
	uint8_t id = extract_uint8_t(serial_input, idx);

	switch (next_cmd) {
	case CMD_ILLUM: //Get current measured illuminance at desk <i>
		if(id == utils->my_id) // if this is the node being asked
            comms::serial_respond(CMD_ILLUM, id, utils->calc_lux());
		else // send the msg to the node
			comms::can_bus_send_cmd(id, CMD_GET, CMD_ILLUM, utils->my_id);
		break;

	case CMD_DUTY_CYCLE:
		if(id == utils->my_id) // if this is the node being asked
			comms::serial_respond(CMD_DUTY_CYCLE, utils->my_id, u_sat);
		else
			comms::can_bus_send_cmd(id, CMD_GET, CMD_DUTY_CYCLE, utils->my_id);
		break;
	
	case CMD_OCCUPANCY:
		if(id == utils->my_id) // if this is the node being asked
			comms::serial_respond(CMD_OCCUPANCY, id, (int)occupancy);
		else
			comms::can_bus_send_cmd(id, CMD_GET, CMD_OCCUPANCY, utils->my_id);
		break;

	case CMD_OCCUPIED_ILLUM:
		break;
	case CMD_UNOCCUPIED_ILLUM:
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
		break;
	case CMD_ACCUM_ENERGY:
		break;
	case CMD_VISIBILITY_ERR:
		break;
	case CMD_FLICKER_ERR:
		break;
	default:
		Serial.println("Invalid command!");
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
        id = extract_uint8_t(serial_input, aux_idx);
        if(id == utils->my_id){
            occupancy = (serial_input[aux_idx] == '0') ? false : true;
            Serial.println("ACK");
        }
        else 
            comms::can_bus_send_cmd(id, CMD_OCCUPANCY, utils->my_id, (serial_input[aux_idx] == '0') ? 0 : 1);
         
		break;
	
	case CMD_OCCUPIED_ILLUM: // Set lower bound on illuminance for Occupied state at desk <i>
        id = extract_uint8_t(serial_input, aux_idx);
        val = extract_float(serial_input, aux_idx);
        Serial.print("FLOAT VALUE : "); Serial.println(val);
		break;
	
	case CMD_UNOCCUPIED_ILLUM: // Set lower bound on illuminance for Unccupied state at desk <i>
	
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

    mcp2515.setFilterMask(MCP2515::MASK0, 0, CAN_SFF_MASK);
    mcp2515.setFilter(MCP2515::RXF0, 0, (uint32_t)utils->my_id);  // own id

    mcp2515.setFilterMask(MCP2515::MASK1, 0, CAN_SFF_MASK);
    mcp2515.setFilter(MCP2515::RXF2, 0, CAN_BROADCAST_ID);  // own id
    // TODO if isHub, set mask to accept everything

    mcp2515.setNormalMode();
    // use mcp2515.setLoopbackMode() for local testing
    // mcp2515.setLoopbackMode();
}