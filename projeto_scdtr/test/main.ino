#include <math.h>
#include <string.h>
#include <EEPROM.h>
#include "configs.h"
#include "simulator.h"
#include "controller.h"

//  /home/gnl/.platformio/penv/bin/pio device monitor --baud 115200

// kp a 0

float C1, m, b; // variables saved in EEPROM

const unsigned long sampInterval = 10000; //microseconds 100Hz

char serial_input[SERIAL_INPUT_SIZE+1];
int serial_input_index = 0;

float static_gain;
float static_b;

float v_i = 0;
unsigned long t_i = 0;

float u_ff = 0;
float x_ref = 0;

bool occupancy = false;
float occupied_lux = 50;
float unoccupied_lux = 20;

Simulator* sim;
Controller* ctrl;

volatile bool flag;

ISR(TIMER1_COMPA_vect);
void load_EEPROM_vars();
int serial_read_lux ();
void process_serial_input_command();
float calc_lux (float V_AD);
void calc_gain ();
float get_voltage();


void setup() {
  Serial.begin(115200);
  load_EEPROM_vars();

  // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR2B = TCCR2B & B11111000 | B00000001;
  pinMode(LED_PIN, OUTPUT);

  calc_gain();
  Serial.print("\n\nGain [lux/dc]: ");
  Serial.println(static_gain);
  sim = new Simulator(m, b, R1, C1, VCC);
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
}


void loop() {
  if (flag) {
    unsigned long init_t = micros();

    if(serial_read_lux()){ // command is ready to be processed
      process_serial_input_command();
      x_ref = (occupancy == true) ? occupied_lux : unoccupied_lux; // decides x_ref, depending if it is occupied or not
      u_ff = (x_ref == 0) ? 0 : (x_ref - static_b)/static_gain; //if x_ref = 0, u_ff = 0
      t_i = micros();
      v_i = get_voltage();
    }
    
    unsigned long t = micros();
    
    // for simulation results
    /*
    float trash;
    float y_ref = sim->calc_LDR_voltage(x_ref, v_i, t_i, t - TETA*1000000, &trash);
    float y = get_voltage();
    u_ff = round(u_ff);
    analogWrite(LED_PIN, u_ff);
    Serial.print(t);
    Serial.print(", ");
    Serial.print(u_ff);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(y_ref);
    Serial.print(", ");
    Serial.println(calc_lux(get_voltage()));
    /**/

    
    float y_ref = sim->calc_LDR_lux(x_ref, v_i, t_i, t);
    float y = calc_lux(get_voltage());

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
}


/**
 * Interruption
 * Activates the flag that is used to run the loop
 */
ISR(TIMER1_COMPA_vect){
  flag = 1; //notify main loop
}

/**
 * Loads the values from the EEPROM to the variables C1, m and b of the program
 */
void load_EEPROM_vars() {
  int address = 0;
  
  EEPROM.get(address, C1);
  address += sizeof(C1);
  EEPROM.get(address, m);
  address += sizeof(m);
  EEPROM.get(address, b);
  address += sizeof(b);

  Serial.println();
  Serial.println("### EEPROM VALUES ###");
  Serial.print("C1 : ");
  Serial.println(C1, 7);
  Serial.print("m : ");
  Serial.println(m, 7);
  Serial.print("b : ");
  Serial.println(b, 7);
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

/**
 * Calculates the lux value from a voltage value
 * @param V_AD Voltage at LDR
 * @returns Lux value at LDR
 */
float calc_lux (float V_AD) {
  float R_lux = (R1/V_AD)*VCC - R1; 
  return pow(10, (log10(R_lux)-b)/m);
}

/**
 * Reads the value from LUX_PIN and converts it to volts
 * @returns voltage at LDR
 */
float get_voltage() {
  return float(analogRead(LUX_PIN)*VCC)/1023;
}

/**
 * Calculates the gain [lux / duty cycle] of the system
 * @returns Gain [lux / duty cycle] to the static_gain global variable
 */
void calc_gain () {
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