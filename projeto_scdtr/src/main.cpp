#include <math.h>
#include <string.h>
#include "configs.h"
#include "simulator.h"
#include "controller.h"

//  /home/gnl/.platformio/penv/bin/pio device monitor --baud 115200

// kp a 0
//aproximar a funcao do tau !!

const unsigned long sampInterval = 10000; //microseconds 100Hz

int counter = 0;
int pwm_pos = 0;
float gain = 0;
int read_lux = 0;

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
int serial_read_lux ();
void process_serial_input_command();
float get_lux ();
void calc_gain ();
float get_voltage();



void setup() {
  Serial.begin(115200);
  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  pinMode(LED_PIN, OUTPUT);

  calc_gain();
  Serial.print("\n\nGain [lux/dc]: ");
  Serial.println(static_gain);
  sim = new Simulator(m, b, R1, C1, VCC);
  ctrl = new Controller(error_margin, true, K1, K2);

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
    
    /* for simulation results
    float y_ref = sim->calc_LDR_voltage(x_ref, v_i, t_i, t);
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
    Serial.println(get_lux());*/

    float y_ref = sim->calc_LDR_lux(x_ref, v_i, t_i, t);
    float y = get_lux();

    float debug[3];

    int u_sat = ctrl->run_controller(y, y_ref, u_ff, debug);
    analogWrite(LED_PIN, u_sat);

    Serial.print(t);
    Serial.print(", ");
    Serial.print(x_ref);
    Serial.print(", ");
    Serial.print(y_ref);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(debug[0]);
    Serial.print(", ");
    Serial.print(debug[1]);
    Serial.print(", ");
    Serial.print(debug[2]);
    Serial.print(", ");
    Serial.print(u_ff);
    Serial.print(", ");
    Serial.print(u_sat);
    Serial.print(", ");
    Serial.print(get_voltage());

    Serial.println();

    unsigned long endTime = micros();
    unsigned long elapsedTime = endTime - init_t;
    if(elapsedTime > sampInterval)
        Serial.println("ERROR: Sampling period was exceeded!");
    flag = 0;
  }
}



ISR(TIMER1_COMPA_vect){
  flag = 1; //notify main loop
}

// reads 1 char from serial and concatenates it into the string serial_input
// returns 1 when the enter key is pressed
// else returns 0
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


//Gets the voltage value from the analog input and returns the lux value
float get_lux () {
  float V_AD = float(analogRead(LUX_PIN)*5)/1023;
  float R_lux = (R1/V_AD)*VCC - R1; 
  return pow(10, (log10(R_lux)-b)/m);
}

float get_voltage() {
  return float(analogRead(LUX_PIN)*VCC)/1023;
}

void calc_gain () {
  int max_pwm = 255;
  int min_pwm = 6;

  analogWrite(LED_PIN, max_pwm);
  delay(5000);
  float max_lux = get_lux();  
  
  /*analogWrite(LED_PIN, min_pwm);
  delay(1000);
  float min_lux = get_lux();
  static_gain = (max_lux - min_lux) / (max_pwm-min_pwm);
  static_b = min_lux - static_gain*min_pwm;  
  */

  static_gain = max_lux/max_pwm;
  Serial.println(max_lux);
  static_b = 0;  
  analogWrite(LED_PIN, 0);
  delay(400);
}
