#include <math.h>
#include <string.h>
#include "configs.h"
#include "simulator.h"
#include "controller.h"

//  /home/gnl/.platformio/penv/bin/pio device monitor --baud 115200

int counter = 0;
int pwm_pos = 0;
float gain = 0;
int read_lux = 0;

float v_i = 0;
unsigned long t_i = 0;

int u_ff = 0;
int x_ref = 0;

Simulator* sim;
Controller* ctrl;

volatile bool flag;

ISR(TIMER1_COMPA_vect);
int serial_read_lux (int *ref);
float get_lux ();
float calc_gain ();
float get_voltage();



void setup() {
  Serial.begin(115200);
  //TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  pinMode(LED_PIN, OUTPUT);

  gain = calc_gain();
  Serial.print("\n\nGain [lux/dc]: ");
  Serial.println(gain);
  sim = new Simulator(gain, m, b, R1, C1, VCC);
  ctrl = new Controller(error_margin, true, K1, K2);

  cli(); //disable interrupts
  TCCR1A = 0; // clear register
  TCCR1B = 0; // clear register
  TCNT1 = 0; //reset counter

  /* for 1 HZ
  OCR1A = 62499; //must be <65536
  TCCR1B |= (1 << WGM12); //CTC On
  // Set prescaler for 256 -
  TCCR1B |= (1<<CS12);
  */
 
  // OCR1A = desired_period/clock_period – 1
  // = clock_freq/desired_freq - 1
  // = (16*10^6 / 64) / (100Hz) – 1
  OCR1A = 2499; //must be <65536
  TCCR1B |= (1 << WGM12); //CTC On
  // Set prescaler for 64 -
  TCCR1B |= (1 << CS01) | (1 << CS00);;

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //enable interrupts
}


void loop() {
  if (flag) {
    unsigned long t = micros();

    if(serial_read_lux(&x_ref) != -1){ // number is ready to be processed
      u_ff = (int) (x_ref/gain);
      t_i = micros();
      v_i = get_voltage();
    }
    
    
    float y_ref = sim->calc_LDR_voltage(x_ref, v_i, t_i, t);
    float y = get_voltage();
    float debug;
    int u_sat = ctrl->run_controller(y, y_ref, u_ff, &debug);
    analogWrite(LED_PIN, u_sat);

    Serial.print(t);
    Serial.print(", ");
    Serial.print(x_ref);
    Serial.print(", ");
    Serial.print(y_ref);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(u_ff);
    Serial.print(", ");
    Serial.print(debug);
    Serial.print(", ");
    Serial.print(u_sat);
    Serial.print(", ");
    Serial.print(get_lux());
    
    Serial.println();

    unsigned long endTime = micros();
    unsigned long elapsedTime = endTime - t;
    if(elapsedTime > sampInterval)
        Serial.println("ERROR: Sampling period was exceeded!");
    flag = 0;
  }
}

// kp a 0
//aproximar a funcao do tau !!

ISR(TIMER1_COMPA_vect){
  flag = 1; //notify main loop
}

// reads from serial the lux value
// returns -1 if number incorrect or not finished
// else returns 0
int serial_read_lux (int *ref) {
   

  if (Serial.available()) {
    char read_byte = Serial.read();
    if(read_byte == 10) { // 10 -> new line
      if(read_lux < 0){
        Serial.println("ERROR: Number overflowed");
        read_lux = 0;
      }
      else{
        *ref = read_lux;
        read_lux = 0;
        return 0;
      }
    }
    else if (read_byte != 13) { //13 -> carriage return
      if(read_byte - 48 >= 0 && read_byte - 48 <= 9){ // makes sure that a number was inserted
        read_lux *= 10;
        read_lux += read_byte - 48; // -48 converts it to an int
      }
      else
        Serial.println("ERROR: Character is not a digit");
    }
  }
  return -1;
}

//Gets the voltage value from the analog input and returns the lux value
float get_lux () {
  float V_AD = float(analogRead(LUX_PIN)*5)/1023;;
  float R_lux = (R1/V_AD)*VCC - R1; 
  return pow(10, (log10(R_lux)-b)/m);
}

float get_voltage() {
  return float(analogRead(LUX_PIN)*VCC)/1023;
}

float calc_gain () {
  int max_pwm = 255;
  analogWrite(LED_PIN, max_pwm);
  delay(400);
  float max_lux = get_lux();
  analogWrite(LED_PIN, 0);
  delay(400);
  return max_lux/max_pwm;
}
