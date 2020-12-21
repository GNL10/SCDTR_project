#include "controller.h"
#include <Arduino.h>

// constructor
Controller::Controller(float _error_margin, float _K1, float _K2, bool _err_margins_en = true) {
    error_margin = _error_margin;
    err_margins_en =_err_margins_en;
    K1 = _K1;
    K2 = _K2;

    es = false; // if windup is not used, then es must always stay false and thus, not affect the controller
    i_ant = 0;
    e_ant = 0;
    y_ant = 0;
}

/**
 * Calculates the error
 * @param y Lux read at the output
 * @param y_ref Lux output from the simulator
 * @returns Error calculated
*/
float Controller::calc_error (float y, float y_ref) {
    float error = y_ref - y;
    error = (err_margins_en == true) ? apply_error_margins(error) : error;
    e_ant = error;
    y_ant = y;
    return error;
} 

/**
 * Ignores error if it is smaller than error margin
 * @param error Error from to be processed
 * @returns Processed error
*/
float Controller::apply_error_margins (float error) {
    if(error < error_margin && error > -error_margin)
      error=0;
    return error;
} 

/**
 * Limits the input value between 0 and 255
 * @param pwm Value to be limited
 * @returns Value between 0 a 255
 */
int Controller::apply_pwm_limits(int pwm) {
  if(pwm > 255)
    return 255;
  else if (pwm < 0) // should never enter here, since lux can never be negative
    return 0;
  return pwm;
}

/**
 * Applies anti windup t the controller
 * @param u_sat Saturated pwm value
 * @param u Non saturated pwm value
 * @returns True if saturated, else false
 */
bool Controller::anti_windup (int u_sat, int u) {
  return (u_sat - u == 0) ? false : true; // if u_sat - u = 0 then there is no saturation
} 

/**
 * Applies the PI to the error
 * @param e Error to be used in the PI
 * @returns Sum of the integrator + proportional term
 */
float Controller::proportional_integrator (float e, float u_ff ) {
  float p = K1*e;
  float i = i_ant + K2*(e + e_ant);

  //i_ant = (es==true) ? 0 : i; //if there was saturation, reset the integral to 0
  
  float i_max = U_MAX - u_ff - K1*e; 
  float i_min = U_MIN - u_ff - K1*e;
  float e_sat;
  
  if(i + p + u_ff > U_MAX) i = i_max; // i > i_max -> e_sat > 0
  if(i + p + u_ff < U_MIN) i = i_min; // i < i_min -> e_sat < 0
  
  /*while (i > i_max || i < i_min){ 
  
    if(i > i_max) e_sat = i - i_max; // i > i_max -> e_sat > 0
    if(i < i_min) e_sat = i - i_min; // i < i_min -> e_sat < 0
    
    i = i - (e_sat*1.0001); //i > i_max: subtract; i < i_min: add
  }
  */
  Serial.print(i_min);
  Serial.print(", ");
  Serial.print(i_max);
  Serial.print(", ");
  Serial.print(p);
  Serial.print(", ");
  Serial.print(i);
  Serial.print(", ");
  Serial.print(p+i);
  Serial.print(", ");
  
  return p+i;
}

/**
 * Runs the controller of the system
 * @param y Last lux reading from output
 * @param y_ref Lux output from the simulator
 * @param u_ff Pwm reference
 * @returns Pwm value to be inputed into the LED
 */
int Controller::run_controller(float y, float y_ref, float u_ff) {
  float e = calc_error(y, y_ref);
  float u_fb = proportional_integrator(e, u_ff);
  int u = u_fb + u_ff;
  
  int u_sat = apply_pwm_limits(u);
  es = anti_windup (u_sat, u);
  return u_sat;
}
