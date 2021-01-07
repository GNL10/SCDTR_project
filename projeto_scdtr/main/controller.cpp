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

    u_max = 255;
    u_min = 0;

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
int Controller::anti_windup (float ufb, float uff, float e, float ui) {
  float ui_max = u_max - uff - K1*e; 
  float ui_min = u_min - uff - K1*e;
  float e_sat;

  while(ui > ui_max || ui < ui_min){        
    if(ui > ui_max) e_sat = ui - ui_max; // ui > ui_max -> e_sat > 0
    if(ui < ui_min) e_sat = ui - ui_min; // ui < ui_min -> e_sat < 0
    
    ui = ui - (e_sat*1.01); //ui > ui_max: subtract; ui < ui_min: add
    ufb = K1*e + ui;       
  }

  i_ant = ui;

  return ufb + uff; 

  //return (u_sat - u == 0) ? false : true; // if u_sat - u = 0 then there is no saturation

} 

/**
 * Applies the PI to the error
 * @param e Error to be used in the PI
 * @returns Sum of the integrator + proportional term
 */
float Controller::proportional_integrator (float e) {
  float p = K1*e;
  i = i_ant + K2*0.5*(e + e_ant);
  //i_ant = (es==true) ? 0 : i; //if there was saturation, reset the integral to 0
  
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
  float u_fb = proportional_integrator(e);
  u_ff = apply_pwm_limits(u_ff);
  int u = anti_windup (u_fb, u_ff, e, i);
  return u;
}
