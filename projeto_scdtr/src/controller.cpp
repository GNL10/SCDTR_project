#include "controller.h"

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

float Controller::calc_error (float y, float y_ref) {
    float error = y_ref - y;
    error = (err_margins_en == true) ? apply_error_margins(error) : error;
    e_ant = error;
    y_ant = y;
    return error;
} 

float Controller::apply_error_margins (float error) {
    if(error < error_margin && error > -error_margin)
      error=0;
    return error;
} 

int Controller::apply_pwm_limits(int pwm) {
  if(pwm > 255)
    return 255;
  else if (pwm < 0) // should never enter here, since lux can never be negative
    return 0;
  return pwm;
}

float Controller::anti_windup (int u_sat, int u) {
  return (u_sat - u == 0) ? false : true; // if u_sat - u = 0 then there is no saturation
} 

float Controller::proportional_integrator (float e) {
  float p = K1*e;
  float i = i_ant + K2*(e + e_ant);
  i_ant = (es==true) ? 0 : i; //if there was saturation, reset the integral to 0
  return p+i;//p+i;
}

int Controller::run_controller(float y, float y_ref, float u_ff, float *u_out) {
  float e = calc_error(y, y_ref);
  float u_fb = proportional_integrator(e);
  int u = u_fb + u_ff;
  *u_out = u;
  int u_sat = apply_pwm_limits(u);
  es = anti_windup (u_sat, u);
  return u_sat;
}
