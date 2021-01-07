#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_
class Controller {
    private:
        float error_margin;
        bool err_margins_en;
        float K1;
        float K2;
        bool es;
        float i_ant; //if there was saturation, reset the integral to 0
        float y_ant;
        float e_ant;
        float i;
        float u_max;
        float u_min;
    public:
        Controller(float _error_margin, float _K1, float _K2, bool _err_margins_en);
        float calc_error (float y, float y_ref);
        float apply_error_margins (float e);
        int apply_pwm_limits(int pwm);
        bool anti_windup (int u_sat, int u);
        float proportional_integrator (float e);
        int run_controller(float y, float y_ref, float u_ff);
        int anti_windup (float ufb, float uff, float e, float ui);
};

#endif