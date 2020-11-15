#include "simulator.h"
#include <math.h>

// constructor
Simulator::Simulator(float _m, float _b, int _R1, float _C1, float _VCC) {
    m = _m;
    b = _b;
    R1 = _R1;
    C1 = _C1;
    VCC = _VCC;
}


float Simulator::calc_R2 (float lux) {
    if(lux == 0)
        lux = 0.0001; //low value other than 0
    return pow(10, m*log10(lux)+b);
}

float Simulator::calc_Req (float lux) {
    float R2 = calc_R2(lux);
    return (R1*R2)/(R1+R2);
}

//returns tau in ms
float Simulator::calc_tau (float lux) {
    return calc_Req(lux)*C1*1000;
    //return 1.38*pow(10, 3)*pow(lux, -0.912);
}

//measure initial voltage at ldr
// times in ms
float Simulator::calc_LDR_voltage(float lux, float v_i, float t_i, float t) {
    float v_f = VCC*R1/(R1+calc_R2(lux));
    t_i/=1000;
    t/=1000;
    return v_f - (v_f - v_i)*exp(-(t-t_i)/(calc_tau(lux)));
}



