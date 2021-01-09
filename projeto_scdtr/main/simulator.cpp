#include "simulator.h"
#include <math.h>

// constructor
/*
Simulator::Simulator(float _m, float _b, int _R1, float _C1, float _VCC) {
    m = _m;
    b = _b;
    R1 = _R1;
    C1 = _C1;
    VCC = _VCC;
}*/

void Simulator::init(float _m, float _b, int _R1, float _C1, float _VCC) {
    m = _m;
    b = _b;
    R1 = _R1;
    C1 = _C1;
    VCC = _VCC;
}
/**
 * Calculates the Resistance 2 value according to the lux value
 * @param lux Lux value used to calculate the resistance
 * @returns Value of the resistance in Ohm
 */
float Simulator::calc_R2 (float lux) {
    if(lux == 0)
        lux = 0.0001; //low value other than 0
    return pow(10, m*log10(lux)+b);
}

/**
 * Calculates the equivalent resistance according to the lux value
 * @param lux Lux value used to calculate the resistance
 * @returns Value of the equivalent resistance in Ohm
 */
float Simulator::calc_Req (float lux) {
    float R2 = calc_R2(lux);
    return (R1*R2)/(R1+R2);
}

/**
 * Calculates the tau according to lux
 * @param lux Lux value used to calculate tau
 * @returns Tau value in ms
 */
float Simulator::calc_tau (float lux) {
    return calc_Req(lux)*C1*1000;
}

/**
 * Calculates theoretical LDR voltage
 * @param lux Lux reference value
 * @param v_i Voltage at time of change of lux reference value
 * @param t_i Time of change of lux reference value
 * @param t current time
 * @returns Theoretical current voltage value
 */
float Simulator::calc_LDR_voltage(float lux, float v_i, float t_i, float t) {
    float v_f = VCC*R1/(R1+calc_R2(lux));
    t_i/=1000;
    t/=1000;
    return v_f - (v_f - v_i)*exp(-(t-t_i)/(calc_tau(lux)));
}

/**
 * Calculates theoretical LDR lux value at current value
 * @param lux Lux reference value
 * @param v_i Voltage at time of change of lux reference value
 * @param t_i Time of change of lux reference value
 * @param t current time
 * @returns Theoretical current lux value
 */
float Simulator::calc_LDR_lux(float lux, float v_i, float t_i, float t) {
    float v = calc_LDR_voltage(lux, v_i, t_i, t);
    float R_lux = (R1/v)*VCC - R1;
    return pow(10, (log10(R_lux)-b)/m);
}



