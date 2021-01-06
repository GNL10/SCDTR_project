#ifndef _SIMULATOR_H_
#define _SIMULATOR_H_
class Simulator {
    private:
        float gain; //static gain of the box
        float m;
        float b;
        int R1;
        float C1;
        float VCC;
    public:
        Simulator(float _m, float _b, int _R1, float _C1, float _VCC);
        float calc_R2 (float lux);
        float calc_Req (float lux);
        float calc_tau (float lux);
        float calc_LDR_voltage(float lux, float v_i, float t_i, float t);
        float calc_LDR_lux(float lux, float v_i, float t_i, float t);
};

#endif