#include "Arduino.h"

class Consensus
{
private:
public:
    float rho;
    int maxiter;
    int id;
    float d[2];
    float d_av[2];
    float y[2];
    float k[2];
    float n;
    float m;
    float c[2];
    float o;
    float L;

  Consensus(uint8_t id_node, float des_lux, float res_lux, float* gains, float cost);
  void evaluate_cost(float* d, int len, float* cost);
  bool check_feasibility(float* d);
  void iterate();
  float norm(float* v);
};