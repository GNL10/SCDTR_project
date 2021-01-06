#ifndef CONSENSUS_H
#define CONSENSUS_H

#include "comms.h"

class Consensus
{
private:
public:
    float rho;
    int maxiter;
    int idx;
    float d[2];
    float d_av[2];
    float prev_av[2];
    float d_aux[2];
    float y[2];
    float k[2];
    float n;
    float m;
    float c[2];
    float o;
    float L;
    int len;
    

    enum class State : byte {
      iterate,
      wait_for_d
    };

    State curr_state;

  Consensus(int node_idx, float des_lux, float res_lux, float* gains, float cost, int n_nodes);
  void evaluate_cost(float* d, float* cost);
  bool check_feasibility(float* d);
  void iterate(float* d_best);
  void compute_d_bl(float* z, float* d_bl);
  void compute_d_b0(float* z, float* d_b0);
  void compute_d_b100(float* z, float* d_b100);
  void compute_d_l0(float* z, float* d_l0);
  void compute_d_l100(float* z, float* d_l100);
  void compute_avg(float* d1, float* d2);
  void compute_y(float* d);
  float norm(float* v);
  bool process_msg_received(uint8_t i, float d);
  bool negotiate(can_frame frame, bool has_data, uint8_t my_id);
};
#endif