#ifndef CONSENSUS_H
#define CONSENSUS_H

#include "comms.h"

#define NORM_THRESHOLD 0.02

class Consensus
{
private:
public:
    float rho;
    int maxiter;
    int idx;
    float d[ID_MAX_NUM];
    float d_av[ID_MAX_NUM];
    float prev_av[ID_MAX_NUM];
    float d_aux[ID_MAX_NUM];
    float y[ID_MAX_NUM];
    float k[ID_MAX_NUM];
    float n;
    float m;
    float c[ID_MAX_NUM];
    float o;
    float L;
    uint8_t len;
    int d_received_ctr; //counts the number of d's received through the can bus
    

    enum class State : byte {
      iterate,
      wait_for_d
    };

    State curr_state;

  //Consensus(int node_idx, float des_lux, float res_lux, float* gains, float cost, uint8_t n_nodes);
  void init(int node_idx, float des_lux, float res_lux, float* gains, float cost, uint8_t n_nodes);
  void evaluate_cost(float* d, float* cost);
  bool check_feasibility(float* d);
  void iterate(float* d_best);
  void compute_d_bl(float* z, float* d_bl);
  void compute_d_b0(float* z, float* d_b0);
  void compute_d_b100(float* z, float* d_b100);
  void compute_d_l0(float* z, float* d_l0);
  void compute_d_l100(float* z, float* d_l100);
  void compute_y(float* d);
  float norm(float* v);
  bool process_msg_received(uint8_t i, float d);
  bool negotiate(can_frame frame, bool has_data, uint8_t my_id);
  bool check_threshold(float res[], uint8_t len);
  float get_desired_lux();
};
#endif