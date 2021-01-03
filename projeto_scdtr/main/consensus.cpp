#include "consensus.h"
#include "op.h"
#include <math.h>

Consensus::Consensus(int node_idx, float des_lux, float res_lux, float* gains, float cost, int n_nodes){

rho = 0.07;

len = n_nodes;
idx = node_idx; 
L = des_lux; 
o = res_lux;
d[0] = 0; d[1] = 0;
d_av[0] = 0; d_av[1] = 0;
y[0] = 0; y[1] = 0;
k[0] = gains[0]; k[1] = gains[1];
n = pow(norm(k), 2);
m = n - pow(k[idx], 2);
c[0] = 0; c[1] = 0;
c[idx] = cost;

}

void Consensus::evaluate_cost(float* d, float* cost){
    float res_sub[len], res_mul;

    op::sub(d, d_av, len, res_sub);
    op::mul(y, res_sub, 1, len, 1, &res_mul);
    *cost = res_mul;
    
    op::mul(c, d, 1, len, 1, &res_mul);
    *cost = *cost + res_mul + rho/2*pow(norm(res_sub), 2);
} 

bool Consensus::check_feasibility(float* d){
    float tol = 0.001, res_mul;

    if(d[idx] < 0-tol) return 0;
    if(d[idx] > 100+tol) return 0;
    op::mul(d, k, 1, len, 1, &res_mul);
    if(res_mul < L-o-tol) return 0;
    return 1;
}

void Consensus::iterate(float* d_best){
    
    float cost_best = 1000000; //large number
    float isfeasible = 1;
    float cost;
    float z[len], d_u[len];

    // Compute z
    op::scalar_mul(rho, d_av, len, 1, z);
    op::sub(z, y, len, z);
    op::sub(z, c, len, z);
    
    // Compute unconstrained minimum  
    op::scalar_mul(1/rho, z, len, 1, d_u);
    /*cout << "d_u: " << d_u[0] << endl;
    cout << "d_u: " << d_u[1] << endl;*/
    isfeasible = check_feasibility(d_u);
    if(isfeasible){
        evaluate_cost(d_u, &cost);
        if(cost < cost_best){
            op::copy(d_best, d_u, len, 1);
            cost_best = cost;
        }
    }
    else{ 
        float d_bl[len], d_b0[len], d_b100[len], d_l0[len], d_l100[len];
        
        // Compute minimum constrained to linear boundary
        compute_d_bl(z, d_bl);
        /*cout << "d_bl: " << d_bl[0] << endl;
        cout << "d_bl: " << d_bl[1] << endl;*/
        isfeasible = check_feasibility(d_bl);
        if(isfeasible){
            evaluate_cost(d_bl, &cost);
            if(cost < cost_best){
                op::copy(d_best, d_bl, len, 1);
                cost_best = cost;
            }
        }
        // Compute minimum constrained to 0 boundary
        compute_d_b0(z, d_b0);
        /*cout << "d_b0: " << d_b0[0] << endl;
        cout << "d_b0: " << d_b0[1] << endl;*/
        isfeasible = check_feasibility(d_b0);
        if(isfeasible){
            evaluate_cost(d_b0, &cost);
            if(cost < cost_best){
                op::copy(d_best, d_b0, len, 1);
                cost_best = cost;
            }
        }
        // Compute minimum constrained to 100 boundary
        compute_d_b100(z, d_b100);
        /*cout << "d_b100: " << d_b100[0] << endl;
        cout << "d_b100: " << d_b100[1] << endl;*/
        isfeasible = check_feasibility(d_b100);
        if(isfeasible){
            evaluate_cost(d_b100, &cost);
            if(cost < cost_best){
                op::copy(d_best, d_b100, len, 1);
                cost_best = cost;
            }
        }
        // Compute minimum constrained to linear and 0 boundary
        compute_d_l0(z, d_l0);
        /*cout << "d_bl0: " << d_l0[0] << endl;
        cout << "d_bl0: " << d_l0[1] << endl;*/
        isfeasible = check_feasibility(d_l0);
        if(isfeasible){
            evaluate_cost(d_l0, &cost);
            if(cost < cost_best){
                op::copy(d_best, d_l0, len, 1);
                cost_best = cost;
            }
        }
        // Compute minimum constrained to linear and 100 boundary
        compute_d_l100(z, d_l100);
        /*cout << "d_l100: " << d_l100[0] << endl;
        cout << "d_l100: " << d_l100[1] << endl;*/
        isfeasible = check_feasibility(d_l100);
        if(isfeasible){
            evaluate_cost(d_l100, &cost);
            if(cost < cost_best){
                op::copy(d_best, d_l100, len, 1);
                cost_best = cost;
            }
        }        
    }

}

void Consensus::compute_d_bl(float* z, float* d_bl){
    float res_mul, res1[len], res2[len];

    op::scalar_mul(1/rho, z, len, 1, res1);
    op::mul(res1, k, 1, len, 1, &res_mul);
    res_mul = o - L + res_mul;
    op::scalar_mul(1/n, k, len, 1, res2);
    op::scalar_mul(res_mul, res2, len, 1, res2);
    op::sub(res1, res2, len, d_bl);
}

void Consensus::compute_d_b0(float* z, float* d_b0){  
    op::scalar_mul(1/rho, z, len, 1, d_b0);
    d_b0[idx] = 0;
}

void Consensus::compute_d_b100(float* z, float* d_b100){
    op::scalar_mul(1/rho, z, len, 1, d_b100);
    d_b100[idx] = 100;
}

void Consensus::compute_d_l0(float* z, float* d_l0){
    float res1[len], res2[len], res3[len], res_mul;

    op::scalar_mul(1/rho, z, len, 1, res1);
    op::scalar_mul((o-L)/m, k, len, 1, res2);
    op::mul(z, k, 1, len, 1, &res_mul);
    res_mul = (1/rho/m)*(k[idx]*z[idx] - res_mul);
    op::scalar_mul(res_mul, k, len, 1, res3);
    op::sub(res1, res2, len, res2);
    op::sum(res2, res3, len, d_l0);
    d_l0[idx] = 0;
}

void Consensus::compute_d_l100(float* z, float* d_l100){
    float res1[len], res2[len], res3[len], res_mul;

    op::scalar_mul(1/rho, z, len, 1, res1);
    op::scalar_mul((o-L+100*k[idx])/m, k, len, 1, res2);
    op::mul(z, k, 1, len, 1, &res_mul);
    res_mul = (1/rho/m)*(k[idx]*z[idx] - res_mul);
    op::scalar_mul(res_mul, k, len, 1, res3);
    op::sub(res1, res2, len, res2);
    op::sum(res2, res3, len, d_l100);
    d_l100[idx] = 100;
}

float Consensus::norm (float* v){
    float n = 0;
    for (int i = 0; i < len; i++){
        n += pow(v[i], 2);
    }

    return sqrt(n);
}

void Consensus::compute_avg(float* d1, float* d2){
    op::sum(d1, d2, len, d_av);
    op::scalar_mul(0.5, d_av, len, 1, d_av);
}

void Consensus::compute_y(float* d){
    float res[len];
    op::sub(d, d_av, len, res);
    op::scalar_mul(rho, res, len, 1, res);
    op::sum(y, res, len, y);
}



