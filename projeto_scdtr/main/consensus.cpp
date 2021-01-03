#include "consensus.h"
#include "op.h"
#include <math.h>

Consensus::Consensus(uint8_t id_node, float des_lux, float res_lux, float* gains, float cost){

rho = 0.07;
maxiter = 50;

id = id_node;
L = des_lux; 
o = res_lux;
d[0] = 0; d[1] = 0;
d_av[0] = 0; d_av[1] = 0;
y[0] = 0; y[1] = 0;
k[0] = gains[0]; k[1] = gains[1];
n = pow(norm(k), 2);
m = n - pow(k[0], 2);
c[0] = cost; c[1] = 0;

}

void Consensus::evaluate_cost(float* d, int len, float* cost){
    float res_sub[len], res_mul;

    op::sub(d, d_av, len, res_sub);
    op::mul(y, res_sub, 1, len, 1, &res_mul);
    *cost = res_mul;
    op::mul(c, d, 1, len, 1, &res_mul);
    *cost = *cost + res_mul + rho/2*pow(norm(res_sub), 2);
    float c = rho/2*pow(norm(res_sub), 2);
    Serial.println(res_sub[0]);
    Serial.println(res_sub[1]);
    

} 

/*bool Consensus::check_feasibility(float* d){
    float tol = 0.001;
    if(d[id] < 0-tol) return 0;
    if(d[id] > 100+tol) return 0;
    if(mul(d, k) < L-o-tol) return 0;
    return 1;
}

float* Consensus::iterate(){
    
    float cost_best = 1000000; //large number
    float d_best[] {-1, -1};
    float isfeasible = 1;
    float cost;

    float* z = sub(sub(mul(rho, d_av), y), c);

    // Compute unconstrained minimum
    float* d_u = mul(1/rho, z); //unconstrained minimum
    isfeasible = check_feasibility(d_u);
    if(isfeasible){
        cost = evaluate_cost(d_u);
        if(cost < cost_best){
            d_best[0] = d_u[0]; d_best[1] = d_u[1];
            cost_best = cost;
        }
        return d_best;
    }
    else{ // Compute minimum constrained to linear boundary
        d_bl = mul(1/rho, z) - mul(mul(k, 1/n),(o-L+mul(mul((1/rho), z), k)));
        Serial.pri
        
    }

}*/

float Consensus::norm (float* v){
    float n = 0;
    for (int i = 0; i < sizeof(v)/sizeof(float*); i++){
        n += pow(v[i], 2);
    }
    return sqrt(n);
}




