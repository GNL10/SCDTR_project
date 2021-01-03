#include "node.h"

Node::Node(int idx, float gains[], float ext, int cost, int des_lux){
    k[0] = gains[0];
    k[1] = gains[1];
    n = sum_squares(k);
    m = n - (k[0]*k[0]);
    c[idx] = cost;
    o = ext;
    L = des_lux;

}

float Node::sum_squares (float v[]){
    float n = 0;
    for (int i = 0; i < sizeof(v)/sizeof(float*); i++){
        n += v[i]*v[i];
    }
    return n;
}