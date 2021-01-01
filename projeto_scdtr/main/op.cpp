#include "op.h"

void op::sum (float *a, float *b, int len, float *res){
    for(int i = 0; i < len; i++)
        res[i] = a[i] + b[i];
}

void op::sub (float *a, float *b, int len, float *res) {
    for(int i = 0; i < len; i++)
        res[i] = a[i] - b[i];
}

// a_cols = b_rows
void op::mul (float *a, float *b, int a_rows, int a_cols, int b_cols, float *res) {
    for(int i = 0; i < a_rows; i++){
        for(int j = 0; j < b_cols; j ++){
            res[i*b_cols + j] = 0;
            for(int k = 0; k < a_cols; k++){
                res[i*b_cols + j] += a[i*a_cols + k] * b[k*b_cols + j];
            }
        }
    }       
}

void op::copy (float *a, float *b, int rows, int cols) {
    memcpy(a, b, sizeof(a[0]) * rows * cols);
}