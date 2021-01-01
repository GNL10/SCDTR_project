#ifndef _OP_H_
#define _OP_H_

#include <Arduino.h>
namespace op {
    void sum (float *a, float *b, int len, float *res);
    void sub (float *a, float *b, int len, float *res);
    void mul (float *a, float *b, int a_rows, int a_cols, int b_cols, float *res);
    void copy (float *a, float *b, int rows, int cols);
};


#endif