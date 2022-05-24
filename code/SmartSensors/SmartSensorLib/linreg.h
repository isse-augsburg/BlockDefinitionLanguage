#ifndef linreg_h
#define linreg_h



#include "Arduino.h"
//#define REAL float
#define REAL float


/*
 * Based on Code from
 * https://stackoverflow.com/questions/5083465/fast-efficient-least-squares-fit-algorithm-in-c
 * and
 * https://helloacm.com/cc-linear-regression-tutorial-using-gradient-descent/
 *
 * n = number of data points
 * x,y  = arrays of data
 * *t = output intercept
 * *m  = output slope
 */

/*
 int main(){
    double x[] = {1, 2, 4, 3, 5};
    double y[] = {1, 3, 3, 2, 5};
    double m= 0, t=0;
    linreg(5, x, y, 4, &m, &t);
    printf("%lf, %lf\n", m, t);
}
*/

void linreg(int n, const REAL x[], const REAL y[], int epoches, REAL* m, REAL* t ) {
    REAL b0 = 0;
    REAL b1 = 0;
    REAL alpha = 0.01;

    for (int i = 0; i < epoches*n; i ++) {
        int idx = i % n;
        REAL p = b0 + b1 * x[idx];
        REAL err = p - y[idx];
        b0 = b0 - alpha * err;
        b1 = b1 - alpha * err * x[idx];
    }
    *m = b1;
    *t = b0;
}
#endif
