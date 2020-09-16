//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xaxpy.cpp
//
//  Code generation for function 'xaxpy'
//


// Include files
#include "xaxpy.h"
#include "calibrateGribber.h"
#include "rt_nonfinite.h"

// Function Definitions
void b_xaxpy(int n, double a, const double x[9], int ix0, double y[3], int iy0)
{
  if (!(a == 0.0)) {
    int ix;
    int iy;
    int i;
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (int k = 0; k <= i; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

void c_xaxpy(int n, double a, const double x[3], int ix0, double y[9], int iy0)
{
  if (!(a == 0.0)) {
    int ix;
    int iy;
    int i;
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (int k = 0; k <= i; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

void xaxpy(int n, double a, int ix0, double y[9], int iy0)
{
  if (!(a == 0.0)) {
    int ix;
    int iy;
    int i;
    ix = ix0 - 1;
    iy = iy0 - 1;
    i = n - 1;
    for (int k = 0; k <= i; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

// End of code generation (xaxpy.cpp)
