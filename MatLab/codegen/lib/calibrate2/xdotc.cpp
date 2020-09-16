//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xdotc.cpp
//
//  Code generation for function 'xdotc'
//


// Include files
#include "xdotc.h"
#include "calibrate2.h"

// Function Definitions
double xdotc(int n, const double x[9], int ix0, const double y[9], int iy0)
{
  double d;
  int ix;
  int iy;
  ix = ix0;
  iy = iy0;
  d = 0.0;
  for (int k = 0; k < n; k++) {
    d += x[ix - 1] * y[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

// End of code generation (xdotc.cpp)
