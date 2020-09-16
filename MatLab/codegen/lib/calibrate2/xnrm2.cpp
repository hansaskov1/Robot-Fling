//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xnrm2.cpp
//
//  Code generation for function 'xnrm2'
//


// Include files
#include "xnrm2.h"
#include "calibrateGribber.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
double b_xnrm2(const double x[3], int ix0)
{
  double y;
  double scale;
  int kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = ix0 + 1;
  for (int k = ix0; k <= kend; k++) {
    double absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      double t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      double t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

double xnrm2(int n, const double x[9], int ix0)
{
  double y;
  double scale;
  int kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = (ix0 + n) - 1;
  for (int k = ix0; k <= kend; k++) {
    double absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      double t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      double t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// End of code generation (xnrm2.cpp)
