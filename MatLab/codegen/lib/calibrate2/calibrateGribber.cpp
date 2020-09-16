//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  calibrateGribber.cpp
//
//  Code generation for function 'calibrateGribber'
//


// Include files
#include "calibrateGribber.h"
#include "rt_nonfinite.h"
#include "svd.h"

// Function Definitions
void calibrateGribber::calibrate2(const coder::array<double, 2U> &pr, const
  coder::array<double, 2U> &pw, double rHat[9], double tHat[3])
{
  double a;
  coder::array<double, 2U> x;
  int vlen;
  int xpageoffset;
  int i;
  double Cpr[3];
  double b_a;
  int k;
  double y[3];
  double pr_idx_1;
  double pr_idx_2;
  boolean_T p;
  double H[9];
  double U[9];
  double V[9];
  a = 1.0 / static_cast<double>(pr.size(1));
  x.set_size(pr.size(1), 3);
  vlen = pr.size(1);
  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    for (i = 0; i < vlen; i++) {
      x[i + x.size(0) * xpageoffset] = pr[xpageoffset + 3 * i];
    }
  }

  vlen = x.size(0);
  if (x.size(0) == 0) {
    Cpr[0] = 0.0;
    Cpr[1] = 0.0;
    Cpr[2] = 0.0;
  } else {
    for (i = 0; i < 3; i++) {
      xpageoffset = i * x.size(0);
      Cpr[i] = x[xpageoffset];
      for (k = 2; k <= vlen; k++) {
        Cpr[i] += x[(xpageoffset + k) - 1];
      }
    }
  }

  b_a = 1.0 / static_cast<double>(pw.size(1));

  //  size(pw,2)
  x.set_size(pw.size(1), 3);
  vlen = pw.size(1);
  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    Cpr[xpageoffset] *= a;
    for (i = 0; i < vlen; i++) {
      x[i + x.size(0) * xpageoffset] = pw[xpageoffset + 3 * i];
    }
  }

  vlen = x.size(0);
  if (x.size(0) == 0) {
    y[0] = 0.0;
    y[1] = 0.0;
    y[2] = 0.0;
  } else {
    for (i = 0; i < 3; i++) {
      xpageoffset = i * x.size(0);
      y[i] = x[xpageoffset];
      for (k = 2; k <= vlen; k++) {
        y[i] += x[(xpageoffset + k) - 1];
      }
    }
  }

  a = pr[0] - Cpr[0];
  y[0] = pw[0] - b_a * y[0];
  pr_idx_1 = pr[1] - Cpr[1];
  y[1] = pw[1] - b_a * y[1];
  pr_idx_2 = pr[2] - Cpr[2];
  y[2] = pw[2] - b_a * y[2];
  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    H[3 * xpageoffset] = a * y[xpageoffset];
    H[3 * xpageoffset + 1] = pr_idx_1 * y[xpageoffset];
    H[3 * xpageoffset + 2] = pr_idx_2 * y[xpageoffset];
  }

  p = true;
  for (k = 0; k < 9; k++) {
    if ((!p) || (rtIsInf(H[k]) || rtIsNaN(H[k]))) {
      p = false;
    }
  }

  if (p) {
    svd(H, U, y, V);
  } else {
    for (xpageoffset = 0; xpageoffset < 9; xpageoffset++) {
      U[xpageoffset] = rtNaN;
      V[xpageoffset] = rtNaN;
    }
  }

  x.set_size(pw.size(1), 3);
  vlen = pw.size(1);
  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    a = V[xpageoffset + 3];
    pr_idx_1 = V[xpageoffset + 6];
    for (i = 0; i < 3; i++) {
      rHat[xpageoffset + 3 * i] = (V[xpageoffset] * U[i] + a * U[i + 3]) +
        pr_idx_1 * U[i + 6];
    }

    for (i = 0; i < vlen; i++) {
      x[i + x.size(0) * xpageoffset] = pw[xpageoffset + 3 * i];
    }
  }

  vlen = x.size(0);
  if (x.size(0) == 0) {
    y[0] = 0.0;
    y[1] = 0.0;
    y[2] = 0.0;
  } else {
    for (i = 0; i < 3; i++) {
      xpageoffset = i * x.size(0);
      y[i] = x[xpageoffset];
      for (k = 2; k <= vlen; k++) {
        y[i] += x[(xpageoffset + k) - 1];
      }
    }
  }

  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    tHat[xpageoffset] = b_a * y[xpageoffset] - ((rHat[xpageoffset] * Cpr[0] +
      rHat[xpageoffset + 3] * Cpr[1]) + rHat[xpageoffset + 6] * Cpr[2]);
  }
}

calibrateGribber::~calibrateGribber()
{
  // (no terminate code required)
}

calibrateGribber::calibrateGribber()
{
  rt_InitInfAndNaN();
}

// End of code generation (calibrateGribber.cpp)
