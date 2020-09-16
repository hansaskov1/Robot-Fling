//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  calibrate2.cpp
//
//  Code generation for function 'calibrate2'
//


// Include files
#include "calibrate2.h"
#include "calibrate2_data.h"
#include "calibrate2_initialize.h"
#include "rt_nonfinite.h"
#include "svd.h"

// Function Definitions
void calibrate2(const coder::array<double, 2U> &pr, const coder::array<double,
                2U> &pw, double rHat[9], double tHat[3])
{
  double a;
  coder::array<double, 2U> qr;
  int vlen;
  int xpageoffset;
  int aoffset;
  double y[3];
  double bkj;
  int k;
  double b_y[3];
  coder::array<double, 2U> qw;
  bool p;
  double H[9];
  double U[9];
  double V[9];
  if (!isInitialized_calibrate2) {
    calibrate2_initialize();
  }

  a = 1.0 / static_cast<double>(pr.size(1));
  qr.set_size(pr.size(1), 3);
  vlen = pr.size(1);
  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    for (aoffset = 0; aoffset < vlen; aoffset++) {
      qr[aoffset + qr.size(0) * xpageoffset] = pr[xpageoffset + 3 * aoffset];
    }
  }

  vlen = qr.size(0);
  if (qr.size(0) == 0) {
    y[0] = 0.0;
    y[1] = 0.0;
    y[2] = 0.0;
  } else {
    for (aoffset = 0; aoffset < 3; aoffset++) {
      xpageoffset = aoffset * qr.size(0);
      y[aoffset] = qr[xpageoffset];
      for (k = 2; k <= vlen; k++) {
        y[aoffset] += qr[(xpageoffset + k) - 1];
      }
    }
  }

  bkj = 1.0 / static_cast<double>(pw.size(1));
  qr.set_size(pw.size(1), 3);
  vlen = pw.size(1);
  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    y[xpageoffset] *= a;
    for (aoffset = 0; aoffset < vlen; aoffset++) {
      qr[aoffset + qr.size(0) * xpageoffset] = pw[xpageoffset + 3 * aoffset];
    }
  }

  vlen = qr.size(0);
  if (qr.size(0) == 0) {
    b_y[0] = 0.0;
    b_y[1] = 0.0;
    b_y[2] = 0.0;
  } else {
    for (aoffset = 0; aoffset < 3; aoffset++) {
      xpageoffset = aoffset * qr.size(0);
      b_y[aoffset] = qr[xpageoffset];
      for (k = 2; k <= vlen; k++) {
        b_y[aoffset] += qr[(xpageoffset + k) - 1];
      }
    }
  }

  b_y[0] *= bkj;
  b_y[1] *= bkj;
  b_y[2] *= bkj;
  qr.set_size(3, pr.size(1));
  vlen = pr.size(0) * pr.size(1);
  for (xpageoffset = 0; xpageoffset < vlen; xpageoffset++) {
    qr[xpageoffset] = pr[xpageoffset] - y[0];
  }

  qw.set_size(3, pw.size(1));
  vlen = pw.size(0) * pw.size(1);
  for (xpageoffset = 0; xpageoffset < vlen; xpageoffset++) {
    qw[xpageoffset] = pw[xpageoffset] - b_y[0];
  }

  vlen = qr.size(1);
  for (int j = 0; j < 3; j++) {
    xpageoffset = j * 3;
    H[xpageoffset] = 0.0;
    H[xpageoffset + 1] = 0.0;
    H[xpageoffset + 2] = 0.0;
    for (k = 0; k < vlen; k++) {
      aoffset = k * 3;
      bkj = qw[k * 3 + j];
      H[xpageoffset] += qr[aoffset] * bkj;
      H[xpageoffset + 1] += qr[aoffset + 1] * bkj;
      H[xpageoffset + 2] += qr[aoffset + 2] * bkj;
    }
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

  qr.set_size(pr.size(1), 3);
  vlen = pr.size(1);
  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    double d;
    bkj = V[xpageoffset + 3];
    d = V[xpageoffset + 6];
    for (aoffset = 0; aoffset < 3; aoffset++) {
      rHat[xpageoffset + 3 * aoffset] = (V[xpageoffset] * U[aoffset] + bkj *
        U[aoffset + 3]) + d * U[aoffset + 6];
    }

    for (aoffset = 0; aoffset < vlen; aoffset++) {
      qr[aoffset + qr.size(0) * xpageoffset] = pr[xpageoffset + 3 * aoffset];
    }
  }

  vlen = qr.size(0);
  if (qr.size(0) == 0) {
    y[0] = 0.0;
    y[1] = 0.0;
    y[2] = 0.0;
  } else {
    for (aoffset = 0; aoffset < 3; aoffset++) {
      xpageoffset = aoffset * qr.size(0);
      y[aoffset] = qr[xpageoffset];
      for (k = 2; k <= vlen; k++) {
        y[aoffset] += qr[(xpageoffset + k) - 1];
      }
    }
  }

  for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
    tHat[xpageoffset] = b_y[xpageoffset] - ((rHat[xpageoffset] * (a * y[0]) +
      rHat[xpageoffset + 3] * (a * y[1])) + rHat[xpageoffset + 6] * (a * y[2]));
  }
}

// End of code generation (calibrate2.cpp)
