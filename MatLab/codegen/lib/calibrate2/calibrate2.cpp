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
#include "svd.h"

// Function Definitions
void calibrate2(const double pr_data[], const int pr_size[2], const double
                pw_data[], const int pw_size[2], double rHat[9], double tHat[3])
{
  double a;
  int x_size_idx_0;
  int xpageoffset;
  int j;
  int aoffset;
  double y[3];
  double qr_data[150];
  double bkj;
  int k;
  double b_y[3];
  double qw_data[150];
  double H[9];
  double U[9];
  double V[9];
  a = 1.0 / static_cast<double>(pr_size[1]);
  x_size_idx_0 = pr_size[1];
  xpageoffset = pr_size[1];
  for (j = 0; j < 3; j++) {
    for (aoffset = 0; aoffset < xpageoffset; aoffset++) {
      qr_data[aoffset + x_size_idx_0 * j] = pr_data[j + 3 * aoffset];
    }
  }

  if (pr_size[1] == 0) {
    y[0] = 0.0;
    y[1] = 0.0;
    y[2] = 0.0;
  } else {
    for (aoffset = 0; aoffset < 3; aoffset++) {
      xpageoffset = aoffset * x_size_idx_0;
      y[aoffset] = qr_data[xpageoffset];
      for (k = 2; k <= x_size_idx_0; k++) {
        y[aoffset] += qr_data[(xpageoffset + k) - 1];
      }
    }
  }

  bkj = 1.0 / static_cast<double>(pw_size[1]);
  x_size_idx_0 = pw_size[1];
  xpageoffset = pw_size[1];
  for (j = 0; j < 3; j++) {
    y[j] *= a;
    for (aoffset = 0; aoffset < xpageoffset; aoffset++) {
      qr_data[aoffset + x_size_idx_0 * j] = pw_data[j + 3 * aoffset];
    }
  }

  if (pw_size[1] == 0) {
    b_y[0] = 0.0;
    b_y[1] = 0.0;
    b_y[2] = 0.0;
  } else {
    for (aoffset = 0; aoffset < 3; aoffset++) {
      xpageoffset = aoffset * x_size_idx_0;
      b_y[aoffset] = qr_data[xpageoffset];
      for (k = 2; k <= x_size_idx_0; k++) {
        b_y[aoffset] += qr_data[(xpageoffset + k) - 1];
      }
    }
  }

  b_y[0] *= bkj;
  b_y[1] *= bkj;
  b_y[2] *= bkj;
  x_size_idx_0 = pr_size[1];
  xpageoffset = pr_size[0] * pr_size[1];
  for (j = 0; j < xpageoffset; j++) {
    qr_data[j] = pr_data[j] - y[0];
  }

  xpageoffset = pw_size[0] * pw_size[1];
  for (j = 0; j < xpageoffset; j++) {
    qw_data[j] = pw_data[j] - b_y[0];
  }

  for (j = 0; j < 3; j++) {
    xpageoffset = j * 3;
    H[xpageoffset] = 0.0;
    H[xpageoffset + 1] = 0.0;
    H[xpageoffset + 2] = 0.0;
    for (k = 0; k < x_size_idx_0; k++) {
      aoffset = k * 3;
      bkj = qw_data[k * 3 + j];
      H[xpageoffset] += qr_data[aoffset] * bkj;
      H[xpageoffset + 1] += qr_data[aoffset + 1] * bkj;
      H[xpageoffset + 2] += qr_data[aoffset + 2] * bkj;
    }
  }

  svd(H, U, y, V);
  x_size_idx_0 = pr_size[1];
  xpageoffset = pr_size[1];
  for (j = 0; j < 3; j++) {
    double d;
    bkj = V[j + 3];
    d = V[j + 6];
    for (aoffset = 0; aoffset < 3; aoffset++) {
      rHat[j + 3 * aoffset] = (V[j] * U[aoffset] + bkj * U[aoffset + 3]) + d *
        U[aoffset + 6];
    }

    for (aoffset = 0; aoffset < xpageoffset; aoffset++) {
      qr_data[aoffset + x_size_idx_0 * j] = pr_data[j + 3 * aoffset];
    }
  }

  if (pr_size[1] == 0) {
    y[0] = 0.0;
    y[1] = 0.0;
    y[2] = 0.0;
  } else {
    for (aoffset = 0; aoffset < 3; aoffset++) {
      xpageoffset = aoffset * x_size_idx_0;
      y[aoffset] = qr_data[xpageoffset];
      for (k = 2; k <= x_size_idx_0; k++) {
        y[aoffset] += qr_data[(xpageoffset + k) - 1];
      }
    }
  }

  for (j = 0; j < 3; j++) {
    tHat[j] = b_y[j] - ((rHat[j] * (a * y[0]) + rHat[j + 3] * (a * y[1])) +
                        rHat[j + 6] * (a * y[2]));
  }
}

// End of code generation (calibrate2.cpp)
