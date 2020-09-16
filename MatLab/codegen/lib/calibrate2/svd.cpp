//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  svd.cpp
//
//  Code generation for function 'svd'
//


// Include files
#include "svd.h"
#include "calibrateGribber.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"
#include <cmath>

// Function Definitions
void svd(const double A[9], double U[9], double s[3], double V[9])
{
  double e[3];
  double work[3];
  int kase;
  boolean_T apply_transform;
  double b_A[9];
  double nrm;
  double b_s[3];
  double r;
  int qp1;
  int qjj;
  int m;
  int q;
  int qq;
  double snorm;
  double sm;
  double sqds;
  e[0] = 0.0;
  work[0] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (kase = 0; kase < 9; kase++) {
    b_A[kase] = A[kase];
    U[kase] = 0.0;
    V[kase] = 0.0;
  }

  apply_transform = false;
  nrm = xnrm2(3, b_A, 1);
  if (nrm > 0.0) {
    apply_transform = true;
    if (b_A[0] < 0.0) {
      r = -nrm;
      b_s[0] = -nrm;
    } else {
      r = nrm;
      b_s[0] = nrm;
    }

    if (std::abs(r) >= 1.0020841800044864E-292) {
      nrm = 1.0 / r;
      for (qp1 = 1; qp1 < 4; qp1++) {
        b_A[qp1 - 1] *= nrm;
      }
    } else {
      for (qp1 = 1; qp1 < 4; qp1++) {
        b_A[qp1 - 1] /= b_s[0];
      }
    }

    b_A[0]++;
    b_s[0] = -b_s[0];
  } else {
    b_s[0] = 0.0;
  }

  for (kase = 2; kase < 4; kase++) {
    qjj = 3 * (kase - 1);
    if (apply_transform) {
      xaxpy(3, -(xdotc(3, b_A, 1, b_A, qjj + 1) / b_A[0]), 1, b_A, qjj + 1);
    }

    e[kase - 1] = b_A[qjj];
  }

  for (qp1 = 1; qp1 < 4; qp1++) {
    U[qp1 - 1] = b_A[qp1 - 1];
  }

  nrm = b_xnrm2(e, 2);
  if (nrm == 0.0) {
    e[0] = 0.0;
  } else {
    if (e[1] < 0.0) {
      e[0] = -nrm;
    } else {
      e[0] = nrm;
    }

    nrm = e[0];
    if (std::abs(e[0]) >= 1.0020841800044864E-292) {
      nrm = 1.0 / e[0];
      for (qp1 = 2; qp1 < 4; qp1++) {
        e[qp1 - 1] *= nrm;
      }
    } else {
      for (qp1 = 2; qp1 < 4; qp1++) {
        e[qp1 - 1] /= nrm;
      }
    }

    e[1]++;
    e[0] = -e[0];
    for (qp1 = 2; qp1 < 4; qp1++) {
      work[qp1 - 1] = 0.0;
    }

    for (kase = 2; kase < 4; kase++) {
      b_xaxpy(2, e[kase - 1], b_A, 3 * (kase - 1) + 2, work, 2);
    }

    for (kase = 2; kase < 4; kase++) {
      c_xaxpy(2, -e[kase - 1] / e[1], work, 2, b_A, 3 * (kase - 1) + 2);
    }
  }

  for (qp1 = 2; qp1 < 4; qp1++) {
    V[qp1 - 1] = e[qp1 - 1];
  }

  apply_transform = false;
  nrm = xnrm2(2, b_A, 5);
  if (nrm > 0.0) {
    apply_transform = true;
    if (b_A[4] < 0.0) {
      r = -nrm;
      b_s[1] = -nrm;
    } else {
      r = nrm;
      b_s[1] = nrm;
    }

    if (std::abs(r) >= 1.0020841800044864E-292) {
      nrm = 1.0 / r;
      for (qp1 = 5; qp1 < 7; qp1++) {
        b_A[qp1 - 1] *= nrm;
      }
    } else {
      for (qp1 = 5; qp1 < 7; qp1++) {
        b_A[qp1 - 1] /= b_s[1];
      }
    }

    b_A[4]++;
    b_s[1] = -b_s[1];
  } else {
    b_s[1] = 0.0;
  }

  for (kase = 3; kase < 4; kase++) {
    if (apply_transform) {
      xaxpy(2, -(xdotc(2, b_A, 5, b_A, 8) / b_A[4]), 5, b_A, 8);
    }
  }

  for (qp1 = 2; qp1 < 4; qp1++) {
    U[qp1 + 2] = b_A[qp1 + 2];
  }

  m = 1;
  b_s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (q = 1; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 3 * q;
    if (b_s[q] != 0.0) {
      for (kase = qp1; kase < 4; kase++) {
        qjj = (q + 3 * (kase - 1)) + 1;
        xaxpy(3 - q, -(xdotc(3 - q, U, qq + 1, U, qjj) / U[qq]), qq + 1, U, qjj);
      }

      for (qp1 = q + 1; qp1 < 4; qp1++) {
        kase = (qp1 + 3 * q) - 1;
        U[kase] = -U[kase];
      }

      U[qq]++;
      if (0 <= q - 1) {
        U[3 * q] = 0.0;
      }
    } else {
      U[3 * q] = 0.0;
      U[3 * q + 1] = 0.0;
      U[3 * q + 2] = 0.0;
      U[qq] = 1.0;
    }
  }

  for (q = 2; q >= 0; q--) {
    if ((q + 1 <= 1) && (e[0] != 0.0)) {
      xaxpy(2, -(xdotc(2, V, 2, V, 5) / V[1]), 2, V, 5);
      xaxpy(2, -(xdotc(2, V, 2, V, 8) / V[1]), 2, V, 8);
    }

    V[3 * q] = 0.0;
    V[3 * q + 1] = 0.0;
    V[3 * q + 2] = 0.0;
    V[q + 3 * q] = 1.0;
  }

  qq = 0;
  snorm = 0.0;
  for (q = 0; q < 3; q++) {
    if (b_s[q] != 0.0) {
      nrm = std::abs(b_s[q]);
      r = b_s[q] / nrm;
      b_s[q] = nrm;
      if (q + 1 < 3) {
        e[q] /= r;
      }

      qjj = 3 * q;
      kase = qjj + 3;
      for (qp1 = qjj + 1; qp1 <= kase; qp1++) {
        U[qp1 - 1] *= r;
      }
    }

    if ((q + 1 < 3) && (e[q] != 0.0)) {
      nrm = std::abs(e[q]);
      r = nrm / e[q];
      e[q] = nrm;
      b_s[q + 1] *= r;
      qjj = 3 * (q + 1);
      kase = qjj + 3;
      for (qp1 = qjj + 1; qp1 <= kase; qp1++) {
        V[qp1 - 1] *= r;
      }
    }

    nrm = std::abs(b_s[q]);
    r = std::abs(e[q]);
    if ((nrm > r) || rtIsNaN(r)) {
      r = nrm;
    }

    if ((!(snorm > r)) && (!rtIsNaN(r))) {
      snorm = r;
    }
  }

  while ((m + 2 > 0) && (qq < 75)) {
    qp1 = m;
    int exitg1;
    do {
      exitg1 = 0;
      q = qp1 + 1;
      if (qp1 + 1 == 0) {
        exitg1 = 1;
      } else {
        nrm = std::abs(e[qp1]);
        if ((nrm <= 2.2204460492503131E-16 * (std::abs(b_s[qp1]) + std::abs
              (b_s[qp1 + 1]))) || (nrm <= 1.0020841800044864E-292) || ((qq > 20)
             && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[qp1] = 0.0;
          exitg1 = 1;
        } else {
          qp1--;
        }
      }
    } while (exitg1 == 0);

    if (qp1 + 1 == m + 1) {
      kase = 4;
    } else {
      boolean_T exitg2;
      qjj = m + 2;
      kase = m + 2;
      exitg2 = false;
      while ((!exitg2) && (kase >= qp1 + 1)) {
        qjj = kase;
        if (kase == qp1 + 1) {
          exitg2 = true;
        } else {
          nrm = 0.0;
          if (kase < m + 2) {
            nrm = std::abs(e[kase - 1]);
          }

          if (kase > qp1 + 2) {
            nrm += std::abs(e[kase - 2]);
          }

          r = std::abs(b_s[kase - 1]);
          if ((r <= 2.2204460492503131E-16 * nrm) || (r <=
               1.0020841800044864E-292)) {
            b_s[kase - 1] = 0.0;
            exitg2 = true;
          } else {
            kase--;
          }
        }
      }

      if (qjj == qp1 + 1) {
        kase = 3;
      } else if (qjj == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        q = qjj;
      }
    }

    switch (kase) {
     case 1:
      r = e[m];
      e[m] = 0.0;
      kase = m + 1;
      for (qp1 = kase; qp1 >= q + 1; qp1--) {
        xrotg(&b_s[qp1 - 1], &r, &sm, &sqds);
        if (qp1 > q + 1) {
          r = -sqds * e[0];
          e[0] *= sm;
        }

        xrot(V, 3 * (qp1 - 1) + 1, 3 * (m + 1) + 1, sm, sqds);
      }
      break;

     case 2:
      {
        r = e[q - 1];
        e[q - 1] = 0.0;
        for (qp1 = q + 1; qp1 <= m + 2; qp1++) {
          double b;
          xrotg(&b_s[qp1 - 1], &r, &sm, &sqds);
          b = e[qp1 - 1];
          r = -sqds * b;
          e[qp1 - 1] = b * sm;
          xrot(U, 3 * (qp1 - 1) + 1, 3 * (q - 1) + 1, sm, sqds);
        }
      }
      break;

     case 3:
      {
        double scale;
        double b;
        kase = m + 1;
        nrm = b_s[m + 1];
        scale = std::abs(nrm);
        r = std::abs(b_s[m]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(e[m]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(b_s[q]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        r = std::abs(e[q]);
        if ((!(scale > r)) && (!rtIsNaN(r))) {
          scale = r;
        }

        sm = nrm / scale;
        nrm = b_s[m] / scale;
        r = e[m] / scale;
        sqds = b_s[q] / scale;
        b = ((nrm + sm) * (nrm - sm) + r * r) / 2.0;
        nrm = sm * r;
        nrm *= nrm;
        if ((b != 0.0) || (nrm != 0.0)) {
          r = std::sqrt(b * b + nrm);
          if (b < 0.0) {
            r = -r;
          }

          r = nrm / (b + r);
        } else {
          r = 0.0;
        }

        r += (sqds + sm) * (sqds - sm);
        nrm = sqds * (e[q] / scale);
        for (qp1 = q + 1; qp1 <= kase; qp1++) {
          xrotg(&r, &nrm, &sm, &sqds);
          if (qp1 > q + 1) {
            e[0] = r;
          }

          nrm = e[qp1 - 1];
          b = b_s[qp1 - 1];
          e[qp1 - 1] = sm * nrm - sqds * b;
          r = sqds * b_s[qp1];
          b_s[qp1] *= sm;
          xrot(V, 3 * (qp1 - 1) + 1, 3 * qp1 + 1, sm, sqds);
          b_s[qp1 - 1] = sm * b + sqds * nrm;
          xrotg(&b_s[qp1 - 1], &r, &sm, &sqds);
          r = sm * e[qp1 - 1] + sqds * b_s[qp1];
          b_s[qp1] = -sqds * e[qp1 - 1] + sm * b_s[qp1];
          nrm = sqds * e[qp1];
          e[qp1] *= sm;
          xrot(U, 3 * (qp1 - 1) + 1, 3 * qp1 + 1, sm, sqds);
        }

        e[m] = r;
        qq++;
      }
      break;

     default:
      if (b_s[q] < 0.0) {
        b_s[q] = -b_s[q];
        qjj = 3 * q;
        kase = qjj + 3;
        for (qp1 = qjj + 1; qp1 <= kase; qp1++) {
          V[qp1 - 1] = -V[qp1 - 1];
        }
      }

      qp1 = q + 1;
      while ((q + 1 < 3) && (b_s[q] < b_s[qp1])) {
        nrm = b_s[q];
        b_s[q] = b_s[qp1];
        b_s[qp1] = nrm;
        xswap(V, 3 * q + 1, 3 * (q + 1) + 1);
        xswap(U, 3 * q + 1, 3 * (q + 1) + 1);
        q = qp1;
        qp1++;
      }

      qq = 0;
      m--;
      break;
    }
  }

  s[0] = b_s[0];
  s[1] = b_s[1];
  s[2] = b_s[2];
}

// End of code generation (svd.cpp)
