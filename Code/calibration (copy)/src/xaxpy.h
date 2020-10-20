//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xaxpy.h
//
//  Code generation for function 'xaxpy'
//


#ifndef XAXPY_H
#define XAXPY_H

// Include files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "calibrate2_types.h"

// Function Declarations
extern void b_xaxpy(int n, double a, const double x[9], int ix0, double y[3],
                    int iy0);
extern void c_xaxpy(int n, double a, const double x[3], int ix0, double y[9],
                    int iy0);
extern void xaxpy(int n, double a, int ix0, double y[9], int iy0);

#endif

// End of code generation (xaxpy.h)
