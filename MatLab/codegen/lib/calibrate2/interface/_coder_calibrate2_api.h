/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_calibrate2_api.h
 *
 * Code generation for function '_coder_calibrate2_api'
 *
 */

#ifndef _CODER_CALIBRATE2_API_H
#define _CODER_CALIBRATE2_API_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void calibrate2(real_T pr_data[], int32_T pr_size[2], real_T pw_data[],
  int32_T pw_size[2], real_T rHat[9], real_T tHat[3]);
extern void calibrate2_api(const mxArray * const prhs[2], int32_T nlhs, const
  mxArray *plhs[2]);
extern void calibrate2_atexit(void);
extern void calibrate2_initialize(void);
extern void calibrate2_terminate(void);
extern void calibrate2_xil_shutdown(void);
extern void calibrate2_xil_terminate(void);

#endif

/* End of code generation (_coder_calibrate2_api.h) */
