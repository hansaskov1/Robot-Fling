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

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void calibrate2(emxArray_real_T *pr, emxArray_real_T *pw, real_T rHat[9],
  real_T tHat[3]);
extern void calibrate2_api(const mxArray * const prhs[2], int32_T nlhs, const
  mxArray *plhs[2]);
extern void calibrate2_atexit(void);
extern void calibrate2_initialize(void);
extern void calibrate2_terminate(void);
extern void calibrate2_xil_shutdown(void);
extern void calibrate2_xil_terminate(void);

#endif

/* End of code generation (_coder_calibrate2_api.h) */
