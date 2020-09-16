//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  calibrate2_initialize.cpp
//
//  Code generation for function 'calibrate2_initialize'
//


// Include files
#include "calibrate2_initialize.h"
#include "calibrate2.h"
#include "calibrate2_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void calibrate2_initialize()
{
  rt_InitInfAndNaN();
  isInitialized_calibrate2 = true;
}

// End of code generation (calibrate2_initialize.cpp)
