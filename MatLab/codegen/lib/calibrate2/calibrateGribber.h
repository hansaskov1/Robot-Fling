//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  calibrateGribber.h
//
//  Code generation for function 'calibrateGribber'
//


#ifndef CALIBRATEGRIBBER_H
#define CALIBRATEGRIBBER_H

// Include files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "calibrate2_types.h"

// Type Definitions
class calibrateGribber
{
 public:
  calibrateGribber();
  ~calibrateGribber();
  void calibrate2(const coder::array<double, 2U> &pr, const coder::array<double,
                  2U> &pw, double rHat[9], double tHat[3]);
};

#endif

// End of code generation (calibrateGribber.h)
