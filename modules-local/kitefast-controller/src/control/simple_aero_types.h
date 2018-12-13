#ifndef CONTROL_SIMPLE_AERO_TYPES_H_
#define CONTROL_SIMPLE_AERO_TYPES_H_

#include "system/labels.h"

#define NUM_SIMPLE_ROTOR_MODEL_COEFFS 3

typedef struct {
  double thrust_coeffs[NUM_SIMPLE_ROTOR_MODEL_COEFFS];
  double J_neutral;
  double J_max;
  double D;
  double D4;
} SimpleRotorModelParams;

typedef struct {
  double dCL_dalpha;
  double dCD_dalpha;
  double CL_0;
  double CD_0;
  double base_flaps[kNumFlaps];  // Flap deflections at which CL_0 is calculated
  double dCL_dflap[kNumFlaps];
  double dCY_dbeta;
  double CY_0;
} SimpleAeroModelParams;

#endif  // CONTROL_SIMPLE_AERO_TYPES_H_
