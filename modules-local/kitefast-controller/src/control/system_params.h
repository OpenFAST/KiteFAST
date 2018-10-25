#ifndef CONTROL_SYSTEM_PARAMS_H_
#define CONTROL_SYSTEM_PARAMS_H_

#include "system_types.h"

typedef struct {
  const double *ts;
  const PhysParams *phys;
  const TetherParams *tether;
  const WingParams *wing;
  const PerchParams *perch;
  const RotorParams *rotors[kNumMotors];
} GlobalSystemParams;

extern const GlobalSystemParams g_sys;

#ifdef __cplusplus
extern "C" {
#endif

const SystemParams *GetSystemParams(void);
SystemParams *GetSystemParamsUnsafe(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_SYSTEM_PARAMS_H_
