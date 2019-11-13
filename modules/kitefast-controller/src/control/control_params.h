#ifndef CONTROL_CONTROL_PARAMS_H_
#define CONTROL_CONTROL_PARAMS_H_

#include "control/control_types.h"

typedef struct {
  const FlightPlan *flight_plan;
  const ControlOption *control_opt;
  const SimpleAeroModelParams *simple_aero_model;
  const JoystickControlParams *joystick_control;
  const RotorControlParams *rotor_control;
} GlobalControlParams;

extern GlobalControlParams g_cont;

#ifdef __cplusplus
extern "C" {
#endif

const ControlParams *GetControlParams(void);
ControlParams *GetControlParamsUnsafe(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CONTROL_PARAMS_H_
