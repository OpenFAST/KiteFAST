#ifndef MOTOR_PARAMS_H_
#define MOTOR_PARAMS_H_

#include "control/physics/motor_param_types.h"

#ifdef __cplusplus
extern "C" {
#endif

const PowerSysSimParams *GetMotorParams(void);
PowerSysSimParams *GetMotorParamsUnsafe(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // MOTOR_PARAMS_H_
