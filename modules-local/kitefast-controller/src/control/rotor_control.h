#ifndef ROTOR_CONTROLLER_H
#define ROTOR_CONTROLLER_H

#include "control/physics/motor_param_types.h"
#include "control/physics/motors.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void InitMotorControl(const double rotor_omegas[]);

void MotorControlStep(const PowerSysSimParams *motor_params,
                      const RotorParams rotor_params[],
                      const PowerSysParams *power_sys_params,
                      const double rotor_cmds[],
                      const double external_torques[],
                      double *rotor_omegas, double *rotor_torques);

#ifdef __cplusplus
}  // extern "C"
#endif
#endif

