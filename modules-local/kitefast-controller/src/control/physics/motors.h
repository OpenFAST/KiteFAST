#ifndef SIM_PHYSICS_MOTORS_H_
#define SIM_PHYSICS_MOTORS_H_

#include "control/physics/motor_param_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct TorqueLimits {
  double lower_limit;
  SimMotorLimit lower_constraint;
  double upper_limit;
  SimMotorLimit upper_constraint;
} TorqueLimits;

// Applies directions to the motor speeds.
void SetMotorDirection(double rotor_omegas[], double rotor_accel[], double rotor_torques[]);

// Calculates upper and lower torque limits based on the voltage,
// rotational velocity, motor parameters, and programmed current
// limits.
TorqueLimits CalcTorqueLimits(double voltage, double rotor_vel,
                              const MotorParams *params);

// Calculate the electrical terminal power for the torque, speed, and voltage
// for the specified motor.
double CalcMotorPower(double voltage, double torque, double rotor_vel,
                      const MotorParams *params);

// Helper function for calculating motor controller loss contribution.
double CalcMotorControllerLoss(double voltage, double peak_phase_current_sq,
                               const MotorParams *params);
#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SIM_PHYSICS_MOTORS_H_
