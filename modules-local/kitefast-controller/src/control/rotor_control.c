#include "control/rotor_control.h"

#include "common/c_math/filter.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "control/system_params.h"
#include "control/physics/motor_params.h"
#include "control/physics/motor_types.h"

#define APPLY_RATE_LIMIT 1

//const double dt_motor_loop = 0.01;  // Loop time
// double motor_speeds[kNumMotors] = {0};  // Maintain motor state locally. NOTE: moved to control_types.h - Jmiller - STI
// double rate_limited_cmd[kNumMotors] = {0};
// double cmd_filter_z[kNumMotors] = {0};
// double omega_cmd_int[kNumMotors] = {0};

void InitMotorControl(MotorState *motor_state) {
  for (int i = 0; i < kNumMotors; i++) {
    // Set speeds and commands to assume steady state behavior
    motor_state->motor_speeds[i] = motor_state->rotor_omegas[i];
    motor_state->rate_limited_cmd[i] = motor_state->rotor_omegas[i];
    motor_state->cmd_filter_z[i] = 0;//motor_state->rotor_omegas[i];
  }
}

// Everything here is in the frame of the motor with positive torque
// accelerating the motor and negative torque decelerating the motor.  Rotor
// commands are assumed to be from control_output, where direction of rotation
// has not yet been taken into account.
__attribute__((optimize(0)))  void MotorControlStep(const PowerSysSimParams *motor_params,
  const RotorParams rotor_params[], const PowerSysParams *power_sys_params,
  const double rotor_cmds[], const double external_torques[],
  MotorState *motor_state) {
  // Motors are independent so we just need to solve over all motors
  for (int32_t motor_number = 0; motor_number < kNumMotors; motor_number++) {

    double omega_cmd = rotor_cmds[motor_number];

    // Apply Rate Limit
    #if APPLY_RATE_LIMIT
     omega_cmd = RateLimit(omega_cmd, -motor_params->omega_cmd_rate_limit,
                           motor_params->omega_cmd_rate_limit,
                           *g_sys.ts, &motor_state->rate_limited_cmd[motor_number]);
    #endif

    // Apply command filter
    double a = -exp(*g_sys.ts * motor_params->speed_cmd_pole);
    double a_array[2] = {1.0, a};
    double b_array[2] = {1 + a, 0.0};
    double omega_cmd_filt = Filter(omega_cmd, 2, a_array, b_array, &motor_state->cmd_filter_z[motor_number]);

    // Calculate reachable motor speeds
    double motor_bus_voltage = power_sys_params->v_source_0 / kNumMotors * 2.0;
    TorqueLimits torque_limit = CalcTorqueLimits(motor_bus_voltage, motor_state->motor_speeds[motor_number],
                              &(motor_params->motor));

    // Apply control with limits and anti-windup
    double omega_error = omega_cmd_filt - motor_state->motor_speeds[motor_number];
    double kp = motor_params->kp_rotor_vel_err;
    double torque = kp * omega_error + motor_state->omega_cmd_int[motor_number];

    double ki = motor_params->ki_rotor_vel_err * *g_sys.ts;
    if (torque >= torque_limit.upper_limit) {
      torque = torque_limit.upper_limit;
    } else if (torque <= torque_limit.lower_limit) {
      torque = torque_limit.lower_limit;
    } else {
      motor_state->omega_cmd_int[motor_number] += ki * omega_error;
    }

    // Assume torque response is instantaneous. 
    // Add in external_torques (assume contribution accelerates propeller).
    // Divide by individual propeller inertia and run discrete integrator.
    // TODO(gdolan): Consider whether a zoh is required on the torque.
    double rotor_torque = torque + external_torques[motor_number]; // external torques coming in should be opposite sign  
    double rotor_accel = rotor_torque / rotor_params[motor_number].I;
    motor_state->motor_speeds[motor_number] += rotor_accel * *g_sys.ts;

    // Copy variables to output
    motor_state->rotor_torques[motor_number] = rotor_torque;
    motor_state->rotor_omegas[motor_number] = motor_state->motor_speeds[motor_number];
    motor_state->rotor_accel[motor_number]  = rotor_accel;
    // motor_state->aero_torque[motor_number] = external_torques[motor_number];
  }
}

// int main(int argc, char *argv[])
// {
//   const SystemParams *sp = GetSystemParams();
//   const PowerSysSimParams *psp = GetMotorParams();

//   FILE *fp = fopen("test_rotor.dat", "w");
//   if (fp == NULL) {
//       printf("Quitting: Unable to open output file\n");
//       return -1;
//   }

//   double rotor_cmds[kNumMotors] = {0};
//   double ext_torques[kNumMotors] = {0};
//   double omegas[kNumMotors] = {30.0, -30.0, 30.0, -30.0, 30.0, -30.0, 30.0, -30.0};
//   double torques[kNumMotors] = {0};

//   InitMotorControl(omegas);

//   for (int i = 0; i < 100000; i++) {
//       // Calculate next speed command
//       double cmd = 120.0 + 90.0 * sin(2.0 * PI * 0.005 * (double)i);

//       for (int motor_i = 0; motor_i < kNumMotors; motor_i++) {
//           // Calculate some torques - Just roughing it for now
//           ext_torques[motor_i] = copysignf(1.0f, omegas[motor_i]) * 1200.0 - 10.0 * omegas[motor_i];

//           // Calculate next speed command
//           rotor_cmds[motor_i] = copysignf(1.0f, omegas[motor_i]) * cmd;
//       }
//       MotorControlStep(psp, &(sp->rotors[0]), &(sp->power_sys),
//                        rotor_cmds, ext_torques, omegas, torques);

//       fwrite(rotor_cmds, sizeof(double), kNumMotors, fp);
//       fwrite(ext_torques, sizeof(double), kNumMotors, fp);
//       fwrite(torques, sizeof(double), kNumMotors, fp);
//       fwrite(omegas, sizeof(double), kNumMotors, fp);
//   }

//   fclose(fp);
//   return 0;
// }

