#ifndef MOTOR_TYPES_H_
#define MOTOR_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "system/labels.h"

typedef struct {

    double motor_speeds[kNumMotors];  // Maintain motor state locally. NOTE: moved to control_types.h - Jmiller - STI
    double rate_limited_cmd[kNumMotors];
    double cmd_filter_z[kNumMotors];
    double omega_cmd_int[kNumMotors];
    double rotor_omegas[kNumMotors];
    double rotor_torques[kNumMotors];
    double rotor_accel[kNumMotors];
    
} MotorState;


#endif