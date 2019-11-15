#ifndef CONTROL_ACTUATOR_TYPES_H_
#define CONTROL_ACTUATOR_TYPES_H_

// #include "avionics/network/aio_labels.h"
#include "common/c_math/vec3.h"
#include "control/simple_aero_types.h"

typedef enum {
  kStackingStateForceSigned = -1,
  kStackingStateNormal,
  kStackingStateFaultBlock1,
  kStackingStateFaultBlock2,
  kStackingStateFaultBlock3,
  kStackingStateFaultBlock4,
  kNumStackingStates
} StackingState;

typedef struct {
  double aileron;
  double inboard_flap;
  double midboard_flap;
  double outboard_flap;
  double elevator;
  double rudder;
} Deltas;

typedef struct {
  double thrust;
  Vec3 moment;
} ThrustMoment;

#define FREESTREAM_VEL_TABLE_LENGTH 5
typedef struct {
  double idle_speed;
  double max_speeds[kNumMotors];
  double min_aero_power;
  bool penalize_symmetric_torsion_mode;
  double regularization_weight;
  double symmetric_torsion_weight;
  double thrust_moment_to_thrusts[kNumMotors][4];
  double thrusts_to_thrust_moment[4][kNumMotors];
  double comm_and_diff_thrusts_to_thrusts[kNumMotors][5];
  double comm_and_diff_thrusts_to_thrust_moment[kNumStackingStates][4][5];
  double constraint_matrix[kNumMotors + 1][5];
  double freestream_vel_table[FREESTREAM_VEL_TABLE_LENGTH];
  double max_thrusts[2][kNumMotors][FREESTREAM_VEL_TABLE_LENGTH];
  double total_power_limit_thrusts[FREESTREAM_VEL_TABLE_LENGTH];
  SimpleRotorModelParams simple_models[kNumMotors];
} RotorControlParams;

#endif  // CONTROL_ACTUATOR_TYPES_H_
