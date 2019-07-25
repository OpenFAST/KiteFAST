#include "control/actuator_util.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/linalg.h"
#include "common/c_math/optim.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/ground_station_frame.h"
#include "control/simple_aero.h"
#include "system/labels.h"
// #include "system/labels_util.h"

bool ActuatorUtilValidateParams(const RotorControlParams *params) {
  assert(params != NULL);

  if (params->idle_speed < 0.0) {
    assert(!(bool)"idle_speed must be non-negative.");
    return false;
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    if (params->idle_speed >= params->max_speeds[i]) {
      assert(!(bool)"max_speeds must be greater than idle_speed.");
      return false;
    }
  }

  if (params->regularization_weight < 0.0) {
    assert(!(bool)"regularization_weight must be non-negative.");
    return false;
  }

  if (params->min_aero_power > -5e5) {
    assert(!(bool)"min_aero_power must be plausibly negative.");
    return false;
  }

  if (params->freestream_vel_table[0] < 0.0) {
    assert(!(bool)"freestream_vel_table must be non-negative.");
    return false;
  }

  for (int32_t j = 1; j < FREESTREAM_VEL_TABLE_LENGTH; ++j) {
    if (params->freestream_vel_table[j - 1] > params->freestream_vel_table[j]) {
      assert(!(bool)"freestream_vel_table must be strictly increasing.");
      return false;
    }
  }

  for (int32_t i = 0; i < 2; i++) {
    for (int32_t j = 0; j < kNumMotors; ++j) {
      for (int32_t k = 1; k < FREESTREAM_VEL_TABLE_LENGTH; ++k) {
        if (params->max_thrusts[i][j][k - 1] < params->max_thrusts[i][j][k]) {
          assert(
              !(bool)"max_thrusts should decrease with freestream velocity.");
          return false;
        }
      }
    }
  }

  if (params->total_power_limit_thrusts[0] < 0.0) {
    assert(!(bool)"total_power_limit_thrusts should be positive in still air.");
    return false;
  }
  for (int32_t j = 1; j < FREESTREAM_VEL_TABLE_LENGTH; ++j) {
    if (params->total_power_limit_thrusts[j - 1] <
        params->total_power_limit_thrusts[j]) {
      assert(!(bool)"total_power_limit_thrusts should decrease with velocity.");
      return false;
    }
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    if (params->simple_models[i].J_max < 0.0) {
      assert(!(bool)"J_max must be non-negative.");
      return false;
    }
    if (params->simple_models[i].J_neutral > params->simple_models[i].J_max) {
      assert(!(bool)"J_neutral must be less than J_max.");
      return false;
    }
    if (params->simple_models[i].D < 0.0) {
      assert(!(bool)"D must be non-negative.");
      return false;
    }
  }

  return true;
}

// Computes the expected maximum thrust for a given apparent wind.
double CalcMaxTotalThrust(double v_app, const RotorControlParams *params) {
  assert(v_app >= 0.0 && params != NULL);
  return Interp1(params->freestream_vel_table,
                 params->total_power_limit_thrusts, FREESTREAM_VEL_TABLE_LENGTH,
                 v_app, kInterpOptionSaturate);
}

// Calculate the maximum thrust for a given rotor based on a speed
// limit and model of thrust limits as a function of freestream
// velocity.
static double CalcMaxThrust(double freestream_vel, double max_speed,
                            double air_density,
                            const double freestream_vel_table[],
                            const double max_thrusts[],
                            const SimpleRotorModelParams *params) {
  return fmin(
      Interp1(freestream_vel_table, max_thrusts, FREESTREAM_VEL_TABLE_LENGTH,
              freestream_vel, kInterpOptionSaturate),
      OmegaToThrust(max_speed, freestream_vel, air_density, params));
}

// Builds the constraint matrix and limits for the
// ThrustMomentToThrusts function.  Also, returns a vector that is
// guaranteed to meet the constraints.
static void SetupThrustPowerConstraints(const Vec *min_thrusts,
                                        const Vec *max_thrusts,
                                        double total_power_limit_thrust,
                                        const RotorControlParams *params,
                                        Mat *C, Vec *lower, Vec *upper,
                                        Vec *comm_and_diff_thrusts) {
  assert(VecIsSize(min_thrusts, kNumMotors));
  assert(VecIsSize(max_thrusts, kNumMotors));

  double max_min_thrusts = -INFINITY;
  double min_max_thrusts = INFINITY;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    assert(VecGet(max_thrusts, i) > VecGet(min_thrusts, i));
    if (VecGet(min_thrusts, i) > max_min_thrusts) {
      max_min_thrusts = VecGet(min_thrusts, i);
    }
    if (VecGet(max_thrusts, i) < min_max_thrusts) {
      min_max_thrusts = VecGet(max_thrusts, i);
    }
  }
  if (max_min_thrusts >= min_max_thrusts) {
    max_min_thrusts = min_max_thrusts - 1.0;
  }

  // Initial guess for common- and differential-mode thrusts that is
  // guaranteed to meet the individual thrust constraints defined
  // below.
  VecZero(comm_and_diff_thrusts);
  *VecPtr(comm_and_diff_thrusts, 0) = (min_max_thrusts + max_min_thrusts) / 2.0;

  // Initialize the constraint matrix.
  MatInit(&params->constraint_matrix[0][0], kNumMotors + 1, 5, C);

  // Set-up lower and upper bounds on individual thrusts.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    *VecPtr(lower, i) = fmin(VecGet(min_thrusts, i), max_min_thrusts);
    *VecPtr(upper, i) = VecGet(max_thrusts, i);
  }

  // Set-up maximum common-mode thrust bounds.  To ensure that the
  // initial guess is within the constraints we set the total thrust
  // bound to be at least as large as one more than the initial guess.
  *VecPtr(lower, kNumMotors) = -INFINITY;
  *VecPtr(upper, kNumMotors) =
      fmax(VecGet(comm_and_diff_thrusts, 0) + 1.0, total_power_limit_thrust);
}

// Convert common- and differential-mode thrusts solution to the
// predicted available thrust-moment.
static void CalcAvailableThrustMoment(StackingState stacking_state,
                                      const Vec *comm_and_diff_thrusts,
                                      const RotorControlParams *params,
                                      ThrustMoment *available_thrust_moment) {
  assert(kStackingStateNormal <= stacking_state &&
         stacking_state < kNumStackingStates);
  MAT_CLONE(
      4, 5, comm_and_diff_thrusts_to_thrust_moment,
      &params->comm_and_diff_thrusts_to_thrust_moment[stacking_state][0][0]);
  VEC_INIT(4, available_thrust_moment_vec, {0});
  MatVecMult(&comm_and_diff_thrusts_to_thrust_moment, comm_and_diff_thrusts,
             &available_thrust_moment_vec);
  available_thrust_moment->thrust = VecGet(&available_thrust_moment_vec, 0);
  available_thrust_moment->moment.x = VecGet(&available_thrust_moment_vec, 1);
  available_thrust_moment->moment.y = VecGet(&available_thrust_moment_vec, 2);
  available_thrust_moment->moment.z = VecGet(&available_thrust_moment_vec, 3);
}

// Solves a constrained least squares problem to approximate the
// required motor thrusts to achieve a desired total thrust and
// moment.
//
// Stacking constraint:
//
// The stacked motor system consists of four blocks, each block
// being a pair of motors:
//
//    PTO    PTI    STI    STO            Bot. Top
//   ( o )  ( o )  ( o )  ( o )  Block 1: SBO  PTO----------+
//     |______|______|______|    Block 2: SBI  PTI \__ inner \__outer
//     |      |      |      |    Block 3: PBI  STI /         /
//   ( o )  ( o )  ( o )  ( o )  Block 4: PBO  STO----------+
//    PBO    PBI    SBI    SBO
//
// Equal current must flow through each block, and the stacking motor
// controller attempts to regulate equal voltage on each block.  As a
// result, each block should be commanded to have equal power.  This
// function approximates this constraint by requesting thrusts such
// that the sum of the motor thrusts is the same for each block.
// During hover, when advance ratios are nearly zero and thrusts are
// nearly equal, this approximation is reasonable.
//
// Torque and advance ratio constraints:
//
// Individual motors have a maximum torque constraint during hover and
// transition-in and a maximum advance ratio constraint, i.e. minimum
// rotor speed before stall, in crosswind.  These constraints may be
// converted to equivalent thrust values using a model of the rotors,
// and thus are modeled as minimum and maximum thrust constraints.
//
// Total power constraint:
//
// The total power available to the rotors is, in general, less than
// the power that would be used if each rotor were at its maximum
// torque limit.  To model this, we pass in parameters for the power
// per unit thrust used by each rotor at the operating point, and use
// this to calculate a total power which must be greater than a given
// minimum (recall that negative power is thrusting).
//
// The decision variables for this optimization problem are:
//
//   x = [common_mode_thrust ; diff_thrust_1 ; ... ; diff_thrust_4]
//
// We minimize ||W*(Ax - b)||^2 where A and b have the following structure:
//
//   If penalizing the symmetric torsion mode:
//     A * x = [diff_thrust_1 ; diff_thrust_2 ; diff_thrust_3 ; diff_thrust_4 ;
//              thrust ; roll_moment ; pitch_moment ; yaw_moment;
//              symmetric_torsion_mode],
//     b = [0 ; 0 ; 0 ; 0 ;
//          thrust_cmd; roll_moment_cmd ; pitch_moment_cmd ; yaw_moment_cmd;
//          0],
//   Otherwise:
//     A * x = [diff_thrust_1 ; diff_thrust_2 ; diff_thrust_3 ; diff_thrust_4 ;
//              thrust ; roll_moment ; pitch_moment ; yaw_moment],
//     b = [0 ; 0 ; 0 ; 0 ;
//          thrust_cmd; roll_moment_cmd ; pitch_moment_cmd ; yaw_moment_cmd],
//
// and W is a diagonal matrix containing the associated weights.  The
// first four entries of these matrices are used to "regularize" the
// problem, ensuring that the cost function is strongly convex and that
// minimizers are unique.  This slightly biases the resulting total
// thrust and moment solutions, even when constraints are not hit.
// The objective of this regularization is to reduce the sensitivity
// of the solutions to small variations in problem inputs.
//
// Args:
//   thrust_moment: Desired thrust-moment command ([N] for thrust and
//       [N-m] for moment).
//   weights: Weight to place on each component ([1/N^2] for thrust and
//       [1/(N^2-m^2)] for moment).
//   min_thrusts: Vector of kNumMotors minimum thrusts.
//   max_thrusts: Vector of kNumMotors maximum thrusts.  Note that to
//       guarantee a valid initial guess for the constrained least
//       squares algorithm, we enforce that the maximum of the
//       min_thrusts is strictly less than the minimum of the
//       max_thrusts.
//   total_power_limit_thrust: Maximum total thrust command [N].  This is set to
//       roughly enforce a total power limit.
//   stacking_state: State of the stacked motor system.
//   params: Set of parameters describing the constraint matrix and
//       the conversions between common- and differential-thrusts and
//       individual thrusts.
//   thrusts: Output vector of kNumMotors thrusts.
//   available_thrust_moment: Output total thrust and moments
//       generated by thrusts.
static void ThrustMomentToThrusts(
    const ThrustMoment *thrust_moment, const ThrustMoment *weights,
    const Vec *min_thrusts, const Vec *max_thrusts,
    double total_power_limit_thrust, StackingState stacking_state,
    const RotorControlParams *params, Vec *thrusts,
    ThrustMoment *available_thrust_moment) {
  assert(kStackingStateNormal <= stacking_state &&
         stacking_state < kNumStackingStates);

  MAT_INIT(kNumMotors + 1, 5, C, {{0}});
  VEC_INIT(kNumMotors + 1, lower, {0});
  VEC_INIT(kNumMotors + 1, upper, {0});
  VEC_INIT(5, comm_and_diff_thrusts, {0});
  SetupThrustPowerConstraints(min_thrusts, max_thrusts,
                              total_power_limit_thrust, params, &C, &lower,
                              &upper, &comm_and_diff_thrusts);

  MAT_INIT(9, 5, A, {{0}});

  // Use the first four rows to select the differential thrusts.
  for (int32_t i = 0; i < 4; ++i) *MatPtr(&A, i, i + 1) = 1.0;

  // The next four rows calculate the thrust and moments.
  MAT_CLONE(
      4, 5, comm_and_diff_thrusts_to_thrust_moment,
      &params->comm_and_diff_thrusts_to_thrust_moment[stacking_state][0][0]);
  MatSubmatSet(&comm_and_diff_thrusts_to_thrust_moment, 0, 0, 4, 5, 4, 0, &A);

  // Use last row to select the symmetric torsion null space mode.
  *MatPtr(&A, 8, 1) = 1.0;
  *MatPtr(&A, 8, 2) = -1.0;
  *MatPtr(&A, 8, 3) = -1.0;
  *MatPtr(&A, 8, 4) = 1.0;

  // See the comment above for the structure of b.
  VEC_INIT(9, b,
           {0.0, 0.0, 0.0, 0.0, thrust_moment->thrust, thrust_moment->moment.x,
            thrust_moment->moment.y, thrust_moment->moment.z, 0.0});

  // See the comment above for the structure of weights_vec.
  VEC_INIT(9, weights_vec,
           {params->regularization_weight, params->regularization_weight,
            params->regularization_weight, params->regularization_weight,
            weights->thrust, weights->moment.x, weights->moment.y,
            weights->moment.z, params->symmetric_torsion_weight});

  // If not penalizing the symmetric torsion mode, only the first 8 rows of the
  // system should be used.
  if (!params->penalize_symmetric_torsion_mode) {
    A.nr = 8;
    b.length = 8;
    weights_vec.length = 8;
  }

  WeightLeastSquaresInputs(&weights_vec, &A, &b);
  ConstrainedLeastSquares(&A, &b, &C, &lower, &upper, &comm_and_diff_thrusts,
                          &comm_and_diff_thrusts);

  // Convert common- and differential-mode thrusts solution to
  // individual thrusts.
  MAT_CLONE(kNumMotors, 5, comm_and_diff_thrusts_to_thrusts,
            params->comm_and_diff_thrusts_to_thrusts);
  MatVecMult(&comm_and_diff_thrusts_to_thrusts, &comm_and_diff_thrusts,
             thrusts);

  CalcAvailableThrustMoment(stacking_state, &comm_and_diff_thrusts, params,
                            available_thrust_moment);
}

void MixRotors(const ThrustMoment *thrust_moment, const ThrustMoment *weights,
               double v_app, const Vec3 *pqr, StackingState stacking_state,
               bool force_zero_advance_ratio, double air_density,
               const RotorParams *const rotor_params[],
               const RotorControlParams *params, double *rotors,
               ThrustMoment *available_thrust_moment, double *v_app_locals) {
  // Determine thrust limits.
  VEC_INIT(kNumMotors, min_thrusts, {0});
  VEC_INIT(kNumMotors, max_thrusts, {0});
  for (int32_t i = 0; i < kNumMotors; ++i) {
    double v_app_local, min_thrust;
    if (force_zero_advance_ratio) {
      v_app_local = 0.0;
      min_thrust = 0.0;
    } else {
      v_app_local =
          CalcLocalAirspeed(v_app, rotor_params[i]->local_pressure_coeff,
                            &rotor_params[i]->pos, pqr);

      min_thrust = CalcMinThrust(v_app_local, params->idle_speed, air_density,
                                 &params->simple_models[i]);
    }

    // Store the estimated local apparent wind for telemetry
    v_app_locals[i] = v_app_local;

    double max_thrust = CalcMaxThrust(
        v_app_local, params->max_speeds[i], air_density,
        params->freestream_vel_table,
        params->max_thrusts[stacking_state != kStackingStateNormal][i],
        &params->simple_models[i]);

    // Coerce min_thrust to be strictly less than max_thrust, which is
    // required by ThrustMomentToThrusts.
    if (min_thrust >= max_thrust) min_thrust = max_thrust - 1.0;
    *VecPtr(&min_thrusts, i) = min_thrust;
    *VecPtr(&max_thrusts, i) = max_thrust;
  }

  // Calculate the maximum common mode thrust.  We enforce a limit on
  // common mode thrust to roughly bound the total system power draw.
  double total_power_limit_thrust = CalcMaxTotalThrust(v_app, params);
  VEC_INIT(kNumMotors, thrusts, {0});
  ThrustMomentToThrusts(thrust_moment, weights, &min_thrusts, &max_thrusts,
                        total_power_limit_thrust, stacking_state, params,
                        &thrusts, available_thrust_moment);

  // Convert thrusts to rotor speeds using the simple propeller
  // models.
  for (int32_t i = 0; i < kNumMotors; ++i) {
    double v_app_local =
        CalcLocalAirspeed(v_app, rotor_params[i]->local_pressure_coeff,
                          &rotor_params[i]->pos, pqr);
    rotors[i] = fmax(ThrustToOmega(VecGet(&thrusts, i), v_app_local,
                                   air_density, &params->simple_models[i]),
                     params->idle_speed);
  }
}

void MixFlaps(const Deltas *deltas, const double *offsets,
              const double *lower_limits, const double *upper_limits,
              double *flaps) {
  assert(deltas != NULL && offsets != NULL && flaps != NULL);
  assert((lower_limits == NULL && upper_limits == NULL) ||
         (lower_limits != NULL && upper_limits != NULL));

  for (int32_t i = 0; i < kNumFlaps; ++i) {
    flaps[i] = offsets[i];
  }

  flaps[kFlapA1] += -deltas->aileron + deltas->outboard_flap;
  flaps[kFlapA2] += -deltas->aileron + deltas->midboard_flap;
  flaps[kFlapA4] += deltas->inboard_flap;
  flaps[kFlapA5] += deltas->inboard_flap;
  flaps[kFlapA7] += deltas->aileron + deltas->midboard_flap;
  flaps[kFlapA8] += deltas->aileron + deltas->outboard_flap;
  flaps[kFlapEle] += deltas->elevator;
  flaps[kFlapRud] += deltas->rudder;

  if (lower_limits != NULL && upper_limits != NULL) {
    for (int32_t i = 0; i < kNumFlaps; ++i) {
      flaps[i] = Saturate(flaps[i], lower_limits[i], upper_limits[i]);
    }
  }
}

void UnmixFlaps(const double *flaps, const Deltas *deltas,
                const double *offsets, double *delta_aileron,
                double *delta_rudder) {
  // The rudder is easy.
  if (delta_rudder != NULL) {
    *delta_rudder = flaps[kFlapRud] - offsets[kFlapRud];
  }

  // The ailerons are trickier. Here we will be generous and record
  // the highest delta seen across all of the ailerons.
  double delta_A1 =
      -(flaps[kFlapA1] - deltas->outboard_flap - offsets[kFlapA1]);
  double delta_A2 =
      -(flaps[kFlapA2] - deltas->midboard_flap - offsets[kFlapA2]);
  double delta_A7 = flaps[kFlapA7] - deltas->midboard_flap - offsets[kFlapA7];
  double delta_A8 = flaps[kFlapA8] - deltas->outboard_flap - offsets[kFlapA8];

  // Choose whichever has the largest absolute magnitude.
  if (delta_aileron != NULL) {
    if (fabs(delta_A1) > fabs(delta_A2) && fabs(delta_A1) > fabs(delta_A7) &&
        fabs(delta_A1) > fabs(delta_A8)) {
      *delta_aileron = delta_A1;
    } else if (fabs(delta_A2) > fabs(delta_A7) &&
               fabs(delta_A2) > fabs(delta_A8)) {
      *delta_aileron = delta_A2;
    } else if (fabs(delta_A7) > fabs(delta_A8)) {
      *delta_aileron = delta_A7;
    } else {
      *delta_aileron = delta_A8;
    }
  }
}

void ServoAnglesToFlapAngles(const double servo_angles[],
                             const ServoParams params[], double flap_angles[]) {
  flap_angles[kFlapA1] =
      servo_angles[kServoA1] * params[kServoA1].linear_servo_to_flap_ratio;
  flap_angles[kFlapA2] =
      servo_angles[kServoA2] * params[kServoA2].linear_servo_to_flap_ratio;
  flap_angles[kFlapA4] =
      servo_angles[kServoA4] * params[kServoA4].linear_servo_to_flap_ratio;
  flap_angles[kFlapA5] =
      servo_angles[kServoA5] * params[kServoA5].linear_servo_to_flap_ratio;
  flap_angles[kFlapA7] =
      servo_angles[kServoA7] * params[kServoA7].linear_servo_to_flap_ratio;
  flap_angles[kFlapA8] =
      servo_angles[kServoA8] * params[kServoA8].linear_servo_to_flap_ratio;
  flap_angles[kFlapEle] =
      0.5 *
      (servo_angles[kServoE1] * params[kServoE1].linear_servo_to_flap_ratio +
       servo_angles[kServoE2] * params[kServoE2].linear_servo_to_flap_ratio);
  flap_angles[kFlapRud] =
      0.5 *
      (servo_angles[kServoR1] * params[kServoR1].linear_servo_to_flap_ratio +
       sin(servo_angles[kServoR1]) *
           params[kServoR1].nonlinear_servo_to_flap_ratio +
       servo_angles[kServoR2] * params[kServoR2].linear_servo_to_flap_ratio +
       sin(servo_angles[kServoR2]) *
           params[kServoR2].nonlinear_servo_to_flap_ratio);
}

static double FlapAngleToServoAngle(double flap_angle,
                                    const ServoParams *params) {
  assert(params->linear_servo_to_flap_ratio == 0.0 ||
         params->nonlinear_servo_to_flap_ratio == 0.0);
  assert(params->linear_servo_to_flap_ratio != 0.0 ||
         params->nonlinear_servo_to_flap_ratio != 0.0);
  if (params->linear_servo_to_flap_ratio != 0.0) {
    return flap_angle / params->linear_servo_to_flap_ratio;
  } else {
    return Asin(flap_angle / params->nonlinear_servo_to_flap_ratio);
  }
}

void FlapAnglesToServoAngles(const double flap_angles[],
                             const ServoParams params[],
                             double servo_angles[]) {
  for (int32_t i = 0; i < kNumServos; ++i) {
    servo_angles[i] =
        FlapAngleToServoAngle(flap_angles[ServoToFlap(i)], &params[i]);
  }
}

double CalcHoverGsTargetAzimuthReel(const Vec3 *wing_pos_g,
                                    const Gs02Params *params,
                                    bool *target_valid) {
  double r_wing = Vec3XyNorm(wing_pos_g);
  double r_anchor = params->anchor_arm_length;

  *target_valid = r_wing > r_anchor;
  return Wrap(
      VecGToAzimuth(wing_pos_g) + Asin(r_anchor / fmax(r_wing, r_anchor)), 0.0,
      2.0 * PI);
}
