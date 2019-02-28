#include "control/physics/motors.h"

#include <assert.h>
#include <float.h>
#include <math.h>

#include "common/c_math/util.h"

void SetMotorDirection(double rotor_omegas[], double rotor_accel[], double rotor_torques[]){
// Kitefast Motor Order -> Kitefast sign convention
// 
// [0] starboard-inboard-top      (-)
// [1] starboard-inboard-bottom   (+)
// [2] starboard-outboard-top     (+)
// [3] starboard-outboard-bottom  (-)
// [4] port-inboard-top           (-)
// [5] port-inboard-bottom        (+)
// [6] port-outboard-top          (+)
// [7] port-outboard-bottom       (-)

// Controller Motor Order -> Makani Sign Convention
// 
// Table:  location, #, name and rotational direction
// (Pos or Neg) of each rotor, from the position of standing in front of and facing the kite.
// 
// 8. STo  | 7. STi|          | 6. PTi  | 5. PTo                       
// Pos     | Neg   |          | Neg     | Pos
// ----------------------------------------------
// Starboard Wing  | Fuselage | Port Wing
// ----------------------------------------------
// 1. SBo  | 2. SBi|          | 3. PBi  | 4. PBo
// Neg     | Pos   |          | Pos     | Neg
// 
// The propellers follow
// the sign convention where positive means
// that the propeller is rotating in a positive direction (i.e. right
// hand rule) about the propeller axis, which is predominately in the
// same direction as the body x-axis.
// 

  double motor_dir[] = {
    -1,   // Motor 1
    1,    // Motor 2 
    1,    // Motor 3
    -1,   // Motor 4
    1,    // Motor 5
    -1,   // Motor 6
    -1,   // Motor 7
    1,    // Motor 8
   };

  // evaluate signs
  for (int i=0; i<kNumMotors; i++){
    // Rotor_speeds
    if (rotor_omegas[i] > 0 && motor_dir[i] > 0){
      // Do nothing sign is correct direction
    }
    else if (rotor_omegas[i] > 0 && motor_dir[i] < 0){
      rotor_omegas[i] = rotor_omegas[i] * motor_dir[i];
    }
    else if (rotor_omegas[i] < 0 && motor_dir[i] > 0){
      rotor_omegas[i] = rotor_omegas[i] * -motor_dir[i];
    }
    else if (rotor_omegas[i] < 0 && motor_dir[i] < 0){
      // Do nothing sign is correct direction
    }

    // Rotor_accel
    if (rotor_accel[i] > 0 && motor_dir[i] > 0){
      // Do nothing sign is correct direction
    }
    else if (rotor_accel[i] > 0 && motor_dir[i] < 0){
      rotor_accel[i] = rotor_accel[i] * motor_dir[i];
    }
    else if (rotor_accel[i] < 0 && motor_dir[i] > 0){
      rotor_accel[i] = rotor_accel[i] * -motor_dir[i];
    }
    else if (rotor_accel[i] < 0 && motor_dir[i] < 0){
      // Do nothing sign is correct direction
    }

    // Rotor_Torque
    if (rotor_torques[i] > 0 && motor_dir[i] > 0){
      // Do nothing sign is correct direction
    }
    else if (rotor_torques[i] > 0 && motor_dir[i] < 0){
      rotor_torques[i] = rotor_torques[i] * motor_dir[i];
    }
    else if (rotor_torques[i] < 0 && motor_dir[i] > 0){
      rotor_torques[i] = rotor_torques[i] * -motor_dir[i];
    }
    else if (rotor_torques[i] < 0 && motor_dir[i] < 0){
      // Do nothing sign is correct direction
    }
  }
}

TorqueLimits CalcTorqueLimits(double voltage, double rotor_vel,
                              const MotorParams *params) {
  assert(voltage > 0.0);

  // For now, the following code does not support saliency.
  assert(fabs(params->Ld - params->Lq) < DBL_EPSILON);

  TorqueLimits limits;

  // Stash values into shorter variable names.
  //
  // Yasa motors do have some saliency but it is believed to be relatively small
  // and is neglected here to simplify calculations. The q-axis inductance is
  // chosen as the motor inductance L because it has a much more substantial
  // impact on the performance when not heavily flux weakening.
  double L = params->Lq;
  double Rs = params->Rs;
  double lambda = params->flux_linkage;
  double i_phase_lim = params->phase_current_cmd_limit;
  int32_t npp = params->num_pole_pairs;
  double omega_e = rotor_vel * npp;

  double vdq_max = voltage * (1.0 / sqrt(3.0)) * params->modulation_limit;
  double z2 = Rs * Rs + L * L * omega_e * omega_e;

  // Initialize with the hard quadrature current command limits.
  double iq_cmd_lower_limit = params->iq_cmd_lower_limit;
  double iq_cmd_upper_limit = params->iq_cmd_upper_limit;
  limits.lower_constraint = kSimMotorLimitPhaseCurrent;
  limits.upper_constraint = kSimMotorLimitPhaseCurrent;

  // Calculate the power limit assuming a non-salient machine.
  double id_center = -omega_e * omega_e * L * lambda / z2;
  double iq_center = -Rs * omega_e * lambda / z2;
  double iq_radius = vdq_max / sqrt(z2);
  if (iq_cmd_lower_limit < iq_center - iq_radius) {
    limits.lower_constraint = kSimMotorLimitPower;
    iq_cmd_lower_limit = iq_center - iq_radius;
  }
  if (iq_cmd_upper_limit > iq_center + iq_radius) {
    limits.upper_constraint = kSimMotorLimitPower;
    iq_cmd_upper_limit = iq_center + iq_radius;
  }

  // Calculate the phase current limit assuming a non-salient machine.
  double cos_idq =
      (vdq_max * vdq_max - z2 * i_phase_lim * i_phase_lim -
       lambda * lambda * omega_e * omega_e) /
      (2.0 * fmax(fabs(omega_e), 1.0) * lambda * i_phase_lim * sqrt(z2));
  cos_idq = Saturate(cos_idq, -1.0, 1.0);
  double theta_delta = acos(cos_idq);
  double theta_ref =
      fabs(omega_e) > DBL_EPSILON ? atan(Rs / (omega_e * L)) : 0.0;

  // Apply lower phase current limit.
  double theta = fmin(theta_ref - theta_delta, -0.5 * PI);
  if (id_center < i_phase_lim * cos(theta) &&
      i_phase_lim * sin(theta) > iq_cmd_lower_limit) {
    limits.lower_constraint = kSimMotorLimitPhaseCurrent;
    iq_cmd_lower_limit = i_phase_lim * sin(theta);
  }

  // Apply upper phase current limit.
  theta = fmax(theta_ref + theta_delta, 0.5 * PI);
  if (id_center < i_phase_lim * cos(theta) &&
      i_phase_lim * sin(theta) < iq_cmd_upper_limit) {
    limits.upper_constraint = kSimMotorLimitPhaseCurrent;
    iq_cmd_upper_limit = i_phase_lim * sin(theta);
  }

  limits.lower_limit = 1.5 * npp * lambda * iq_cmd_lower_limit;
  limits.upper_limit = 1.5 * npp * lambda * iq_cmd_upper_limit;

  return limits;
}

// Direct calculation of motor and electric drive losses to determine motor
// electrical power consumption or generation.  This is alternative to method
// previously provided in powertrain_database which relies on an efficiency
// lookup table.
// TODO(gdolan): Ignores torque generated by hysteresis and eddy current
// losses.  Can improve in future to return the actual torque produced.
// Sign convention is positive power for generation.
double CalcMotorPower(double voltage, double torque, double rotor_vel,
                      const MotorParams *params) {
  assert(voltage > 0.0);

  // For now, the following code does not support saliency.
  assert(fabs(params->Ld - params->Lq) < DBL_EPSILON);


  // Stash values into shorter variable names.
  //
  // Yasa motors do have some saliency but it is believed to be relatively small
  // and is neglected here to simplify calculations. The q-axis inductance is
  // chosen as the motor inductance L because it has a much more substantial
  // impact on the performance when not heavily flux weakening.
  double L = params->Lq;
  double Rs = params->Rs;
  double lambda = params->flux_linkage;
  int32_t npp = params->num_pole_pairs;
  double omega_e = rotor_vel * npp;

  double vdq_max = voltage * (1.0 / sqrt(3.0)) * params->modulation_limit;
  double z2 = Rs * Rs + L * L * omega_e * omega_e;

  // This is a simplification.  Ignores saliency and magnetic loss torque.
  double iq = torque / (1.5 * npp * lambda);

  // Now assume we follow path minimizing phase current for given torque.
  // Follow id = 0 line, transitioning to impedance limited behavior.  For
  // simplicity, use nearest point to calculate loss if for some reason the
  // impedance limit is exceeded.  Use id = short_circuit_current.  Limits are
  // assumed to be correctly applied and mild violations are okay.
  double id_center = -omega_e * omega_e * L * lambda / z2;
  double iq_center = -Rs * omega_e * lambda / z2;
  double iq_radius = vdq_max / sqrt(z2);

  // Assume phase current is all q current to start
  double peak_phase_current_sq = iq * iq;

  double iq_height = iq - iq_center;
  // If point is unreachable, assume max id and proceed
  if (fabs(iq_height) > iq_radius) {
    peak_phase_current_sq = peak_phase_current_sq + id_center * id_center;
  } else {
    double id = id_center + sqrt(iq_radius * iq_radius - iq_height * iq_height);
    if (id < 0) {
      peak_phase_current_sq = peak_phase_current_sq + id * id;
    }
  }

  // Power [W] convention has positive power for generation.
  double mechanical_power = -torque * rotor_vel;

  // Loss [W} for 3 phases.  Divide by two to convert peak to rms.
  double resistive_loss = -3 / 2 * peak_phase_current_sq * Rs;

  // Speed Loss [W] based on 2nd order polynomial fit.
  const double p_w_loss[3] = {params->omega_loss_coefficient_cubic,
                              params->omega_loss_coefficient_sq,
                              params->omega_loss_coefficient_lin};
  double speed_loss = -(p_w_loss[0] * rotor_vel * rotor_vel +
                        p_w_loss[1] * rotor_vel + p_w_loss[2]) *
                      rotor_vel;

  // Hysteresis and eddy current losses [W].
  // 0.5 factor to account for peak to rms conversion.
  double hysteresis_loss = -0.5 * params->hysteresis_loss_coefficient *
                           peak_phase_current_sq * rotor_vel * rotor_vel;

  // Calculate motor controller loss
  double controller_loss =
      CalcMotorControllerLoss(voltage, peak_phase_current_sq, params);

  return mechanical_power + resistive_loss + speed_loss + hysteresis_loss +
         controller_loss;
}

// Model for motor controller loss.  Based on model at:
// docs/spreadsheets/d/1fbCH_8zUq6EYXMfOGzC6kEj6e-5wtaob1VNaavI7xlU
// Sign convention is negative for loss.
double CalcMotorControllerLoss(double voltage, double peak_phase_current_sq,
                               const MotorParams *params) {
  // Conduction assumes 3 phases and synchronous switching guarantees one
  // leg of each half bridge is always conducting.
  double conduction_loss = -3 / 2 * peak_phase_current_sq * params->rds_on;

  // Switching turn on and turn off losses are broken into 2 parts:
  // TODO(gdolan): Does not take into account ripple current at switching
  // frequency.

  // - Loss associated with commutating current.  These are specified as being
  //   proportional to the bus voltage times the average phase current.
  double variable_switching_loss_per_cycle =
      -(3.0 * 2.0 / PI * voltage * sqrt(peak_phase_current_sq) *
        params->specific_switching_loss);

  // - Loss associated with the output capacitance.  This is tricky since the
  //   output capacitance decreases with increasing voltage though a linear
  //   model should give us sufficient accuracy over the operating range.
  double fixed_switching_loss_per_cycle =
      -3.0 *
      (params->fixed_loss_sq_coeff * voltage + params->fixed_loss_lin_coeff) *
      voltage;

  // Sum everything up
  return conduction_loss +
         params->switching_frequency * (variable_switching_loss_per_cycle +
                                       fixed_switching_loss_per_cycle);
}
