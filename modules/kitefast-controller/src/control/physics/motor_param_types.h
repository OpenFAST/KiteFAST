#ifndef MOTOR_PARAM_TYPES_H_
#define MOTOR_PARAM_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "system/labels.h"

typedef enum {
  kSimMotorLimitNone,
  kSimMotorLimitGroundPower,
  kSimMotorLimitPhaseCurrent,
  kSimMotorLimitPower
} SimMotorLimit;


typedef struct {
  double modulation_limit;
  double phase_current_cmd_limit;
  double iq_cmd_lower_limit;
  double iq_cmd_upper_limit;
  double Lq;
  double Ld;
  double Rs;
  double flux_linkage;
  int32_t num_pole_pairs;
  double hysteresis_loss_coefficient;
  double omega_loss_coefficient_cubic;
  double omega_loss_coefficient_sq;
  double omega_loss_coefficient_lin;
  double rds_on;
  double specific_switching_loss;
  double fixed_loss_sq_coeff;
  double fixed_loss_lin_coeff;
  double switching_frequency;
} MotorParams;

typedef struct {
  double current_filter_cutoff_freq;
  double ground_voltage_pole;
  double min_ground_voltage_compensation;
  double max_ground_voltage_compensation;
  double kp_rotor_vel_err;
  double ki_rotor_vel_err;
  double rotor_vel_err_pole;
  double kp_voltage_err;
  double voltage_control_pole;
  double voltage_control_zero;
  double fc_stacking_speed_correction;
  double kp_stacking_speed_correction;
  double cap_drain_conductance;
  MotorParams motor;
  double omega_cmd_rate_limit;
  double speed_cmd_pole;
  double min_tether_current;
  double kp_excess_tether_current;
} PowerSysSimParams;

#endif  // SIM_SIM_TYPES_H_
