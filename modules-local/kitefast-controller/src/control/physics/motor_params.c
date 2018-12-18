#include "control/physics/motor_params.h"
#include "control/physics/motor_param_types.h"

static PowerSysSimParams motor_params = {
  .kp_voltage_err = 90.0,
  .min_ground_voltage_compensation = -700.0,
  .speed_cmd_pole = -100.53096491487338,
  .voltage_control_zero = -125.66370614359172,
  .cap_drain_conductance = 10.0,
  .voltage_control_pole = -6.283185307179586,
  .max_ground_voltage_compensation = 300.0,
  .rotor_vel_err_pole = -0.18849555921538758,
  .omega_cmd_rate_limit = 500.0,
  .ki_rotor_vel_err = 287.0,
  .motor = {
    .modulation_limit = 0.9215,
    .iq_cmd_lower_limit = -225.0,
    .switching_frequency = 15000.0,
    .phase_current_cmd_limit = 225.0,
    .Rs = 0.10490458,
    .omega_loss_coefficient_sq = 0.009943,
    .Ld = 0.0009,
    .flux_linkage = 0.14994,
    .num_pole_pairs = 15,
    .hysteresis_loss_coefficient = 1.851851852e-06,
    .omega_loss_coefficient_lin = 2.347,
    .fixed_loss_sq_coeff = 3.8e-09,
    .Lq = 0.0009,
    .omega_loss_coefficient_cubic = 8.373e-05,
    .fixed_loss_lin_coeff = 3.372e-06,
    .iq_cmd_upper_limit = 225.0,
    .specific_switching_loss = 7.53e-08,
    .rds_on = 0.003625
  },
  .min_tether_current = -323.52941176470586,
  .ground_voltage_pole = -188.49555921538757,
  .kp_stacking_speed_correction = 0.26666666666666666,
  .kp_excess_tether_current = 6.0,
  .fc_stacking_speed_correction = 3.0,
  .kp_rotor_vel_err = 48.0,
  .current_filter_cutoff_freq = 20.0
};


const PowerSysSimParams *GetMotorParams(void) { 
  return &motor_params; 
}
PowerSysSimParams *GetMotorParamsUnsafe(void) { return &motor_params; }
