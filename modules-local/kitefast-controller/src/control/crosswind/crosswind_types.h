#ifndef CONTROL_CROSSWIND_CROSSWIND_TYPES_H_
#define CONTROL_CROSSWIND_CROSSWIND_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "filter.h"
#include "vec3.h"
#include "actuator_types.h"
#include "system_types.h"
#include "labels.h"

typedef enum {
  kCrosswindNormalGateForceSigned = -1,
  kCrosswindNormalGateSpeed,
  kCrosswindNormalGateTension,
  kCrosswindNormalGateAltitude,
  kCrosswindNormalGateAirspeed,
  kCrosswindNormalGateFlightMode,
  kNumCrosswindNormalGates
} CrosswindNormalGate;

typedef enum {
  kCrosswindPrepTransOutGateForceSigned = -1,
  kCrosswindPrepTransOutGateWindSpeed,
  kNumCrosswindPrepTransOutGates
} CrosswindPrepTransOutGate;

// TODO(kennyjensen): This gate belongs with the hover controller, but
// since it relies on loop angle and whether the path has arrived we
// currently put it in the crosswind controller.
typedef enum {
  kCrosswindHoverTransOutGateForceSigned = -1,
  kCrosswindHoverTransOutGateAirspeed,
  kCrosswindHoverTransOutGateAlpha,
  kCrosswindHoverTransOutGatePathType,
  kCrosswindHoverTransOutGateStillAccelerating,
  kNumCrosswindHoverTransOutGates
} CrosswindHoverTransOutGate;

typedef enum {
  kCrosswindInnerMinAirspeed,
  kCrosswindInnerNominalAirspeed,
  kCrosswindInnerMaxAirspeed,
  kCrosswindInnerNumAirspeeds,
} CrosswindInnerAirspeeds;

typedef enum {
  kCrosswindLongitudinalInputElevator,
  kCrosswindLongitudinalInputMotorPitch,
  kNumCrosswindLongitudinalInputs
} CrosswindLongitudinalInputs;

typedef enum {
  kCrosswindLongitudinalStatePositionGroundZ,
  kCrosswindLongitudinalStateVelocityGroundZ,
  kCrosswindLongitudinalStateAngleOfAttack,
  kCrosswindLongitudinalStatePitchRate,
  kCrosswindLongitudinalStateIntegratedAngleOfAttack,
  kNumCrosswindLongitudinalStates
} CrosswindLongitudinalStates;

typedef enum {
  kCrosswindLateralInputAileron,
  kCrosswindLateralInputRudder,
  kCrosswindLateralInputMotorYaw,
  kNumCrosswindLateralInputs
} CrosswindLateralInputs;

typedef enum {
  kCrosswindLateralStateTetherRoll,
  kCrosswindLateralStateSideslip,
  kCrosswindLateralStateRollRate,
  kCrosswindLateralStateYawRate,
  kCrosswindLateralStateIntegratedTetherRoll,
  kCrosswindLateralStateIntegratedSideslip,
  kNumCrosswindLateralStates
} CrosswindLateralStates;

typedef enum { kLoopDirectionCw = -1, kLoopDirectionCcw = 1 } LoopDirection;

typedef enum {
  kCrosswindPathNormal,
  kCrosswindPathPrepareTransitionOut
} CrosswindPathType;

#define CROSSWIND_SCHEDULE_TABLE_LENGTH 50
typedef struct {
  double wind_speed;
  double alpha_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double beta_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double airspeed_lookup[CROSSWIND_SCHEDULE_TABLE_LENGTH];
  double path_radius_target;
  double elevation;
  double azi_offset;
  double lookup_loop_angle[CROSSWIND_SCHEDULE_TABLE_LENGTH];
} PlaybookEntry;

#define NUM_PLAYBOOK_ENTRIES 15
typedef struct {
  int num_entries;
  PlaybookEntry entries[NUM_PLAYBOOK_ENTRIES];
} Playbook;

typedef struct { bool enable_alternate_gains; } ExperimentalCrosswindConfig;

#define NUM_EXPERIMENTAL_CROSSWIND_CONFIGS 64
typedef struct {
  double transition_smooth_time;
  double ele_tuner_safe;
  double min_airspeed;
  double max_airspeed;
  double ele_min;
  double ele_safe;
  double ele_max;
  double ele_rate_lim;
  double azi_rate_lim;
  double azi_rate_lim_high_wind;
  double azi_min;
  double azi_max;
  double loop_angle_path_switch_max;
  double loop_angle_path_switch_min;
  double transout_airspeed_cmd_high;
  double transout_airspeed_cmd_low;
  double transout_airspeed_crossfade_angle_end;
  double transout_elevation_cmd;
  double experimental_crosswind_airspeed_rate;
} CrosswindPowerParams;

typedef struct {
  bool mode_active;
  ExperimentalCrosswindConfig current_config;
} ExperimentalCrosswindState;

typedef struct {
  double path_azimuth;
  double path_elevation;
  CrosswindPathType path_type;
  double dloop_angle;
} CrosswindPowerState;

#define CROSSWIND_PATH_CURVATURE_TABLE_LENGTH 9
typedef struct {
  LoopDirection loop_dir;
  double k_tab[CROSSWIND_PATH_CURVATURE_TABLE_LENGTH];
  double commanded_curvature_time;
  double current_curvature_time;
  double time_horizon_speed_limit;
  double min_turning_radius;
  double fc_k_geom_curr;
  double fc_speed;
  double fc_k_aero_cmd;
  double fc_k_geom_cmd;
  PidParams crosstrack_pid;
  double experimental_crosswind_max_radius_error;
  double max_k_aero_error;
  double path_radius_rate;
} CrosswindPathParams;

typedef struct {
  double k_geom_curr_f_z1;
  double speed_f_z1;
  double k_geom_cmd_f_z1;
  double k_aero_cmd_f_z1;
  double current_curvature_time;
  double int_crosstrack;
  double path_radius_target;
} CrosswindPathState;

typedef struct {
  double alpha_min;
  double alpha_min_airspeed;
  double dalpha_dairspeed;
  double alpha_cmd_rate_min;
  double alpha_cmd_rate_max;
  double beta_cmd_rate_min;
  double beta_cmd_rate_max;
  double tether_roll_max_excursion_low;
  double tether_roll_max_excursion_high;
  double tether_roll_nom;
  double tether_roll_tension_low;
  double tether_roll_tension_high;
  double tether_roll_large_excursion;
  double tether_roll_ff_amplitude;
  double tether_roll_ff_phase;
  double alpha_cmd_min;
  double alpha_cmd_max;
  double alpha_cmd_max_flare;
  double dCL_cmd_max;
  double beta_cmd_min;
  double beta_cmd_max;
  double fc_tension;
  double kp_tension_hf;
  double transout_tether_tension_cmd;
  double transout_tether_roll_cmd;
  double transout_tether_roll_cmd_rate_limit;
  double preptransout_alpha_cmd;
  double preptransout_alpha_rate;
  double transout_flare_alpha_cmd;
  double transout_flare_alpha_rate;
  double transout_flare_beta_cmd;
  double transout_flare_beta_rate;
  double transout_flare_airspeed;
} CrosswindCurvatureParams;

typedef struct {
  double tension_hpf_z1;
  double tension_z1;
  double alpha_cmd_z1;
  double beta_cmd_z1;
  double transout_flare_time;
} CrosswindCurvatureState;

typedef struct {
  double elevator_flap_ratio;
  double delevator_dalpha;
  double kp_flap;
  Mat3 B_flaps_to_pqr;
  double airspeed_table[kCrosswindInnerNumAirspeeds];
  double longitudinal_gains_min_airspeed[kNumCrosswindLongitudinalInputs]
                                        [kNumCrosswindLongitudinalStates];
  double longitudinal_gains_nominal_airspeed[kNumCrosswindLongitudinalInputs]
                                            [kNumCrosswindLongitudinalStates];
  double longitudinal_gains_max_airspeed[kNumCrosswindLongitudinalInputs]
                                        [kNumCrosswindLongitudinalStates];
  double int_alpha_min;
  double int_alpha_max;
  double lateral_gains_min_airspeed[kNumCrosswindLateralInputs]
                                   [kNumCrosswindLateralStates];
  double lateral_gains_nominal_airspeed[kNumCrosswindLateralInputs]
                                       [kNumCrosswindLateralStates];
  double lateral_gains_max_airspeed[kNumCrosswindLateralInputs]
                                   [kNumCrosswindLateralStates];
  double lateral_gains_alt_min_airspeed[kNumCrosswindLateralInputs]
                                       [kNumCrosswindLateralStates];
  double lateral_gains_alt_nominal_airspeed[kNumCrosswindLateralInputs]
                                           [kNumCrosswindLateralStates];
  double lateral_gains_alt_max_airspeed[kNumCrosswindLateralInputs]
                                       [kNumCrosswindLateralStates];
  double int_tether_roll_min;
  double int_tether_roll_max;
  double int_beta_min;
  double int_beta_max;
  PidParams airspeed_pid;
  double max_airspeed_error;
  double max_airspeed_control_power_gen;
  double max_airspeed_control_power_motor;
  double max_airspeed_control_thrust_rate;
  double initial_thrust;
  double airspeed_error_spoiler_on;
  double airspeed_error_spoiler_off;
  double delta_spoiler_on_rate;
  double delta_spoiler_off_rate;
  double delta_spoiler;
  double alt_gain_fade_rate;
  double beta_harmonic_gain;
  double beta_harmonic_integrator_max;
  bool enable_acceleration_ff;
} CrosswindInnerParams;

typedef struct {
  double int_tether_roll_error;
  double int_alpha_error;
  double int_beta_error;
  double int_thrust;
  bool spoiler_on;
  double delta_spoiler_z1;
  double alt_lateral_gains_0;
  double thrust_cmd_z1;
  double loop_angle_z1;
  double beta_harmonic_state[2];
} CrosswindInnerState;

#define CROSSWIND_RUDDER_LIMIT_BETAS 5
#define CROSSWIND_RUDDER_LIMIT_AIRSPEEDS 6
typedef struct {
  double rudder_limit_betas[CROSSWIND_RUDDER_LIMIT_BETAS];
  double rudder_limit_airspeeds[CROSSWIND_RUDDER_LIMIT_AIRSPEEDS];
  double rudder_limits_lower[CROSSWIND_RUDDER_LIMIT_AIRSPEEDS]
                            [CROSSWIND_RUDDER_LIMIT_BETAS];
  double rudder_limits_upper[CROSSWIND_RUDDER_LIMIT_AIRSPEEDS]
                            [CROSSWIND_RUDDER_LIMIT_BETAS];
  ThrustMoment thrust_moment_weights;
  double flap_offsets[kNumFlaps];
  double lower_flap_limits[kNumFlaps];
  double upper_flap_limits[kNumFlaps];
  double lower_flap_limits_flare[kNumFlaps];
  double release_wait_period;
  double release_aileron_cmd;
} CrosswindOutputParams;

typedef struct {
  double detwist_loop_angle;
  int32_t detwist_rev_count;
  double prerelease_timer;
  bool prerelease_flag;
} CrosswindOutputState;

typedef struct {
  bool reset_int;
  bool loadcell_fault;
  bool alpha_beta_fault;
  bool extreme_wind;
  bool spoiler_enabled;
  bool alternate_gains;
} CrosswindFlags;

#define CROSSWIND_TRANS_OUT_SPEED_TABLE_LENGTH 6
typedef struct {
  double min_wing_speed;
  double min_tension;
  double max_wing_pos_g_z;
  double high_wind_speed;
  double loop_angle_table[CROSSWIND_TRANS_OUT_SPEED_TABLE_LENGTH];
  double max_trans_out_speed_table[CROSSWIND_TRANS_OUT_SPEED_TABLE_LENGTH];
  double min_time_in_accel;
  double max_time_in_accel;
  double acc_slow_down_threshold;
  double min_airspeed_return_to_crosswind;
  double transout_max_time_in_flare;
  double transout_airspeed;
  double transout_alpha;
} CrosswindModeParams;

typedef struct {
  PlaybookEntry entry;
  double crossfade_rate;
  double crossfade_throttle;
} PlaybookFallbackParams;

typedef struct {
  LoopDirection loop_dir;
  CrosswindPowerParams power;
  CrosswindPathParams path;
  CrosswindCurvatureParams curvature;
  CrosswindInnerParams inner;
  CrosswindOutputParams output;
  CrosswindModeParams mode;
  ExperimentalCrosswindConfig
      experimental_crosswind_config[NUM_EXPERIMENTAL_CROSSWIND_CONFIGS];
  Playbook playbook;
  PlaybookFallbackParams playbook_fallback;
  bool enable_new_pqr_cmd;
} CrosswindParams;

typedef struct {
  CrosswindPowerState power;
  CrosswindPathState path;
  CrosswindCurvatureState curvature;
  CrosswindInnerState inner;
  CrosswindOutputState output;
  ExperimentalCrosswindState experimental_crosswind;
  double tether_roll_cmd_zs[2];
  double playbook_fallback_crossfade;
} CrosswindState;

#ifdef __cplusplus
extern "C" {
#endif

const char *CrosswindNormalGateToString(CrosswindNormalGate gate);
const char *CrosswindPrepTransOutGateToString(CrosswindPrepTransOutGate gate);
const char *CrosswindHoverTransOutGateToString(CrosswindHoverTransOutGate gate);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_TYPES_H_
