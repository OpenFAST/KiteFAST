#ifndef CONTROL_ESTIMATOR_ESTIMATOR_TYPES_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_TYPES_H_

#include <stdint.h>
//#include "avionics/common/plc_messages.h"
//#include "../../common/c_math/kalman.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec3.h"
#include "common/c_math/vec2.h"
#include "control/sensor_types.h"
#include "control/system_types.h"
#include "common/c_math/vec3.h"

typedef enum {
  kEstimatorVelocitySolutionTypeGps,
  kEstimatorVelocitySolutionTypeGlas,
  kEstimatorVelocitySolutionTypeDeadReckoned
} EstimatorVelocitySolutionType;

typedef enum {
  kEstimatorPosMeasurementTypeForceSigned = -1,
  kEstimatorPosMeasurementBaro,
  kEstimatorPosMeasurementGps,
  kEstimatorPosMeasurementGlas,
  kNumEstimatorPosMeasurements
} EstimatorPosMeasType;

typedef enum {
  kEstimatorVelMeasurementTypeForceSigned = -1,
  kEstimatorVelMeasurementGps,
  kEstimatorVelMeasurementGlas,
  kNumEstimatorVelMeasurements
} EstimatorVelMeasType;

typedef enum {
  kAttitudeStateLabelForceSigned = -1,
  kAttitudeStateAttX,
  kAttitudeStateAttY,
  kAttitudeStateAttZ,
  kAttitudeStateBiasGX,
  kAttitudeStateBiasGY,
  kAttitudeStateBiasGZ,
  kNumAttitudeStates
} AttitudeStateLabel;

typedef enum {
  kAttitudeNoiseLabelForceSigned = -1,
  kAttitudeNoiseGyroX,
  kAttitudeNoiseGyroY,
  kAttitudeNoiseGyroZ,
  kAttitudeNoiseBiasGRwX,
  kAttitudeNoiseBiasGRwY,
  kAttitudeNoiseBiasGRwZ,
  kNumAttitudeNoises
} AttitudeNoiseLabel;

typedef enum {
  kPositionStateLabelForceSigned = -1,
  kPositionStateVelX,
  kPositionStateVelY,
  kPositionStateVelZ,
  kPositionStatePosX,
  kPositionStatePosY,
  kPositionStatePosZ,
  kNumPositionStates
} PositionStateLabel;

typedef enum {
  kPositionNoiseLabelForceSigned = -1,
  kPositionNoiseAccelX,
  kPositionNoiseAccelY,
  kPositionNoiseAccelZ,
  kNumPositionNoises
} PositionNoiseLabel;

typedef enum {
  kMahonyVecForceSigned = -1,
  kMahonyVecMag,
  kMahonyVecAppWind,
  kMahonyVecGravity,
  kNumMahonyVecs
} MahonyVecType;

typedef struct {
  Quat q;
  Vec3 bias;
  Vec3 ef[kNumMahonyVecs];
} MahonyState;

typedef enum {
  kApparentWindSolutionTypeForceSigned = -1,
  kApparentWindSolutionTypeInertialAndWind,
  kApparentWindSolutionTypeMixed,
  kApparentWindSolutionTypeFixedAngles,
  kApparentWindSolutionTypeLoadcell,
  kApparentWindSolutionTypePitot,
  kNumApparentWindSolutionTypes
} ApparentWindSolutionType;

typedef enum {
  kWindSolutionTypeForceSigned = -1,
  kWindSolutionTypeNone,
  kWindSolutionTypeGroundStationSensor,
  kWindSolutionTypeHardcoded,
  kWindSolutionTypePitotAndInertial,
  kNumWindSolutionTypes
} WindSolutionType;

typedef struct {
  // Last valid measurements for each GSG axis [rad].
  GsgData last_valid_gsg;
  // Last valid levelwind elevation measurement [rad].
  double last_valid_levelwind_ele;
  // Last valid perch azimuth measurement [rad].
  double last_valid_perch_azi;
} EstimatorEncodersState;

typedef struct {
  GsgData gsg;
  bool gsg_azi_valid;
  bool gsg_ele_valid;
  double levelwind_ele;
  bool levelwind_ele_valid;
  double perch_azi;
  bool perch_azi_valid;
} EncodersEstimate;

typedef struct {
  double pitot_upwash_alpha_bias;
  double pitot_upwash_alpha_scale;
  double v_low, v_high;
  double ang_est_low, ang_est_high;
  double ang_fly_low, ang_fly_high;
  double fc_v;
  double fc_alpha;
  double fc_beta;
  double fc_comp;
} EstimatorApparentWindParams;

typedef struct {
  // Apparent wind estimate with individually filtered airspeed [m/s],
  // angle-of-attach [rad] and side-slip-angle [rad].
  ApparentWindSph sph_f_z1;
  Vec3 apparent_wind_b_lpf_z1;
  Vec3 apparent_wind_b_hpf_z1;
} EstimatorApparentWindState;

typedef struct {
  Quat q_g2b_0;
  double coarse_init_kp;
  double coarse_init_kp_acc;
  double coarse_init_kp_mag;
  double sigma_attitude_0;
  double sigma_gyro_bias_0;
  double sigma_gyro_noise;
  double sigma_gyro_bias_instability;
  double gyro_bias_time_constant;
  double nominal_angle_of_attack;
  double nominal_angle_of_sideslip;
  double v_app_relative_err;
  double v_app_relative_err_min_airspeed;
  double mag_relative_err;
  double plumb_bob_relative_err;
  double plumb_bob_g_err_scale;
  double max_gyro_bias;
  double fc_acc;
  bool enable_apparent_wind_correction;
  bool enable_gps_vector_correction;
  double wing_port_to_star_sigma;
  double wing_wingtip_to_center_sigma;
  double wing_vector_sigma_ratio;
  double max_gps_position_sigma;
  double max_gps_vector_error;
  double gps_vector_relative_err;
  int32_t gps_vector_timeout_cycles;
} EstimatorAttitudeParams;

typedef struct {
  // Kalman filter innovation.
  double dz;
  // Kalman filter innovation covariance.
  double pzz;
  // Kalman filter error state correction.
  double dx_plus[kNumAttitudeStates];
} EstimatorAttitudeCorrection;

typedef struct {
  EstimatorAttitudeCorrection x;
  EstimatorAttitudeCorrection y;
  EstimatorAttitudeCorrection z;
} EstimatorAttitudeCorrection3;

typedef struct {
  EstimatorAttitudeCorrection3 gps_port_to_star;
  EstimatorAttitudeCorrection3 gps_port_to_center;
  EstimatorAttitudeCorrection3 gps_star_to_center;
  EstimatorAttitudeCorrection3 apparent_wind;
  EstimatorAttitudeCorrection3 gravity_vector;
  EstimatorAttitudeCorrection3 magnetometer;
} EstimatorAttitudeCorrections;

typedef struct {
  // Cycles [#] since last GPS vector correction.
  int32_t gps_vector_timer_cycles;
  // Gyro biases [rad/s].
  Vec3 gyro_bias;
  // Quaternion encoding the rotation from g coordinates to b coordinates.
  Quat q_g2b;
  // UD factorization of the Kalman filter covariance.  The first
  // three error states are given by the attitude error encoded as an
  // Euler vector.  The remaining three states are for the gyro
  // biases.
  double ud[(kNumAttitudeStates + 1) * kNumAttitudeStates / 2];
} EstimatorAttitudeFilterState;

typedef struct {
  // Filter on the accelerometer vector before using it as a vector measurement.
  Vec3 acc_f_z1;
  EstimatorAttitudeFilterState filter;
} EstimatorAttitudeState;

typedef struct { int32_t num_debounce; } EstimatorGroundStationParams;

typedef struct {
  // Indicates that the fixed position has been detected.
  bool position_fixed;
  // Filtered ground station GPS position [m].
  Vec3 pos_ecef;
  // Rotatation matrix from ECEF to g.
  Mat3 dcm_ecef2g;

  //int32_t mode_counts[kNumGroundStationModes];
  //GroundStationMode last_confirmed_mode;
} EstimatorGroundStationState;

typedef struct {
  int32_t joystick_num_debounce;
  double fc_throttle;
  double fc_pitch;
} EstimatorJoystickParams;

typedef struct {
  // Most recent valid data.
  JoystickData last_valid_data;
  // State of the throttle filter.
  double throttle_f_z1;
  // State of the pitch filter.
  double pitch_f_z1;
  // Counter for debouncing the joystick release.
  int32_t debounce_joystick_release_count;
  // Joystick switch position at the previous time step.
  JoystickSwitchPositionLabel switch_position_z1;
  // Counters for debounching the joystick switch position.
  int32_t switch_up_count, switch_mid_count, switch_down_count;
} EstimatorJoystickState;

typedef struct {
  double max_angle_vel;
  double fc_angle_vel;
  double damping_ratio_angle_vel;
} EstimatorPerchAziParams;

typedef struct {
  // Last perch azimuth angle [rad] recorded without any faults.
  double last_valid_perch_azi_angle;
  // State of the second order derivative filter used to estimate the
  // perch azimuth angular rate.
  double angle_vel_filter_state[2];
} EstimatorPerchAziState;

typedef struct {
  double min_relative_sigma_pos_g;
  double sigma_per_weight_over_tension_ratio;
  double max_weight_over_tension_ratio;
  double gsg_bias_fc;
  double gsg_bias_tension_lim;
  GsgData bias_low, bias_high;
} EstimatorPositionGlasParams;

typedef struct {
  int32_t outage_hold_num;
  double min_disagreement_distance_hover;
  double min_disagreement_distance;
  double disagreement_hysteresis_ratio;
  double sigma_hysteresis;
  double relative_sigma_threshold;
} EstimatorPositionGpsParams;

typedef struct {
  //BoundedKalman1dEstimatorParams kalman_est;
  double sigma_Xg_z_bias_0;
  double sigma_Xg_z;
} EstimatorPositionBaroParams;

typedef struct {
  double sigma_vel_g_0;
  double gps_sigma_multiplier;
  double min_gps_sigma_vel_g;
  double min_gps_sigma_pos_g;
  double sigma_wing_accel_hover;
  double sigma_wing_accel_dynamic;
  bool baro_enabled;
  bool glas_enabled;
  int32_t gps_position_timeout_cycles;
} EstimatorPositionFilterParams;

typedef struct {
  EstimatorPositionBaroParams baro;
  EstimatorPositionFilterParams filter;
  EstimatorPositionGlasParams glas;
  EstimatorPositionGpsParams gps;
} EstimatorPositionParams;

typedef struct {
  // Kalman filter innovation.
  double dz;
  // Kalman filter innovation covariance.
  double pzz;
  // Kalman filter error state correction.
  double dx_plus[kNumPositionStates];
} EstimatorPositionCorrection;

typedef struct {
  EstimatorPositionCorrection x;
  EstimatorPositionCorrection y;
  EstimatorPositionCorrection z;
} EstimatorPositionCorrection3;

typedef struct {
  // Use last_center_gps_receiver to determine the receiver identity of the
  // center channel.
  EstimatorPositionCorrection3 gps_center_position;
  EstimatorPositionCorrection3 gps_center_velocity;
  EstimatorPositionCorrection3 gps_port_position;
  EstimatorPositionCorrection3 gps_port_velocity;
  EstimatorPositionCorrection3 gps_star_position;
  EstimatorPositionCorrection3 gps_star_velocity;
  EstimatorPositionCorrection3 glas_position;
  EstimatorPositionCorrection baro;
} EstimatorPositionCorrections;

typedef struct {
  // Cycles [#] since last GPS position correction.
  int32_t gps_position_timer_cycles;
  // Position [m] estimate.
  Vec3 pos_g;
  // Velocity [m/s] estimate resolve in g coordinates.
  Vec3 vel_g;
  // UD factorization of the Kalman filter covariance.  The first
  // three error states are given by the velocity and the next three
  // are the position error both resolved in the g coordinate system.
  double ud[(kNumPositionStates + 1) * kNumPositionStates / 2];
} EstimatorPositionFilterState;

typedef struct {
  // Number [#] of control cycles to ignore GPS data.
  int32_t outage_timer;
  // Last time of week [ms], used to detect new data.
  int32_t time_of_week_z1;
} EstimatorPositionGpsState;

typedef struct {
  // Last valid wing g-frame z estimate [m].
  double last_valid_Xg_z;
  // Estimate of the bias [m].
  double Xg_z_bias;
  // Covariance for the bias [m^2].
  double cov_Xg_z_bias;
} EstimatorPositionBaroState;

typedef struct {
  // Last valid wing position [m] and velocity [m/s].
  Vec3 last_valid_wing_pos_g;
  // Biases [rad] for the GSG angles.
  GsgData gsg_bias;
} EstimatorPositionGlasState;

typedef struct {
  // Barometric altitude state.
  EstimatorPositionBaroState baro;
  // GLAS estimate state.
  EstimatorPositionGlasState glas;
  // Center GPS receiver chosen on the previous iteration.
  WingGpsReceiverLabel last_center_gps_receiver;
  // GPS estimate state.
  //EstimatorPositionGpsState gps[kNumWingGpsReceivers];
  // Adaptive complementary filter state.
  EstimatorPositionFilterState filter;
} EstimatorPositionState;

typedef struct {
  // If false, do not use Xg.
  bool wing_pos_valid;
  // Position estimate [m].
  Vec3 Xg;
  // Position standard devation [m].
  Vec3 sigma_Xg;
} EstimatorPositionGlasEstimate;

typedef struct {
  // If false, do not use Xg_z.
  bool valid;
  // Position estimate [m].
  double Xg_z;
  // Standard deviation [m] on the estimate based on
  // the quality of initial biasing.
  double sigma_Xg_z;
} EstimatorPositionBaroEstimate;

typedef struct {
  // If false, do not apply a measurement correction.
  bool new_data;
  // If false do not use Xg.
  bool wing_pos_valid;
  // If false, do not use Vg.
  bool wing_vel_valid;
  // Wing position estimate in g-coordinates [m].
  Vec3 Xg;
  // Standard deviations [m] for the wing position estimate.
  Vec3 sigma_Xg;
  // Wing velocity [m/s] estimate in g coordinates.
  Vec3 Vg;
  // Standard deviations [m/s] for the wing velocity in ground coordinates.
  Vec3 sigma_Vg;
} EstimatorPositionGpsEstimate;

#define ESTIMATOR_VIBRATION_FILTER_ORDER 2
#define ESTIMATOR_VB_FILTER_ORDER 2

typedef struct {
  // Label of the IMU to trust if all IMUs are currently faulted.
  ImuLabel last_used_imu;
  // Attitude estimator state.
  EstimatorAttitudeState attitude[kNumImus];
  // Position estimator state.
  EstimatorPositionState position;
  // Filtered kite velocity [m/s] for attitude determination in crosswind.
  Vec3 Vg_f_z1;
  // Velocity solution type from previous control sample.
  EstimatorVelocitySolutionType vel_type_z1;
  // States of the third-order low-pass filter on the body angular
  // rates and accelerations.
  double p_filter[ESTIMATOR_VIBRATION_FILTER_ORDER];
  double q_filter[ESTIMATOR_VIBRATION_FILTER_ORDER];
  double r_filter[ESTIMATOR_VIBRATION_FILTER_ORDER];
  double acc_b_x_filter[ESTIMATOR_VIBRATION_FILTER_ORDER];
  double acc_b_y_filter[ESTIMATOR_VIBRATION_FILTER_ORDER];
  double acc_b_z_filter[ESTIMATOR_VIBRATION_FILTER_ORDER];
  // State of the acceleration norm low-pass filter.
  Vec3 acc_norm_f_z1;
  // State for the anti-vibration velocity filter.
  double Vb_x_filter_state[ESTIMATOR_VB_FILTER_ORDER];
  double Vb_y_filter_state[ESTIMATOR_VB_FILTER_ORDER];
  double Vb_z_filter_state[ESTIMATOR_VB_FILTER_ORDER];
  // Expected magnetometer measurement [Gauss] in g-coordinates.
  Vec3 mag_g;
} EstimatorNavState;

typedef struct {
  EstimatorAttitudeParams attitude;
  EstimatorPositionParams position;
  double fc_Vg;
  double fc_acc_norm;
  double vibration_filter_a[ESTIMATOR_VIBRATION_FILTER_ORDER + 1];
  double vibration_filter_b[ESTIMATOR_VIBRATION_FILTER_ORDER + 1];
  double Vb_filter_a[ESTIMATOR_VB_FILTER_ORDER + 1];
  double Vb_filter_b[ESTIMATOR_VB_FILTER_ORDER + 1];
} EstimatorNavParams;

#undef ESTIMATOR_VIBRATION_FILTER_ORDER
#undef ESTIMATOR_VB_FILTER_ORDER

typedef struct { double last_valid_position; } EstimatorTetherElevationState;

typedef struct {
  // Most recent loadcell measurement when no fault was reported.
  double last_valid_loadcells[kNumLoadcellSensors];
  // Tension filter state [N].
  Vec3 vector_f_z1;
} EstimatorTetherForceState;

typedef struct { double fc_tension; } EstimatorTetherForceParams;

typedef struct {
  // Value of winch_pos [m] recorded from the last time flight_mode was
  // kFlightModePerched.
  double position_perched;
  // Last valid recorded winch_pos [m].
  double last_valid_position;
  // Last valid proximity sensor value.
  bool last_valid_proximity;
} EstimatorWinchState;

typedef struct {
  Vec3 hard_coded_wind_g;
  double fc_initialize;
  double fc_vector;
  double fc_vector_slow;
  double fc_speed;
  double fc_speed_playbook;
  double fc_dir;
} EstimatorWindParams;

typedef struct {
  // Last wind vector [m/s] in the ground coordinates for which there
  // were no sensor faults.
  Vec3 last_valid_wind_g;
  // State of the wind low-pass filter [m/s] used to generate vector_f.
  Vec3 vector_f_z1;
  // State of the wind low-pass filter [m/s] used to generate vector_f_slow.
  Vec3 vector_f_slow_z1;
  // State of the speed low-pass filter [m/s].
  double speed_f_z1;
  double speed_f_pb_z1;
  // State of the wind low-pass filter [m/s] used to generate dir_f.
  Vec3 wind_direction_vector_f_z1;
} EstimatorWindState;

typedef struct {
  double t_initialize;
  double hard_coded_initial_payout;
  EstimatorApparentWindParams apparent_wind;
  //EstimatorGroundStationParams ground_station;
  //EstimatorJoystickParams joystick;
  //EstimatorNavParams nav;
  ////EstimatorPerchAziParams perch_azi;
  //EstimatorTetherForceParams tether_force;
  EstimatorWindParams wind;
} EstimatorParams;

typedef struct {
  // Indicates the sensors used in solving for apparent wind
  // when flying-like-an-airplane.
  int32_t solution_type;  // See ApparentWindSolutionType.
  // Apparent wind estimate in Cartesian coordinates.
  Vec3 vector;
  // Apparent wind estimate in spherical coordinates.
  ApparentWindSph sph;
  // Apparent wind estimate in spherical coordinates with individually
  // low-pass filtered components.
  ApparentWindSph sph_f;
} ApparentWindEstimate;

typedef struct {
  // Only use the ground station pose estimate if this flag is true.
  bool valid;
  // Position [m] of the ground-station origin in ECEF.
  Vec3 pos_ecef;
  // Rotation matrix from ECEF to g.
  Mat3 dcm_ecef2g;
} GroundStationPoseEstimate;

typedef struct {
  GroundStationPoseEstimate pose;
  //GroundStationMode mode;
} GroundStationEstimate;

typedef struct {
  // Whether to trust this estimate.
  bool valid;
  // Most recent valid joystick data.
  //JoystickData data;
  // Low-pass filtered throttle.
  double throttle_f;
  // Low-pass filtered pitch stick.
  double pitch_f;
} JoystickEstimate;

typedef struct {
  // Whether to trust this estimate.
  bool valid;
  // Most recent valid perch azimuth angle [rad].
  double angle;
  // Estimate of the perch azimuth angular rate [rad/s].
  double angle_vel_f;
} PerchAziEstimate;

typedef struct {
  // Whether to trust this estimate.
  bool valid;
  // Tether force estimate in Cartesian coordinates [N].
  Vec3 vector;
  // Tether force estimate in spherical coordinates.
  TetherForceSph sph;
  // Low-pass filtered tension [N].
  double tension_f;
  Vec3 vector_f;
  // Port bridle force estimate in Cartesian coordinates [N].
  Vec3 bridle_port_vector;
  // Starboard bridle force estimate in Cartesian coordinates [N].
  Vec3 bridle_star_vector;
} TetherForceEstimate;

typedef struct {
  bool valid;
  // Negative one times the nominal tether length [m] that must be
  // payed out to reach the crosswind orientation (defined to be zero
  // in the crosswind orientation).
  //
  // This value is negative when reeled-in, increases during pay-out
  // and must be non-negative before the ground station experiences
  // crosswind loads.
  double position;
  // Nominal tether length [m] that has been payed-out since last
  // being on the perch (defined to be zero when perched).
  double payout;
  // Whether or not the proximity flag is valid.
  // TODO(tobenkin): Use payout as a backup proximity sensor.
  bool proximity_valid;
  // Whether or not the proximity sensor is active.
  bool proximity;
} WinchEstimate;

typedef struct {
  bool valid;
  double position;
} TetherElevationEstimate;

typedef struct {
  // Whether to trust this estimate.
  bool valid;
  // Where this estimate came from.
  int32_t solution_type;  // See WindSolutionType.
  // Wind speed estimate [m/s].
  Vec3 vector;
  // Low-pass filtered wind speed estimate [m/s].
  Vec3 vector_f;
  // More heavily low-pass filtered wind speed estimate [m/s].
  Vec3 vector_f_slow;
  // Low-pass filtered angle [rad] indicating the direction the wind
  // is coming from (i.e. defined to be 0 when the wind vector is
  // aligned with the negative x-axis, and increasing according to the
  // right-hand rule around the down axis).  This value is filtered in
  // a wind speed dependent way so as to vary more slowly in
  // low-winds.
  double dir_f;
  // Filtered wind speed estimate [m/s].
  double speed_f;
  double speed_f_playbook;
} WindEstimate;

// Structure of the internal state of the estimator.
typedef struct {
  bool tether_release_latched;
  uint8_t experimental_crosswind_config;
  double time;
  EstimatorApparentWindState apparent_wind;
  EstimatorEncodersState encoders;
  EstimatorGroundStationState ground_station;
  EstimatorJoystickState joystick;
  EstimatorNavState nav;
  EstimatorPerchAziState perch_azi;
  EstimatorTetherElevationState tether_elevation;
  EstimatorTetherForceState tether_force;
  EstimatorWinchState winch;
  EstimatorWindState wind;
  EstimatorWindState wind_aloft;
  double rho_z1;
} EstimatorState;

typedef struct {
  int32_t stacking_state;  // See StackingState.

  // Attitude and angular rate estimates.
  Mat3 dcm_g2b;
  Vec3 pqr;

  // Filtered values.
  double acc_norm_f;
  Vec3 pqr_f;

  // Position, velocity, and acceleration estimates.
  Vec3 Xg, Vg, Vg_f, Vb, Vb_f, Ag, Ab_f;
  bool gps_active;

  // Whether the tether has been released. This field latches and is only set to
  // true when an unfaulted loadcell indicates that release has occurred.
  bool tether_released;

  // Air density at weather station (on ground station) [kg/m^3].
  // This value is not used by the controller, but is provided for
  // convenience of flight data analysis.
  // TODO(tfricke): It would be more useful to estimate the density at
  // the wing; however, the wing currently does not have a good
  // outside air temperature sensor. Another approach would be to
  // apply an atmospheric model (i.e. adiabatic lapse rate) to
  // extrapolate the density to the wing altitude. Care should be
  // taken to not introduce unintentional feedback from position to
  // density to motor thrust.
  double rho;

  // Apparent wind estimate resolved in body coordinates.
  ApparentWindEstimate apparent_wind;

  // Joystick estimate.
  //JoystickEstimate joystick;

  // Perch azimuth estimate.
  //PerchAziEstimate perch_azi;

  // Tether tension information resolved in body coordinates.
  TetherForceEstimate tether_force_b;

  // Winch position / payout estimate.
  //WinchEstimate winch;

  // Tether elevation.
  //TetherElevationEstimate tether_elevation;

  // Wind estimates resolved in ground coordinates.
  WindEstimate wind_g;

  // Estimate of the wind at the kite.
  WindEstimate wind_aloft_g;

  // Experimental crosswind config.
  uint8_t experimental_crosswind_config;

  // Ground station mode.
  //GroundStationMode gs_mode;
} StateEstimate;

#ifdef __cplusplus
extern "C" {
#endif

const char *ApparentWindSolutionTypeToString(ApparentWindSolutionType type);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_TYPES_H_
