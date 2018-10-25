#ifndef CONTROL_SYSTEM_TYPES_H_
#define CONTROL_SYSTEM_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

//#include "avionics/network/aio_labels.h"
//#include "avionics/network/aio_node.h"
#include "mat2.h"
#include "mat3.h"
#include "util.h"
#include "vec3.h"
#include "labels.h"

typedef enum {
  kActuatorHitlLevelReal,
  kActuatorHitlLevelSimulated,
} ActuatorHitlLevel;

typedef enum {
  kBridleLabelForceSigned = -1,
  kBridlePort,
  kBridleStar,
  kNumBridles
} BridleLabel;

typedef enum {
  kFlightPlanForceSigned = -1,
  kFlightPlanDisengageEngage,
  kFlightPlanHoverAndAccel,
  kFlightPlanHoverInPlace,
  kFlightPlanLaunchPerch,
  kFlightPlanManual,
  kFlightPlanRemotePerchHover,
  kFlightPlanRemotePerchCrosswind,
  kFlightPlanStartDownwind,
  kFlightPlanTurnKey,
  kNumFlightPlans
} FlightPlan;

typedef struct {
  int32_t sim_level;  // SimulatorHitlLevel
  bool use_software_joystick;
  int32_t gs02_level;  // ActuatorHitlLevel
  double gs02_timeout_sec;
  int32_t motor_level;  // ActuatorHitlLevel
  double motor_timeout_sec;
  bool send_dyno_commands;
  int32_t servo_levels[kNumServos];  // ActuatorHitlLevel
  double servo_timeout_sec;
  int32_t tether_release_level;  // ActuatorHitlLevel
  double tether_release_timeout_sec;
} HitlConfiguration;

typedef struct { HitlConfiguration config; } HitlParams;

typedef enum {
  kPropVersionForceSigned = -1,
  kPropVersionRev1,
  kPropVersionRev1Trimmed,
  kPropVersionRev2,
  kPropVersionRev3NegativeX,
  kPropVersionRev3PositiveX,
  kPropVersionRev4NegativeX,
  kPropVersionRev4PositiveX,
  kNumPropVersions
} PropVersion;

// These values are used directly and must not be changed.
typedef enum { kNegativeX = -1, kPositiveX = 1 } RotorDirection;

typedef enum {
  kSimulatorHitlLevelNone,
  kSimulatorHitlLevelAsync,
  kSimulatorHitlLevelSync,
} SimulatorHitlLevel;

typedef enum {
  kTestSiteForceSigned = -1,
  kTestSiteAlameda,
  kTestSiteChinaLake,
  kTestSiteParkerRanch,
  kTestSiteNorway,
  kNumTestSites
} TestSite;

typedef enum {
  kWingModelForceSigned = -1,
  kWingModelYm600,
  kWingModelM600a,
  kNumWingModels
} WingModel;

typedef enum {
  kWingSerialForceSigned = -1,
  kWingSerial01,
  kWingSerial02,
  kWingSerial02Final,
  kWingSerial03Hover,
  kWingSerial03Crosswind,
  kWingSerial04Hover,
  kWingSerial04Crosswind,
  kNumWingSerials
} WingSerial;

typedef enum {
  kGroundStationModelForceSigned = -1,
  kGroundStationModelGSv1,
  kGroundStationModelGSv2,
  kGroundStationModelTopHat,
  kNumGroundStationModels
} GroundStationModel;

typedef struct {
  double g, rho, P_atm, R_dry_air, R_water_vapor;
  Vec3 g_g, mag_ned;
} PhysParams;

typedef struct {
  double A;
  double b;
  double c;
  double m;
  double m_tail;
  Mat3 I;
  Mat3 I_inv;
  Mat3 i_tail;
  Vec3 center_of_mass_pos;
  Vec3 tail_cg_pos;
  Vec3 bridle_pos[kNumBridles];
  double bridle_rad;
  double bridle_y_offset;
  Vec3 horizontal_tail_pos;
  Vec3 tail_spike_pos;
  Vec3 remote_perch_peg_pos[2];
  Vec3 proboscis_pos;
  Vec3 pylon_pos[4];
  double b_pylon;
} WingParams;

typedef struct {
  Vec3 gsg_pos_drum;
  double drum_radius;
  Vec3 drum_origin_p;
  Vec3 perched_wing_pos_p;
  double racetrack_tether_length;
  double anchor_arm_length;
  double boom_azimuth_p;
  double detwist_elevation;
} Gs02Params;

typedef struct {
  double azi_ref_offset;
  double ground_z;
  Vec3 gps_antenna_pos_g;
  double heading;
  Gs02Params gs02;
} GroundStationParams;

typedef struct {
  double length;
  double linear_density;
  double outer_diameter;
  double section_drag_coeff;
  double tensile_stiffness;
  double bending_stiffness;
  double gsg_ele_to_termination;
} TetherParams;

typedef struct {
  RotorDirection dir;
  Vec3 axis;
  Mat3 dcm_b2r;
  Vec3 pos;
  double I;
  double D;
  double local_pressure_coeff;
  PropVersion version;
} RotorParams;

typedef enum {
  kSensorHitlLevelReal,
  kSensorHitlLevelSimulated,
} SensorHitlLevel;

typedef struct {
  bool use_ground_voltage_compensation;
  double P_source;
  double R_tether;
  double R_source;
  double v_source_0;
  double C_block;
} PowerSysParams;

typedef struct {
  CalParams v_bus_cal, i_bus_cal, v_380_cal, v_batt_48_cal;
  CalParams v_release_cal, i_release_cal;
  CalParams temperature_cal;
} PowerSensorParams;

typedef struct {
  Vec3 pos;
  Mat3 dcm_b2m;
  CalParams acc_cal[3], gyro_cal[3], mag_cal[3], pressure_cal;
} ImuParams;

// Provides indices needed to access the strain reading for a given loadcell in
// an array of all LoadcellMessages, ordered via LoadcellNodeLabel.
typedef struct {
  int32_t i_msg;     // Index of the LoadcellMessage.
  int32_t i_strain;  // Index into LoadcellMessage::device::strain.
} StrainLocation;

#define NUM_LOADCELL_CHANNELS 2

typedef struct {
  CalParams cal;
  StrainLocation strain_location;
} LoadcellChannelParams;

typedef struct {
  LoadcellChannelParams channels[NUM_LOADCELL_CHANNELS];
  Mat3 dcm_loadcell2b;
  Mat2 channels_to_force_local_xy;
} LoadcellParams;

typedef struct {
  double max_pressure;
  CalParams stat_cal, alpha_cal, beta_cal, dyn_cal;
} PitotSensorParams;

typedef struct {
  Vec3 pos;
  Mat3 dcm_b2p;
  double port_angle;
  PitotSensorParams sensors[kNumPitotSensors];
  double local_pressure_coeff;
} PitotParams;

typedef struct {
  double ele_axis_z_g;
  double ele_axis_horiz_offset_g;

  CalParams azi_cal;
  CalParams ele_cal;
} GsgParams;

typedef struct {
  Vec3 pos_parent;
  Mat3 dcm_parent2ws;
  bool on_perch;
} WindSensorParams;

typedef struct {
  double linear_servo_to_flap_ratio;
  double nonlinear_servo_to_flap_ratio;
} ServoParams;

typedef struct {
  CalParams throttle, roll, pitch, yaw;  // Switches are not calibrated.
} JoystickCalParams;

typedef struct { JoystickCalParams cal; } JoystickParams;

typedef struct {
  Vec3 antenna_dir;
  Vec3 pos;
} GpsParams;

typedef struct { CalParams cal; } RotorSensorParams;

typedef struct {
  double drum_angle_to_vertical_travel;
  double pivot_axis_to_bridle_point;
  double azimuth_offset;
  double pulley_engage_drum_angle;
  double elevation_backlash;
  double elevation_nominal;
} LevelwindParams;

typedef struct {
  Vec3 perch_origin_g;
  Vec3 winch_drum_origin_p;
  Vec3 gsg_pos_wd;
  Vec3 levelwind_origin_p_0;
  double I_perch_and_drum;
  double I_drum;
  double b_perch;
  double kinetic_friction_perch;
  double b_drum;
  Vec3 perched_wing_pos_p;
  CalParams heading_cal;
} PerchParams;

typedef struct {
  double r_drum;
  double transmission_ratio;
  CalParams velocity_cmd_cal;
  CalParams position_cal;
  CalParams drum_velocity_cal;
} WinchParams;

typedef struct {
  char aio_telemetry_remote_addr[16];
  uint16_t aio_telemetry_1_remote_port;
  uint16_t aio_telemetry_2_remote_port;
  uint16_t aio_telemetry_3_remote_port;
  char flight_gear_remote_addr[16];
  uint16_t flight_gear_remote_port;
  uint16_t joystick_input_remote_port;
} UdpioParams;

typedef struct {
  UdpioParams udpio;
  uint16_t aio_port;
} CommsParams;

//typedef struct {
//  //FlightComputerLabel pitot_fc_labels[kNumPitotSensors];
//} SensorLayoutParams;

typedef struct {
  Vec3 origin_g;
  double azimuth_offset;
  Vec3 port_platform_center_local;
  Vec3 starboard_platform_center_local;
  Vec3 platform_dimensions_local;
  Vec3 port_fence_center_local;
  Vec3 starboard_fence_center_local;
  Vec3 fence_dimensions_local;
} RemotePerchParams;

typedef struct {
  TestSite test_site;
  WingModel wing_model;
  WingSerial wing_serial;
  GroundStationModel gs_model;
  HitlParams hitl;
  double ts;
  FlightPlan flight_plan;
  PhysParams phys;
  WingParams wing;
  GroundStationParams ground_station;
  TetherParams tether;
  RotorParams rotors[kNumMotors];
  ImuParams imus[kNumImus];
  LoadcellParams loadcells[kNumBridles];
  PitotParams pitot;
  GsgParams gsg;
  WindSensorParams wind_sensor;
  ServoParams servos[kNumServos];
  JoystickParams joystick;
  PowerSysParams power_sys;
  PowerSensorParams power_sensor;
  GpsParams gps[kNumWingGpsReceivers];
  RotorSensorParams rotor_sensors[kNumMotors];
  LevelwindParams levelwind;
  PerchParams perch;
  WinchParams winch;
  CommsParams comms;
  //SensorLayoutParams sensor_layout;
  RemotePerchParams remote_perch;
} SystemParams;

#ifdef __cplusplus
extern "C" {
#endif

const char *FlightPlanToString(FlightPlan flight_plan);
const char *GroundStationModelToString(GroundStationModel gs);
const char *TestSiteToString(TestSite test_site);
const char *WingSerialToString(WingSerial wing_serial);
bool IsLowAltitudeFlightPlan(FlightPlan flight_plan);

LoadcellSensorLabel BridleAndChannelToLoadcellSensorLabel(BridleLabel bridle,
                                                          int32_t channel);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_SYSTEM_TYPES_H_
