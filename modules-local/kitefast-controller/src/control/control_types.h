#ifndef CONTROL_CONTROL_TYPES_H_
#define CONTROL_CONTROL_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/vec3.h"
//#include "vec3.h"
//#include "vec3.h"
//#include "vec3.h"
#include "control/actuator_types.h"
//#include "control/avionics/avionics_interface_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator_types.h"
//#include "control/fault_detection/fault_detection_types.h"
//#include "control/hover/hover_types.h"
//#include "control/manual/manual_types.h"
#include "control/sensor_types.h"
#include "control/simple_aero_types.h"
#include "control/system_types.h"
//#include "control/trans_in/trans_in_types.h"
#include "system/labels.h"

typedef enum {
  kControllerForceSigned = -1,
  kControllerNone,
  kControllerCrosswind,
  kControllerHover,
  kControllerManual,
  kControllerTransIn
} ControllerType;

typedef enum {
  kInitializationStateFirstEntry,
  kInitializationStateWaitForValidData,
  kInitializationStateFirstLoop,
  kInitializationStateRunning
} InitializationState;

typedef enum {
  kControlOptHardCodeWind = 1 << 0,
  kControlOptHardCodeInitialPayout = 1 << 1,
} ControlOption;

// These values should never be changed (except for kNumFlightModes).
// New flight modes should be appended to the end.
typedef enum {
  kFlightModeForceSigned = -1,
  kFlightModePilotHover = 0,
  kFlightModePerched = 1,
  kFlightModeHoverAscend = 2,
  kFlightModeHoverPayOut = 3,
  kFlightModeHoverFullLength = 4,
  kFlightModeHoverAccel = 5,
  kFlightModeTransIn = 6,
  kFlightModeCrosswindNormal = 7,
  kFlightModeCrosswindPrepTransOut = 8,
  kFlightModeHoverTransOut = 9,
  kFlightModeHoverReelIn = 10,
  kFlightModeHoverDescend = 11,
  kFlightModeOffTether = 12,
  kFlightModeHoverTransformGsUp = 13,
  kFlightModeHoverTransformGsDown = 14,
  kNumFlightModes
} FlightMode;

typedef struct {
  uint8_t sequence;
  int32_t flight_mode;
} ControlSyncData;

typedef struct {
  FlightMode flight_mode;
  FlightMode last_flight_mode;
  bool flight_mode_first_entry;
  double flight_mode_time;
} FlightStatus;

typedef struct {
  double temperature;  // [C]
  double pressure;     // [Pa]
  double humidity;     // [fraction]
} GsWeather;

typedef struct {
  //ControlSyncData sync[kNumControllers];
  //GsgData gsg[kNumDrums];
  GsSensorData gs_sensors;
  GpsData wing_gps[kNumWingGpsReceivers];
  JoystickData joystick;
  double loadcells[kNumLoadcellSensors];
  bool tether_released;
  ImuData imus[kNumImus];
  PitotData pitots[kNumPitotSensors];
  double flaps[kNumFlaps];    // [rad]
  double rotors[kNumMotors];  // [rad/s]
  int32_t stacking_state;     // See StackingState.
  Vec3 wind_ws;
  GsGpsData gs_gps;
  //PerchData perch;
  GsWeather weather;
  bool force_hover_accel;
  uint8_t experimental_crosswind_config;
} ControlInput;

typedef struct {
  ControlSyncData sync;
  double flaps[kNumFlaps];    // [rad]
  double rotors[kNumMotors];  // [rad/s]
  double winch_vel_cmd;       // [m/s]
  double detwist_cmd;         // [rad]
  bool stop_motors;           // Set by inner loop; overrides run_motors.
  bool run_motors;            // Set by outer loop.
  bool tether_release;
  bool light;
  //GroundStationMode gs_mode_request;

  // Target azimuth [rad], tether elevation [rad], and dead zone [rad].
  // TODO(claridge): Separate this into named fields.
  Vec3 gs_targeting_data;

  // When this is set by one of the controllers, ControlOutputStep will hold the
  // value of gs_targeting_data.
  bool hold_gs_targeting_data;
} ControlOutput;

// Common parameters.

typedef struct {
  double ascend_payout_throttle;
  double return_to_crosswind_throttle;
  double prep_trans_out_throttle;
  double descend_reel_in_throttle;
  double e_stop_throttle;
  double e_stop_throttle_latch_time;
} JoystickControlParams;

typedef struct {
  ControlInput max;
  ControlInput min;
} SensorLimitsParams;

typedef struct {
  FlightStatus flight_status;
  FlightMode autonomous_flight_mode;
  double flight_mode_zero_throttle_timer;
  uint8_t controller_sync_sequence;
} PlannerState;

typedef struct {
  double airbrake_flap_angle;
  double flaps_min[kNumFlaps];
  double flaps_max[kNumFlaps];
  double rate_limit;
} ControlOutputParams;

typedef struct {
  bool run_motors_latched;
  double flaps_z1[kNumFlaps];
  double last_valid_detwist_cmd;
  Vec3 gs_targeting_data_z1;
} ControlOutputState;

typedef struct {
  double sim_init_timeout;
  double sim_update_timeout;
} HitlControlParams;

typedef struct {
  FlightPlan flight_plan;
  ControlOption control_opt;
  SimpleAeroModelParams simple_aero_model;
  RotorControlParams rotor_control;
  SensorLimitsParams sensor_limits;
  EstimatorParams estimator;
  //HoverParams hover;
  //TransInParams trans_in;
  CrosswindParams crosswind;
  //ManualParams manual;
  ControlOutputParams control_output;
  JoystickControlParams joystick_control;
  //FaultDetectionParams fault_detection;
  HitlControlParams hitl;
} ControlParams;

typedef struct {
  int32_t buffer_counter;
  int64_t max_usecs[2];
} LoopTimeState;

typedef struct {
  InitializationState init_state;
  FlightStatus flight_status;
  //ControlInputMessages input_messages;
  double time;
  //FaultMask faults[kNumSubsystems];
  //AvionicsInterfaceState avionics_interface;
  PlannerState planner;
  EstimatorState estimator;
  //HoverState hover;
  //TransInState trans_in;
  CrosswindState crosswind;
  //ManualState manual;
  ControlOutputState control_output;
  LoopTimeState loop_time;
} ControlState;

#ifdef __cplusplus
extern "C" {
#endif

//int32_t GetNumFlightModeGates(FlightMode flight_mode);
//const char *InitializationStateToString(InitializationState init_state);
//const char *FlightModeToString(FlightMode flight_mode);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CONTROL_TYPES_H_
