#ifndef SYSTEM_LABELS_H_
#define SYSTEM_LABELS_H_

#include <stdbool.h>

// Flaps A1 through A8 correspond to the main wing flaps,
// i.e. ailerons and center flaps, going from port to starboard.
// Flaps A3 and A6 do not exist because they are fixed on the M600.
typedef enum {
  kFlapLabelForceSigned = -1,
  kFlapA1,
  kFlapA2,
  kFlapA4,
  kFlapA5,
  kFlapA7,
  kFlapA8,
  kFlapEle,
  kFlapRud,
  kNumFlaps
} FlapLabel;

typedef enum {  // added here becuase definition not found elesewhere - Jmiller - STI
  kMotorForceSigned = -1,
  kMotor1,
  kMotor2,
  kMotor3,
  kMotor4,
  kMotor5,
  kMotor6,
  kMotor7,
  kMotor8,
  kNumMotors
} MotorLabel;

typedef enum { // // added here becuase definition not found elesewhere - Jmiller - STI
  kServoForceSigned = -1,
  kServoA1,
  kServoA2,
  kServoA3,
  kServoA4,
  kServoA5,
  kServoA6,
  kServoA7,
  kServoA8,
  kServoE1,
  kServoE2,
  kServoR1,
  kServoR2,
  kNumServos
} ServoLabel;

typedef enum {
  kImuLabelForceSigned = -1,
  kImuA,
  kImuB,
  kImuC,
  kNumImus
} ImuLabel;

typedef enum {
  kPitotSensorLabelForceSigned = -1,
  kPitotSensorHighSpeed,
  kPitotSensorLowSpeed,
  kNumPitotSensors
} PitotSensorLabel;

typedef enum {
  kWingGpsReceiverLabelForceSigned = -1,
  kWingGpsReceiverCrosswind,
  kWingGpsReceiverHover,
  kWingGpsReceiverPort,
  kWingGpsReceiverStar,
  kNumWingGpsReceivers
} WingGpsReceiverLabel;

typedef enum {
  kJoystickChannelLabelForceSigned = -1,
  kJoystickChannelPitch,
  kJoystickChannelRoll,
  kJoystickChannelYaw,
  kJoystickChannelThrottle,
  kJoystickChannelSwitches,
  kNumJoystickChannels
} JoystickChannelLabel;

typedef enum {
  kJoystickSwitchPositionLabelForceSigned = -1,
  kJoystickSwitchPositionUp,
  kJoystickSwitchPositionMiddle,
  kJoystickSwitchPositionDown,
  kNumJoystickSwitchPositions
} JoystickSwitchPositionLabel;

typedef enum {
  kLoadcellSensorLabelForceSigned = -1,
  kLoadcellSensorPort0,
  kLoadcellSensorPort1,
  kLoadcellSensorStarboard0,
  kLoadcellSensorStarboard1,
  kNumLoadcellSensors
} LoadcellSensorLabel;

typedef enum {
  kProximitySensorLabelForceSigned = -1,
  kProximitySensorEarlyA,
  kProximitySensorEarlyB,
  kProximitySensorFinalA,
  kProximitySensorFinalB,
  kNumProximitySensors
} ProximitySensorLabel;

#endif  // SYSTEM_LABELS_H_
