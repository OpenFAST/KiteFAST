#include "system/labels_util.h"

#include <assert.h>
#include <stdbool.h>

// #include "avionics/common/gps_receiver.h"
// #include "avionics/network/aio_labels.h"
// #include "avionics/network/aio_node.h"
#include "system/labels.h"

FlapLabel ServoToFlap(ServoLabel servo_label) {
  switch (servo_label) {
    case kServoA1: return kFlapA1;
    case kServoA2: return kFlapA2;
    case kServoA4: return kFlapA4;
    case kServoA5: return kFlapA5;
    case kServoA7: return kFlapA7;
    case kServoA8: return kFlapA8;
    case kServoE1: return kFlapEle;
    case kServoE2: return kFlapEle;
    case kServoR1: return kFlapRud;
    case kServoR2: return kFlapRud;
    default:
    // case kServoLabelForceSigned:
    case kNumServos:
      assert(false);
      return kFlapA1;
  }
}

// TODO(scarito): Consider deprecating this.  Our code doesn't currently handle
// non-flap servos and we don't use them anyway.
bool IsFlapServo(ServoLabel servo_label) {
  return kServoA1 <= servo_label && servo_label <= kServoR2;
}

const char *LoadcellSensorLabelToString(LoadcellSensorLabel loadcell_label) {
  switch (loadcell_label) {
    case kLoadcellSensorPort0:
      return "Port0";
    case kLoadcellSensorPort1:
      return "Port1";
    case kLoadcellSensorStarboard0:
      return "Starboard0";
    case kLoadcellSensorStarboard1:
      return "Starboard1";
    default:
    case kLoadcellSensorLabelForceSigned:
    case kNumLoadcellSensors:
      assert(false);
      return "<Unknown>";
  }
}

// // TODO(tfricke): Move this to config/m600/sensor_layout.py
// AioNode WingGpsReceiverLabelToAioNode(WingGpsReceiverLabel wing_gps_receiver) {
//   switch (wing_gps_receiver) {
//     case kWingGpsReceiverCrosswind:
//       return kAioNodeFcA;
//     case kWingGpsReceiverHover:
//       return kAioNodeFcB;
//     case kWingGpsReceiverPort:
//       return kAioNodeLightPort;
//     case kWingGpsReceiverStar:
//       return kAioNodeLightStbd;
//     case kWingGpsReceiverLabelForceSigned:
//     case kNumWingGpsReceivers:
//     default:
//       assert(false);
//       return kAioNodeUnknown;
//   }
// }

// GpsReceiverType WingGpsReceiverLabelToGpsReceiverType(
//     WingGpsReceiverLabel wing_gps_receiver) {
//   switch (wing_gps_receiver) {
//     case kWingGpsReceiverCrosswind:
//       return kGpsReceiverTypeNovAtel;
//     case kWingGpsReceiverHover:
//       return kGpsReceiverTypeNovAtel;
//     case kWingGpsReceiverPort:
//       return kGpsReceiverTypeNovAtel;
//     case kWingGpsReceiverStar:
//       return kGpsReceiverTypeNovAtel;
//     case kWingGpsReceiverLabelForceSigned:
//     case kNumWingGpsReceivers:
//     default:
//       assert(false);
//       return kGpsReceiverTypeNone;
//   }
// }
