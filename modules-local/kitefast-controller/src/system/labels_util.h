#ifndef SYSTEM_LABELS_UTIL_H_
#define SYSTEM_LABELS_UTIL_H_

#include <stdbool.h>

// #include "avionics/common/gps_receiver.h"
// #include "avionics/network/aio_labels.h"
// #include "avionics/network/aio_node.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

// Converts a servo label to the corresponding flap label.  Note that
// multiple servos can attach to the same flap on the M600.
FlapLabel ServoToFlap(ServoLabel servo_label);

// Returns true if the servo is attached to a flap on the wing.
bool IsFlapServo(ServoLabel servo_label);

const char *LoadcellSensorLabelToString(LoadcellSensorLabel loadcell_label);

// AioNode WingGpsReceiverLabelToAioNode(WingGpsReceiverLabel wing_gps_receiver);
// GpsReceiverType WingGpsReceiverLabelToGpsReceiverType(
//     WingGpsReceiverLabel wing_gps_receiver);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SYSTEM_LABELS_UTIL_H_
