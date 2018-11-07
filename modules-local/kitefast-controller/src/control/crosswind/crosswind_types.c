#include "control/crosswind/crosswind_types.h"

#include <assert.h>
#include <stdbool.h>

const char *CrosswindNormalGateToString(CrosswindNormalGate gate) {
  switch (gate) {
    case kCrosswindNormalGateSpeed:
      return "Speed";
    case kCrosswindNormalGateTension:
      return "Tension";
    case kCrosswindNormalGateAltitude:
      return "Altitude";
    case kCrosswindNormalGateAirspeed:
      return "Airspeed";
    case kCrosswindNormalGateFlightMode:
      return "Flight Mode";
    default:
    case kCrosswindNormalGateForceSigned:
    case kNumCrosswindNormalGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *CrosswindPrepTransOutGateToString(CrosswindPrepTransOutGate gate) {
  switch (gate) {
    case kCrosswindPrepTransOutGateWindSpeed:
      return "WindSpeed";
    default:
    case kCrosswindPrepTransOutGateForceSigned:
    case kNumCrosswindPrepTransOutGates:
      assert(false);
      return "<Unknown>";
  }
}

const char *CrosswindHoverTransOutGateToString(
    CrosswindHoverTransOutGate gate) {
  switch (gate) {
    case kCrosswindHoverTransOutGateAirspeed:
      return "Airspeed";
    case kCrosswindHoverTransOutGateAlpha:
      return "Angle of Attack";
    case kCrosswindHoverTransOutGatePathType:
      return "PathType";
    case kCrosswindHoverTransOutGateStillAccelerating:
      return "StillAccelerating";
    default:
    case kCrosswindHoverTransOutGateForceSigned:
    case kNumCrosswindHoverTransOutGates:
      assert(false);
      return "<Unknown>";
  }
}
