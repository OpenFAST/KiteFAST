#include "control/estimator/estimator_types.h"

#include <assert.h>
#include <stdbool.h>

const char *ApparentWindSolutionTypeToString(ApparentWindSolutionType type) {
  switch (type) {
    case kApparentWindSolutionTypeInertialAndWind:
      return "InertialAndWind";
    case kApparentWindSolutionTypeMixed:
      return "Mixed";
    case kApparentWindSolutionTypeFixedAngles:
      return "FixedAngles";
    case kApparentWindSolutionTypeLoadcell:
      return "Loadcell";
    case kApparentWindSolutionTypePitot:
      return "Pitot";
    case kApparentWindSolutionTypeComplementary:
      return "Complementary";
    default:
    case kApparentWindSolutionTypeForceSigned:
    case kNumApparentWindSolutionTypes:
      assert(false);
      return "<Unknown>";
  }
}
