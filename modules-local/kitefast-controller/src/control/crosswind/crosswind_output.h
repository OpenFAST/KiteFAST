#ifndef CONTROL_CROSSWIND_CROSSWIND_OUTPUT_H_
#define CONTROL_CROSSWIND_CROSSWIND_OUTPUT_H_

#include "control/actuator_util.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator_types.h"


#ifdef __cplusplus
extern "C" {
#endif

#define TETHER_DETWIST_REVS 1.0f // Taken from /avionics/common/avionics_messages.h (line:1223) - jmiller - STI
#define TETHER_DETWIST_BITS 12 // Taken from /avionics/common/avionics_messages.h (line:1223) - jmiller - STI

// Validate the parameters of the output stage.
bool CrosswindOutputValidateParams(const CrosswindOutputParams *params);

// Initializes the state of the output stage.
void CrosswindOutputInit(const CrosswindOutputParams *params,
                         CrosswindOutputState *state,
                         double previous_detwist_loop_angle,
                         int32_t previous_detwist_rev_count);

// Updates the state of the output stage by a single time step.
// Returns the final control output.
void CrosswindOutputStep(LoopDirection loop_dir, double loop_angle,
                         bool allow_flare, const ThrustMoment *thrust_moment,
                         const Deltas *deltas, const StateEstimate *state_est,
                         const Vec3 *path_center_g,
                         const CrosswindOutputParams *params,
                         CrosswindOutputState *state,
                         ControlOutput *control_output);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_OUTPUT_H_
