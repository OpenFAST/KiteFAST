// The crosswind controller is composed of a hierarchy of successively
// closed loops, with long time scale loops that determine power
// production and path on the outside and with short time scale loops
// such as angle-of-attack control on the inside.  The specific
// sequence of loop closures and their inputs and outputs are:
//
//         POWER          PATH     CURVATURE       INNER
//   --------|-------------|-----------|-------------|---------
//    power  | path type   | curvature | alpha, beta | delta_i
//    temp.  | path center |           | teth_roll   | tau_i
//    pilot  |             |           | p, q, r     |
//    tuners |             |           |             |
//           | airspeed    |           | airspeed    | thrust
//
// Finally, there is an output stage that converts the control outputs
// (logical flap groupings and collective rotor thrusts and moments)
// to individual flap and rotor commands:
//
//          OUTPUT
//   ---------|---------
//    delta_i | flap_i
//    tau_i   | rotor_i
//    thrust  |

#ifndef CONTROL_CROSSWIND_CROSSWIND_H_
#define CONTROL_CROSSWIND_CROSSWIND_H_

#include <stdbool.h>
#include <stdint.h>

#include "vec3.h"
#include "control_types.h"
#include "crosswind_types.h"
#include "estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the crosswind controller.
void CrosswindInit(const StateEstimate *state_est, const double *flaps_z1,
                   double previous_detwist_loop_angle,
                   int32_t previous_detwist_rev_count,
                   const CrosswindParams *params, CrosswindState *state);

// Determines whether the controller is ready to switch into
// kFlightModeCrosswindNormal or kFlightModeCrosswindPrepTransOut.
//bool CrosswindIsReadyForMode(FlightMode flight_mode,
//                             const FlightStatus *flight_status,
//                             const StateEstimate *state_est,
//                             const CrosswindParams *params,
//                             const CrosswindState *state);

// Updates the state crosswind controller by a single time step.
// Returns the output commands to the actuators.
void CrosswindStep(const FlightStatus *flight_status,
                   const StateEstimate *state_est,
                   const CrosswindParams *params, CrosswindState *state,
                   ControlOutput *control_output);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_H_
