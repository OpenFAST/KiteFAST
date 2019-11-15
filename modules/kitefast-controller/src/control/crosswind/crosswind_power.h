// The crosswind power loop collects information on wind speed, motor
// temperatures, controller state, and pilot requests and determines
// how hard to push the system, i.e. whether to generate as much power
// as possible or to run the system more conservatively.  The output
// of the power loop is a flight path type and center and an airspeed.
//
//                        ___________
//                       |           |--> path_type
//          ele_tuner -->|  SetPath  |--> path_center_g
//                       |___________|
//
//                        ___________________
//                       |                   |
//     airspeed_tuner -->|  SetMeanAirspeed  |--> mean_airspeed_cmd
//                       |___________________|
//
//                        _______________
//                       |               |
//  mean_airspeed_cmd -->|  SetAirspeed  |--> airspeed_cmd
//                       |_______________|
//

#ifndef CONTROL_CROSSWIND_CROSSWIND_POWER_H_
#define CONTROL_CROSSWIND_CROSSWIND_POWER_H_

#include <stdbool.h>

#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_playbook_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the state of the power loop.
void CrosswindPowerInit(const Vec3 *wing_pos_g, const Vec3 *wing_vel_g,
                        double wind_dir_f, const PlaybookEntry *playbook_entry,
                        const CrosswindPowerParams *params,
                        CrosswindPowerState *state);

// Returns the center of the flight path in ground coordinates.
void CrosswindPowerGetPathCenter(const CrosswindPowerState *state,
                                 Vec3 *path_center_g);

// Returns the path radius target.
double CrosswindPowerGetPathRadiusTarget(const CrosswindPowerState *state);

// Updates the state of the power loop by a single time step.  Returns
// the path type and center and an airspeed command.
void CrosswindPowerStep(
    const FlightStatus *flight_status, const StateEstimate *state_est,
    const CrosswindFlags *flags, const CrosswindPowerParams *params,
    const PlaybookEntry *playbook_entry, CrosswindPowerState *state,
    CrosswindPathType *path_type, Vec3 *path_center_g, double *airspeed_cmd,
    double *d_airspeed_d_loopangle, double *alpha_nom, double *beta_nom);

double SetAlpha(double loop_angle, const PlaybookEntry *playbook_entry);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_POWER_H_
