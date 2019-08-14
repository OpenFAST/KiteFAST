#include "control/crosswind/crosswind_power.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/geometry.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
// #include "control/common.h"
#include "control/control_params.h"
// #include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_frame.h"
#include "control/crosswind/crosswind_playbook.h"
#include "control/crosswind/crosswind_types.h"
#include "control/crosswind/crosswind_util.h"
#include "control/estimator/estimator_types.h"
#include "control/ground_station_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"

// Attempts to reduce the transient between transition-in and
// crosswind by selecting a path center where the current wing
// velocity and position align with the circular path.
static void SelectStartingAzimuthElevation(const Vec3 *wing_pos_g,
                                           const Vec3 *wing_vel_g,
                                           double path_azimuth_setpoint,
                                           double path_elevation_setpoint,
                                           double path_radius,
                                           const CrosswindPowerParams *params,
                                           double *azimuth, double *elevation) {
  assert(-PI <= path_azimuth_setpoint && path_azimuth_setpoint <= PI);
  assert(0.0 <= path_elevation_setpoint && path_elevation_setpoint <= PI / 2.0);
  assert(params->ele_min <= params->ele_max);
  assert(path_radius > 0.0);

  // Calculate offset from the current wing position to the ideal path
  // center.
  Vec3 wing_to_path_center_g;
  Vec3Cross(wing_pos_g, wing_vel_g, &wing_to_path_center_g);
  Vec3Normalize(&wing_to_path_center_g, &wing_to_path_center_g);
  Vec3Scale(&wing_to_path_center_g, path_radius, &wing_to_path_center_g);

  // Convert the ideal path center to azimuth and elevation values.
  Vec3 path_center_g;
  Vec3Add(&wing_to_path_center_g, wing_pos_g, &path_center_g);
  double unused_radius;
  VecGToSph(&path_center_g, azimuth, elevation, &unused_radius);

  // Saturate the azimuth and elevation values both by an allowed
  // angular offset from the starting azimuth and elevation set-points.
  double azimuth_offset = Wrap(*azimuth - path_azimuth_setpoint, -PI, PI);
  *azimuth = path_azimuth_setpoint + Saturate(azimuth_offset, -0.4, 0.4);
  *azimuth = Wrap(*azimuth, -PI, PI);

  double elevation_offset = Wrap(*elevation - path_elevation_setpoint, -PI, PI);
  *elevation = path_elevation_setpoint + Saturate(elevation_offset, -0.2, 0.2);
  *elevation = Saturate(*elevation, params->ele_min, params->ele_max);
}

void CrosswindPowerInit(const Vec3 *wing_pos_g, const Vec3 *wing_vel_g,
                        double wind_dir_f, const PlaybookEntry *playbook_entry,
                        const CrosswindPowerParams *params,
                        CrosswindPowerState *state) {
  assert(-PI <= wind_dir_f && wind_dir_f <= PI);

  memset(state, 0, sizeof(*state));
  state->azi_setpoint =
      GetPlaybookEntryAzimuthWithLimits(wind_dir_f, playbook_entry);
  SelectStartingAzimuthElevation(wing_pos_g, wing_vel_g, state->azi_setpoint,
                                 playbook_entry->elevation,
                                 playbook_entry->path_radius_target, params,
                                 &state->path_azimuth, &state->path_elevation);
  state->path_type = kCrosswindPathNormal;

  // Check that the loop angle is provided on an equispaced grid. This is
  // important for getting a smooth derivative from SetAirspeedDerivative.
  state->dloop_angle = playbook_entry->lookup_loop_angle[1] -
                       playbook_entry->lookup_loop_angle[0];
  for (int32_t i = 0; i < ARRAYSIZE(playbook_entry->lookup_loop_angle) - 1;
       ++i) {
    double dloop_angle = playbook_entry->lookup_loop_angle[i + 1] -
                         playbook_entry->lookup_loop_angle[i];
    assert(fabs(dloop_angle - state->dloop_angle) < 1e-12);
  }
}

static void UpdatePathCenter(double wind_dir_f,
                             const PlaybookEntry *playbook_entry,
                             FlightMode flight_mode,
                             const CrosswindPowerParams *params,
                             CrosswindPowerState *state, Vec3 *path_center_g) {
  assert(-PI <= wind_dir_f && wind_dir_f <= PI);
  assert(0 <= flight_mode && flight_mode < kNumFlightModes);
  assert(path_center_g != NULL);
  assert(params->ele_min <= params->ele_max);

  if (flight_mode == kFlightModeCrosswindNormal) {
    // Update the path azimuth and elevation when in crosswind normal.

    // Calculate the set-point for the center of the flight path.
    // Some test sites or equipment have azimuth limitations.
    state->azi_setpoint =
        GetPlaybookEntryAzimuthWithLimits(wind_dir_f, playbook_entry);

    // Rate limit how fast elevation and azimuth set-point can change.
    RateLimitCircular(state->azi_setpoint, -params->azi_rate_lim,
                      params->azi_rate_lim, -PI, PI, *g_sys.ts,
                      &state->path_azimuth);
    RateLimitCircular(playbook_entry->elevation, -params->ele_rate_lim,
                      params->ele_rate_lim, -PI, PI, *g_sys.ts,
                      &state->path_elevation);
  } else if (flight_mode == kFlightModeCrosswindPrepTransOut) {
    // Update only the path elevation in prep transout.
    RateLimitCircular(params->transout_elevation_cmd, -params->ele_rate_lim,
                      params->ele_rate_lim, -PI, PI, *g_sys.ts,
                      &state->path_elevation);
  }

  CrosswindPowerGetPathCenter(state, path_center_g);
}

// Chooses which path to use in the path controller.
static CrosswindPathType UpdatePathType(FlightMode flight_mode,
                                        double loop_angle,
                                        const CrosswindPowerParams *params,
                                        CrosswindPowerState *state) {
  if (flight_mode == kFlightModeCrosswindPrepTransOut) {
    // We change from kCrosswindPathNormal to PrepareTransitionOut near the
    // bottom of the loop. Over the rest of the loop, we leave the path type
    // unmodified. This handles situations where the vehicle passes loop_angle
    // == 0 during trans-out or we enter kFlightModeCrosswindPrepTransOut in
    // what would otherwise be the final quarter loop.
    // TODO(scarito): Wait for reaching azi command.
    // TODO(mabraham): Wait for reaching radius command.
    if (params->loop_angle_path_switch_min < loop_angle &&
        loop_angle <= params->loop_angle_path_switch_max &&
        fabs(params->transout_elevation_cmd - state->path_elevation) < 0.01) {
      state->path_type = kCrosswindPathPrepareTransitionOut;
    }
  } else {
    state->path_type = kCrosswindPathNormal;
  }

  return state->path_type;
}

// Move the branch point of loop_angle from the middle of the up stroke to the
// top of the circle and reverse the direction.
static double LoopAngleTop(double loop_angle) {
  return Wrap(1.5 * PI - loop_angle, 0.0, 2.0 * PI);
}

// TODO(kennyjensen): This assumes a specific loop direction.
static double SetAirspeed(double loop_angle, CrosswindPathType path_type,
                          const PlaybookEntry *playbook_entry,
                          const CrosswindPowerParams *params) {
  assert(0.0 <= loop_angle && loop_angle <= 2.0 * PI);

  double normal_airspeed_cmd = CircularInterp1(
      playbook_entry->lookup_loop_angle, playbook_entry->airspeed_lookup,
      ARRAYSIZE(playbook_entry->lookup_loop_angle), loop_angle);
  normal_airspeed_cmd =
      Saturate(normal_airspeed_cmd, params->min_airspeed, params->max_airspeed);

  double airspeed_cmd = normal_airspeed_cmd;
  if (path_type == kCrosswindPathPrepareTransitionOut) {
    // Create a new loop angle measured from the top of the circle and
    // increasing with progress around the loop.
    double loop_angle_top = LoopAngleTop(loop_angle);
    // The deceleration begins at the same loop angle where the path changes so
    // that the airspeed command crossfade results in a continuous signal.
    double loop_angle_top_start =
        LoopAngleTop(params->loop_angle_path_switch_max);
    double loop_angle_top_end =
        LoopAngleTop(params->transout_airspeed_crossfade_angle_end);
    airspeed_cmd =
        Crossfade(normal_airspeed_cmd, params->transout_airspeed_target,
                  loop_angle_top, loop_angle_top_start, loop_angle_top_end);
  }

  return airspeed_cmd;
}

// Return the derivative of airspeed_cmd with respect to loop_angle.
static double GetAirspeedDerivative(double loop_angle, double dloop_angle,
                                    CrosswindPathType path_type,
                                    const PlaybookEntry *playbook_entry,
                                    const CrosswindPowerParams *params) {
  // We take the derivative numerically so that all saturations in SetAirspeed
  // are automatically accounted for.
  return (SetAirspeed(Wrap(loop_angle + dloop_angle, 0.0, 2.0 * PI), path_type,
                      playbook_entry, params) -
          SetAirspeed(Wrap(loop_angle - dloop_angle, 0.0, 2.0 * PI), path_type,
                      playbook_entry, params)) /
         (2.0 * dloop_angle);
}

double SetAlpha(double loop_angle, const PlaybookEntry *playbook_entry) {
  return CircularInterp1(playbook_entry->lookup_loop_angle,
                         playbook_entry->alpha_lookup,
                         ARRAYSIZE(playbook_entry->alpha_lookup), loop_angle);
}

static double SetBeta(double loop_angle, const PlaybookEntry *playbook_entry) {
  return CircularInterp1(playbook_entry->lookup_loop_angle,
                         playbook_entry->beta_lookup,
                         ARRAYSIZE(playbook_entry->beta_lookup), loop_angle);
}

void CrosswindPowerGetPathCenter(const CrosswindPowerState *state, Vec3 *path_center_g) {
  assert(state != NULL && path_center_g != NULL);
  assert(-PI <= state->path_elevation && state->path_elevation <= PI);
  // TODO(kennyjensen): Add an assert for the path azimuth being in
  // [-pi, pi) once all the wrapping issues are resolved.

  SphToVecG(state->path_azimuth, state->path_elevation, g_sys.tether->length,
            path_center_g);
}

// TODO(kennyjensen): This code does not check state_est->wind_g.valid.
void CrosswindPowerStep(
    const FlightStatus *flight_status, const StateEstimate *state_est,
    const CrosswindFlags *flags, const CrosswindPowerParams *params,
    const PlaybookEntry *playbook_entry, CrosswindPowerState *state,
    CrosswindPathType *path_type, Vec3 *path_center_g, double *airspeed_cmd,
    double *d_airspeed_d_loopangle, double *alpha_nom, double *beta_nom) {
  assert(flight_status != NULL && state_est != NULL && flags != NULL &&
         params != NULL && state != NULL && path_type != NULL &&
         path_center_g != NULL && airspeed_cmd != NULL && alpha_nom != NULL);

  UpdatePathCenter(state_est->wind_g.dir_f, playbook_entry,
                   flight_status->flight_mode, params, state, path_center_g);

  double loop_angle = CalcLoopAngle(path_center_g, &state_est->Xg);
  *path_type =
      UpdatePathType(flight_status->flight_mode, loop_angle, params, state);

  *airspeed_cmd = SetAirspeed(loop_angle, *path_type, playbook_entry, params);
  *alpha_nom = SetAlpha(loop_angle, playbook_entry);
  *beta_nom = SetBeta(loop_angle, playbook_entry);

  *d_airspeed_d_loopangle = GetAirspeedDerivative(
      loop_angle, state->dloop_angle, *path_type, playbook_entry, params);

  // // Update telemetry.
  // CrosswindTelemetry *cwt = GetCrosswindTelemetry();
  // cwt->azi_offset = playbook_entry->azi_offset;
  // cwt->azi_target = state->azi_setpoint;
  // cwt->elevation = state->path_elevation;
  // cwt->path_type = (int32_t)*path_type;
  // cwt->path_center_g = *path_center_g;
  // cwt->loop_angle = loop_angle;
}
