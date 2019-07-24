#include "control/crosswind/crosswind.h"

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "common/c_math/geometry.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
//#include "../actuator_util.h"
//#include "../common.h"
#include "control/control_params.h"
//#include "../control_telemetry.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_curvature.h"
#include "crosswind_frame.h"
#include "control/crosswind/crosswind_inner.h"
//#include "crosswind_mode.h"
#include "control/crosswind/crosswind_output.h"
#include "crosswind_path.h"
#include "control/crosswind/crosswind_playbook.h"
#include "control/crosswind/crosswind_power.h"
#include "control/crosswind/crosswind_types.h"
#include "control/crosswind/crosswind_util.h"
#include "control/sensor_util.h"
#include "control/simple_aero.h"
#include "control/system_params.h"
#include "control/system_types.h"

void CrosswindInit(const StateEstimate *state_est, const double *flaps_z1,
                   double previous_detwist_loop_angle,
                   int32_t previous_detwist_rev_count,
                   const CrosswindParams *params, CrosswindState *state) {
  assert(state_est != NULL && params != NULL && state != NULL);

  memset(state, 0, sizeof(*state));

  PlaybookEntry playbook_entry;
  // TODO(scarito): Handle the wind_aloft_g valid flag here.
  GetPlaybookEntry(&params->playbook, NULL,
                  state_est->wind_aloft_g.speed_f_playbook, 0.0,
                  &playbook_entry);
  CrosswindPowerInit(&state_est->Xg, &state_est->Vg, state_est->wind_g.dir_f,
                    &playbook_entry, &params->power, &state->power);

  // Calculate the initial loop angle.
  Vec3 path_center_g;
  CrosswindPowerGetPathCenter(&state->power, &path_center_g);
  // path_center_g.x = -170.6151;
  // path_center_g.y = -267.3352;
  // path_center_g.z = -284.1268; // need to ask about path_center_g
  const double loop_angle = CalcLoopAngle(&path_center_g, &state_est->Xg);
  printf("   loop_angle: %0.4f \n", loop_angle);
  CrosswindPathInit(state_est, &params->path, playbook_entry.path_radius_target,
                   &state->path);
  CrosswindCurvatureInit(state_est->tether_force_b.sph.tension,
                        state_est->apparent_wind.sph_f.alpha,
                        state_est->apparent_wind.sph_f.beta,
                        &params->curvature, &state->curvature);

  // Calculate the aileron,p elevator, and rudder commands that produce
  // equivalent deflections to the final deflections from the
  // transition-in controller.
  Deltas deltas_0 = {
	  .aileron = (flaps_z1[kFlapA8] - flaps_z1[kFlapA1]) / 2.0,
	  .inboard_flap = 0.0,
	  .midboard_flap = 0.0,
	  .outboard_flap = 0.0,
	  .elevator = flaps_z1[kFlapEle] - params->output.flap_offsets[kFlapEle],
	  .rudder = flaps_z1[kFlapRud] - params->output.flap_offsets[kFlapRud] };

  double alpha_0 = SetAlpha(loop_angle, &playbook_entry);
  // double alpha_0 = 0; // added by Jmiller STI
  CrosswindInnerInit(state_est, &deltas_0, alpha_0, loop_angle, &params->inner,
                     &state->inner);
  CrosswindOutputInit(&params->output, &state->output,
                     previous_detwist_loop_angle, previous_detwist_rev_count);

  state->experimental_crosswind.mode_active = false;
  state->experimental_crosswind.current_config =
     params->experimental_crosswind_config[0];

  state->playbook_fallback_crossfade = 0.0;
}

// bool CrosswindIsReadyForMode(FlightMode proposed_flight_mode,
//                             const FlightStatus *floop_anglelight_status,
//                             const StateEstimate *state_est,
//                             const CrosswindParams *params,
//                             const CrosswindState *state) {
//  double trans_out_flare_time = CrosswindCurvatureFlareTimer(&state->curvature);

//  return CrosswindModeIsReadyFor(proposed_flight_mode, flight_status, state_est,
//                                 trans_out_flare_time, state->power.path_type,
//                                 &params->mode);
// }

// Returns control flags that are used throughout the crosswind
// controller.
static void GetFlags(const StateEstimate *state_est, FlightMode flight_mode,
                    CrosswindPathType path_type, CrosswindFlags *flags) {
 // These flags simplify what constitutes a fault in the crosswind controller.
 //
 // The control features that are tied directly to the loadcells are
 // turned off if there is a fault.
 flags->loadcell_fault = !state_est->tether_force_b.valid;

 // When true, this flag indicates that we should trust our alpha / beta
 // estimates less (e.g. by turning off integrators).
 switch (state_est->apparent_wind.solution_type) {
   case kApparentWindSolutionTypeLoadcell:
   case kApparentWindSolutionTypePitot:
     flags->alpha_beta_fault = false;
     break;
   default:
   case kNumApparentWindSolutionTypes:
   case kApparentWindSolutionTypeForceSigned:
   case kApparentWindSolutionTypeInertialAndWind:
   case kApparentWindSolutionTypeMixed:
   case kApparentWindSolutionTypeFixedAngles:
     flags->alpha_beta_fault = true;
     break;
 }

 // This is somewhat of a hack that was necessary to pass the extreme
 // gust IEC cases.  The extreme wind flag forces a high elevation,
 // high azimuth path, and faster wind direction tracking.  This must
 // use the unfiltered wind speed value or it cannot respond to
 // extreme wind events quickly enough.
 //
 // NOTE(kennyjensen): This is turned off for the first flights,
 // which will be in low wind, for simplicity.  If it is activated
 // again, hysteresis should be added.
 // flags->extreme_wind = false;

 // Only use the spoiler when we are preparing to transition out.  In
 // the future, we may want to try using it during normal crosswind
 // flight to slow down the wing to put it in sync with the ideal
 // velocity curve around the loop.
 if (flight_mode == kFlightModeCrosswindPrepTransOut &&
     path_type == kCrosswindPathPrepareTransitionOut) {
   flags->spoiler_enabled = true;
 } else {
   flags->spoiler_enabled = false;
 }
}

static void UpdateExperimentalConfig(uint8_t config_index, double pitch_f,
                                    const CrosswindParams *params,
                                    ExperimentalCrosswindState *exp_state) {
 assert(-1.0 <= pitch_f && pitch_f <= 1.0);

 ExperimentalCrosswindConfig *config = &exp_state->current_config;
 if (pitch_f < -0.7 && !exp_state->mode_active) {
   // Turn on experimental crosswind mode.
   assert(config_index < NUM_EXPERIMENTAL_CROSSWIND_CONFIGS);
   if (config_index < NUM_EXPERIMENTAL_CROSSWIND_CONFIGS) {
     exp_state->mode_active = true;
     *config = params->experimental_crosswind_config[config_index];
   }
 } else if (pitch_f > -0.3 && exp_state->mode_active) {
   // Turn off experimental crosswind mode.
   exp_state->mode_active = false;
   *config = params->experimental_crosswind_config[0];
 }

 // Update telemetry.
 //CrosswindTelemetry *cwt = GetCrosswindTelemetry();
 //cwt->experimental_config_active = exp_state->mode_active;
}

__attribute__((optimize(0))) void CrosswindStep(const FlightStatus *flight_status,
                   const StateEstimate *state_est,
                   const CrosswindParams *params, CrosswindState *state,
                   ControlOutput *control_output) {
#ifdef DEBUG //DEBUG preproc found in kfc.h
  // printf("    crosswindstep \n");
#endif
  CrosswindFlags flags;
  GetFlags(state_est, flight_status->flight_mode, state->power.path_type, &flags);

  UpdateExperimentalConfig(state_est->experimental_crosswind_config,
                          state_est->joystick.pitch_f, params,
                          &state->experimental_crosswind);

  PlaybookEntry playbook_entry;
  // Crossfade to fallback playbook when throttle is sufficiently high.  Stop
  // crossfade in place when mode changes to PrepTransOut to allow pilot to
  // trans out in fallback configuration.
  if (flight_status->flight_mode == kFlightModeCrosswindNormal) {
   double crossfade_rate = params->playbook_fallback.crossfade_rate;
   double crossfade_throttle = params->playbook_fallback.crossfade_throttle;
   assert(crossfade_rate > 0.0);
   assert(crossfade_throttle >= 0.0 && crossfade_throttle <= 1.0);
   RateLimit(state_est->joystick.throttle_f > crossfade_throttle,
             -crossfade_rate, crossfade_rate, *g_sys.ts,
             &state->playbook_fallback_crossfade);
  }
  GetPlaybookEntry(&params->playbook, &params->playbook_fallback.entry,
                 state_est->wind_aloft_g.speed_f_playbook,
                  state->playbook_fallback_crossfade, &playbook_entry);
 
  CrosswindPathType path_type;
  Vec3 path_center_g;
  double airspeed_cmd, d_airspeed_d_loopangle, alpha_nom, beta_nom;

  CrosswindPowerStep(flight_status, state_est, &flags, &params->power,
                     &playbook_entry, &state->power, &path_type, &path_center_g,
                     &airspeed_cmd, &d_airspeed_d_loopangle, &alpha_nom,
                     &beta_nom);
  // added to plotting routine to monitor outputs of powerstep (7/16/19)
  control_output->airspeed_cmd_power_out = airspeed_cmd;
  control_output->d_airspeed_d_loopangle_power_out = d_airspeed_d_loopangle;
  control_output->alpha_nom_power_out = alpha_nom;
  control_output->beta_nom_power_out = beta_nom;

  double loop_angle = CalcLoopAngle(&path_center_g, &state_est->Xg);

  double k_aero_cmd, k_geom_cmd, k_aero_curr, k_geom_curr;
  CrosswindPathStep(flight_status->flight_mode, &path_center_g, path_type,
                    state_est, &playbook_entry, &params->path, &state->path,
                    &k_aero_cmd, &k_geom_cmd, &k_aero_curr, &k_geom_curr);

  // added to plotting routine to monitor outputs of pathrstep (7/16/19)
  control_output->k_aero_cmd_path_out = k_aero_cmd;
  control_output->k_geom_cmd_path_out = k_geom_cmd;
  control_output->k_aero_curr_path_out = k_aero_curr;
  control_output->k_geom_curr_path_out = k_geom_curr;

  Vec3 pqr_cmd;
  double dCL_cmd, alpha_cmd, beta_cmd, tether_roll_cmd;

  bool flaring = false;
  
  CrosswindCurvatureStep(
      k_aero_cmd, k_geom_cmd, state_est->tether_force_b.sph.tension,
      state_est->apparent_wind.sph_f.v, alpha_nom, beta_nom, loop_angle,
      path_type, &state_est->Vg, flight_status, &flags, &params->curvature,
      &state->curvature, &pqr_cmd, &flaring, &alpha_cmd, &dCL_cmd, &beta_cmd,
      &tether_roll_cmd);
  // added to plotting routine to monitor outputs of pathrstep (7/16/19)
  control_output->pqr_cmd_curv_out = pqr_cmd;
  control_output->flaring_curv_out = flaring;
  control_output->alpha_cmd_curv_out = alpha_cmd;
  control_output->beta_cmd_curv_out = beta_cmd;
  control_output->tether_roll_cmd_curv_out = tether_roll_cmd;
  control_output->dCL_cmd_curv_out = dCL_cmd;

  // TODO(kennyjensen): Eventually, this should go in state_estimation
  // as a fallback to the loadcell-based tether roll calculation.
  double tether_roll = state_est->tether_force_b.sph.roll;
  // tether_roll = -0.0072; // Added - jmiller
  if (flags.loadcell_fault) { 
   tether_roll = CurvatureToTetherRoll(
       k_aero_curr, AlphaToCL(state_est->apparent_wind.sph_f.alpha,
                              g_cont.simple_aero_model),
       BetaToCY(state_est->apparent_wind.sph_f.beta,
                g_cont.simple_aero_model));
  }

  const double path_radius = playbook_entry.path_radius_target;
  control_output->path_radius_playbook_out = path_radius;  // Calculate the required acceleration to track the airspeed schedule.
  // TODO(tfricke): Check that wind_g is valid and do something
  // sensible if it is not.
  Vec3 wing_vel_g;
  //const double kite_accel_ff = 0; // Added - jmiller
	
  const double kite_accel_ff = CalcLoopKinematics(
      &state_est->wind_g.vector_f, &path_center_g, loop_angle, path_radius,
      airspeed_cmd, d_airspeed_d_loopangle, &wing_vel_g, NULL);
  
  control_output->kite_accel_ff_loop_kin_out = kite_accel_ff;
  // TODO(tfricke): Estimate the derivative of the tether roll command
  // analytically instead of numerically.
  const double tether_roll_dot =
      DiffLpf2(tether_roll_cmd, 1.0, 0.7, *g_sys.ts, state->tether_roll_cmd_zs);

  Vec3 pqr_cmd_new;
  const double kitespeed = Vec3Norm(&wing_vel_g);
  CalcCrosswindPqr(&state_est->wind_g.vector_f, &path_center_g, loop_angle,
                   path_radius, tether_roll, tether_roll_dot, alpha_cmd,
                   beta_cmd, airspeed_cmd, d_airspeed_d_loopangle, kitespeed,
                   &pqr_cmd_new);
  control_output->pqr_cmd_new_cross_pqr_out = pqr_cmd_new;
  //GetCrosswindTelemetry()->pqr_cmd_old = pqr_cmd;
  //GetCrosswindTelemetry()->pqr_cmd_new = pqr_cmd_new;
  //if (params->enable_new_pqr_cmd) {
  //  pqr_cmd = pqr_cmd_new;
  //}

  // Calculate pqr_cmd_dot.
  Vec3 pqr_cmd_dot;
  CalcCrosswindPqrDot(&state_est->wind_g.vector_f, &path_center_g, loop_angle,
                     path_radius, tether_roll, tether_roll_dot, alpha_cmd,
                     beta_cmd, airspeed_cmd, d_airspeed_d_loopangle, kitespeed,
                     &pqr_cmd_dot);
  //GetCrosswindTelemetry()->pqr_cmd_dot = pqr_cmd_dot;

  // Estimate aero forces.
  // TODO(tfricke): Currently this uses as-measured quantities;
  // instead, consider using as-predicted quantities, which may be
  // cleaner.
  Vec3 aero_force_b;
  CalcAeroForce(&state_est->apparent_wind.vector, g_sys.phys->rho,
                g_sys.wing->A, g_cont.simple_aero_model, &aero_force_b);
  // GetCrosswindTelemetry()->aero_force_b = aero_force_b;

  Deltas deltas;
  ThrustMoment thrust_moment;
  double lateral_gains[kNumCrosswindLateralInputs][kNumCrosswindLateralStates];

  CrosswindInnerStep(
      tether_roll_cmd, tether_roll, alpha_cmd,
      state_est->apparent_wind.sph_f.alpha, dCL_cmd, beta_cmd,
      state_est->apparent_wind.sph_f.beta, airspeed_cmd,
      state_est->apparent_wind.sph_f.v, &pqr_cmd, &state_est->pqr_f, loop_angle,
      &state_est->dcm_g2b, kite_accel_ff, &flags,
      &state->experimental_crosswind.current_config, &params->inner,
      &state->inner, lateral_gains, &deltas, &thrust_moment);

  control_output->thrust_moment_inner_out = thrust_moment;
  control_output->delta_inner_out = deltas;

  // Convert control variables to actuator commands.
  CrosswindOutputStep(params->loop_dir, loop_angle, flaring, &thrust_moment,
                     &deltas, state_est, &path_center_g, &params->output,
                     &state->output, control_output);

  //Backsolve for the crosswind lateral integrators.
  double delta_aileron_avail, delta_rudder_avail;
  UnmixFlaps(control_output->flaps, &deltas, params->output.flap_offsets,
            &delta_aileron_avail, &delta_rudder_avail);
  BacksolveLateralIntegrators(
     delta_aileron_avail, deltas.aileron, delta_rudder_avail, deltas.rudder,
     lateral_gains, &params->inner, &state->inner.int_tether_roll_error,
     &state->inner.int_beta_error);

  
  //GetCrosswindTelemetry()->int_tether_roll_error =
  //    state->inner.int_tether_roll_error;
  //GetCrosswindTelemetry()->int_beta_error = state->inner.int_beta_error;
}
