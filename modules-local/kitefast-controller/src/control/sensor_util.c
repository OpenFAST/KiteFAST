// This file is for stateless, math-like transformations to sensor
// data.

#include "sensor_util.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "coord_trans.h"
#include "geometry.h"
#include "mat3.h"
#include "quaternion.h"
#include "util.h"
#include "vec2.h"
#include "vec3.h"
#include "macros.h"
#include "ground_station_frame.h"
#include "perch_frame.h"
#include "sensor_types.h"
#include "system_params.h"
#include "system_types.h"
#include "tether_util.h"
#include "labels.h"

void GsGpsPosEcefToGsPosEcef(const Vec3 *gs_gps_pos_ecef,
                             const Vec3 *gs_gps_pos_g, double gs_heading,
                             Vec3 *gs_pos_ecef, Mat3 *dcm_ecef2g) {
  // We assume that the GS GPS antenna is sufficiently close to the ground
  // station origin that the NED-to-ECEF rotation obtained using gs_gps_pos_ecef
  // as the reference point is negligibly different from the rotation with
  // gs_pos_ecef as its reference point.
  CalcDcmEcefToG(gs_gps_pos_ecef, gs_heading, dcm_ecef2g);

  Vec3 r_ecef_gps_antenna;
  Mat3TransVec3Mult(dcm_ecef2g, gs_gps_pos_g, &r_ecef_gps_antenna);
  Vec3Sub(gs_gps_pos_ecef, &r_ecef_gps_antenna, gs_pos_ecef);
}

void ConvSigmaEcefToLocal(const Vec3 *sigma_X_ecef, const Vec3 *sigma_V_ecef,
                          const Mat3 *dcm_ecef2g, Vec3 *sigma_Xg,
                          Vec3 *sigma_Vg) {
  Mat3 cov_X_ecef = {{{sigma_X_ecef->x * sigma_X_ecef->x, 0.0, 0.0},
                      {0.0, sigma_X_ecef->y * sigma_X_ecef->y, 0.0},
                      {0.0, 0.0, sigma_X_ecef->z * sigma_X_ecef->z}}};
  Mat3 cov_V_ecef = {{{sigma_V_ecef->x * sigma_V_ecef->x, 0.0, 0.0},
                      {0.0, sigma_V_ecef->y * sigma_V_ecef->y, 0.0},
                      {0.0, 0.0, sigma_V_ecef->z * sigma_V_ecef->z}}};
  Mat3 cov_Xg, cov_Vg;

  RotateCov(&cov_X_ecef, dcm_ecef2g, &cov_Xg);
  RotateCov(&cov_V_ecef, dcm_ecef2g, &cov_Vg);

  // TODO(kennyjensen): This is an approximation until we figure out
  // the appropriate thing to do with the covariance matrix in the
  // state estimator.
  sigma_Xg->x = Sqrt(cov_Xg.d[0][0]);
  sigma_Xg->y = Sqrt(cov_Xg.d[1][1]);
  sigma_Xg->z = Sqrt(cov_Xg.d[2][2]);
  sigma_Vg->x = Sqrt(cov_Vg.d[0][0]);
  sigma_Vg->y = Sqrt(cov_Vg.d[1][1]);
  sigma_Vg->z = Sqrt(cov_Vg.d[2][2]);
}

void TetherForceSphToCart(const TetherForceSph *sph, Vec3 *cart) {
  cart->x = sph->tension * cos(sph->roll) * sin(sph->pitch);
  cart->y = sph->tension * -sin(sph->roll);
  cart->z = sph->tension * cos(sph->roll) * cos(sph->pitch);
}

void TetherForceCartToSph(const Vec3 *cart, TetherForceSph *sph) {
  sph->tension = Vec3Norm(cart);
  sph->roll = atan2(-cart->y, Vec3XzNorm(cart));
  sph->pitch = atan2(cart->x, cart->z);
}

// Determines the z-component of force measured by a loadcell in the loadcell's
// frame.
//
// If we assume that the bridles are rigid and under tension, then the force
// vector on a given loadcell must:
//   (1) Be in the same direction as the loadcell-to-knot vector.
//   (2) Form a fixed angle phi (the "bridle angle") with the
//       loadcell-to-other-loadcell vector.
//
// In local loadcell coordinates, let F = [f1, f2, f3]^T be the force vector,
// and x = [x1, x2, x3]^T be the loadcell-to-other-loadcell vector. Note that f1
// and f2 are measured directly by the loadcell. Then
//     F * x = |F| |x| cos(phi).
// Squaring both sides yields a quadratic equation in f3.
//
// For a full description of the method, see
//     http://go/makani-doc/controls/tension_estimation.md.
//
// Args:
//   force_x: The x-component of the force in the loadcell's frame.
//   force_y: The y-component of the force in the loadcell's frame.
//   cos_bridle_angle: Cosine of the bridle angle, the angle between the
//       loadcell-to-knot and loadcell-to-other-loadcell vectors.
//   this_to_other: Vector from this loadcell to the other loadcell,
//       in this loadcell's frame.
static double LoadcellForceBZ(double force_x, double force_y,
                              double cos_bridle_angle,
                              const Vec3 *this_to_other) {
  double s1 = force_x * this_to_other->x + force_y * this_to_other->y;
  double s2 = Square(force_x) + Square(force_y);
  double s3_squared = Square(Vec3Norm(this_to_other) * cos_bridle_angle);

  double a = Square(this_to_other->z) - s3_squared;
  double b = 2.0 * s1 * this_to_other->z;
  double c = Square(s1) - s2 * s3_squared;

  // This assertion is stronger than needed, but if a starts getting small,
  // someone should give the algebra some more careful thought. It's value is
  // determined entirely by configuration parameters.
  assert(fabs(a) >= 1.0);

  if (fabs(a) < DBL_TOL) {
    return 0.0;
  } else {
    return (-b - Sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
  }
}

static double CalcCosAngle(const Vec3 *u, const Vec3 *v) {
  double u_norm = Vec3Norm(u);
  double v_norm = Vec3Norm(v);
  if (u_norm < DBL_TOL || v_norm < DBL_TOL) {
    assert(false);
    return 0;
  }
  return Vec3Dot(u, v) / (u_norm * v_norm);
}

// Computes the force exerted on a loadcell by its bridle in the body frame.
static void CalcLoadcellForceB(const Vec2 *channel_measurements,
                               const Vec3 *this_bridle_pos_b,
                               const Vec3 *other_bridle_pos_b,
                               const Vec3 *knot0_pos_b,
                               const LoadcellParams *loadcell_params,
                               Vec3 *force_b) {
  // Incorporate the force components measured by the loadcell.
  Vec2 force_local_xy;
  Mat2Vec2Mult(&loadcell_params->channels_to_force_local_xy,
               channel_measurements, &force_local_xy);

  Vec3 this_to_other_b, this_to_other_local;
  Vec3Sub(other_bridle_pos_b, this_bridle_pos_b, &this_to_other_b);
  Mat3TransVec3Mult(&loadcell_params->dcm_loadcell2b, &this_to_other_b,
                    &this_to_other_local);
  Vec3 this_to_knot0_b;
  Vec3Sub(knot0_pos_b, this_bridle_pos_b, &this_to_knot0_b);

  Vec3 force_local = {
      force_local_xy.x, force_local_xy.y,
      LoadcellForceBZ(force_local_xy.x, force_local_xy.y,
                      CalcCosAngle(&this_to_other_b, &this_to_knot0_b),
                      &this_to_other_local)};

  Mat3Vec3Mult(&loadcell_params->dcm_loadcell2b, &force_local, force_b);
}

COMPILE_ASSERT(NUM_LOADCELL_CHANNELS == 2, NUM_LOADCELL_CHANNELS_must_be_2);

void LoadcellsToTetherForce(const double loadcells[],
                            const WingParams *wing_params,
                            const LoadcellParams loadcell_params[], Vec3 *cart,
                            TetherForceSph *sph, Vec3 *port_force_b,
                            Vec3 *star_force_b) {
  assert(loadcells != NULL && wing_params != NULL && loadcell_params != NULL &&
         cart != NULL && sph != NULL);

  const Vec3 *port_pos_b = &wing_params->bridle_pos[kBridlePort];
  const Vec3 *star_pos_b = &wing_params->bridle_pos[kBridleStar];

  // "knot0" refers to the position of the knot at zero pitch and zero roll.
  Vec3 knot0_pos_b = {
      (port_pos_b->x + star_pos_b->x) / 2.0, wing_params->bridle_y_offset,
      wing_params->bridle_rad + (port_pos_b->z + star_pos_b->z) / 2.0};

  const Vec2 port_channels = {loadcells[kLoadcellSensorPort0],
                              loadcells[kLoadcellSensorPort1]};
  CalcLoadcellForceB(&port_channels, port_pos_b, star_pos_b, &knot0_pos_b,
                     &loadcell_params[kBridlePort], port_force_b);

  const Vec2 star_channels = {loadcells[kLoadcellSensorStarboard0],
                              loadcells[kLoadcellSensorStarboard1]};
  CalcLoadcellForceB(&star_channels, star_pos_b, port_pos_b, &knot0_pos_b,
                     &loadcell_params[kBridleStar], star_force_b);

  Vec3Add(port_force_b, star_force_b, cart);
  TetherForceCartToSph(cart, sph);
}

double GsHeadingPerchHeadingToPerchAzi(double gs_heading,
                                       double perch_heading) {
  assert(0.0 <= gs_heading && gs_heading < 2.0 * PI);
  assert(0.0 <= perch_heading && perch_heading < 2.0 * PI);

  return remainder(perch_heading - gs_heading, 2.0 * PI);
}

double WinchPosToDrumAngle(double winch_pos, const WinchParams *params) {
  return winch_pos / params->r_drum;
}

// Incidence angles follow conventions from Etkin (pg. 11):
//
//   alpha = arctan(w / u)
//   beta  = arcsin(v / |V|)
//
// where (u, v, w) are the cartesian components of the body velocity,
// i.e. opposite the apparent wind.
void ApparentWindSphToCart(const ApparentWindSph *sph, Vec3 *cart) {
  cart->x = -sph->v * cos(sph->alpha) * cos(sph->beta);
  cart->y = -sph->v * sin(sph->beta);
  cart->z = -sph->v * sin(sph->alpha) * cos(sph->beta);
}

void ApparentWindCartToSph(const Vec3 *cart, ApparentWindSph *sph) {
  sph->v = Vec3Norm(cart);
  sph->alpha = atan2(-cart->z, -cart->x);
  // The following is equivalent to asin(-cart->y / sph->v) but
  // avoids the potential for division by zero.
  sph->beta = atan2(-cart->y, Vec3XzNorm(cart));
}

void PitotToApparentWindSph(const PitotDifferentialData *diff, const Vec3 *pqr,
                            const PitotParams *params, ApparentWindSph *sph) {
  // The current Pitot tube has a hemispherical tip, and for small
  // angles the expected pressure differences measured between the
  // Pitot ports can be approximated by the pressure distribution on a
  // sphere giving:
  //
  // dyn_press =
  //     dynamic_pressure * (1 - 9/4 * (1 - cos(alpha)^2 * cos(beta)^2)),
  // alpha_press =
  //     dynamic_pressure * (9/2) * sin(2 * port_angle) * cos(beta)^2
  //     * cos(alpha) * sin(alpha),
  // beta_press =
  //     dynamic_pressure * (9/2) * sin(2 * port_angle) * cos(alpha)
  //     * cos(beta) * sin(beta).
  //
  // The wind vector has components:
  //
  //   u = V * cos(alpha) * cos(beta),
  //   v = V * sin(beta),
  //   w = V * cos(beta) * sin(alpha).
  //
  // If we write c = rho * 9/4 * sin(2 * port_angle), then:
  //
  //   a = alpha_press / c = u * w
  //   b = beta_press / c = u * v
  //   d = dyn_press / (0.5 * rho) = u^2 - 5/4 * (v^2 + w^2)
  //                               = u^2 - 5/4 * (a^2 + b^2) / u^2.
  //
  // We can thus solve a quadratic equation for u^2, and then use this
  // to find v and w.
  double c = g_sys.phys->rho * (9.0 / 4.0) * sin(2.0 * params->port_angle);
  double a = diff->alpha_press / c;
  double b = diff->beta_press / c;
  double d = diff->dyn_press / (0.5 * g_sys.phys->rho);

  Vec3 apparent_wind_p;
  apparent_wind_p.x = -Sqrt(0.5 * (d + Sqrt(d * d + 5.0 * (a * a + b * b))));
  apparent_wind_p.y = b / fmin(-1.0, apparent_wind_p.x);
  apparent_wind_p.z = a / fmin(-1.0, apparent_wind_p.x);

  // Transform measurement to body coordinates.
  Vec3 apparent_wind_b;
  Mat3TransVec3Mult(&params->dcm_b2p, &apparent_wind_p, &apparent_wind_b);

  // Correct for offset of Pitot tube.
  Vec3 offset_correction_b;
  Vec3Cross(&params->pos, pqr, &offset_correction_b);
  Vec3Sub(&apparent_wind_b, &offset_correction_b, &apparent_wind_b);

  ApparentWindCartToSph(&apparent_wind_b, sph);
}

void WindWsToWindG(const Vec3 *wind_ws, double perch_azi_angle,
                   double perch_azi_omega, const WindSensorParams *params,
                   Vec3 *wind_g) {
  if (params->on_perch) {
    // Rotate the measured wind velocity into the perch frame.
    Vec3 wind_p;
    Mat3TransVec3Mult(&params->dcm_parent2ws, wind_ws, &wind_p);

    // Remove the apparent wind from the rotating perch.
    Vec3 omega_g2p = {0.0, 0.0, perch_azi_omega};
    Vec3 wind_sensor_vel_p;
    Vec3Cross(&omega_g2p, &params->pos_parent, &wind_sensor_vel_p);
    Vec3Add(&wind_p, &wind_sensor_vel_p, &wind_p);

    // Rotate wind from perch to ground-station frame.
    RotPToG(&wind_p, perch_azi_angle, wind_g);
  } else {
    // The parent frame is the ground station frame in this case.
    Mat3TransVec3Mult(&params->dcm_parent2ws, wind_ws, wind_g);
  }
}

// TODO(kennyjensen): This does not account for the offset between the
// azimuth axis and the elevation axis.  Also, assumes tether length
// is measured from GSG origin.
static void XgsgToXwd(const GsgData *gsg, const Vec3 *X_gsg, Vec3 *X_wd) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  assert(GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
         GetSystemParams()->gs_model == kGroundStationModelTopHat);
  Mat3 dcm_wd2gsg;
  AngleToDcm(PI + gsg->azi, gsg->ele, 0.0, kRotationOrderZyx, &dcm_wd2gsg);
  Mat3TransVec3Mult(&dcm_wd2gsg, X_gsg, X_wd);
  Vec3Add(X_wd, &g_sys.perch->gsg_pos_wd, X_wd);
}

void XgsgToXp(const GsgData *gsg, const Vec3 *X_gsg, double drum_angle,
              Vec3 *Xp) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  assert(GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
         GetSystemParams()->gs_model == kGroundStationModelTopHat);
  Vec3 X_wd;
  XgsgToXwd(gsg, X_gsg, &X_wd);
  WdToP(&X_wd, drum_angle, Xp);
}

static void XwdToGsg(const Vec3 *X_wd, GsgData *gsg) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  assert(GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
         GetSystemParams()->gs_model == kGroundStationModelTopHat);
  Vec3 tether_direction_wd;
  Vec3Sub(X_wd, &g_sys.perch->gsg_pos_wd, &tether_direction_wd);
  TetherDirectionWdToGsgAziEle(&tether_direction_wd, &gsg->azi, &gsg->ele);
}

void XpToGsg(const Vec3 *Xp, double drum_angle, GsgData *gsg) {
  // TODO(b/109812013): Rethink GSG frame with GSv2.
  assert(GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
         GetSystemParams()->gs_model == kGroundStationModelTopHat);
  Vec3 X_wd;
  PToWd(Xp, drum_angle, &X_wd);
  XwdToGsg(&X_wd, gsg);
}

void TetherDirectionWdToGsgAziEle(const Vec3 *tether_direction_wd, double *azi,
                                  double *ele) {
  *azi =
      Wrap(atan2(tether_direction_wd->y, tether_direction_wd->x) - PI, -PI, PI);
  *ele = atan2(-tether_direction_wd->z, Vec3XyNorm(tether_direction_wd));
}

bool IsLevelwindEngaged(double drum_angle, const LevelwindParams *params) {
  return drum_angle < params->pulley_engage_drum_angle;
}

void LevelwindToXp(double payout, double drum_angle, double levelwind_ele,
                   double tension, const LevelwindParams *params, Vec3 *Xp) {
  // free_len is the distance between the levelwind and the wing's
  // center.
  double free_len = payout + g_sys.wing->bridle_rad +
                    g_sys.wing->bridle_pos[kBridlePort].z +
                    params->pivot_axis_to_bridle_point;
  Vec3 X_lw = {0.0, -free_len * sin(params->azimuth_offset),
               -free_len * cos(params->azimuth_offset)};

  // Perform the catenary calculations in a new "catenary frame"
  // that is parallel to the perch-frame but with the same origin
  // as the levelwind-frame.
  Vec3 X_cat_no_cat, X_cat_including_cat;
  RotLwToP(&X_lw, levelwind_ele, &X_cat_no_cat);  // lw to catenary frame
  IncludeCatenary(&X_cat_no_cat, tension, free_len, &X_cat_including_cat);
  RotPToLw(&X_cat_including_cat, levelwind_ele, &X_lw);  // catenary to lw frame
  LwToP(&X_lw, levelwind_ele, drum_angle, Xp);
}

void IncludeCatenary(const Vec3 *X_no_cat, double tension, double free_len,
                     Vec3 *X_including_cat) {
  Vec3 X = *X_no_cat;  // Ensure safe for argument reuse.
  double xy_no_cat = Vec3XyNorm(&X);
  TetherParabola tp;
  TensionAndAngleToParabola(fmax(tension, 100.0), 0.0, atan2(-X.z, xy_no_cat),
                            g_sys.phys->g * g_sys.tether->linear_density, &tp);
  X.z = -ParabolaHeight(&tp, xy_no_cat);
  Vec3Scale(Vec3Normalize(&X, &X), free_len, &X);
  // TODO(chubb): Could clean this up by adding arc-length based
  // functions to the tether_util library (easier said than done).

  *X_including_cat = X;
}

double PressureToAltitude(double pressure, const PhysParams *params) {
  return (params->P_atm - pressure) / (params->rho * params->g);
}

double CalcDryAirDensity(double pressure_pascals, double temperature_celcius) {
  const PhysParams *const phys = g_sys.phys;

  // Absolute temperature [Kelvin].
  const double T = temperature_celcius + 273.15;

  // Density of dry air [kg/m^3].
  const double rho_dry_air = pressure_pascals / (phys->R_dry_air * T);

  return rho_dry_air;
}

// Saturation partial pressure of water vapor [Pa] using Tetens' Formula,
// given temperature in degrees Celcius.
static double CalcSaturationVaporPressure(double temperature_celcius) {
  const double c[] = {6.1078, 7.5, 237.3};
  const double p_sat = 100.0 * c[0] * Exp10((c[1] * temperature_celcius) /
                                            (c[2] + temperature_celcius));
  return p_sat;
}

double CalcAirDensity(double pressure_pascals, double temperature_celcius,
                      double relative_humidity, bool *valid) {
  if (valid != NULL) {
    // Sanity check that the arguments are in a meteorologically
    // plausible range; this function is actually correct over a much
    // larger range of inputs.
    *valid = (0.0 <= relative_humidity && relative_humidity <= 1.0) &&
             (-40.0 <= temperature_celcius && temperature_celcius <= 70.0) &&
             (500e2 <= pressure_pascals && pressure_pascals <= 1500e2);
  }

  const PhysParams *const phys = g_sys.phys;

  // Absolute temperature [Kelvin].
  const double T = temperature_celcius + 273.15;

  // Saturation partial pressure of water vapor [Pa].
  const double p_sat = CalcSaturationVaporPressure(temperature_celcius);

  // Partial pressure of water vapor [Pa].
  const double p_v = relative_humidity * p_sat;

  // Partial pressure of dry air [Pa].
  const double p_d = pressure_pascals - p_v;

  // Density [kg/m^3], accounting for effect of humidity.
  const double rho =
      p_d / (phys->R_dry_air * T) + p_v / (phys->R_water_vapor * T);

  return rho;
}
