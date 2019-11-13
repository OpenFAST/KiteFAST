#include "control/crosswind/crosswind_util.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>

#include "common/c_math/geometry.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "control/crosswind/crosswind_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"

double CalcCurvature(const Vec3 *acc_b, const Vec3 *vel_b) {
  assert(acc_b != NULL && vel_b != NULL);

  Vec3 acc_cross_vel_b;
  Vec3Cross(acc_b, vel_b, &acc_cross_vel_b);
  // TODO(kennyjensen): Remove arbitrary 1e-5.
  double norm_vel_b = Vec3NormBound(vel_b, 1e-5);
  return Sign(acc_b->y) * Vec3Norm(&acc_cross_vel_b) /
         (norm_vel_b * norm_vel_b * norm_vel_b);
}

double CurvatureToTetherRoll(double aero_curvature, double lift_coeff,
                             double side_force_coeff) {
  assert(lift_coeff > 0.0);

  // Forces on the wing must accelerate both the wing and the tether.
  // The effective mass of the tether as if it were concentrated at
  // the wing is derived by modeling the tether as a rigid rod of
  // uniform density.
  const double effective_mass =
      g_sys.wing->m + g_sys.tether->linear_density * g_sys.tether->length / 3.0;

  double acc_coeff =
      aero_curvature * effective_mass / (0.5 * g_sys.phys->rho * g_sys.wing->A);
  return Saturate(
      (side_force_coeff - acc_coeff) / fmax(lift_coeff, DBL_EPSILON), -PI / 2.0,
      PI / 2.0);
}

double CalcLoopKinematics(const Vec3 *wind_g, const Vec3 *path_center_g,
                          double loop_angle, double loop_radius,
                          double airspeed, double d_airspeed_d_loopangle,
                          Vec3 *vel_g, Vec3 *acc_g) {
  // Get the loop geometry.
  Vec3 pos_g, vel_g_hat, acc_g_perp_hat;
  CalcLoopGeometry(path_center_g, loop_angle, loop_radius, &pos_g, &vel_g_hat,
                   &acc_g_perp_hat);

  // Verify that these are indeed unit vectors.
  assert(fabs(Vec3Norm(&vel_g_hat) - 1.0) < 1e-6);
  assert(fabs(Vec3Norm(&acc_g_perp_hat) - 1.0) < 1e-6);

  // Verify that the given acceleration is indeed centripetal.
  assert(fabs(Vec3Dot(&vel_g_hat, &acc_g_perp_hat)) < 1e-6);

  // Compute parallel and perpendicular components of the wind.
  //
  // It's useful to split up the wind vector into components that are
  // parallel and perpendicular to the kite velocity.  These obey the
  // following scalar equation:
  //
  //     wind^2 = wind_par^2 + wind_perp^2

  const double wind = Vec3Norm(wind_g);
  const double wind_par = Vec3Dot(&vel_g_hat, wind_g);
  const double wind_perp = Sqrt(Square(wind) - Square(wind_par));

  // Calculate the kite speed we need to achieve a given airspeed.
  //
  // Given wind_par and wind_perp, this becomes a purely scalar
  // equation:
  //
  //     airspeed^2 = (kitespeed - wind_par)^2 + wind_perp^2

  const double kitespeed =
      wind_par + Sqrt(Square(airspeed) - Square(wind_perp));

  // Calculate the rate of change of wind_par.
  //
  // We assume that the wind vector is constant.

  const double wind_par_dot =
      Vec3Dot(&acc_g_perp_hat, wind_g) * kitespeed / loop_radius;

  // Calculate the rate of change of airspeed.
  //
  // This is simply the chain rule:
  //
  //     airspeed_dot = (d airspeed / d loop angle) * (d loop angle / d time)
  //
  // where (d loop angle / d time) is -kitespeed / loop_radius, with
  // the minus sign appearing because we fly around the loop in the
  // direction of decreasing loop angle.

  const double airspeed_dot =
      d_airspeed_d_loopangle * (-kitespeed / loop_radius);

  // Finally we can compute the required acceleration to track the
  // changing airspeed command.
  //
  //     airspeed^2 = (kitespeed - wind_par)^2 + wind_perp^2
  //     airspeed^2 = (kitespeed - wind_par)^2 + wind^2 - wind_par^2
  //
  // Now take the derivative
  //
  //    2 airspeed airspeed_dot =
  //                 2 (kitespeed - wind_par) * (kitespeed_dot - wind_par_dot)
  //               + 2 wind * wind_dot - 2 wind_par * wind_par_dot
  //
  // which can be solved for kitespeed_dot = kiteaccel.

  const double kiteaccel =
      (airspeed * airspeed_dot + kitespeed * wind_par_dot) /
      (kitespeed - wind_par);

  if (vel_g != NULL) {
    Vec3Scale(&vel_g_hat, kitespeed, vel_g);
  }

  if (acc_g != NULL) {
    Vec3LinComb(kiteaccel, &vel_g_hat, Square(kitespeed) / loop_radius,
                &acc_g_perp_hat, acc_g);
  }

  return kiteaccel;
}

void CalcLoopGeometry(const Vec3 *path_center_g, double loop_angle,
                      double loop_radius, Vec3 *pos_g, Vec3 *vel_hat_g,
                      Vec3 *acc_perp_hat_g) {
  Vec3 wing_pos_cw = {.x = 0.0, .y = cos(loop_angle), .z = sin(loop_angle)};
  Vec3 wing_vel_cw = {.x = 0.0, .y = sin(loop_angle), .z = -cos(loop_angle)};
  Vec3 wing_acc_cw = {.x = 0.0, .y = -cos(loop_angle), .z = -sin(loop_angle)};

  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  Vec3Scale(&wing_pos_cw, loop_radius, &wing_pos_cw);
  Mat3TransVec3Mult(&dcm_g2cw, &wing_pos_cw, pos_g);

  if (pos_g != NULL) {
    Vec3Add(pos_g, path_center_g, pos_g);
  }
  if (vel_hat_g != NULL) {
    Mat3TransVec3Mult(&dcm_g2cw, &wing_vel_cw, vel_hat_g);
  }
  if (acc_perp_hat_g != NULL) {
    Mat3TransVec3Mult(&dcm_g2cw, &wing_acc_cw, acc_perp_hat_g);
  }
}

double CalcLoopAngle(const Vec3 *path_center_g, const Vec3 *wing_pos_g) {
  assert(path_center_g != NULL && wing_pos_g != NULL);

  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  Vec3 wing_pos_cw;
  Vec3 wing_point_vec;
  Vec3Sub(wing_pos_g, path_center_g, &wing_point_vec);
  Mat3Vec3Mult(&dcm_g2cw, &wing_point_vec, &wing_pos_cw);
  return PI - atan2(wing_pos_cw.z, -wing_pos_cw.y);
}

void CalcCrosswindAttitude(const Vec3 *wind_g, const Vec3 *path_center_g,
                           double loop_angle, double path_radius,
                           double tether_roll, double alpha_cmd,
                           double beta_cmd, double airspeed_cmd,
                           double d_airspeed_d_loopangle, Mat3 *dcm_g2b) {
  // Calculate required crosswind geometry and kinematics.
  Vec3 wing_vel_g;
  CalcLoopKinematics(wind_g, path_center_g, loop_angle, path_radius,
                     airspeed_cmd, d_airspeed_d_loopangle, &wing_vel_g, NULL);
  Mat3 dcm_g2t;
  CalcDcmGToT(path_center_g, loop_angle, &dcm_g2t);

  // Roll out of the crosswind plane.  TODO(tfricke): Here we assume
  // that "tether roll angle" is about the loop tangent, which is not
  // exactly correct.  We may wish to resolve this by redefining the
  // control variable from tether roll angle to tangent roll angle.
  {
    Mat3 dcm_roll;
    const double half_cone_angle = asin(path_radius / g_sys.tether->length);
    AngleToDcm(0.0, 0.0, half_cone_angle - tether_roll, kRotationOrderZyx,
               &dcm_roll);
    Mat3Mat3Mult(&dcm_roll, &dcm_g2t, &dcm_g2t);
  }

  // Calculate the apparent wind in the tangent frame and in the body
  // frame (given alpha and beta commands).
  Vec3 apparent_wind_b, apparent_wind_t;
  {
    Vec3 apparent_wind_g;
    Vec3Sub(wind_g, &wing_vel_g, &apparent_wind_g);
    Mat3Vec3Mult(&dcm_g2t, &apparent_wind_g, &apparent_wind_t);

    ApparentWindSph apparent_wind_sph = {
        .alpha = alpha_cmd, .beta = beta_cmd, .v = 1.0};
    ApparentWindSphToCart(&apparent_wind_sph, &apparent_wind_b);
  }

  // Calculate the rotation from the tangent frame to the body frame
  // to get the desired apparent wind vector.
  Mat3 dcm_t2b;
  Vec3Vec3ToDcm(&apparent_wind_t, &apparent_wind_b, &dcm_t2b);
  Mat3Mat3Mult(&dcm_t2b, &dcm_g2t, dcm_g2b);
}

void CalcCrosswindPqr(const Vec3 *wind_g, const Vec3 *path_center_g,
                      double loop_angle, double path_radius, double tether_roll,
                      double tether_roll_dot, double alpha_cmd, double beta_cmd,
                      double airspeed_cmd, double d_airspeed_d_loopangle,
                      double kitespeed, Vec3 *pqr) {
  // Use central difference to compute the differential rotation
  // between (loop_angle-h) and (loop_angle+h).
  const double d_loopangle = 1e-3;  // Radians.
  const double d_loopangle_d_time = -kitespeed / path_radius;
  const double d_roll_d_loopangle = tether_roll_dot / d_loopangle_d_time;

  Mat3 dcm_g2b_m, dcm_g2b_p;
  CalcCrosswindAttitude(
      wind_g, path_center_g, loop_angle - d_loopangle, path_radius,
      tether_roll - d_roll_d_loopangle * d_loopangle, alpha_cmd, beta_cmd,
      airspeed_cmd - d_airspeed_d_loopangle * d_loopangle,
      d_airspeed_d_loopangle, &dcm_g2b_m);

  CalcCrosswindAttitude(
      wind_g, path_center_g, loop_angle + d_loopangle, path_radius,
      tether_roll + d_roll_d_loopangle * d_loopangle, alpha_cmd, beta_cmd,
      airspeed_cmd + d_airspeed_d_loopangle * d_loopangle,
      d_airspeed_d_loopangle, &dcm_g2b_p);

  // Now we have a DCM corresponding to a differential rotation; here
  // we turn it into a pqr vector.  With a differential rotation, the
  // Euler angles act like a vector.
  Mat3 dcm_m2p;
  Mat3Mult(&dcm_g2b_p, kNoTrans, &dcm_g2b_m, kTrans, &dcm_m2p);
  DcmToAngle(&dcm_m2p, kRotationOrderZyx, &pqr->z, &pqr->y, &pqr->x);

  Vec3Scale(pqr, d_loopangle_d_time / (2 * d_loopangle), pqr);
}

// TODO(tfricke): Combine this with CalcCrosswindPqr to reduce the
// number of evaluations of CalcCrosswindAttitude from four to three.
void CalcCrosswindPqrDot(const Vec3 *wind_g, const Vec3 *path_center_g,
                         double loop_angle, double path_radius,
                         double tether_roll, double tether_roll_dot,
                         double alpha_cmd, double beta_cmd, double airspeed_cmd,
                         double d_airspeed_d_loopangle, double kitespeed,
                         Vec3 *pqr_cmd_dot) {
  // Use central difference to compute the differential rotation
  // between (loop_angle-h) and (loop_angle+h).
  const double d_loopangle = 1e-2;  // Radians.
  const double d_loopangle_d_time = -kitespeed / path_radius;
  const double d_roll_d_loopangle = tether_roll_dot / d_loopangle_d_time;

  Vec3 pqr_m, pqr_p;
  CalcCrosswindPqr(wind_g, path_center_g, loop_angle - d_loopangle, path_radius,
                   tether_roll - d_roll_d_loopangle * d_loopangle,
                   tether_roll_dot, alpha_cmd, beta_cmd,
                   airspeed_cmd - d_airspeed_d_loopangle * d_loopangle,
                   d_airspeed_d_loopangle, kitespeed, &pqr_m);

  CalcCrosswindPqr(wind_g, path_center_g, loop_angle + d_loopangle, path_radius,
                   tether_roll + d_roll_d_loopangle * d_loopangle,
                   tether_roll_dot, alpha_cmd, beta_cmd,
                   airspeed_cmd + d_airspeed_d_loopangle * d_loopangle,
                   d_airspeed_d_loopangle, kitespeed, &pqr_p);

  const double dt = d_loopangle / d_loopangle_d_time;
  Vec3Sub(&pqr_p, &pqr_m, pqr_cmd_dot);
  Vec3Scale(pqr_cmd_dot, 1.0 / (2.0 * dt), pqr_cmd_dot);
}

double HarmonicIntegrator(double u, double angle, double d_angle, double ki,
                          double int_max, double state[]) {
  state[0] += ki * sin(angle) * u * d_angle;
  state[1] += ki * cos(angle) * u * d_angle;

  state[0] = Saturate(state[0], -int_max, int_max);
  state[1] = Saturate(state[1], -int_max, int_max);

  return state[0] * sin(angle) + state[1] * cos(angle);
}
