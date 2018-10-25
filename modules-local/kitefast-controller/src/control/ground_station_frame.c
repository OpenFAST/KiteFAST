#include "ground_station_frame.h"

#include <math.h>

#include "coord_trans.h"
#include "geometry.h"
#include "mat3.h"
#include "util.h"
#include "vec3.h"
#include "system_params.h"
#include "system_types.h"

// Conversions between North-East-Down and Ground station.

const Mat3 *CalcDcmNedToG(double gs_heading, Mat3 *dcm_ned2g) {
  return AngleToDcm(gs_heading, 0.0, 0.0, kRotationOrderZyx, dcm_ned2g);
}

const Vec3 *NedToG(const Vec3 *X_ned, double gs_heading, Vec3 *Xg) {
  Mat3 dcm_ned2g;
  CalcDcmNedToG(gs_heading, &dcm_ned2g);
  return PoseTransform(&dcm_ned2g, &kVec3Zero, X_ned, Xg);
}

const Vec3 *RotNedToG(const Vec3 *V_ned, double gs_heading, Vec3 *Vg) {
  Mat3 dcm_ned2g;
  CalcDcmNedToG(gs_heading, &dcm_ned2g);
  return Mat3Vec3Mult(&dcm_ned2g, V_ned, Vg);
}

const Vec3 *GToNed(const Vec3 *Xg, double gs_heading, Vec3 *X_ned) {
  Mat3 dcm_ned2g;
  CalcDcmNedToG(gs_heading, &dcm_ned2g);
  return InversePoseTransform(&dcm_ned2g, &kVec3Zero, Xg, X_ned);
}

const Vec3 *RotGToNed(const Vec3 *Vg, double gs_heading, Vec3 *V_ned) {
  Mat3 dcm_ned2g;
  CalcDcmNedToG(gs_heading, &dcm_ned2g);
  return Mat3TransVec3Mult(&dcm_ned2g, Vg, V_ned);
}

// Conversions between ECEF and Ground-station.

const Mat3 *CalcDcmEcefToG(const Vec3 *gs_pos, double gs_heading,
                           Mat3 *dcm_ecef2g) {
  Mat3 dcm_ned2g, dcm_ecef2ned;
  return Mat3Mat3Mult(CalcDcmNedToG(gs_heading, &dcm_ned2g),
                      CalcDcmEcefToNed(gs_pos, &dcm_ecef2ned), dcm_ecef2g);
}

const Vec3 *EcefToG(const Vec3 *X_ecef, const Vec3 *gs_pos, double gs_heading,
                    Vec3 *Xg) {
  Vec3 X_ned;
  return NedToG(EcefToNed(X_ecef, gs_pos, &X_ned), gs_heading, Xg);
}

const Vec3 *RotEcefToG(const Vec3 *V_ecef, const Vec3 *gs_pos,
                       double gs_heading, Vec3 *Vg) {
  Vec3 V_ned;
  return RotNedToG(RotEcefToNed(V_ecef, gs_pos, &V_ned), gs_heading, Vg);
}

const Vec3 *GToEcef(const Vec3 *Xg, const Vec3 *gs_pos, double gs_heading,
                    Vec3 *X_ecef) {
  Vec3 X_ned;
  return NedToEcef(GToNed(Xg, gs_heading, &X_ned), gs_pos, X_ecef);
}

const Vec3 *RotGToEcef(const Vec3 *Vg, const Vec3 *gs_pos, double gs_heading,
                       Vec3 *V_ecef) {
  Vec3 V_ned;
  return RotNedToEcef(RotGToNed(Vg, gs_heading, &V_ned), gs_pos, V_ecef);
}

double VecGToAzimuth(const Vec3 *pos_g) {
  return Wrap(atan2(pos_g->y, pos_g->x) +
                  GetSystemParams()->ground_station.azi_ref_offset,
              -PI, PI);
}

void CylToVecG(double azi, double r, double z, Vec3 *pos_g) {
  CylToCart(azi - GetSystemParams()->ground_station.azi_ref_offset, r, z,
            pos_g);
}

void SphToVecG(double azi, double ele, double r, Vec3 *pos_g) {
  // Negate the elevation because positive elevation corresponds to the
  // -z-direction.
  SphToCart(azi - GetSystemParams()->ground_station.azi_ref_offset, -ele, r,
            pos_g);
}

void VecGToSph(const Vec3 *pos_g, double *azi, double *ele, double *r) {
  CartToSph(pos_g, azi, ele, r);
  *azi = Wrap(*azi + GetSystemParams()->ground_station.azi_ref_offset, -PI, PI);
  // Negate the elevation because positive elevation corresponds to the
  // -z-direction.
  *ele = -*ele;
}
