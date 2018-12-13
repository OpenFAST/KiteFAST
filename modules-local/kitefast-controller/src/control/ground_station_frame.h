#ifndef CONTROL_GROUND_STATION_FRAME_H_
#define CONTROL_GROUND_STATION_FRAME_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Conversion between North-East-Down and Ground station.

const Mat3 *CalcDcmNedToG(double gs_heading, Mat3 *dcm_ned2g);
const Vec3 *NedToG(const Vec3 *X_ned, double gs_heading, Vec3 *Xg);
const Vec3 *RotNedToG(const Vec3 *V_ned, double gs_heading, Vec3 *Vg);
const Vec3 *GToNed(const Vec3 *Xg, double gs_heading, Vec3 *X_ned);
const Vec3 *RotGToNed(const Vec3 *Vg, double gs_heading, Vec3 *V_ned);

// Conversion between ECEF and Ground-station.

const Mat3 *CalcDcmEcefToG(const Vec3 *gs_pos, double gs_heading,
                           Mat3 *dcm_ecef2g);
const Vec3 *EcefToG(const Vec3 *X_ecef, const Vec3 *gs_pos, double gs_heading,
                    Vec3 *Xg);
const Vec3 *RotEcefToG(const Vec3 *V_ecef, const Vec3 *gs_pos,
                       double gs_heading, Vec3 *Vg);
const Vec3 *GToEcef(const Vec3 *Xg, const Vec3 *gs_pos, double gs_heading,
                    Vec3 *X_ecef);
const Vec3 *RotGToEcef(const Vec3 *Vg, const Vec3 *gs_pos, double gs_heading,
                       Vec3 *V_ecef);

// Conversion between the g-frame and cylindrical or spherical coordinates.
// The definition of azimuth is system-dependent -- see
// GroundStationParams.azi_offset.
//
// NOTE: We'd more naturally have PosGToCyl in place of VecGToAzimuth, but as it
// stands, we would never use the other outputs.
double VecGToAzimuth(const Vec3 *pos_g);
double VecGToElevation(const Vec3 *pos_g);
void CylToVecG(double azi, double r, double z, Vec3 *pos_g);
void VecGToCyl(const Vec3 *pos_g, double *azi, double *r, double *z);
void SphToVecG(double azi, double ele, double r, Vec3 *pos_g);
void VecGToSph(const Vec3 *pos_g, double *azi, double *ele, double *r);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_GROUND_STATION_FRAME_H_
