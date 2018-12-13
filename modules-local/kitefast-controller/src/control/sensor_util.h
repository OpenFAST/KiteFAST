#ifndef CONTROL_SENSOR_UTIL_H_
#define CONTROL_SENSOR_UTIL_H_

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/sensor_types.h"
#include "control/system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPS functions.

// Converts the GS GPS position reading to the position of the
// ground-station origin in ECEF and calculates the transformation
// matrix from ECEF to G.
void GsGpsPosEcefToGsPosEcef(const Vec3 *gs_gps_pos_ecef,
                             const Vec3 *gs_gps_pos_g, double gs_heading,
                             Vec3 *gs_pos_ecef, Mat3 *dcm_ecef2g);

// Converts a position and velocity uncertainties from the ECEF frame
// to the ground frame.
void ConvSigmaEcefToLocal(const Vec3 *sigma_X_ecef, const Vec3 *sigma_V_ecef,
                          const Mat3 *dcm_ecef2g, Vec3 *sigma_Xg,
                          Vec3 *sigma_Vg);

// Tether force functions.

// Converts a tension, tether roll, and tether pitch to a tether force
// vector in body coordinates.
void TetherForceSphToCart(const TetherForceSph *sph, Vec3 *cart);

// Converts a tether force vector in body coordinates to a tension,
// tether roll, and tether pitch.
void TetherForceCartToSph(const Vec3 *cart, TetherForceSph *sph);

// Converts the loadcell readings into bridle force vectors, then calculates
// tether tension, roll, and pitch.
void LoadcellsToTetherForce(const double loadcells[],
                            const WingParams *wing_params,
                            const LoadcellParams loadcell_params[], Vec3 *cart,
                            TetherForceSph *sph, Vec3 *port_force_b,
                            Vec3 *star_force_b);

// Apparent wind functions.

// Converts airspeed, angle-of-attack, and sideslip angle to the
// apparent wind vector in body coordinates.
void ApparentWindSphToCart(const ApparentWindSph *sph, Vec3 *cart);

// Converts the apparent wind vector to alpha, beta, and airspeed.
void ApparentWindCartToSph(const Vec3 *cart, ApparentWindSph *sph);

// Converts the pitot measurements to angles and airspeed.
void PitotToApparentWindSph(const PitotDifferentialData *diff, const Vec3 *pqr,
                            const PitotParams *params, ApparentWindSph *sph);

// Rotates a wind speed measurement in the wind sensor frame into
// ground coordinates.
//
// Args:
//   wind_ws: Wind measurement [m/s] in the wind sensor frame.
//   perch_azi_angle: Angle [rad] of the perch to the ground frame.
//   perch_azi_omega: Rate of change of perch_azi_angle [rad/s].
//   params: Wind sensor parameters including location of the sensor.
//   wind_g: Output wind measurement [m/s] in the ground coordinates.
void WindWsToWindG(const Vec3 *wind_ws, double perch_azi_angle,
                   double perch_azi_omega, const WindSensorParams *params,
                   Vec3 *wind_g);

// Converts the ground station heading and perch heading to the perch
// azimuth angle.
//
// Args:
//   gs_heading: Heading [rad] of the ground station relative to NED.  Should
//       lie in [0, 2 pi).
//   perch_heading: Heading [rad] of the perch relative to NED.  Should be
//       lie in [0, 2 pi).
//
// Returns:
//   The perch azimuth angle [rad] in range (-pi, pi).
double GsHeadingPerchHeadingToPerchAzi(double gs_heading, double perch_heading);

// Converts a winch position [m] to an equivalent drum angle [rad].
double WinchPosToDrumAngle(double winch_pos, const WinchParams *params);

// Alternate GLAS-based position functions.

// Converts a position in GSG coordinates to perch coordinates.
void XgsgToXp(const GsgData *gsg, const Vec3 *X_gsg, double drum_angle,
              Vec3 *Xp);

// Converts wing position in perch coordinates to an equivalent
// GSG reading ([azi, ele]') assuming a perfect GSG, and a
// straight-line tether.
void XpToGsg(const Vec3 *Xp, double drum_angle, GsgData *gsg);

// Determines the azimuth and elevation angles that correspond to the vector
// describing the departure direction of the tether from the GSG.
void TetherDirectionWdToGsgAziEle(const Vec3 *tether_direction_wd, double *azi,
                                  double *ele);

// Returns true if levelwind should be engaged based on drum angle.
bool IsLevelwindEngaged(double drum_angle, const LevelwindParams *params);

// Estimates wing position in perch coordinates using levelwind
// elevation and winch position.
void LevelwindToXp(double payout, double drum_angle, double levelwind_ele,
                   double tension, const LevelwindParams *params, Vec3 *Xp);

// Converts a position based on a straight line assumption from
// line-angle sensing to a position that also accounts for the
// catenary in the tether.  This function may be used with any
// coordinate system where gravity is in the positive z direction.
void IncludeCatenary(const Vec3 *X_no_cat, double tension, double free_len,
                     Vec3 *X_including_cat);

// Converts static pressure to altitude (positive is up) above mean
// sea level.
double PressureToAltitude(double pressure, const PhysParams *params);

// Calculate air density [kg/m^3] given pressure [Pa] and temperature [°C],
// assuming dry air.
double CalcDryAirDensity(double pressure_pascals, double temperature_celcius);

// Calculate air density [kg/m^3] given pressure [Pa], temperature
// [°C], and relative humidity [as a fraction, from 0.0 to 1.0]. The
// validity flag, if not NULL, is set indicating whether the input
// arguments are in a meteorologically plausible range.
double CalcAirDensity(double pressure_pascals, double temperature_celcius,
                      double relative_humidity, bool *valid);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_SENSOR_UTIL_H_
