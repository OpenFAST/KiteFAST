#ifndef COMMON_C_MATH_VOTING_H_
#define COMMON_C_MATH_VOTING_H_

#include "common/c_math/quaternion.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

double Median3(double a, double b, double c);
const Vec3 *Median3Vec3(const Vec3 *a, const Vec3 *b, const Vec3 *c, Vec3 *m);
const Quat *Median3Quat(const Quat *a, const Quat *b, const Quat *c, Quat *m);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_VOTING_H_
