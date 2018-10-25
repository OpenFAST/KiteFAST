#ifndef COMMON_C_MATH_FORCE_MOMENT_H_
#define COMMON_C_MATH_FORCE_MOMENT_H_

#include "mat3.h"
#include "vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ForceMoment { Vec3 force, moment; } ForceMoment;

typedef struct ForceMomentPos { Vec3 force, moment, pos; } ForceMomentPos;

extern const ForceMoment kForceMomentZero;
extern const ForceMomentPos kForceMomentPosZero;

// ForceMoment.

const ForceMoment *ForceMomentRef(const ForceMoment *fm_in,
                                  const Vec3 *ref_disp, ForceMoment *fm_out);
const ForceMoment *ForceMomentAdd(const ForceMoment *fm1,
                                  const ForceMoment *fm2, ForceMoment *fm_out);
const ForceMoment *ForceMomentLinComb(double c0, const ForceMoment *fm0,
                                      double c1, const ForceMoment *fm1,
                                      ForceMoment *fm_out);
const ForceMoment *ForceMomentScale(const ForceMoment *fm_in, double scale,
                                    ForceMoment *fm_out);

// ForceMomentPos.

const ForceMomentPos *ForceMomentPosRef(const ForceMomentPos *fm_in,
                                        ForceMomentPos *fm_out);
const ForceMomentPos *ForceMomentPosAdd(const ForceMomentPos *fm1,
                                        const ForceMomentPos *fm2,
                                        ForceMomentPos *fm_out);
const ForceMomentPos *ForceMomentPosAdd3(const ForceMomentPos *fm1,
                                         const ForceMomentPos *fm2,
                                         const ForceMomentPos *fm3,
                                         ForceMomentPos *fm_out);
const ForceMomentPos *ForceMomentPosPoseTransform(const Mat3 *dcm_a2b,
                                                  const Vec3 *R_a_b_origin,
                                                  const ForceMomentPos *fmx_a,
                                                  ForceMomentPos *fmx_b);
const ForceMomentPos *ForceMomentPosInversePoseTransform(
    const Mat3 *dcm_a2b, const Vec3 *R_a_b_origin, const ForceMomentPos *fmx_b,
    ForceMomentPos *fmx_a);
const ForceMoment *ForceMomentPosToForceMoment(const ForceMomentPos *fmx,
                                               ForceMoment *fm);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_FORCE_MOMENT_H_
