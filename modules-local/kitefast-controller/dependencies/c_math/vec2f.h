// The Vec2f library provides functions for performing simple
// arithmetic operations (addition, subtraction, scaling, dot product,
// norm, etc.) on two-dimensional vectors.

#ifndef COMMON_C_MATH_VEC2F_H_
#define COMMON_C_MATH_VEC2F_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Vec2f { float x, y; } Vec2f;

extern const Vec2f kVec2fZero;
extern const Vec2f kVec2fOnes;

#define VEC2F_DISP(v) \
  printf("%s:%u %s = [%.12f, %.12f]'\n", __FILE__, __LINE__, #v, (v).x, (v).y)

const Vec2f *Vec2fAdd(const Vec2f *v0, const Vec2f *v1, Vec2f *v_out);
const Vec2f *Vec2fAdd3(const Vec2f *v0, const Vec2f *v1, const Vec2f *v2,
                       Vec2f *v_out);
const Vec2f *Vec2fSub(const Vec2f *v0, const Vec2f *v1, Vec2f *v_out);
const Vec2f *Vec2fScale(const Vec2f *v_in, float scale, Vec2f *v_out);
const Vec2f *Vec2fLinComb(float c0, const Vec2f *v0, float c1, const Vec2f *v1,
                          Vec2f *v_out);
const Vec2f *Vec2fLinComb3(float c0, const Vec2f *v0, float c1, const Vec2f *v1,
                           float c2, const Vec2f *v2, Vec2f *v_out);
const Vec2f *Vec2fMult(const Vec2f *v0, const Vec2f *v1, Vec2f *v_out);
float Vec2fDot(const Vec2f *v0, const Vec2f *v1);
float Vec2fNorm(const Vec2f *v);
float Vec2fNormBound(const Vec2f *v, float low);
float Vec2fNormSquared(const Vec2f *v);
const Vec2f *Vec2fNormalize(const Vec2f *v_in, Vec2f *v_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_VEC2F_H_
