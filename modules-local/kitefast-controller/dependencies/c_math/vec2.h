// The Vec2 library provides functions for performing simple
// arithmetic operations (addition, subtraction, scaling, dot product,
// norm, etc.) on two-dimensional vectors.

#ifndef COMMON_C_MATH_VEC2_H_
#define COMMON_C_MATH_VEC2_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Vec2 { double x, y; } Vec2;

extern const Vec2 kVec2Zero;
extern const Vec2 kVec2Ones;

#define VEC2_DISP(v) \
  printf("%s:%u %s = [%.12lf, %.12lf]'\n", __FILE__, __LINE__, #v, (v).x, (v).y)

const Vec2 *Vec2Add(const Vec2 *v0, const Vec2 *v1, Vec2 *v_out);
const Vec2 *Vec2Add3(const Vec2 *v0, const Vec2 *v1, const Vec2 *v2,
                     Vec2 *v_out);
const Vec2 *Vec2Sub(const Vec2 *v0, const Vec2 *v1, Vec2 *v_out);
const Vec2 *Vec2Scale(const Vec2 *v_in, double scale, Vec2 *v_out);
const Vec2 *Vec2LinComb(double c0, const Vec2 *v0, double c1, const Vec2 *v1,
                        Vec2 *v_out);
const Vec2 *Vec2LinComb3(double c0, const Vec2 *v0, double c1, const Vec2 *v1,
                         double c2, const Vec2 *v2, Vec2 *v_out);
const Vec2 *Vec2Mult(const Vec2 *v0, const Vec2 *v1, Vec2 *v_out);
double Vec2Dot(const Vec2 *v0, const Vec2 *v1);
double Vec2Norm(const Vec2 *v);
double Vec2NormBound(const Vec2 *v, double low);
double Vec2NormSquared(const Vec2 *v);
const Vec2 *Vec2Normalize(const Vec2 *v_in, Vec2 *v_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_VEC2_H_
