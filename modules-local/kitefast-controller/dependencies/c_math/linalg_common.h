#ifndef COMMON_C_MATH_LINALG_COMMON_H_
#define COMMON_C_MATH_LINALG_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

// Enum for matrix operation types.
typedef enum { kTrans, kNoTrans } TransposeType;

typedef enum {
  kLinalgErrorNone,
  kLinalgErrorSingularMat,
  kLinalgErrorMaxIter
} LinalgError;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_LINALG_COMMON_H_
