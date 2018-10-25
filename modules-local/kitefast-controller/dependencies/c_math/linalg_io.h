#ifndef COMMON_C_MATH_UTIL_LINALG_IO_H_
#define COMMON_C_MATH_UTIL_LINALG_IO_H_

#include <stdio.h>

#include "linalg.h"

#ifdef __cplusplus
extern "C" {
#endif

void VecDisp(const Vec *v);
void VecScan(FILE *f, Vec *v);

void MatDisp(const Mat *m);
void MatScan(FILE *f, Mat *m);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // COMMON_C_MATH_UTIL_LINALG_IO_H_
