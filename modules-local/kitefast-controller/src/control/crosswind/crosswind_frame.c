#include "crosswind_frame.h"

#include <math.h>

#include "geometry.h"
#include "mat3.h"
#include "util.h"
#include "vec3.h"

void CalcDcmGToCw(const Vec3 *path_center_g, Mat3 *dcm_g2cw) {
  Vec3 ex = *path_center_g, ey, ez;
  Vec3Scale(&ex, -1.0 / Vec3NormBound(&ex, DBL_TOL), &ex);
  Vec3Cross(&kVec3Z, &ex, &ey);
  Vec3Scale(&ey, 1.0 / Vec3NormBound(&ey, DBL_TOL), &ey);
  Vec3Cross(&ex, &ey, &ez);
  dcm_g2cw->d[0][0] = ex.x;
  dcm_g2cw->d[0][1] = ex.y;
  dcm_g2cw->d[0][2] = ex.z;
  dcm_g2cw->d[1][0] = ey.x;
  dcm_g2cw->d[1][1] = ey.y;
  dcm_g2cw->d[1][2] = ey.z;
  dcm_g2cw->d[2][0] = ez.x;
  dcm_g2cw->d[2][1] = ez.y;
  dcm_g2cw->d[2][2] = ez.z;
}
void CalcDcmCwToT(double loop_angle, Mat3 *dcm_cw2t) {
  AngleToDcm(0.0, PI / 2.0, loop_angle, kRotationOrderXyz, dcm_cw2t);
}
void CalcDcmGToT(const Vec3 *path_center_g, double loop_angle, Mat3 *dcm_g2t) {
  Mat3 dcm_g2cw, dcm_cw2t;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  CalcDcmCwToT(loop_angle, &dcm_cw2t);
  Mat3Mat3Mult(&dcm_cw2t, &dcm_g2cw, dcm_g2t);
}
void TransformGToCw(const Vec3 *point_g, const Vec3 *path_center_g,
                    Vec3 *point_cw) {
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  PoseTransform(&dcm_g2cw, path_center_g, point_g, point_cw);
}
void RotateGToCw(const Vec3 *vector_g, const Vec3 *path_center_g,
                 Vec3 *vector_cw) {
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  Mat3Vec3Mult(&dcm_g2cw, vector_g, vector_cw);
}
void TransformCwToG(const Vec3 *point_cw, const Vec3 *path_center_g,
                    Vec3 *point_g) {
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  InversePoseTransform(&dcm_g2cw, path_center_g, point_cw, point_g);
}
void RotateCwToG(const Vec3 *vector_cw, const Vec3 *path_center_g,
                 Vec3 *vector_g) {
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  Mat3TransVec3Mult(&dcm_g2cw, vector_cw, vector_g);
}
