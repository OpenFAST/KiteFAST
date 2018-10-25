#include "tether_util.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "util.h"

// TODO(kennyjensen): Remove arbitrary 1e-6.
void TensionAndPointToParabola(double tension_x, double r, double h, double mu,
                               TetherParabola *tp) {
  assert(tension_x > 0.0);
  assert(mu > 0.0);
  assert(tp != NULL);

  if (-1e-6 < r && r < 1e-6) {
    // The problem is ill conditioned if the radius is near zero.
    assert(false);
    tp->a = 0.0;
    tp->b = 0.0;
  } else {
    tp->a = mu / (2.0 * fmax(tension_x, 1e-6));
    tp->b = (h - tp->a * r * r) / r;
  }
}

void TensionAndAngleToParabola(double tension_x, double r, double angle,
                               double mu, TetherParabola *tp) {
  assert(tension_x > 0.0);
  assert(mu > 0.0);
  assert(tp != NULL);

  tp->a = mu / (2.0 * fmax(tension_x, 1e-6));
  double dh_dr = Saturate(tan(angle), -1e9, 1e9);
  tp->b = dh_dr - 2.0 * tp->a * r;
}

double ParabolaHeight(const TetherParabola *tp, double r) {
  assert(tp != NULL && tp->a > 0.0);
  return tp->a * r * r + tp->b * r;
}

double ParabolaAngle(const TetherParabola *tp, double r) {
  assert(tp != NULL && tp->a > 0.0);
  return atan(2.0 * tp->a * r + tp->b);
}

double ParabolaVertex(const TetherParabola *tp) {
  assert(tp != NULL && tp->a > 0.0);
  return -tp->b / (2.0 * fmax(tp->a, 1e-6));
}

double ParabolaMinimum(const TetherParabola *tp) {
  assert(tp != NULL && tp->a > 0.0);
  return ParabolaHeight(tp, ParabolaVertex(tp));
}

double ParabolaHorizontalTension(const TetherParabola *tp, double mu) {
  assert(tp != NULL && tp->a > 0.0 && mu > 0.0);
  return mu / (2.0 * fmax(tp->a, 1e-6));
}

double ParabolaVerticalTension(const TetherParabola *tp, double mu, double r) {
  assert(tp != NULL && tp->a > 0.0 && mu > 0.0);
  return ParabolaHorizontalTension(tp, mu) * (2.0 * tp->a * r + tp->b);
}

double ParabolaTension(const TetherParabola *tp, double mu, double r) {
  assert(tp != NULL && tp->a > 0.0 && mu > 0.0);
  return hypot(ParabolaHorizontalTension(tp, mu),
               ParabolaVerticalTension(tp, mu, r));
}
