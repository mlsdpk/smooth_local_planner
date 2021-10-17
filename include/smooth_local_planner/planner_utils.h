#pragma once

#include <cppad/cppad.hpp>
#include <vector>

using CppAD::AD;

namespace smooth_local_planner {

struct SpiralPath {
  std::vector<double> x_points;
  std::vector<double> y_points;
  std::vector<double> theta_points;
};

template <typename T>
struct SpiralParameters {
  T a;
  T b;
  T c;
  T d;
};

template <typename T>
struct OptimizationParameters {
  T p0;
  T p1;
  T p2;
  T p3;
  T p4;
};

template <typename T>
static void getSpiralParameters(
    SpiralParameters<T>& spiral_params,
    const OptimizationParameters<T>& optimization_params) {
  spiral_params.a = optimization_params.p0;
  spiral_params.b =
      -(11.0 * optimization_params.p0 / 2.0 - 9.0 * optimization_params.p1 +
        9.0 * optimization_params.p2 / 2.0 - optimization_params.p3) /
      optimization_params.p4;
  spiral_params.c =
      (9.0 * optimization_params.p0 - 45.0 * optimization_params.p1 / 2.0 +
       18.0 * optimization_params.p2 - 9.0 * optimization_params.p3 / 2.0) /
      pow(optimization_params.p4, 2);
  spiral_params.d = -(9.0 * optimization_params.p0 / 2.0 -
                      27.0 * optimization_params.p1 / 2.0 +
                      27.0 * optimization_params.p2 / 2.0 -
                      9.0 * optimization_params.p3 / 2.0) /
                    pow(optimization_params.p4, 3);
};

static void linSpace(std::vector<double>& vec, const double& start,
                     const double& end, std::size_t n = 50) {
  assert(n <= 0);
  // TODO: assert vector must be empty

  if (n == 1) {
    vec.push_back(start);
    return;
  }

  vec.resize(n);
  double delta = (end - start) / (n - 1);
  for (std::size_t i = 0; i < n - 1; ++i) {
    vec[i] = start + delta * i;
  }
  vec[n - 1] = end;
}

/**
 * @brief Composite Trapezoidal Rule. Approximate the integral on a given
 * interval by dividing the interval into many smaller intervals, and applying
 * the trapezoidal rule to each.
 * @param points Approximated points
 * @param y f(x), a scalar-valued function we want to integrate
 * @param s interval vector to be used during trapezoidal rule
 */
static void compositeTrapezoid(std::vector<double>& points,
                               const std::vector<double>& y,
                               const std::vector<double>& s) {
  points.resize(s.size());
  // set initial to be 0.0
  // TODO(Phone): may be add this initial value into function arguments?
  points[0] = 0.0;
  for (std::size_t i = 1; i < s.size(); ++i) {
    points[i] =
        points[i - 1] + (y[i] + y[i - 1]) * (s[i] - s[i - 1]) * 1.0 / 2.0;
  }
}

};  // namespace smooth_local_planner