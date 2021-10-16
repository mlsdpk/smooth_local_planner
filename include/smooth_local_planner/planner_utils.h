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

struct SpiralParameters {
  AD<double> a;
  AD<double> b;
  AD<double> c;
  AD<double> d;
};

struct OptimizationParameters {
  AD<double> p0;
  AD<double> p1;
  AD<double> p2;
  AD<double> p3;
  AD<double> p4;
};

static void getSpiralParameters(
    SpiralParameters& spiral_params,
    const OptimizationParameters& optimization_params) {
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

};  // namespace smooth_local_planner