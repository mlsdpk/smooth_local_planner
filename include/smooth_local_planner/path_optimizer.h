#pragma once

#include <smooth_local_planner/planner_utils.h>

#include <Eigen/Core>
#include <cppad/ipopt/solve.hpp>
#include <iostream>  // remove later

using CppAD::AD;

namespace smooth_local_planner {

class FG_eval {
 public:
  typedef CppAD::vector<AD<double>> ADvector;
  void operator()(ADvector& fg, const ADvector& x) {
    assert(fg.size() == 1);
    assert(x.size() == 3);

    AD<double> p1 = x[0];
    AD<double> p2 = x[1];
    AD<double> sf = x[2];

    fg[0] = objectiveFunc(p1, p2, sf);
  }

  void setFinalPose(const double& xf, const double& yf, const double& thetaf) {
    xf_ = xf;
    yf_ = yf;
    thetaf_ = thetaf;
  };

 private:
  AD<double> objectiveFunc(const AD<double>& p1, const AD<double>& p2,
                           const AD<double>& sf) const;
  AD<double> fbe(const OptimizationParameters& p) const;
  AD<double> fxf(const OptimizationParameters& op) const;
  AD<double> fyf(const OptimizationParameters& op) const;
  AD<double> ftf(const OptimizationParameters& op) const;

  double xf_;
  double yf_;
  double thetaf_;
};

class PathOptimizer {
 public:
  /**
   * @brief Constructor
   */
  PathOptimizer();

  /**
   * @brief Destructor
   */
  ~PathOptimizer();

  /**
   * @brief Sets up the optimization problem to compute a spiral to a given goal
   * point.
   * @param spiral Sampled spiral path
   * @param xf goal point x
   * @param yf goal point y
   * @param thetaf goal point heading
   */
  void optimizeSpiral(SpiralPath& spiral, const double& xf, const double& yf,
                      const double& thetaf);

  /**
   * @brief Samples a set of points along the spiral given the optimization
   * parameters.
   * @param spiral Sampled spiral path
   * @param p optimization parameters
   */
  void sampleSpiral(SpiralPath& spiral, const OptimizationParameters& p);

 private:
  typedef CppAD::vector<double> Dvector;

  // cppad related
  std::string options_;
  Dvector xi_;
  Dvector xl_;
  Dvector xu_;
  Dvector gl_;
  Dvector gu_;
  FG_eval fg_eval_;
};

};  // namespace smooth_local_planner