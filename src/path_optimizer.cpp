#include <smooth_local_planner/path_optimizer.h>

namespace smooth_local_planner {

AD<double> FG_eval::objectiveFunc(const AD<double>& p1, const AD<double>& p2,
                                  const AD<double>& sf) const {
  /*
    Objective Function
      - Bending energy objectve
      - Soften the inequality constraints (curvature constraints) by penalizing
        deviation in objective function

    minimize
      fbe(a,b,c,d,sf) +
      alpha(xs(p4)-xf) + beta(ys(p4)-yf) + gamma(thetas(p4)-thetaf)

    subject to
      |p1| <= Kmax
      |p2| <= Kmax

    See:
    https://www.coursera.org/learn/motion-planning-self-driving-cars/lecture/9MonW/lesson-2-path-planning-optimization
  */

  OptimizationParameters p;
  p.p0 = 0.0;
  p.p1 = p1;
  p.p2 = p2;
  p.p3 = 0.0;
  p.p4 = sf;
  // TODO(Phone): remove hardcoded coefficients and allows to set from outside
  // using setters (probably with ros params)
  return fbe(p) + 25.0 * (fxf(p) + fyf(p)) + 30.0 * ftf(p);
}

AD<double> FG_eval::fbe(const OptimizationParameters& p) const {
  SpiralParameters spiral_params;
  getSpiralParameters(spiral_params, p);

  auto& a = spiral_params.a;
  auto& b = spiral_params.b;
  auto& c = spiral_params.c;
  auto& d = spiral_params.d;
  auto& x = p.p4;

  return ((1.0 / 7.0) * pow(d, 2) * pow(x, 7)) +
         ((1.0 / 3.0) * c * d * pow(x, 6)) +
         ((1.0 / 5.0) * (pow(c, 2) + 2.0 * b * d) * pow(x, 5)) +
         ((1.0 / 2.0) * (b * c + a * d) * pow(x, 4)) +
         (1.0 / 3.0 * (pow(b, 2) + 2.0 * a * c) * pow(x, 3)) +
         (a * b * pow(x, 2)) + (pow(a, 2) * x);
}

AD<double> FG_eval::fxf(const OptimizationParameters& op) const {
  // TODO(Phone): find the compact mathematical form for this
  std::vector<AD<double>> p{0.0, op.p1, op.p2, 0.0, op.p4};
  const auto t2 = p[0] * (1.1E1 / 2.0);
  const auto t3 = p[1] * 9.0;
  const auto t4 = p[2] * (9.0 / 2.0);
  const auto t5 = p[0] * (9.0 / 2.0);
  const auto t6 = p[1] * (2.7E1 / 2.0);
  const auto t7 = p[2] * (2.7E1 / 2.0);
  const auto t8 = p[3] * (9.0 / 2.0);
  const auto t9 = t5 - t6 + t7 - t8;
  const auto t10 = p[0] * 9.0;
  const auto t11 = p[1] * (4.5E1 / 2.0);
  const auto t12 = p[2] * 1.8E1;
  const auto t13 = t8 - t10 + t11 - t12;
  const auto t14 = p[3] - t2 + t3 - t4;
  const auto t15 =
      xf_ -
      p[4] *
          (cos(p[0] * p[4] - p[4] * t9 * (1.0 / 4.0) -
               p[4] * t13 * (1.0 / 3.0) + p[4] * t14 * (1.0 / 2.0)) +
           cos(p[0] * p[4] * (1.0 / 2.0) - p[4] * t9 * (1.0 / 6.4E1) -
               p[4] * t13 * (1.0 / 2.4E1) + p[4] * t14 * (1.0 / 8.0)) *
               2.0 +
           cos(p[0] * p[4] * (3.0 / 4.0) - p[4] * t9 * 7.91015625E-2 -
               p[4] * t13 * (9.0 / 6.4E1) + p[4] * t14 * (9.0 / 3.2E1)) *
               2.0 +
           cos(p[0] * p[4] * (1.0 / 4.0) - p[4] * t9 * 9.765625E-4 -
               p[4] * t13 * (1.0 / 1.92E2) + p[4] * t14 * (1.0 / 3.2E1)) *
               2.0 +
           cos(p[0] * p[4] * (3.0 / 8.0) - p[4] * t9 * 4.94384765625E-3 -
               p[4] * t13 * (9.0 / 5.12E2) + p[4] * t14 * (9.0 / 1.28E2)) *
               4.0 +
           cos(p[0] * p[4] * (1.0 / 8.0) - p[4] * t9 * 6.103515625E-5 -
               p[4] * t13 * 6.510416666666667E-4 +
               p[4] * t14 * (1.0 / 1.28E2)) *
               4.0 +
           cos(p[0] * p[4] * (5.0 / 8.0) - p[4] * t9 * 3.814697265625E-2 -
               p[4] * t13 * 8.138020833333333E-2 +
               p[4] * t14 * (2.5E1 / 1.28E2)) *
               4.0 +
           cos(p[0] * p[4] * (7.0 / 8.0) - p[4] * t9 * 1.4654541015625E-1 -
               p[4] * t13 * 2.233072916666667E-1 +
               p[4] * t14 * (4.9E1 / 1.28E2)) *
               4.0 +
           1.0) *
          (1.0 / 2.4E1);
  const auto t0 = t15 * t15;
  return t0;
}

AD<double> FG_eval::fyf(const OptimizationParameters& op) const {
  // TODO(Phone): find the compact mathematical form for this
  std::vector<AD<double>> p{0.0, op.p1, op.p2, 0.0, op.p4};
  const auto t2 = p[0] * (1.1E1 / 2.0);
  const auto t3 = p[1] * 9.0;
  const auto t4 = p[2] * (9.0 / 2.0);
  const auto t5 = p[0] * (9.0 / 2.0);
  const auto t6 = p[1] * (2.7E1 / 2.0);
  const auto t7 = p[2] * (2.7E1 / 2.0);
  const auto t8 = p[3] * (9.0 / 2.0);
  const auto t9 = t5 - t6 + t7 - t8;
  const auto t10 = p[0] * 9.0;
  const auto t11 = p[1] * (4.5E1 / 2.0);
  const auto t12 = p[2] * 1.8E1;
  const auto t13 = t8 - t10 + t11 - t12;
  const auto t14 = p[3] - t2 + t3 - t4;
  const auto t15 =
      yf_ -
      p[4] *
          (sin(p[0] * p[4] - p[4] * t9 * (1.0 / 4.0) -
               p[4] * t13 * (1.0 / 3.0) + p[4] * t14 * (1.0 / 2.0)) +
           sin(p[0] * p[4] * (1.0 / 2.0) - p[4] * t9 * (1.0 / 6.4E1) -
               p[4] * t13 * (1.0 / 2.4E1) + p[4] * t14 * (1.0 / 8.0)) *
               2.0 +
           sin(p[0] * p[4] * (3.0 / 4.0) - p[4] * t9 * 7.91015625E-2 -
               p[4] * t13 * (9.0 / 6.4E1) + p[4] * t14 * (9.0 / 3.2E1)) *
               2.0 +
           sin(p[0] * p[4] * (1.0 / 4.0) - p[4] * t9 * 9.765625E-4 -
               p[4] * t13 * (1.0 / 1.92E2) + p[4] * t14 * (1.0 / 3.2E1)) *
               2.0 +
           sin(p[0] * p[4] * (3.0 / 8.0) - p[4] * t9 * 4.94384765625E-3 -
               p[4] * t13 * (9.0 / 5.12E2) + p[4] * t14 * (9.0 / 1.28E2)) *
               4.0 +
           sin(p[0] * p[4] * (1.0 / 8.0) - p[4] * t9 * 6.103515625E-5 -
               p[4] * t13 * 6.510416666666667E-4 +
               p[4] * t14 * (1.0 / 1.28E2)) *
               4.0 +
           sin(p[0] * p[4] * (5.0 / 8.0) - p[4] * t9 * 3.814697265625E-2 -
               p[4] * t13 * 8.138020833333333E-2 +
               p[4] * t14 * (2.5E1 / 1.28E2)) *
               4.0 +
           sin(p[0] * p[4] * (7.0 / 8.0) - p[4] * t9 * 1.4654541015625E-1 -
               p[4] * t13 * 2.233072916666667E-1 +
               p[4] * t14 * (4.9E1 / 1.28E2)) *
               4.0) *
          (1.0 / 2.4E1);
  const auto t0 = t15 * t15;
  return t0;
}

AD<double> FG_eval::ftf(const OptimizationParameters& op) const {
  // TODO(Phone): find the compact mathematical form for this
  std::vector<AD<double>> p{0.0, op.p1, op.p2, 0.0, op.p4};
  const auto t2 =
      thetaf_ - p[0] * p[4] +
      p[4] * (p[0] * (1.1E1 / 2.0) - p[1] * 9.0 + p[2] * (9.0 / 2.0) - p[3]) *
          (1.0 / 2.0) +
      p[4] *
          (p[0] * (9.0 / 2.0) - p[1] * (2.7E1 / 2.0) + p[2] * (2.7E1 / 2.0) -
           p[3] * (9.0 / 2.0)) *
          (1.0 / 4.0) -
      p[4] *
          (p[0] * 9.0 - p[1] * (4.5E1 / 2.0) + p[2] * 1.8E1 -
           p[3] * (9.0 / 2.0)) *
          (1.0 / 3.0);
  const auto t0 = t2 * t2;
  return t0;
}

PathOptimizer::PathOptimizer() : xi_(3), xl_(3), xu_(3), gl_(0), gu_(0) {
  // initial value of p1 and p2
  xi_[0] = 0.0;
  xi_[1] = 0.0;

  // lower and upper limits for p1 and p2 (curvature bounds)
  // their curvature needs to lie within [-0.5, 0.5]
  // TODO(Phone): this must be set from ros parameter
  xl_[0] = -5.0;
  xu_[0] = 5.0;
  xl_[1] = -5.0;
  xu_[1] = 5.0;

  // options
  // turn off any printing
  options_ += "Integer print_level  0\n";
  options_ += "String  sb           yes\n";
  // maximum number of iterations
  options_ += "Integer max_iter     10\n";
  // approximate accuracy in first order necessary conditions;
  // see Mathematical Programming, Volume 106, Number 1,
  // Pages 25-57, Equation (6)
  options_ += "Numeric tol          1e-6\n";
  // derivative testing
  options_ += "String  derivative_test            second-order\n";
  // maximum amount of random pertubation; e.g.,
  // when evaluation finite diff
  options_ += "Numeric point_perturbation_radius  0.\n";
}

PathOptimizer::~PathOptimizer() {}

void PathOptimizer::optimizeSpiral(SpiralPath& spiral, const double& xf,
                                   const double& yf, const double& thetaf) {
  // this function assumes xf, yf and thetaf are in the robot coordinate frame
  // user must transform the global path segment into robot frame before calling
  // this function

  // find lower bound of the spiral arc length
  // which is just a straight line
  const double sf_0 = Eigen::Vector2d(xf, yf).norm();

  // set initial value for sf
  xi_[2] = sf_0;

  // set lower and upper limits for sf
  xl_[2] = sf_0;   // lower limit
  xu_[2] = +1e19;  // no upper limit

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  fg_eval_.setFinalPose(xf, yf, thetaf);
  CppAD::ipopt::solve<Dvector, FG_eval>(options_, xi_, xl_, xu_, gl_, gu_,
                                        fg_eval_, solution);
  /*
    for (std::size_t i = 0; i < 3; ++i) {
      std::cout << solution.x[i] << std::endl;
    }

    std::cout << solution.obj_value << std::endl;

    OptimizationParameters p;
    p.p0 = 0.0;
    p.p1 = solution.x[0];
    p.p2 = solution.x[1];
    p.p3 = 0.0;
    p.p4 = solution.x[2];
    SpiralParameters params;
    getSpiralParameters(params, p);
    std::cout << params.a << std::endl;
    std::cout << params.b << std::endl;
    std::cout << params.c << std::endl;
    std::cout << params.d << std::endl;
    */

  // we define the path as cubic polynomial spiral i.e., the curvature of the
  // path is a cubic polynomial function of arc length
  // K(s) = a + bs + cs^2 + ds^3
  // path parameterization: p = [p0 p1 p2 p3 p4/sf]

  // TODO(Phone): samples the spiral along its arc length to generate a
  // discrete set of x, y, and theta points for a path.
}

};  // namespace smooth_local_planner