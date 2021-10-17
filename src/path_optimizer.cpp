#include <smooth_local_planner/path_optimizer.h>

namespace smooth_local_planner {

AD<double> FG_eval::objectiveFunc(const AD<double>& p1, const AD<double>& p2,
                                  const AD<double>& sf) const {
  /*
    We define the path as cubic polynomial spiral i.e., the curvature of the
    path is a cubic polynomial function of arc length:

        K(s) = a + bs + cs^2 + ds^3

    Path parameterization: p = [p0 p1 p2 p3 p4/sf]

    Objective Function
      - Bending energy objectve
      - Soften the equality constraints of final spiral positions by adding as
        penalty functions into our objective
      - penalty functions here use the quadratic loss function

    minimize
      fbe(a,b,c,d,sf) +
      alpha * (xs(p4)-xf)^2 + beta * (ys(p4)-yf)^2 +
      gamma * (thetas(p4)-thetaf)^2

    subject to curvature bounds
      |p1| <= Kmax
      |p2| <= Kmax

    See:
    https://www.coursera.org/learn/motion-planning-self-driving-cars/lecture/9MonW/lesson-2-path-planning-optimization
  */

  OptimizationParameters<AD<double>> p;
  p.p0 = 0.0;
  p.p1 = p1;
  p.p2 = p2;
  p.p3 = 0.0;
  p.p4 = sf;
  // TODO(Phone): remove hardcoded coefficients and allows to set from outside
  // using setters (probably with ros params)
  const double alpha = 25.0;
  const double beta = 25.0;
  const double gamma = 30.0;
  return fbe(p) + alpha * pow(xf_ - xs(p), 2) + beta * pow(yf_ - ys(p), 2) +
         gamma * pow(thetaf_ - thetas(p, p.p4), 2);
}

AD<double> FG_eval::fbe(const OptimizationParameters<AD<double>>& p) const {
  SpiralParameters<AD<double>> spiral_params;
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

AD<double> FG_eval::xs(const OptimizationParameters<AD<double>>& p) const {
  // Simpson's rule with n=8
  // TODO: this parameter "n" must be set from outside of the class (with ros
  // param)
  auto& s = p.p4;

  return (s / 24.0) * (cos(thetas(p, 0)) + 4 * cos(thetas(p, s / 8.0)) +
                       2 * cos(thetas(p, 2.0 * s / 8.0)) +
                       4 * cos(thetas(p, 3.0 * s / 8.0)) +
                       2 * cos(thetas(p, 4.0 * s / 8.0)) +
                       4 * cos(thetas(p, 5.0 * s / 8.0)) +
                       2 * cos(thetas(p, 6.0 * s / 8.0)) +
                       4 * cos(thetas(p, 7.0 * s / 8.0)) + cos(thetas(p, s)));
}

AD<double> FG_eval::ys(const OptimizationParameters<AD<double>>& p) const {
  // Simpson's rule with n=8
  // TODO: this parameter "n" must be set from outside of the class (with ros
  // param)
  auto& s = p.p4;

  return (s / 24.0) * (sin(thetas(p, 0)) + 4 * sin(thetas(p, s / 8.0)) +
                       2 * sin(thetas(p, 2.0 * s / 8.0)) +
                       4 * sin(thetas(p, 3.0 * s / 8.0)) +
                       2 * sin(thetas(p, 4.0 * s / 8.0)) +
                       4 * sin(thetas(p, 5.0 * s / 8.0)) +
                       2 * sin(thetas(p, 6.0 * s / 8.0)) +
                       4 * sin(thetas(p, 7.0 * s / 8.0)) + sin(thetas(p, s)));
}

AD<double> FG_eval::thetas(const OptimizationParameters<AD<double>>& p,
                           const AD<double>& s) const {
  SpiralParameters<AD<double>> spiral_params;
  getSpiralParameters(spiral_params, p);
  return thetas(spiral_params, s);
}

template <typename T>
T FG_eval::thetas(const SpiralParameters<T>& spiral_params, const T& s) const {
  // K(s) = a + bs + cs^2 + ds^3
  // theta(s) = integral of K(s) (has closed-form solution)
  // theta(s) = a*s + b/2*s^2 + c/3*s^3 + d/4*s^4
  return (spiral_params.a * s) + (spiral_params.b * pow(s, 2) * 1.0 / 2.0) +
         (spiral_params.c * pow(s, 3) * 1.0 / 3.0) +
         (spiral_params.d * pow(s, 4) * 1.0 / 4.0);
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

  // for (std::size_t i = 0; i < 3; ++i) {
  //   std::cout << solution.x[i] << std::endl;
  // }

  // std::cout << solution.obj_value << std::endl;

  OptimizationParameters<double> p;
  p.p0 = 0.0;
  p.p1 = solution.x[0];
  p.p2 = solution.x[1];
  p.p3 = 0.0;
  p.p4 = solution.x[2];

  /*
  SpiralParameters<double> params;
  getSpiralParameters(params, p);
  std::cout << params.a << std::endl;
  std::cout << params.b << std::endl;
  std::cout << params.c << std::endl;
  std::cout << params.d << std::endl;
  */

  // TODO(Phone): samples the spiral along its arc length to generate a
  // discrete set of x, y, and theta points for a path.
  sampleSpiral(spiral, p);
}

void PathOptimizer::sampleSpiral(SpiralPath& spiral,
                                 const OptimizationParameters<double>& p) {
  // Set the s_points (list of s values along the spiral) to be from 0.0 to p4
  // (final arc length)
  std::vector<double> s_points;
  // TODO(Phone): Here we're currently using default n size (50)
  // Allows it to be set from ros parameter
  linSpace(s_points, 0.0, p.p4);

  // convert from optimization space to spiral space
  SpiralParameters<double> spiral_params;
  getSpiralParameters(spiral_params, p);

  // reserve memory to reduce allocation time (since we know vector size)
  std::size_t s_size = s_points.size();
  spiral.x_points.reserve(s_size);
  spiral.y_points.reserve(s_size);
  spiral.theta_points.reserve(s_size);

  std::vector<double> cos_thetas, sin_thetas;
  cos_thetas.reserve(s_size);
  sin_thetas.reserve(s_size);

  // we start with theta first (use theta(s) equation)
  for (std::size_t i = 0; i < s_size; ++i) {
    spiral.theta_points.push_back(fg_eval_.thetas(spiral_params, s_points[i]));
    cos_thetas.push_back(std::cos(spiral.theta_points[i]));
    sin_thetas.push_back(std::sin(spiral.theta_points[i]));
  }

  // Use numerical integration to generate points along the spiral
  // path for x_points and y_points, here we use cumulative trapezoidal rule
  // see: https://en.wikipedia.org/wiki/Trapezoidal_rule;
  compositeTrapezoid(spiral.x_points, cos_thetas, s_points);
  compositeTrapezoid(spiral.y_points, sin_thetas, s_points);
}

};  // namespace smooth_local_planner