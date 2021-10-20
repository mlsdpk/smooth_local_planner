#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cppad/cppad.hpp>
#include <vector>

using CppAD::AD;

namespace smooth_local_planner {

struct SpiralPath {
  std::string frame_id;
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
inline void getSpiralParameters(
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

namespace planner_utils {

inline void linSpace(std::vector<double>& vec, const double& start,
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
inline void compositeTrapezoid(std::vector<double>& points,
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

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(const geometry_msgs::Pose& pos1,
                                 const geometry_msgs::Pose& pos2) {
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;
  double dz = pos1.position.z - pos2.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(const geometry_msgs::PoseStamped& pos1,
                                 const geometry_msgs::PoseStamped& pos2) {
  return euclidean_distance(pos1.pose, pos2.pose);
}

/**
 * Find element in iterator with the minimum calculated value
 */
template <typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal) {
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

inline void convertToQuaternion(const double& angle,
                                geometry_msgs::Quaternion& quat) {
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);
  quat = tf2::toMsg(q);
}

};  // namespace planner_utils

};  // namespace smooth_local_planner