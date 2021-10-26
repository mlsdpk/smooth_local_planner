#pragma once

#include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <smooth_local_planner/Trajectory2DMsg.h>
#include <smooth_local_planner/planner_utils.h>

namespace smooth_local_planner {

class VelocityPlanner {
 public:
  /**
   * @brief Constructor
   */
  VelocityPlanner(const std::string& name,
                  base_local_planner::OdometryHelperRos* odom_helper);

  /**
   * @brief Destructor
   */
  ~VelocityPlanner();

  void computeVelocityProfile(Trajectory2DMsg& trajectory_msg,
                              const SpiralPath& best_path,
                              const double& desired_vel);

  double calculateDistance(const double& v_i, const double& v_f,
                           const double& a) const;

  double calculateFinalSpeed(const double& v_i, const double& a,
                             const double& d) const;

 private:
  // provides an interface to receive the current velocity from the robot
  base_local_planner::OdometryHelperRos* odom_helper_;

  double max_vel_x_;
  double acc_lim_x_;
};

};  // namespace smooth_local_planner