#pragma once

#include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <smooth_local_planner/Trajectory2DMsg.h>
#include <smooth_local_planner/planner_utils.h>
#include <tf2_ros/buffer.h>

#include <limits>

namespace smooth_local_planner {

/**
 * @brief Path tracking controller class.
 * We intend to develop two geometric path tracking controllers: pure pursuit
 * and stanley. Current version only includes pure pursuit.
 */
class Controller {
 public:
  /**
   * @brief Constructor
   */
  Controller(const std::string& name, tf2_ros::Buffer* tf,
             const std::string& robot_base_frame);

  /**
   * @brief Destructor
   */
  ~Controller();

  void updateTrajectory(const Trajectory2DMsg& trajectory_msg);

  void updateControls(geometry_msgs::Twist& cmd_vel,
                      const geometry_msgs::PoseStamped& pose);

  geometry_msgs::Pose getLookAheadPose() const { return lookahead_pose_; };
  Trajectory2DMsg getCurrentTrajectory() const { return trajectory_; };

 private:
  void updateLookAheadPose();
  void updateDesiredVelocity(double& vx,
                             const geometry_msgs::PoseStamped& pose);

  tf2_ros::Buffer* tf_;
  std::string robot_base_frame_;
  Trajectory2DMsg trajectory_;
  geometry_msgs::Pose lookahead_pose_;

  double lookahead_dist_;
};

};  // namespace smooth_local_planner