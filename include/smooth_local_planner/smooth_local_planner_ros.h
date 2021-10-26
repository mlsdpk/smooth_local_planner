#pragma once

#include <angles/angles.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <smooth_local_planner/DebugMsg.h>
#include <smooth_local_planner/Trajectory2DMsg.h>
#include <smooth_local_planner/collision_checker.h>
#include <smooth_local_planner/conformal_lattice_planner.h>
#include <smooth_local_planner/controller.h>
#include <smooth_local_planner/velocity_planner.h>
#include <smooth_local_planner/visualizer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace smooth_local_planner {

class SmoothLocalPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  SmoothLocalPlannerROS();

  ~SmoothLocalPlannerROS();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  bool isGoalReached() override;

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros) override;

 private:
  bool transformGlobalPlan(nav_msgs::Path& transformed_plan,
                           const geometry_msgs::PoseStamped& pose);
  bool transformPose(const geometry_msgs::PoseStamped& in_pose,
                     geometry_msgs::PoseStamped& out_pose,
                     const std::string& frame) const;

  /**
   * @brief Find the best lattice path based on the defined scored function.
   * @param idx Index of the best lattice path.
   * @param paths Lattice paths produced by the optimizer.
   * @param lookahead_goal_pose Lookahead goal pose on the global path.
   * @param collision_status Collision status of the lattice paths.
   */
  bool getBestPathIndex(std::size_t& idx, const std::vector<SpiralPath>& paths,
                        const geometry_msgs::PoseStamped& lookahead_goal_pose,
                        const std::vector<bool>& collision_status) const;

  /**
   * @brief Return true if the robot is within the goal region, based on two
   * user-defined paramters: xy_goal_tolerance_ and yaw_goal_tolerance_.
   */
  bool isRobotWithinGoalTolerances();

  /**
   * @brief Find the lookahead goal pose from the transformed global plan. This
   * function also sets the ref_vel_ velocity to this lookahead goal pose. If
   * lookahead goal pose is at the end of the global path, velocity is set as
   * zero. The lookahead distance is decided based on the current robot velocity
   * and user defined parameters: lookahead_base_dist_ and lookahead_time_.
   * @param goal_pose Lookahead goal pose to be set.
   * @param global_plan Transformed global plan into local frame.
   */
  void setLookAheadGoalPoseAndSpeed(GoalPoseWithSpeed& goal_pose,
                                    const nav_msgs::Path& global_plan);

  // ros parameters
  double global_path_distance_bias_;
  double collidiing_path_distance_bias_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double ref_vel_;
  double lookahead_base_dist_;
  double lookahead_time_;
  bool lattice_paths_pub_;
  bool debug_;
  double planning_frequency_;

  // publishers
  ros::Publisher debug_msg_pub_;

  // main classes
  std::shared_ptr<ConformalLatticePlanner> conformal_lattice_planner_;
  std::shared_ptr<GridCollisionChecker> collision_checker_;
  std::shared_ptr<visualizer::Visualizer> visualizer_;
  std::shared_ptr<Controller> controller_;
  std::shared_ptr<base_local_planner::OdometryHelperRos>
      odom_helper_;  // provides an interface to receive the current velocity
                     // from the robot
  std::shared_ptr<VelocityPlanner> velocity_planner_;

  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  geometry_msgs::PoseStamped current_pose_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  ros::Time planning_init_time_;

  // flags
  bool initialized_;
  bool goal_reached_;
};

};  // namespace smooth_local_planner
