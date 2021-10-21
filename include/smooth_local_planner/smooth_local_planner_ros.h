#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <smooth_local_planner/DebugMsg.h>
#include <smooth_local_planner/collision_checker.h>
#include <smooth_local_planner/conformal_lattice_planner.h>
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
  void transformGlobalPlan(nav_msgs::Path& transformed_plan,
                           const geometry_msgs::PoseStamped& pose);
  bool transformPose(const geometry_msgs::PoseStamped& in_pose,
                     geometry_msgs::PoseStamped& out_pose,
                     const std::string& frame) const;
  bool getBestPathIndex(std::size_t& idx, const std::vector<SpiralPath>& paths,
                        const geometry_msgs::PoseStamped& lookahead_goal_pose,
                        const std::vector<bool>& collision_status) const;

  // parameters
  bool lattice_paths_pub_;
  double global_path_distance_bias_;
  double collidiing_path_distance_bias_;
  bool debug_;

  // publishers
  ros::Publisher debug_msg_pub_;

  // main classes
  std::shared_ptr<ConformalLatticePlanner> conformal_lattice_planner_;
  std::shared_ptr<GridCollisionChecker> collision_checker_;
  std::shared_ptr<visualizer::Visualizer> visualizer_;

  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  geometry_msgs::PoseStamped current_pose_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  bool initialized_;
};

};  // namespace smooth_local_planner
