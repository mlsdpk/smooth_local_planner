#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <smooth_local_planner/path_optimizer.h>
#include <smooth_local_planner/planner_utils.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

namespace smooth_local_planner {

class ConformalLatticePlanner {
 public:
  /**
   * @brief Constructor
   */
  ConformalLatticePlanner(const std::string& name);

  /**
   * @brief Destructor
   */
  ~ConformalLatticePlanner();

  bool plan(std::vector<SpiralPath>& paths,
            geometry_msgs::PoseStamped& lookahead_goal_pose,
            std::vector<geometry_msgs::PoseStamped>& goal_poses,
            std::vector<double>& obj_costs,
            const nav_msgs::Path& global_plan) const;

 private:
  void generateGoalSet(
      std::vector<geometry_msgs::PoseStamped>& goal_poses,
      const geometry_msgs::PoseStamped& lookahead_goal_pose) const;

  std::shared_ptr<PathOptimizer> optimizer_;

  double lookahead_goal_dist_;
  int lattice_path_samples_;
  double lattice_path_offset_;
  double min_turning_radius_;
  double penalty_alpha_;
  double penalty_beta_;
  double penalty_gamma_;
  int simpson_intervals_;
};

};  // namespace smooth_local_planner