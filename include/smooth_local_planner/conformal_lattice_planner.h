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
  ConformalLatticePlanner(std::string name, tf2_ros::Buffer* tf,
                          costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destructor
   */
  ~ConformalLatticePlanner();

  bool plan(std::vector<SpiralPath>& paths,
            const nav_msgs::Path& global_plan) const;

 private:
  void generateGoalSet(
      std::vector<geometry_msgs::PoseStamped>& goal_poses,
      const geometry_msgs::PoseStamped& lookahead_goal_pose) const;

  tf2_ros::Buffer* tf_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  std::shared_ptr<PathOptimizer> optimizer_;

  ros::Publisher markers_pub_;

  double lookahead_goal_dist_;
  int lattice_path_samples_;
  double lattice_path_offset_;
  bool lattice_paths_pub_;
  double max_curvature_;
  double penalty_alpha_;
  double penalty_beta_;
  double penalty_gamma_;
  int simpson_intervals_;
};

};  // namespace smooth_local_planner