#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>
#include <smooth_local_planner/path_optimizer.h>
#include <tf2_ros/buffer.h>

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
  std::shared_ptr<PathOptimizer> optimizer_;
};

};  // namespace smooth_local_planner
