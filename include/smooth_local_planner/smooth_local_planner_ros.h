#pragma once

#include <nav_core/base_local_planner.h>

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
};

};  // namespace smooth_local_planner
