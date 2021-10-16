#include <pluginlib/class_list_macros.h>
#include <smooth_local_planner/smooth_local_planner_ros.h>
// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(smooth_local_planner::SmoothLocalPlannerROS,
                       nav_core::BaseLocalPlanner)

namespace smooth_local_planner {

SmoothLocalPlannerROS::SmoothLocalPlannerROS() {}

SmoothLocalPlannerROS::~SmoothLocalPlannerROS() {}

void SmoothLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                                       costmap_2d::Costmap2DROS* costmap_ros) {
  optimizer_ = std::make_shared<PathOptimizer>();

  // this is just for testing optimizer
  SpiralPath spiral;
  optimizer_->optimizeSpiral(spiral, 6.0, -1.5, 0.0);
}

bool SmoothLocalPlannerROS::computeVelocityCommands(
    geometry_msgs::Twist& cmd_vel) {
  return true;
}

bool SmoothLocalPlannerROS::isGoalReached() { return true; }

bool SmoothLocalPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  return true;
}

};  // namespace smooth_local_planner