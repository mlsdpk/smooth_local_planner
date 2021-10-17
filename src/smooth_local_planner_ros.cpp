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

  // auto init_time = std::chrono::system_clock::now();
  optimizer_->optimizeSpiral(spiral, 6.0, -1.5, 0.0);
  // auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
  //                      std::chrono::system_clock::now() - init_time)
  //                      .count();
  // std::cout << "time diff: " << time_diff << std::endl;

  // std::cout << "y_points size: " << spiral.y_points.size() << std::endl;
  // for (const auto& point : spiral.y_points) {
  //   std::cout << point << ", ";
  // }
  // std::cout << std::endl;
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