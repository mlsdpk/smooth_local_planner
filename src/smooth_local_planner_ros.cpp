#include <smooth_local_planner/smooth_local_planner_ros.h>

namespace smooth_local_planner {

void initialize(std::string name, tf2_ros::Buffer* tf,
                costmap_2d::Costmap2DROS* costmap_ros) {}

bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {}

bool isGoalReached() {}

bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {}

};  // namespace smooth_local_planner

#include <pluginlib/class_list_macros.h>
// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(smooth_local_planner::SmoothLocalPlannerROS,
                       nav_core::BaseLocalPlanner)