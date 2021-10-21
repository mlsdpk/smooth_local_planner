#include <smooth_local_planner/visualizer.h>

namespace smooth_local_planner {
namespace visualizer {

Visualizer::Visualizer(const std::string& name) {
  ros::NodeHandle private_nh("~/" + name);
  global_path_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
  local_path_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  markers_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(
      "lattice_path_markers", 10);
}

Visualizer::~Visualizer() {}

void Visualizer::publishGlobalPlan(const nav_msgs::Path& plan) {
  global_path_pub_.publish(plan);
}

void Visualizer::publishLocalPlan(const std::vector<SpiralPath>& paths,
                                  const std::size_t& idx) {
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = paths[idx].frame_id;
  path_msg.header.stamp = ros::Time::now();
  std::size_t s_size = paths[idx].x_points.size();
  path_msg.poses.resize(s_size);
  for (std::size_t i = 0; i < s_size; ++i) {
    path_msg.poses[i].pose.position.x = paths[idx].x_points[i];
    path_msg.poses[i].pose.position.y = paths[idx].y_points[i];
    planner_utils::convertToQuaternion(paths[idx].theta_points[i],
                                       path_msg.poses[i].pose.orientation);
  }
  local_path_pub_.publish(path_msg);
}

void Visualizer::publishMarkers(
    const std::vector<SpiralPath>& paths,
    const std::vector<geometry_msgs::PoseStamped>& goal_poses,
    const std::vector<bool>& collision_status) {
  if (markers_pub_.getNumSubscribers()) {
    visualization_msgs::MarkerArray markers;

    // lattice path marker
    for (std::size_t i = 0; i < paths.size(); ++i) {
      visualization_msgs::Marker path_marker;
      path_marker.header.frame_id = paths[i].frame_id;
      path_marker.header.stamp = ros::Time::now();
      path_marker.ns = "lattice_path";
      path_marker.id = i;
      path_marker.action = visualization_msgs::Marker::ADD;
      path_marker.type = visualization_msgs::Marker::LINE_STRIP;
      path_marker.pose.orientation.w = 1.0;
      path_marker.scale.x = 0.005;

      std::size_t n_points = paths[i].x_points.size();
      path_marker.points.reserve(n_points);
      path_marker.colors.reserve(n_points);

      for (std::size_t j = 0; j < n_points; ++j) {
        geometry_msgs::Point p;
        p.x = paths[i].x_points[j];
        p.y = paths[i].y_points[j];
        p.z = 0.0;
        path_marker.points.push_back(p);

        if (collision_status[i])
          path_marker.colors.push_back(getColorRed());
        else
          path_marker.colors.push_back(getColorGreen());
      }

      markers.markers.emplace_back(std::move(path_marker));
    }

    // goal poses markers
    for (std::size_t i = 0; i < goal_poses.size(); ++i) {
      visualization_msgs::Marker goal_pose_marker;
      goal_pose_marker.header.frame_id = goal_poses[i].header.frame_id;
      goal_pose_marker.header.stamp = ros::Time::now();
      goal_pose_marker.ns = "lattice_goal_poses";
      goal_pose_marker.id = i;
      goal_pose_marker.action = visualization_msgs::Marker::ADD;
      goal_pose_marker.type = visualization_msgs::Marker::SPHERE;
      goal_pose_marker.pose = goal_poses[i].pose;
      goal_pose_marker.scale.x = 0.02;
      goal_pose_marker.scale.y = 0.02;
      goal_pose_marker.scale.z = 0.005;
      goal_pose_marker.color = getColorBlue();

      markers.markers.emplace_back(std::move(goal_pose_marker));
    }

    markers_pub_.publish(markers);
  }
}

}  // namespace visualizer
}  // namespace smooth_local_planner