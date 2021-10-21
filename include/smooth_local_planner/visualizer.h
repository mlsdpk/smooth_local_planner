#pragma once

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <smooth_local_planner/planner_utils.h>
#include <visualization_msgs/MarkerArray.h>

namespace smooth_local_planner {
namespace visualizer {

inline std_msgs::ColorRGBA getColorRed() {
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 1.0;
  return color;
};

inline std_msgs::ColorRGBA getColorGreen() {
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;
  return color;
};

inline std_msgs::ColorRGBA getColorBlue() {
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 1.0;
  return color;
};

class Visualizer {
 public:
  /**
   * @brief Constructor
   */
  Visualizer(const std::string& name);

  /**
   * @brief Destructor
   */
  ~Visualizer();

  void publishGlobalPlan(const nav_msgs::Path& plan);
  void publishLocalPlan(const std::vector<SpiralPath>& paths,
                        const std::size_t& idx);
  void publishMarkers(const std::vector<SpiralPath>& paths,
                      const std::vector<geometry_msgs::PoseStamped>& goal_poses,
                      const std::vector<bool>& collision_status);

 private:
  ros::Publisher global_path_pub_;
  ros::Publisher local_path_pub_;
  ros::Publisher markers_pub_;
};

};  // namespace visualizer
};  // namespace smooth_local_planner