#include <smooth_local_planner/controller.h>

namespace smooth_local_planner {

Controller::Controller(const std::string& name, tf2_ros::Buffer* tf,
                       const std::string& robot_base_frame)
    : tf_{tf}, robot_base_frame_{robot_base_frame} {
  ros::NodeHandle private_nh("~/" + name);

  private_nh.param("lookahead_dist", lookahead_dist_, 0.2);
}

Controller::~Controller() {}

void Controller::updateTrajectory(const Trajectory2DMsg& trajectory_msg) {
  trajectory_ = trajectory_msg;
}

void Controller::updateControls(geometry_msgs::Twist& cmd_vel,
                                const geometry_msgs::PoseStamped& pose) {
  // find the linear velocity we need to follow
  // get the velocity of the nearest index waypoint in the trajectory
  double vx = 0.0;
  updateDesiredVelocity(vx, pose);

  // TODO(Phone): Choose the lookahead distance based on desired velocity
  // use the lookahead gain (This need to be tuned and set as ros param)

  // first find the lookahead point to track
  // we'll use the given trajectory to find the lookahead point
  // the resultant lookahead point is in robot base frame
  updateLookAheadPose();

  // find the curvature of arc between robot base and lookahead
  // K = 2 * y / l^2
  double lookahead_dist2 = lookahead_dist_ * lookahead_dist_;

  double curvature = 0.0;
  if (lookahead_dist2 > 0.001) {
    curvature = 2.0 * lookahead_pose_.position.y / lookahead_dist2;
  }

  cmd_vel.linear.x = vx;
  cmd_vel.angular.z = vx * curvature;
}

void Controller::updateLookAheadPose() {
  // the trajectory must be first transformed into robot frame
  // to obtain the lookahead pose
  nav_msgs::Path transformed_trajectory;
  transformed_trajectory.header.frame_id = robot_base_frame_;

  std::size_t s_size = trajectory_.poses.size();
  transformed_trajectory.poses.resize(s_size);
  for (std::size_t i = 0; i < s_size; ++i) {
    geometry_msgs::PoseStamped temp_pose;
    temp_pose.header.frame_id = trajectory_.header.frame_id;
    temp_pose.pose.position.x = trajectory_.poses[i].position.x;
    temp_pose.pose.position.y = trajectory_.poses[i].position.y;
    temp_pose.pose.orientation = trajectory_.poses[i].orientation;
    try {
      tf_->transform(temp_pose, transformed_trajectory.poses[i],
                     robot_base_frame_);
      transformed_trajectory.poses[i].header.frame_id = robot_base_frame_;
    } catch (tf2::TransformException& ex) {
      ROS_ERROR("Exception in transformPose: %s", ex.what());
    }
  }

  auto pose_it =
      std::find_if(transformed_trajectory.poses.begin(),
                   transformed_trajectory.poses.end(), [&](const auto& ps) {
                     return std::hypot(ps.pose.position.x,
                                       ps.pose.position.y) >= lookahead_dist_;
                   });

  // If the no pose is not far enough, take the last pose
  if (pose_it == transformed_trajectory.poses.end()) {
    pose_it = std::prev(transformed_trajectory.poses.end());
  }

  lookahead_pose_ = (*pose_it).pose;
}

void Controller::updateDesiredVelocity(double& vx,
                                       const geometry_msgs::PoseStamped& pose) {
  std::size_t min_idx = 0;
  double min_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < trajectory_.poses.size(); ++i) {
    auto dist =
        planner_utils::euclidean_distance(trajectory_.poses[i], pose.pose);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  vx = trajectory_.velocity[min_idx];
}

}  // namespace smooth_local_planner