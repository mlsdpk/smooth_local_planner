#include <smooth_local_planner/controller.h>

namespace smooth_local_planner {

Controller::Controller(const std::string& name) {
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
  // the trajectory must be in robot frame
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
  auto pose_it = std::find_if(
      trajectory_.poses.begin(), trajectory_.poses.end(), [&](const auto& ps) {
        return std::hypot(ps.position.x, ps.position.y) >= lookahead_dist_;
      });

  // If the no pose is not far enough, take the last pose
  if (pose_it == trajectory_.poses.end()) {
    pose_it = std::prev(trajectory_.poses.end());
  }

  lookahead_pose_ = *pose_it;
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