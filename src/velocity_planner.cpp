#include <smooth_local_planner/velocity_planner.h>

namespace smooth_local_planner {

VelocityPlanner::VelocityPlanner(
    const std::string& name, base_local_planner::OdometryHelperRos* odom_helper)
    : odom_helper_{odom_helper} {
  ros::NodeHandle private_nh("~/" + name);

  private_nh.param("max_vel_x", max_vel_x_, 0.22);
  private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
}

VelocityPlanner::~VelocityPlanner() {}

void VelocityPlanner::computeVelocityProfile(Trajectory2DMsg& trajectory_msg,
                                             const SpiralPath& best_path,
                                             const double& desired_vel) {
  // get the current robot speed
  geometry_msgs::PoseStamped curr_robot_vel;
  odom_helper_->getRobotVel(curr_robot_vel);
  double curr_vel_x = curr_robot_vel.pose.position.x;

  // compute distance travelled from start speed to desired speed using a
  // constant acceleration.
  double accel_distance;
  if (curr_vel_x >= desired_vel)
    accel_distance = calculateDistance(curr_vel_x, desired_vel, -acc_lim_x_);
  else
    accel_distance = calculateDistance(curr_vel_x, desired_vel, acc_lim_x_);

  // Here we will compute the end of the ramp for our velocity profile.
  // At the end of the ramp, we will maintain our final speed.
  std::size_t ramp_end_index = 0;
  double distance = 0.0;
  while ((ramp_end_index < best_path.x_points.size() - 1) &&
         (distance < accel_distance)) {
    distance += planner_utils::euclidean_distance(
        best_path.x_points[ramp_end_index], best_path.y_points[ramp_end_index],
        best_path.x_points[ramp_end_index + 1],
        best_path.y_points[ramp_end_index + 1]);
    ramp_end_index += 1;
  }

  // Here we will actually compute the velocities along the ramp.
  auto vi = curr_vel_x;
  for (std::size_t i = 0; i < ramp_end_index; ++i) {
    double vf = 0.0;
    double dist = planner_utils::euclidean_distance(
        best_path.x_points[i], best_path.y_points[i], best_path.x_points[i + 1],
        best_path.y_points[i + 1]);

    if (curr_vel_x >= desired_vel) {
      vf = calculateFinalSpeed(vi, -acc_lim_x_, dist);
      // clamp speed to desired speed
      if (vf < desired_vel) vf = desired_vel;
    } else {
      vf = calculateFinalSpeed(vi, acc_lim_x_, dist);
      // clamp speed to desired speed
      if (vf > desired_vel) vf = desired_vel;
    }

    geometry_msgs::Pose pose;
    pose.position.x = best_path.x_points[i];
    pose.position.y = best_path.y_points[i];
    planner_utils::convertToQuaternion(best_path.theta_points[i],
                                       pose.orientation);
    trajectory_msg.poses.emplace_back(pose);
    trajectory_msg.velocity.push_back(vi);
    vi = vf;
  }

  // If the ramp is over, then for the rest of the profile we should
  // track the desired speed
  for (std::size_t i = ramp_end_index; i < best_path.x_points.size(); ++i) {
    geometry_msgs::Pose pose;
    pose.position.x = best_path.x_points[i];
    pose.position.y = best_path.y_points[i];
    planner_utils::convertToQuaternion(best_path.theta_points[i],
                                       pose.orientation);
    trajectory_msg.poses.emplace_back(pose);
    trajectory_msg.velocity.push_back(desired_vel);
  }

  // Interpolate between the zeroth state and the first state.
  // This prevents the myopic controller from getting stuck at the zeroth state.
  if (trajectory_msg.velocity.size() > 1) {
    trajectory_msg.poses[0].position.x = (trajectory_msg.poses[1].position.x -
                                          trajectory_msg.poses[0].position.x) *
                                             0.1 +
                                         trajectory_msg.poses[0].position.x;
    trajectory_msg.poses[0].position.y = (trajectory_msg.poses[1].position.y -
                                          trajectory_msg.poses[0].position.y) *
                                             0.1 +
                                         trajectory_msg.poses[0].position.y;

    trajectory_msg.velocity[0] =
        (trajectory_msg.velocity[1] - trajectory_msg.velocity[0]) * 0.1 +
        trajectory_msg.velocity[0];
  }

  trajectory_msg.header.frame_id = best_path.frame_id;
  trajectory_msg.header.stamp = ros::Time::now();
}

double VelocityPlanner::calculateDistance(const double& v_i, const double& v_f,
                                          const double& a) const {
  return (std::pow(v_f, 2) - std::pow(v_i, 2)) / (2.0 * a);
}

double VelocityPlanner::calculateFinalSpeed(const double& v_i, const double& a,
                                            const double& d) const {
  return std::sqrt(std::pow(v_i, 2) + 2.0 * a * d);
}

}  // namespace smooth_local_planner