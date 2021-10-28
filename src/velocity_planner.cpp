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
                                             const nav_msgs::Path& best_path,
                                             const double& desired_vel,
                                             const double& ref_vel) {
  // get the current robot speed
  geometry_msgs::PoseStamped curr_robot_vel;
  odom_helper_->getRobotVel(curr_robot_vel);
  const double curr_vel_x = curr_robot_vel.pose.position.x;
  const std::size_t path_size = best_path.poses.size();

  // calculate total path length
  double path_length = 0.0;
  for (std::size_t i = 0; i < path_size - 1; ++i) {
    path_length += planner_utils::euclidean_distance(
        best_path.poses[i].pose.position.x, best_path.poses[i].pose.position.y,
        best_path.poses[i + 1].pose.position.x,
        best_path.poses[i + 1].pose.position.y);
  }

  // when current velocity < desired velocity
  // linear ramp-up profile
  if (curr_vel_x < desired_vel) {
    // linear ramp-up
    double ramp_distance =
        calculateDistance(curr_vel_x, desired_vel, acc_lim_x_);

    // find the ramp end index which stops accelerating
    std::size_t ramp_end_idx = 0;
    double temp_dist = 0.0;
    while ((ramp_end_idx < path_size - 1) && (temp_dist < ramp_distance)) {
      temp_dist += planner_utils::euclidean_distance(
          best_path.poses[ramp_end_idx].pose.position.x,
          best_path.poses[ramp_end_idx].pose.position.y,
          best_path.poses[ramp_end_idx + 1].pose.position.x,
          best_path.poses[ramp_end_idx + 1].pose.position.y);
      ramp_end_idx += 1;
    }

    // compute the velocities along the ramp
    auto vi = curr_vel_x;
    for (std::size_t i = 0; i < ramp_end_idx + 1; ++i) {
      double dist = planner_utils::euclidean_distance(
          best_path.poses[i].pose.position.x,
          best_path.poses[i].pose.position.y,
          best_path.poses[i + 1].pose.position.x,
          best_path.poses[i + 1].pose.position.y);
      double vf = calculateFinalSpeed(vi, acc_lim_x_, dist);
      // clamp speed to desired speed
      if (vf > desired_vel) vf = desired_vel;

      geometry_msgs::Pose pose;
      pose.position.x = best_path.poses[i].pose.position.x;
      pose.position.y = best_path.poses[i].pose.position.y;
      pose.orientation = best_path.poses[i].pose.orientation;
      trajectory_msg.poses.emplace_back(pose);
      trajectory_msg.velocity.push_back(vi);
      vi = vf;
    }

    // add constant velocities to the rest of the profile
    for (std::size_t i = ramp_end_idx + 1; i < path_size; ++i) {
      geometry_msgs::Pose pose;
      pose.position.x = best_path.poses[i].pose.position.x;
      pose.position.y = best_path.poses[i].pose.position.y;
      pose.orientation = best_path.poses[i].pose.orientation;
      trajectory_msg.poses.emplace_back(pose);
      trajectory_msg.velocity.push_back(desired_vel);
    }
  }
  // when current velocity >= desired velocity
  // trapezoidal profile
  else {
    // acceleration/deceleration distance from current velocity to reference
    // velocity
    double initial_distance = 0.0;
    if (curr_vel_x >= ref_vel)
      initial_distance = calculateDistance(curr_vel_x, ref_vel, -acc_lim_x_);
    else
      initial_distance = calculateDistance(curr_vel_x, ref_vel, acc_lim_x_);

    // deceleration distance from reference velocity to desired velocity
    // TODO(Phone): ref_vel must be always greater than or equal to desired_vel
    double decel_distance =
        calculateDistance(ref_vel, desired_vel, -acc_lim_x_);

    // if the summation of these two distances larger than total path length
    // we just need to build linear ramp-down profile from current velocity to
    // desired velocity, going backwards
    if (initial_distance + decel_distance > path_length) {
      trajectory_msg.poses.resize(path_size);
      trajectory_msg.velocity.resize(path_size);
      auto vf = desired_vel;
      for (std::size_t i = path_size - 2; i >= 0; i--) {
        double dist = planner_utils::euclidean_distance(
            best_path.poses[i].pose.position.x,
            best_path.poses[i].pose.position.y,
            best_path.poses[i + 1].pose.position.x,
            best_path.poses[i + 1].pose.position.y);
        // here we use position acceleration
        // because we are finding the initial speed instead of final speed
        double vi = calculateFinalSpeed(vf, acc_lim_x_, dist);
        // clamp to current speed
        if (vi > curr_vel_x) vi = curr_vel_x;
        trajectory_msg.poses[i].position.x = best_path.poses[i].pose.position.x;
        trajectory_msg.poses[i].position.y = best_path.poses[i].pose.position.y;
        trajectory_msg.poses[i].orientation =
            best_path.poses[i].pose.orientation;
        trajectory_msg.velocity[i] = vi;
        vf = vi;
      }

      // add the last pose and velocity
      trajectory_msg.poses[path_size - 1].position.x =
          best_path.poses[path_size - 1].pose.position.x;
      trajectory_msg.poses[path_size - 1].position.y =
          best_path.poses[path_size - 1].pose.position.y;
      trajectory_msg.poses[path_size - 1].orientation =
          best_path.poses[path_size - 1].pose.orientation;
      trajectory_msg.velocity[path_size - 1] = desired_vel;
    }

    // otherwise, we build a complete trapezoidal profile with three regions:
    // accelerating/decelerating to reference velocity, constant reference
    // velocity and decelerating to final desired velocity
    else {
      // compute decel_idx at which we start decelerating to final speed
      std::size_t decel_idx = path_size - 1;
      double temp_dist = 0.0;
      while (decel_idx > 0 && temp_dist < decel_distance) {
        temp_dist += planner_utils::euclidean_distance(
            best_path.poses[decel_idx].pose.position.x,
            best_path.poses[decel_idx].pose.position.y,
            best_path.poses[decel_idx - 1].pose.position.x,
            best_path.poses[decel_idx - 1].pose.position.y);
        decel_idx -= 1;
      }

      // initial index to stop accelerating/decelerating to reference speed from
      // current speed
      std::size_t initial_idx = 0;
      temp_dist = 0.0;
      while (initial_idx < decel_idx && temp_dist < initial_distance) {
        temp_dist += planner_utils::euclidean_distance(
            best_path.poses[initial_idx].pose.position.x,
            best_path.poses[initial_idx].pose.position.y,
            best_path.poses[initial_idx + 1].pose.position.x,
            best_path.poses[initial_idx + 1].pose.position.y);
        initial_idx += 1;
      }

      // The speeds from the start to initial_idx should be a linear ramp from
      // the current speed up/down to the reference speed,
      // accelerating/decelerating at acc_lim_x_/-acc_lim_x_.
      double vi = curr_vel_x;
      for (std::size_t i = 0; i < initial_idx; ++i) {
        double dist = planner_utils::euclidean_distance(
            best_path.poses[i].pose.position.x,
            best_path.poses[i].pose.position.y,
            best_path.poses[i + 1].pose.position.x,
            best_path.poses[i + 1].pose.position.y);
        double vf = 0.0;
        if (curr_vel_x >= ref_vel) {
          vf = calculateFinalSpeed(vi, -acc_lim_x_, dist);
          if (vf < ref_vel) vf = ref_vel;
        } else {
          vf = calculateFinalSpeed(vi, acc_lim_x_, dist);
          if (vf > ref_vel) vf = ref_vel;
        }
        geometry_msgs::Pose pose;
        pose.position.x = best_path.poses[i].pose.position.x;
        pose.position.y = best_path.poses[i].pose.position.y;
        pose.orientation = best_path.poses[i].pose.orientation;
        trajectory_msg.poses.emplace_back(pose);
        trajectory_msg.velocity.push_back(vi);
        vi = vf;
      }

      // in this portion of the profile, we are maintaining the reference speed
      for (std::size_t i = initial_idx; i < decel_idx; ++i) {
        geometry_msgs::Pose pose;
        pose.position.x = best_path.poses[i].pose.position.x;
        pose.position.y = best_path.poses[i].pose.position.y;
        pose.orientation = best_path.poses[i].pose.orientation;
        trajectory_msg.poses.emplace_back(pose);
        trajectory_msg.velocity.push_back(ref_vel);
      }

      // The speeds from the decel_idx to end of the path should be a linear
      // ramp from the reference speed down to the 0, decelerating at
      // -acc_lim_x_.
      vi = ref_vel;
      for (std::size_t i = decel_idx; i < path_size; ++i) {
        double dist = planner_utils::euclidean_distance(
            best_path.poses[i].pose.position.x,
            best_path.poses[i].pose.position.y,
            best_path.poses[i + 1].pose.position.x,
            best_path.poses[i + 1].pose.position.y);
        double vf = calculateFinalSpeed(vi, -acc_lim_x_, dist);
        // clamp speed to desired speed
        if (vf < desired_vel) vf = desired_vel;

        geometry_msgs::Pose pose;
        pose.position.x = best_path.poses[i].pose.position.x;
        pose.position.y = best_path.poses[i].pose.position.y;
        pose.orientation = best_path.poses[i].pose.orientation;
        trajectory_msg.poses.emplace_back(pose);
        trajectory_msg.velocity.push_back(vi);
        vi = vf;
      }
    }
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

  trajectory_msg.header.frame_id = best_path.header.frame_id;
  trajectory_msg.header.stamp = ros::Time::now();
}

double VelocityPlanner::calculateDistance(const double& v_i, const double& v_f,
                                          const double& a) const {
  return (std::pow(v_f, 2) - std::pow(v_i, 2)) / (2.0 * a);
}

double VelocityPlanner::calculateFinalSpeed(const double& v_i, const double& a,
                                            const double& d) const {
  double radical = std::pow(v_i, 2) + 2.0 * a * d;
  if (radical > 0) return std::sqrt(std::pow(v_i, 2) + 2.0 * a * d);
  return 0.0;
}

}  // namespace smooth_local_planner