#include <smooth_local_planner/smooth_local_planner_ros.h>

namespace smooth_local_planner {

SmoothLocalPlannerROS::SmoothLocalPlannerROS() : initialized_{false} {}

SmoothLocalPlannerROS::~SmoothLocalPlannerROS() {}

void SmoothLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                                       costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);

    // ros parameters
    // most of the parameters will be handled in its own classes
    private_nh.param("lattice_paths_pub", lattice_paths_pub_, false);
    private_nh.param("global_path_distance_bias", global_path_distance_bias_,
                     1.0);
    private_nh.param("collidiing_path_distance_bias",
                     collidiing_path_distance_bias_, 1.0);
    private_nh.param("debug", debug_, false);

    debug_msg_pub_ =
        private_nh.advertise<smooth_local_planner::DebugMsg>("debug_msg", 10);

    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // get the current robot pose in the map frame
    costmap_ros_->getRobotPose(current_pose_);

    conformal_lattice_planner_ =
        std::make_shared<ConformalLatticePlanner>(name);

    collision_checker_ = std::make_shared<GridCollisionChecker>(
        costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint());

    visualizer_ = std::make_shared<visualizer::Visualizer>(name);

    initialized_ = true;
  }
}

bool SmoothLocalPlannerROS::computeVelocityCommands(
    geometry_msgs::Twist& cmd_vel) {
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  // transform the global plan into robot frame
  // this also removes part of the path that is outside of the local
  // costmap and also prunes the path behind the robot
  nav_msgs::Path transformed_plan;
  transformGlobalPlan(transformed_plan, current_pose_);

  // conformal lattice planning
  // resultant lattice paths and lookahead_goal_pose are now in robot frame
  std::vector<SpiralPath> lattice_paths;
  geometry_msgs::PoseStamped lookahead_goal_pose;
  std::vector<geometry_msgs::PoseStamped> goal_poses;
  std::vector<double> obj_costs;
  conformal_lattice_planner_->plan(lattice_paths, lookahead_goal_pose,
                                   goal_poses, obj_costs, transformed_plan);

  if (debug_) {
    DebugMsg msg;
    msg.objective_costs.resize(obj_costs.size());
    msg.objective_costs.swap(obj_costs);
    debug_msg_pub_.publish(msg);
  }

  // once the lattice paths are generated, we perform collision checking for
  // each path and remove the collision ones

  // auto init_time1 = std::chrono::system_clock::now();

  // first we need to convert paths into global frame
  // TODO(Phone): need to make this conversion faster
  std::vector<nav_msgs::Path> paths;
  paths.resize(lattice_paths.size());
  const std::string global_frame_id = costmap_ros_->getGlobalFrameID();

  for (std::size_t i = 0; i < paths.size(); i++) {
    nav_msgs::Path temp_path;
    temp_path.header.frame_id = global_frame_id;
    // TODO: make sure spiral x, y and theta points have same length
    std::size_t s_size = lattice_paths[i].x_points.size();
    temp_path.poses.resize(s_size);
    for (std::size_t j = 0; j < s_size; ++j) {
      geometry_msgs::PoseStamped temp_pose;
      temp_pose.header.frame_id = lattice_paths[i].frame_id;
      temp_pose.pose.position.x = lattice_paths[i].x_points[j];
      temp_pose.pose.position.y = lattice_paths[i].y_points[j];
      planner_utils::convertToQuaternion(lattice_paths[i].theta_points[j],
                                         temp_pose.pose.orientation);
      if (!transformPose(temp_pose, temp_path.poses[j], global_frame_id)) {
        throw("Unable to transform robot pose into global plan's frame");
      }
    }
    paths[i].header.frame_id = temp_path.header.frame_id;
    paths[i].poses.swap(temp_path.poses);
  }

  // auto time_diff1 = std::chrono::duration_cast<std::chrono::microseconds>(
  //                       std::chrono::system_clock::now() - init_time1)
  //                       .count();
  // std::cout << "path converting time diff: " << time_diff1 << std::endl;

  std::vector<bool> collision_status;
  collision_checker_->checkCollision(collision_status, paths);

  // now choose the best lattice path which minimizes the given objective.

  std::size_t best_path_idx;
  bool local_path_exists = getBestPathIndex(
      best_path_idx, lattice_paths, lookahead_goal_pose, collision_status);

  // visualize everything
  if (lattice_paths_pub_) {
    visualizer_->publishMarkers(lattice_paths, goal_poses, collision_status);
  }
  visualizer_->publishGlobalPlan(transformed_plan);
  // temporary conditional statement for visualizing local path
  if (local_path_exists)
    visualizer_->publishLocalPlan(lattice_paths, best_path_idx);

  return true;
}

void SmoothLocalPlannerROS::transformGlobalPlan(
    nav_msgs::Path& transformed_plan, const geometry_msgs::PoseStamped& pose) {
  if (global_plan_.empty()) {
    throw("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(pose, robot_pose, global_plan_[0].header.frame_id)) {
    throw("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  const double max_costmap_dim =
      std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());
  const double max_transform_dist =
      max_costmap_dim * costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin = planner_utils::min_by(
      global_plan_.begin(), global_plan_.end(),
      [&robot_pose](const geometry_msgs::PoseStamped& ps) {
        return planner_utils::euclidean_distance(robot_pose, ps);
      });

  // Find points definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
      transformation_begin, std::end(global_plan_),
      [&](const auto& global_plan_pose) {
        return planner_utils::euclidean_distance(robot_pose, global_plan_pose) >
               max_transform_dist;
      });

  // Lambda to transform a PoseStamped from global frame to local
  auto transform_global_pose_to_local = [&](const auto& global_plan_pose) {
    geometry_msgs::PoseStamped stamped_pose, transformed_pose;
    stamped_pose.header.frame_id = global_plan_[0].header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    transformPose(stamped_pose, transformed_pose,
                  costmap_ros_->getBaseFrameID());
    return transformed_pose;
  };

  // Transform the near part of the global plan into the robot's frame of
  // reference.
  std::transform(transformation_begin, transformation_end,
                 std::back_inserter(transformed_plan.poses),
                 transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.erase(std::begin(global_plan_), transformation_begin);

  if (transformed_plan.poses.empty()) {
    throw("Resulting plan has 0 poses in it.");
  }
}

bool SmoothLocalPlannerROS::transformPose(
    const geometry_msgs::PoseStamped& in_pose,
    geometry_msgs::PoseStamped& out_pose, const std::string& frame) const {
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("Exception in transformPose: %s", ex.what());
  }
  return false;
}

bool SmoothLocalPlannerROS::getBestPathIndex(
    std::size_t& idx, const std::vector<SpiralPath>& paths,
    const geometry_msgs::PoseStamped& lookahead_goal_pose,
    const std::vector<bool>& collision_status) const {
  // current score function does not consider distance from obstacles yet
  // we will see how we can add more objectives in our score function in the
  // future releases
  //
  // score function =
  //      path_distance_bias * (distance from global path) +
  //      collidiing_path_distance_bias * (distance from other colliding paths
  //      if exists) +
  bool success = false;
  double best_score = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < paths.size(); ++i) {
    double score = std::numeric_limits<double>::infinity();

    if (!collision_status[i]) {
      // calculate distance from global path
      std::size_t i_size = paths[i].x_points.size();
      double dist_from_global =
          std::sqrt(std::pow(paths[i].x_points[i_size - 1] -
                                 lookahead_goal_pose.pose.position.x,
                             2) +
                    std::pow(paths[i].y_points[i_size - 1] -
                                 lookahead_goal_pose.pose.position.y,
                             2));

      // distance from other colliding paths
      double dist_from_colliding_paths = 0.0;
      for (std::size_t j = 0; j < paths.size(); ++j) {
        if (j == i) continue;
        if (collision_status[j]) {
          std::size_t j_size = paths[j].x_points.size();
          dist_from_colliding_paths +=
              std::sqrt(std::pow(paths[i].x_points[i_size - 1] -
                                     paths[j].x_points[j_size - 1],
                                 2) +
                        std::pow(paths[i].y_points[i_size - 1] -
                                     paths[j].y_points[j_size - 1],
                                 2));
        }
      }

      // total score
      score = global_path_distance_bias_ * dist_from_global +
              collidiing_path_distance_bias_ * dist_from_colliding_paths;
    }

    // set the best index to be the path index with the lowest score
    if (score < best_score) {
      best_score = score;
      idx = i;
      success = true;
    }
  }
  return success;
}

bool SmoothLocalPlannerROS::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }
  return false;
}

bool SmoothLocalPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_ERROR(
        "This planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }
  global_plan_ = plan;
  return true;
}

};  // namespace smooth_local_planner

#include <pluginlib/class_list_macros.h>
// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(smooth_local_planner::SmoothLocalPlannerROS,
                       nav_core::BaseLocalPlanner)