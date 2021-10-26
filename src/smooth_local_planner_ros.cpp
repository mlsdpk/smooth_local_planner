#include <smooth_local_planner/smooth_local_planner_ros.h>

namespace smooth_local_planner {

SmoothLocalPlannerROS::SmoothLocalPlannerROS()
    : initialized_{false}, goal_reached_{false} {}

SmoothLocalPlannerROS::~SmoothLocalPlannerROS() {}

void SmoothLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                                       costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);

    // ros parameters
    // most of the parameters will be handled in its own classes
    std::string odom_topic = "odom";
    private_nh.param("odom_topic", odom_topic, odom_topic);
    private_nh.param("lattice_paths_pub", lattice_paths_pub_, false);
    private_nh.param("global_path_distance_bias", global_path_distance_bias_,
                     1.0);
    private_nh.param("collidiing_path_distance_bias",
                     collidiing_path_distance_bias_, 1.0);
    private_nh.param("debug", debug_, false);
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);
    private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.2);
    private_nh.param("ref_vel", ref_vel_, 0.15);
    private_nh.param("lookahead_base_dist", lookahead_base_dist_, 1.0);
    private_nh.param("lookahead_time", lookahead_time_, 1.0);

    // Assuming this planner is being run within the navigation stack, we can
    // just do an upward search for the frequency at which its being run. This
    // also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    double controller_frequency = 0.0;
    if (!private_nh.searchParam("controller_frequency",
                                controller_frequency_param_name)) {
      ROS_ERROR(
          "controller_frequency parameter not found. Make sure it has been "
          "set. Exiting...");
      exit(1);
    } else {
      private_nh.param(controller_frequency_param_name, controller_frequency,
                       controller_frequency);
      if (controller_frequency <= 0) {
        ROS_ERROR("A controller_frequency must be greater than 0.");
        exit(1);
      }
    }
    ROS_INFO("controller_frequency is set to %.2f", controller_frequency);

    // planning frequency for lattice planning and velocity profile generation
    // this must be lower than controller_frequency
    // usually set as half of the controller_frequency
    private_nh.param("planning_frequency", planning_frequency_, 10.0);
    if (planning_frequency_ > controller_frequency) {
      ROS_WARN(
          "A smooth_local_planner planning_frequency greater than "
          "controller_frequency has been set. Ignoring the parameter, assuming "
          "the same rate of %.2fHz as controller_frequency.",
          controller_frequency);
      planning_frequency_ = controller_frequency;
    }
    ROS_INFO("smooth_local_planner planning_frequency is set to %.2f",
             planning_frequency_);

    // publishers
    debug_msg_pub_ =
        private_nh.advertise<smooth_local_planner::DebugMsg>("debug_msg", 10);

    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // get the current robot pose in the map frame
    costmap_ros_->getRobotPose(current_pose_);

    // main classes initialization
    conformal_lattice_planner_ =
        std::make_shared<ConformalLatticePlanner>(name);

    collision_checker_ = std::make_shared<GridCollisionChecker>(
        costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint());

    visualizer_ = std::make_shared<visualizer::Visualizer>(name);

    odom_helper_ =
        std::make_shared<base_local_planner::OdometryHelperRos>(odom_topic);

    velocity_planner_ =
        std::make_shared<VelocityPlanner>(name, odom_helper_.get());

    controller_ = std::make_shared<Controller>(name);

    planning_init_time_ = ros::Time::now();
    initialized_ = true;
  }
}

bool SmoothLocalPlannerROS::computeVelocityCommands(
    geometry_msgs::Twist& cmd_vel) {
  if (!costmap_ros_->getRobotPose(current_pose_)) {
    ROS_ERROR("Could not get robot pose");
    return false;
  }

  // transform the global plan into local frame
  // this also removes part of the path that is outside of the local
  // costmap and also prunes the path behind the robot
  nav_msgs::Path transformed_plan;
  if (!transformGlobalPlan(transformed_plan, current_pose_)) {
    ROS_WARN(
        "Could not transform the global plan to the frame of the controller");
    return false;
  }

  // here we check whether the robot is within the goal region or not
  // TODO: make sure robot stopped with zero velocity
  if (isRobotWithinGoalTolerances()) {
    goal_reached_ = true;
    return true;
  }

  // behavior planning, lattice planning and velocity profile generation must
  // operate in lower rate than controller
  if ((ros::Time::now() - planning_init_time_).toSec() >
      (1.0 / planning_frequency_)) {
    // Here we perform behavior planning.
    // We obtain the lookahead goal pose from the transformed local plan and
    // set the speed to the lookahead goal pose.
    // In the future, this can be integrated with dynamic object detections to
    // generate different behaviors. This lookahead goal pose distance should be
    // chosen based on robot current speed.
    GoalPoseWithSpeed goal_pose_with_speed;
    setLookAheadGoalPoseAndSpeed(goal_pose_with_speed, transformed_plan);

    // conformal lattice planning
    // resultant lattice paths and lookahead_goal_pose are now in local frame
    std::vector<SpiralPath> lattice_paths;
    std::vector<geometry_msgs::PoseStamped> goal_poses;
    std::vector<double> obj_costs;
    conformal_lattice_planner_->plan(lattice_paths, goal_poses, obj_costs,
                                     goal_pose_with_speed.pose);

    if (debug_) {
      DebugMsg msg;
      msg.objective_costs.resize(obj_costs.size());
      msg.objective_costs.swap(obj_costs);
      debug_msg_pub_.publish(msg);
    }

    // once the lattice paths are generated, we perform collision checking for
    // each path and remove the collision ones

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

    std::vector<bool> collision_status;
    collision_checker_->checkCollision(collision_status, paths);

    // now choose the best lattice path which minimizes the given objective.
    std::size_t best_path_idx;
    bool local_path_exists =
        getBestPathIndex(best_path_idx, lattice_paths,
                         goal_pose_with_speed.pose, collision_status);

    // velocity profile generation
    Trajectory2DMsg trajectory_msg;
    velocity_planner_->computeVelocityProfile(trajectory_msg,
                                              lattice_paths[best_path_idx],
                                              goal_pose_with_speed.velocity);

    // std::cout << "velocity" << std::endl;
    // for (std::size_t i = 0; i < trajectory_msg.velocity.size(); ++i) {
    //   std::cout << trajectory_msg.velocity[i] << std::endl;
    // }
    // std::cout << "---" << std::endl;

    // TODO: linear interpolation between trajectory points to obtain a fine
    // resolution

    // if local path exists, update the controller
    if (local_path_exists) controller_->updateTrajectory(trajectory_msg);

    // visualize lattice paths
    if (lattice_paths_pub_) {
      visualizer_->publishMarkers(lattice_paths, goal_poses, collision_status);
    }

    planning_init_time_ = ros::Time::now();
  }

  // controller
  controller_->updateControls(cmd_vel, current_pose_);

  // visualize transformed and pruned global plan
  visualizer_->publishGlobalPlan(transformed_plan);
  // TODO(Phone): visualize local path that the controller is currently
  // attempting to follow
  // if (local_path_exists)
  //   visualizer_->publishLocalPlan(lattice_paths, best_path_idx);

  return true;
}

bool SmoothLocalPlannerROS::transformGlobalPlan(
    nav_msgs::Path& transformed_plan, const geometry_msgs::PoseStamped& pose) {
  if (global_plan_.empty()) {
    ROS_ERROR("Received plan with zero length");
    return false;
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(pose, robot_pose, global_plan_[0].header.frame_id)) {
    ROS_ERROR("Unable to transform robot pose into global plan's frame");
    return false;
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
    ROS_ERROR("Resulting plan has 0 poses in it.");
    return false;
  }
  return true;
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
  if (goal_reached_) {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

bool SmoothLocalPlannerROS::isRobotWithinGoalTolerances() {
  const geometry_msgs::PoseStamped& plan_goal_pose = global_plan_.back();
  geometry_msgs::TransformStamped plan_to_global_transform =
      tf_->lookupTransform(costmap_ros_->getGlobalFrameID(), ros::Time(),
                           plan_goal_pose.header.frame_id,
                           plan_goal_pose.header.stamp,
                           plan_goal_pose.header.frame_id, ros::Duration(0.5));

  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, plan_to_global_transform);

  const double dx = global_goal.pose.position.x - current_pose_.pose.position.x;
  const double dy = global_goal.pose.position.y - current_pose_.pose.position.y;
  const double dth = angles::shortest_angular_distance(
      tf2::getYaw(current_pose_.pose.orientation),
      tf2::getYaw(global_goal.pose.orientation));

  if (fabs(std::sqrt(dx * dx + dy * dy)) < xy_goal_tolerance_ &&
      fabs(dth) < yaw_goal_tolerance_) {
    return true;
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

  // reset goal_reached_ flag
  goal_reached_ = false;
  return true;
}

void SmoothLocalPlannerROS::setLookAheadGoalPoseAndSpeed(
    GoalPoseWithSpeed& goal_pose, const nav_msgs::Path& global_plan) {
  // get the current robot speed
  geometry_msgs::PoseStamped curr_robot_vel;
  odom_helper_->getRobotVel(curr_robot_vel);

  // find the first pose in the tranformed local plan which is at a distance
  // greater than the lookahead distance
  // calculate lookahead distance based on current robot speed
  const double lookahead_dist =
      lookahead_base_dist_ + curr_robot_vel.pose.position.x * lookahead_time_;

  goal_pose.velocity = ref_vel_;
  auto goal_pose_it = std::find_if(
      global_plan.poses.begin(), global_plan.poses.end(), [&](const auto& ps) {
        return std::hypot(ps.pose.position.x, ps.pose.position.y) >=
               lookahead_dist;
      });

  // If no pose is not far enough, take the last pose
  // also set the final goal pose speed to be zero
  if (goal_pose_it == global_plan.poses.end()) {
    goal_pose_it = std::prev(global_plan.poses.end());
    goal_pose.velocity = 0.0;
  }
  goal_pose.pose.header.frame_id = global_plan.header.frame_id;
  goal_pose.pose = *goal_pose_it;
}

};  // namespace smooth_local_planner

#include <pluginlib/class_list_macros.h>
// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(smooth_local_planner::SmoothLocalPlannerROS,
                       nav_core::BaseLocalPlanner)