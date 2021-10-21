#include <smooth_local_planner/conformal_lattice_planner.h>

namespace smooth_local_planner {

ConformalLatticePlanner::ConformalLatticePlanner(const std::string& name) {
  ros::NodeHandle private_nh("~/" + name);

  // conformal lattice planner related parameters
  private_nh.param("lookahead_goal_dist", lookahead_goal_dist_, 1.0);
  private_nh.param("lattice_path_samples", lattice_path_samples_, 7);
  private_nh.param("lattice_path_offset", lattice_path_offset_, 0.1);

  // optimizer related parameters
  private_nh.param("min_turning_radius", min_turning_radius_, 0.2);
  private_nh.param("penalty_alpha", penalty_alpha_, 25.0);
  private_nh.param("penalty_beta", penalty_beta_, 25.0);
  private_nh.param("penalty_gamma", penalty_gamma_, 30.0);

  // TODO(Phone): check whether it is even or not
  // warn the user if not even and use the default one
  private_nh.param("simpson_intervals", simpson_intervals_, 8);

  // initialize path optimizer
  double max_curvature = 1e19;
  if (min_turning_radius_ > 0.0) max_curvature = 1.0 / min_turning_radius_;
  optimizer_ = std::make_shared<PathOptimizer>(max_curvature);
  optimizer_->setPenaltyConstants(penalty_alpha_, penalty_beta_,
                                  penalty_gamma_);
  optimizer_->setSimpsonN(static_cast<unsigned>(simpson_intervals_));
}

ConformalLatticePlanner::~ConformalLatticePlanner() {}

bool ConformalLatticePlanner::plan(
    std::vector<SpiralPath>& paths,
    geometry_msgs::PoseStamped& lookahead_goal_pose,
    std::vector<geometry_msgs::PoseStamped>& goal_poses,
    std::vector<double>& obj_costs, const nav_msgs::Path& global_plan) const {
  // now we find the local goal pose
  // find the first pose in the tranformed local plan which is at a distance
  // greater than the lookahead distance

  auto goal_pose_it = std::find_if(
      global_plan.poses.begin(), global_plan.poses.end(), [&](const auto& ps) {
        return std::hypot(ps.pose.position.x, ps.pose.position.y) >=
               lookahead_goal_dist_;
      });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == global_plan.poses.end()) {
    goal_pose_it = std::prev(global_plan.poses.end());
  }

  lookahead_goal_pose = *goal_pose_it;

  // we now get the local lookahead goal pose
  // based on the number of sampled paths and offset distance defined by the
  // user, we find more goal poses which are laterally offset with respect to
  // the goal heading.
  generateGoalSet(goal_poses, lookahead_goal_pose);

  paths.resize(goal_poses.size());
  obj_costs.resize(goal_poses.size());
  for (std::size_t i = 0; i < goal_poses.size(); ++i) {
    paths[i].frame_id = global_plan.header.frame_id;
    obj_costs[i] = optimizer_->optimizeSpiral(
        paths[i], goal_poses[i].pose.position.x, goal_poses[i].pose.position.y,
        tf2::getYaw(goal_poses[i].pose.orientation));
  }
  return true;
}

void ConformalLatticePlanner::generateGoalSet(
    std::vector<geometry_msgs::PoseStamped>& goal_poses,
    const geometry_msgs::PoseStamped& lookahead_goal_pose) const {
  const auto goal_theta = tf2::getYaw(lookahead_goal_pose.pose.orientation);
  for (auto i = 0; i < lattice_path_samples_; ++i) {
    auto offset = (i - lattice_path_samples_ / 2) * lattice_path_offset_;
    auto x_offset = offset * std::cos(goal_theta + M_PI / 2.0);
    auto y_offset = offset * std::sin(goal_theta + M_PI / 2.0);

    geometry_msgs::PoseStamped temp_goal;
    temp_goal.header = lookahead_goal_pose.header;
    temp_goal.pose.position.x = lookahead_goal_pose.pose.position.x + x_offset;
    temp_goal.pose.position.y = lookahead_goal_pose.pose.position.y + y_offset;
    temp_goal.pose.position.z = lookahead_goal_pose.pose.position.z;
    temp_goal.pose.orientation = lookahead_goal_pose.pose.orientation;

    goal_poses.emplace_back(std::move(temp_goal));
  }
}

}  // namespace smooth_local_planner