# smooth_local_planner package

> This local planner is currently in progress.

The smooth_local_planner ROS package implements the custom local planner plugin to the base_local_planner of the 2D navigation stack. This planner is suitable mostly for car-like robots with curvature constraints however the planner has no restriction to the type of the robot. Although we use the term "local planner", the underlying planner utilizes the hierarchical manner, composed of various planning components to achieve the collison-free optimal motion planning. Specifically, smooth_local_planner mainly consists of three components: conformal lattice planning, velocity profile generation and path tracking.

## Published Topics

- ~\<name>\/global_plan (nav_msgs/Path)  
    The portion of the global plan that the local planner is currently attempting to follow. Used primarily for visualization purposes.

- ~\<name>\/local_plan (nav_msgs/Path)  
    The local plan or trajectory that the smooth_local_planner optimizes and follows. Used primarily for visualization purposes.

- ~\<name>\/lattice_path_markers (visualization_msgs/MarkerArray)  
    The local path segments produced by the lattice planner. Used primarily for visualization purposes.

- ~\<name>\/debug_msg (smooth_local_planner/DebugMsg)  
    The debug message contains the cost values of the objective function used by the optimizer. Used primarily for evaluation and debugging. Parameter ~<name>/debug must be enabled.

## Parameters

### Conformal Lattice Planner Parameters

- ~\<name>\/lookahead_base_dist (double, default: 1.0)  
    The based lookahead goal distance in meters

- ~\<name>\/lookahead_time (double, default: 1.0)  
    Number of seconds to lookahead in the future after lookahead_base_dist

- ~\<name>\/lattice_path_samples (int, default: 7)  
    The number of sampled lattice paths

- ~\<name>\/lattice_path_offset (int, default: 0.1)  
    The offset distance between goal poses of the sampled lattice paths in meters

- ~\<name>\/lattice_paths_pub (bool, default: false)  
    Whether or not publish the lattice path markers

### Optimizer Parameters

- ~\<name>\/min_turning_radius (double, default: 0.2)  
    Minimum turning radius of a carlike robot (set to zero for a diff-drive robot)

- ~\<name>\/penalty_alpha (double, default: 25.0)  
    The weight of the penalty function for x position

- ~\<name>\/penalty_beta (double, default: 25.0)  
    The weight of the penalty function for y position

- ~\<name>\/penalty_gamma (double, default: 30.0)  
    The weight of the penalty function for orientation

- ~\<name>\/simpson_intervals (int, default: 8)  
    The n intervals of Simpson's rule (n must be EVEN) 

### Path Scoring Function Parameters

- ~\<name>\/global_path_distance_bias (double, default: 1.0)  
    The weighting for how much the local planner should stay close to the global path it was given

- ~\<name>\/collidiing_path_distance_bias (double, default: 1.0)  
    The weighting for how much the local planner should stay away from the other colliding lattice paths

### Velocity Profile Generation Parameters

- ~\<name>\/ref_vel (double, default: 0.15)  
    The reference velocity for the robot to follow the global path in m/s

- ~\<name>\/max_vel_x (double, default: 0.22)  
    The maximum translational velocity of the robot in x-direction in m/s

- ~\<name>\/acc_lim_x (double, default: 2.5)  
    The maximum acceleration of the robot in x-direction in m/s^2

### Controller Parameters

- ~\<name>\/lookahead_dist (double, default: 0.2)  
    The lookahead distance of the pure pursuit path tracking algorithm in meters
    
### Goal Tolerance Parameters

- ~\<name>\/xy_goal_tolerance (double, default: 0.2)  
    Allowed final euclidean distance to the goal position in meters

- ~\<name>\/yaw_goal_tolerance (double, default: 0.2)  
    Allowed final orientation error in radians

### Other Parameters

- ~\<name>\/odom_topic (string, default: "odom")  
    Topic name of the odometry message, provided by the robot driver or simulator

- ~\<name>\/planning_frequency (double, default: 10.0)  
    The frequency at which the lattice planning and velocity profile generation will be called in Hz. This frequency cannot be greater than controller_frequency, otherwise it will be set the same as controller_frequency

- ~\<name>\/debug (bool, default: false)  
    Publish debugging messages if it is set to true

## References

- [Coursera Self-Driving Cars Specialization: Course 4 Motion Planning for Self-Driving Cars](https://www.coursera.org/learn/motion-planning-self-driving-cars?specialization=self-driving-cars)
- Kelly, A., & Nagy, B. (2003). Reactive nonholonomic trajectory generation via parametric optimal control. The International Journal of Robotics Research, 22(7-8), 583-601.
- McNaughton, M., Urmson, C., Dolan, J. M., & Lee, J. W. (2011, May). Motion planning for autonomous driving with a conformal spatiotemporal lattice. In 2011 IEEE International Conference on Robotics and Automation (pp. 4889-4895). IEEE.