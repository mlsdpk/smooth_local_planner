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
    Publish all the debugging messages if *debug* ros parameter is set to True.

## Parameters

### Conformal Lattice Planner related parameters

- ~\<name>\/lookahead_goal_dist (double, default: 1.0)  
    The lookahead goal distance in meters

- ~\<name>\/lattice_path_samples (int, default: 7)  
    The number of sampled lattice paths

- ~\<name>\/lattice_path_offset (int, default: 0.1)  
    The offset distance between goal poses of the sampled lattice paths in meters

- ~\<name>\/lattice_paths_pub (bool, default: false)  
    Whether or not publish the lattice path markers

### Optimizer related parameters

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

### Path Scoring Function related parameters

- ~\<name>\/global_path_distance_bias (double, default: 1.0)  
    The weighting for how much the local planner should stay close to the global path it was given

- ~\<name>\/collidiing_path_distance_bias (double, default: 1.0)  
    The weighting for how much the local planner should stay away from the other colliding lattice paths

### Debuging related parameters

- ~\<name>\/debug (bool, default: false)  
    Publish debugging messages if it is set to true.

## References

- [Coursera Self-Driving Cars Specialization: Course 4 Motion Planning for Self-Driving Cars](https://www.coursera.org/learn/motion-planning-self-driving-cars?specialization=self-driving-cars)
- Kelly, A., & Nagy, B. (2003). Reactive nonholonomic trajectory generation via parametric optimal control. The International Journal of Robotics Research, 22(7-8), 583-601.
- McNaughton, M., Urmson, C., Dolan, J. M., & Lee, J. W. (2011, May). Motion planning for autonomous driving with a conformal spatiotemporal lattice. In 2011 IEEE International Conference on Robotics and Automation (pp. 4889-4895). IEEE.