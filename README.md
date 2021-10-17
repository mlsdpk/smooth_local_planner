# smooth_local_planner package

> This local planner is currently in progress.

The smooth_local_planner ROS package implements the custom local planner plugin to the base_local_planner of the 2D navigation stack. This planner is suitable mostly for car-like robots with curvature constraints however the planner has no restriction to the type of the robot. Although we use the term "local planner", the underlying planner utilizes the hierarchical manner, composed of various planning components to achieve the collison-free optimal motion planning. Specifically, smooth_local_planner mainly consists of three components: conformal lattice planning, velocity profile generation and path tracking.

## References

- [Coursera Self-Driving Cars Specialization: Course 4 Motion Planning for Self-Driving Cars](https://www.coursera.org/learn/motion-planning-self-driving-cars?specialization=self-driving-cars)
- Kelly, A., & Nagy, B. (2003). Reactive nonholonomic trajectory generation via parametric optimal control. The International Journal of Robotics Research, 22(7-8), 583-601.
- McNaughton, M., Urmson, C., Dolan, J. M., & Lee, J. W. (2011, May). Motion planning for autonomous driving with a conformal spatiotemporal lattice. In 2011 IEEE International Conference on Robotics and Automation (pp. 4889-4895). IEEE.