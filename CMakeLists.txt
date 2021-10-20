cmake_minimum_required(VERSION 3.0.2)
project(smooth_local_planner)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nav_core
  nav_msgs
  base_local_planner
  costmap_2d
  geometry_msgs
  std_msgs
  visualization_msgs
  cmake_modules
  dynamic_reconfigure
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(CppAD REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(IPOPT REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CPPAD_INCLUDE_DIRS}
  ${IPOPT_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS 
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES 
    smooth_local_planner
  CATKIN_DEPENDS
    roscpp
    pluginlib
    nav_core
    nav_msgs
    base_local_planner
    costmap_2d
    geometry_msgs
    visualization_msgs
    std_msgs
    dynamic_reconfigure
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

add_library(smooth_local_planner
  src/smooth_local_planner_ros.cpp
  src/path_optimizer.cpp
  src/conformal_lattice_planner.cpp
  src/visualizer.cpp
)

add_dependencies(smooth_local_planner 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
	smooth_local_planner
	${catkin_LIBRARIES}
  ${CPPAD_LIBRARY_DIRS}
  ${IPOPT_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  smooth_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  smooth_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

