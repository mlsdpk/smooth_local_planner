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
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    smooth_local_planner
  CATKIN_DEPENDS
    roscpp
    pluginlib
    base_local_planner
    nav_msgs
)

add_library(smooth_local_planner
  src/smooth_local_planner_ros.cpp
)

add_dependencies(smooth_local_planner 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
	smooth_local_planner
	${catkin_LIBRARIES}
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

