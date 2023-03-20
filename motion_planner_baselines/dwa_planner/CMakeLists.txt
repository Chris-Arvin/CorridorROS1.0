cmake_minimum_required(VERSION 3.0.2)
project(dwa_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_local_planner
  costmap_2d
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
  tf2
)

# Eigen3
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dwa_planner
  CATKIN_DEPENDS 
    base_local_planner 
    costmap_2d
    nav_core 
    nav_msgs 
    pluginlib 
    roscpp 
    tf 
    tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/dwa_planner_ros.cpp src/dwa_planner.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})