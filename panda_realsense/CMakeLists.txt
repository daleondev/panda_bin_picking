cmake_minimum_required(VERSION 2.8.3)
project(panda_realsense)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)
catkin_package(
  LIBRARIES path_planning
  CATKIN_DEPENDS roscpp moveit_core moveit_visual_tools moveit_ros_planning_interface tf2_geometry_msgs
  DEPENDS system_lib Eigen3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)