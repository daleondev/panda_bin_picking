cmake_minimum_required(VERSION 3.0.2)
project(panda_bin_picking)

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
  gpd_ros
  std_msgs
  std_srvs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)

catkin_package(
  CATKIN_DEPENDS roscpp moveit_core moveit_visual_tools moveit_ros_planning_interface tf2_geometry_msgs sensor_msgs pcl pcl_conversions pcl_ros std_msgs std_srvs
  DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# add_executable(path_planning_node src/path_planning_node.cpp)

# target_link_libraries(path_planning_node
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

add_executable(panda_bin_picking src/panda_bin_picking.cpp src/FrankaPanda.cpp src/RealsenseL515.cpp src/Visualizer.cpp)
add_dependencies(panda_bin_picking ${catkin_EXPORTED_TARGETS})
target_link_libraries(panda_bin_picking
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)