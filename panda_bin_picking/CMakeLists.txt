cmake_minimum_required(VERSION 3.0.2)
project(panda_bin_picking)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_visual_tools
  moveit_ros_planning_interface
  std_msgs
  std_srvs
  franka_gripper
  gpd_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(CATKIN_DEPENDS 
  roscpp
  moveit_visual_tools 
  moveit_ros_planning_interface
  std_msgs 
  std_srvs
  franka_gripper
  gpd_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(panda_bin_picking_server src/panda_bin_picking_server.cpp src/franka_panda.cpp src/realsense_l515.cpp src/visualizer.cpp)
add_dependencies(panda_bin_picking_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(panda_bin_picking_server
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(panda_bin_picking_client src/panda_bin_picking_client.cpp)
target_link_libraries(panda_bin_picking_client
  ${catkin_LIBRARIES}
)

add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/panda_bin_picking_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)