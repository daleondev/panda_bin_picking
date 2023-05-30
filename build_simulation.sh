#!/bin/bash

# install ros packages
sudo apt install ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-rospy-message-converter ros-$ROS_DISTRO-effort-controllers ros-$ROS_DISTRO-joint-state-controller ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-moveit-visual-tools ros-$ROS_DISTRO-timed-roslaunch ros-$ROS_DISTRO-realsense2-* ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers ros-$ROS_DISTRO-libfranka ros-$ROS_DISTRO-franka-ros ros-$ROS_DISTRO-gazebo*

source /opt/ros/$ROS_DISTRO/setup.bash

# clean
cd ..
sudo rm -r franka_ros gpd_ros panda_moveit_config .rosinstall*

# install dependencies
wstool init
wstool merge panda_bin_picking/dependencies_simulation.rosinstall
wstool up
cd .. 
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# modify for simulation
cp src/panda_bin_picking/panda_realsense_simulation/resources/*.xacro src/franka_ros/franka_description/robots/
cp src/panda_bin_picking/panda_realsense_simulation/resources/*.yaml src/panda_moveit_config/config
cp src/panda_bin_picking/panda_realsense_simulation/resources/*.launch.xml src/panda_moveit_config/launch
mkdir ~/.gazebo/models
cp -r src/panda_bin_picking/panda_realsense_simulation/models/* ~/.gazebo/models

# build
catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-DSIMULATION"
catkin clean -y
catkin build gpd_ros
catkin build
