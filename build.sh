#!/bin/bash

sudo apt remove "*libfranka*"

# install ros packages
sudo apt install ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-rospy-message-converter ros-$ROS_DISTRO-effort-controllers ros-$ROS_DISTRO-joint-state-controller ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-moveit-visual-tools ros-$ROS_DISTRO-timed-roslaunch ros-$ROS_DISTRO-realsense2-* ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers

# install librealsense

sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key #F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
pip2 install pyrealsense2

source /opt/ros/$ROS_DISTRO/setup.bash

# clean
cd ..
sudo rm -r franka_ros gpd_ros panda_moveit_config .rosinstall*

# install dependencies
wstool init
wstool merge panda_bin_picking/dependencies.rosinstall
wstool up
cd .. 
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys libfranka

# modify for reality
cp src/panda_bin_picking/panda_realsense/resources/*.srdf.xacro src/panda_moveit_config/config/
cp src/panda_bin_picking/panda_realsense/resources/*.yaml src/panda_moveit_config/config
cp src/panda_bin_picking/panda_realsense/resources/*.launch.xml src/panda_moveit_config/launch

# build
catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
catkin clean -y
catkin build gpd_ros
catkin build
