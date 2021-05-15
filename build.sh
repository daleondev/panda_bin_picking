#!/bin/bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros-control ros-$ROS_DISTRO-rospy-message-converter ros-$ROS_DISTRO-effort-controllers ros-$ROS_DISTRO-joint-state-controller ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-moveit-visual-tools ros-$ROS_DISTRO-timed-roslaunch ros-$ROS_DISTRO-realsense2-* ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers

sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

source /opt/ros/$ROS_DISTRO/setup.bash
cd ..
wstool init
wstool merge panda_realsense_simulation/dependencies.rosinstall
wstool up

cd .. 
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
