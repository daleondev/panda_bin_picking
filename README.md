# Panda Bin Picking

## Description
This ROS metapackage provides a bin-picking-application which can either be used inside the gazebo simulator or with the real robot and camera. The used robot is the Panda from FRANKA EMIKA and the deapth camera is the Intel RealSense LiDAR Camera L515.

The application sequence is the following:
1. Capture pointclouds of the objects in the workspace
2. Send the fused pointcloud to the Grasp Pose Detection package (gpd_ros) and receive the generated grasp poses
3. Try approaching the poses (ordered by score calculated by gpd_ros)
4. Pick the object
5. Lift the object
6. Place the object
7. Go to step 1

## Installation (Ubuntu 18.04)

### Install ROS Melodic
- Follow the instructions in this link: http://wiki.ros.org/melodic/Installation/Ubuntu

### Build libfranka from source (version 0.8.0)
- Follow the instructions in this link: https://frankaemika.github.io/docs/installation_linux.html

### Create a caktin workspace and clone the repository
```
mkdir -p ~/pbp_ws/src
cd ~/pbp_ws/src
git clone https://code.fbi.h-da.de/istdaleon/panda_bin_picking.git
```

### Install dependencies
- Install OpenCV
```
mkdir ~/gpd_builds
cd ~/gpd_builds
cp ~/pbp_ws/src/panda_bin_picking/install/opencv_install.sh ~/gpd_builds
./opencv_install.sh
```
- Install PCL
```
cp ~/pbp_ws/src/panda_bin_picking/install/pcl_install.sh ~/gpd_builds
./pcl_install.sh
```
- Install GPD
```
cp ~/pbp_ws/src/panda_bin_picking/install/gpd_install.sh ~/gpd_builds
./gpd_install.sh
```

### Build the application
- First you have to decide wether you want to use the gazebo simulation or the real robot
- You can only build for one option in one workspace. So if you want to use the simulation and the real robot, you would need to create a second workspace (e.g. pbp2_ws)
- The build scripts first install all the necessary ROS packages, the realsense SDK, and the 3rd-party packages. Afterwards they build the workspace (rarely it will fail, then you have to retry by using the command `catkin build`)

##### Simulation
- Build for simulation
```
cd ~/pbp_ws/src/panda_bin_picking
./build_simulation.sh
```

##### Reality
- Build for reality
```
cd ~/pbp_ws/src/panda_bin_picking
./build.sh
```

## Usage

### Configuration
- The gpd configuration can be changed by editing the file _~/pbp_ws/src/panda_bin_picking/panda_bin_picking/config/gpd_params.cfg_. Make sure the **weights_file**-option is set correctly (depends on your username)
- The forces of the gripper in simulation can be adjustet by editing the gains in the file _~/pbp_ws/src/panda_bin_picking/panda_realsense_simulation/resources/ros_controllers.yaml_ and rerunning `./build_simulation.sh`
- The octomap configurations can be modified by editing the sensor-files in the resources folders (simulation and reality)
- The gazebo camera parameters can be changed by editing the file _~/pbp_ws/src/panda_bin_picking/panda_realsense_simulation/urdf/realsense_gazebo.xacro_
- The real camera parameters can be changed either by editing the file _~/pbp_ws/src/panda_bin_picking/panda_bin_picking/config/custom.json_ or the file _~/pbp_ws/src/panda_bin_picking/panda_realsense/launch/moveit.launch_

### Start Moveit
- If you built for simulation, follow the simulation instructions. Otherwise follow the reality instructions

##### Simulation
- For the simulation the package panda_realsense_simulation is used to start the simulation and moveit
- Use the launch file _simulation_moveit.launch_ to start the simulation and moveit at once
```
cd ~/pbp_ws/
source devel setup.bash
roslaunch panda_realsense_simulation simulation_moveit.launch
```

##### Reality
- For the reality the package panda_realsense is used to connect to the robot and start moveit
- Use the launch file _moveit.launch_ to connect to the robot and start moveit at once
```
cd ~/pbp_ws/
source devel setup.bash
roslaunch panda_realsense moveit.launch
```

### Start the bin picking application
- The application works the same for simulation and reality
- The application can either be controlled by using the GUI (recommended) or command line (care, using command line doesn't offer much control)
- Start a new Terminal for this application

##### GUI
- Use the launch file _panda_bin_picking.launch_ without additional arguments to launch the _panda_bin_picking_server_ and the _panda_bin_picking_gui_ nodes at the same time
```
cd ~/pbp_ws/
source devel setup.bash
roslaunch panda_bin_picking panda_bin_picking.launch
```

##### Command Line
- To use the command line control, run the launch file _panda_bin_picking.launch_ with additional arguments
- Also start the _panda_bin_picking_client_ node in another Terminal (here you can control the process)
- You can optionally start rviz to visualize the process when not using the GUI
```
cd ~/pbp_ws/
source devel setup.bash
roslaunch panda_bin_picking panda_bin_picking.launch gui:=false
```
```
cd ~/pbp_ws/
source devel setup.bash
rosrun panda_bin_picking panda_bin_picking_client
```

### Camera calibration
- The script to estimate the cameras pose can be found under _~/pbp_ws/src/panda_bin_picking/panda_realsense/scripts/estimate_camera_pose.py_
- Move the real robot in position to view the chessboard and start MoveIt
- Run the script and press space to estimate the pose (only works if the chessboard is actually detected)
- The calculated pose will be saved to a file called _camera_pose.txt_
- Feed the poses coordinates into the script _~/pbp_ws/src/panda_bin_picking/panda_realsense/scripts/eef_to_camera_pose.py_ and you receive a file called _eef_to_camera.txt_ containing the desired transformation
- Edit the file _~/pbp_ws/src/panda_bin_picking/panda_realsense/urdf/panda_realsense.urdf.xacro_ with the calculated transformation

## Documentation
- The automatically generated source code documentation can be found under _~/pbp_ws/src/panda_bin_picking/panda_bin_picking/doc_
- The C++ code is fully documented in doxygen style, while the Python code lacks some comments

## Improvements / Expansions
- Feel free to improve the current software and add new features
- Here are some features that could be improved/added

| What | Where/How |
| ------ | ------ |
| Support for objects in a bin | RealsenseL515 C++ class (_~/pbp_ws/src/panda_bin_picking/panda_bin_picking/src/realsensel515.cpp_) |
| Placing picked objects | FrankaPanda C++ class (_~/pbp_ws/src/panda_bin_picking/panda_bin_picking/src/frankapanda.cpp_) |
| Camera pose calibration | External app to fill _~/pbp_ws/src/panda_bin_picking/panda_realsense/urdf/panda_realsense.urdf.xacro_ |
| ros2_grasp_library | Only if switching to ROS2 to replace gpd_ros https://intel.github.io/ros2_grasp_library/docs/index.html |

## Author
David Leonhardt, 2021

david.leonhardt@stud.h-da.de
