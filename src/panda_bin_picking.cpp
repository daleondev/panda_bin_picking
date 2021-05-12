#include "franka_panda.h"
#include "visualizer.h"

#include <ros/ros.h>

#define NODE_NAME "panda_bin_picking"

static void error(const char* msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  Visualizer::init(n);
  FrankaPanda panda(n);

  while (true) {
  
    GraspConfigList grasps;
    if(!panda.detect(grasps)) {
      ROS_WARN("detection failed");
      break;
    }

    if(!panda.pickAndPlace(grasps)) {
      error("pick and place failed");
    }

  }

  ros::shutdown();
  return 0;
}

static void error(const char* msg)
{
  ROS_ERROR(msg);
  ros::shutdown();
  exit(1);
}