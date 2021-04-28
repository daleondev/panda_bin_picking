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
  
  GraspConfigList grasps;
  if(!panda.detect(grasps)) {
    error("detection failed");
  }

  if(!panda.pick(grasps)) {
    error("picking failed");
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