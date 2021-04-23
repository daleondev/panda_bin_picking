#include <ros/ros.h>
#include <atomic>

#include "FrankaPanda.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_fusion_node");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  FrankaPanda panda;
  MoveGroupInterface::Plan plan;

  std::atomic_bool running(true);
  boost::thread publisher([&]() -> void {
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/fused_pointcloud", 1000);
    while (running) {    
      ros::Duration(0.1).sleep();
      pub.publish(panda.getRealsensePointcloud());
    }
  });  

  std::vector<double> target_joints = {-M_PI_4, 0, -M_PI_2, -M_PI_2, M_PI_4, M_PI_2, M_PI_4};
  if(!panda.planMotion(target_joints, plan)) {
    ROS_ERROR("error while moving");
    ros::shutdown();
    return 1;
  }
  panda.executeMotion(plan);
  panda.captureRealsensePointcloud();  

  target_joints = {M_PI_4, 0, -M_PI_2, -M_PI_2, -M_PI_4, M_PI_2, M_PI_4};
  if(!panda.planMotion(target_joints, plan)) {
    ROS_ERROR("error while planning");
    ros::shutdown();
    return 1;
  }
  panda.executeMotion(plan);
  panda.captureRealsensePointcloud();

  target_joints = {0, 0, -M_PI_2, -M_PI_2, 0, M_PI_2, M_PI_4};
  if(!panda.planMotion(target_joints, plan)) {
    ROS_ERROR("error while planning");
    ros::shutdown();
    return 1;
  }
  panda.executeMotion(plan);
  panda.captureRealsensePointcloud();

  ros::Duration(3).sleep();
  running = false;
  publisher.join();

  panda.saveRealsensePointcloud();

  ros::shutdown();
  return 0;
}