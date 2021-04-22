#include <ros/ros.h>

#include "FrankaPanda.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_fusion_node");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/fused_pointcloud", 1000);

  FrankaPanda panda;
  MoveGroupInterface::Plan plan;

  std::vector<double> target_joints = {-M_PI_4, 0, -M_PI_2, -M_PI_2, M_PI_4, M_PI_2, M_PI_4};
  if(!panda.planMotion(target_joints, plan)) {
    ROS_ERROR("error while moving");
    ros::shutdown();
    return 1;
  }
  panda.executeMotion(plan);
  panda.realsenseCapture();
  

  target_joints = {M_PI_4, 0, -M_PI_2, -M_PI_2, -M_PI_4, M_PI_2, M_PI_4};
  if(!panda.planMotion(target_joints, plan)) {
    ROS_ERROR("error while planning");
    ros::shutdown();
    return 1;
  }
  panda.executeMotion(plan);
  panda.realsenseCapture();

  target_joints = {0, 0, -M_PI_2, -M_PI_2, 0, M_PI_2, M_PI_4};
  if(!panda.planMotion(target_joints, plan)) {
    ROS_ERROR("error while planning");
    ros::shutdown();
    return 1;
  }
  panda.executeMotion(plan);
  panda.realsenseCapture();

  sensor_msgs::PointCloud2 msg;
  if (!panda.getRealsensePointcloud(msg)) {
      ROS_ERROR("error while tranforming");
      ros::shutdown();
      return 1;
  }
  ROS_INFO("%i", msg.data.size());

  for (int i = 0; i < 5; ++i) {
    ros::Duration(0.5).sleep();

    pub.publish(msg);
  }

  ros::shutdown();
  return 0;
}