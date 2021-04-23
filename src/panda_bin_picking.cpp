#include "FrankaPanda.h"

#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <eigen_conversions/eigen_msg.h>

#include <gpd_ros/GraspConfigList.h>

#define NODE_NAME         "panda_bin_picking"
#define PC_CURRENT_TOPIC  "/cloud_current"
#define PC_STITCHED_TOPIC "/cloud_stitched"
#define GRASPS_TOPIC      "/detect_grasps/clustered_grasps"

#define FLOAT_MIN std::numeric_limits<float>::min()
#define FLOAT_MAX std::numeric_limits<float>::max()

static void capturePointclouds(ros::NodeHandle& n, FrankaPanda& panda);
static void grabObject(ros::NodeHandle& n, FrankaPanda& panda);
static void error(const char* msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  FrankaPanda panda;
  
  capturePointclouds(n, panda);
  grabObject(n, panda);

  ros::shutdown();
  return 0;
}

static void capturePointclouds(ros::NodeHandle& n, FrankaPanda& panda)
{
  ros::Publisher pub_current = n.advertise<sensor_msgs::PointCloud2>(PC_CURRENT_TOPIC, 1000);
  ros::Publisher pub_stitched = n.advertise<sensor_msgs::PointCloud2>(PC_STITCHED_TOPIC, 1000);

  MoveGroupInterface::Plan plan;

  std::vector<double> target_joints = {-M_PI_4, 0, -M_PI_2, -M_PI_2, M_PI_4, M_PI_2, M_PI_4};
  if(!panda.planArmMotion(target_joints, plan)) {
    error("error while planning");
  }
  panda.executeArmMotion(plan);
  panda.captureRealsensePointcloud();
  pub_current.publish(panda.getRealsensePointcloud());  

  target_joints = {M_PI_4, 0, -M_PI_2, -M_PI_2, -M_PI_4, M_PI_2, M_PI_4};
  if(!panda.planArmMotion(target_joints, plan)) {
    error("error while planning");
  }
  panda.executeArmMotion(plan);
  panda.captureRealsensePointcloud();
  pub_current.publish(panda.getRealsensePointcloud());

  target_joints = {0, 0, -M_PI_2, -M_PI_2, 0, M_PI_2, M_PI_4};
  if(!panda.planArmMotion(target_joints, plan)) {
    error("error while planning");
  }
  panda.executeArmMotion(plan);
  panda.captureRealsensePointcloud();
  pub_current.publish(panda.getRealsensePointcloud());

  pub_stitched.publish(panda.getRealsensePointcloud());

  // panda.saveRealsensePointcloud();
}

static void grabObject(ros::NodeHandle& n, FrankaPanda& panda)
{
  gpd_ros::GraspConfigList::ConstPtr grasps = ros::topic::waitForMessage<gpd_ros::GraspConfigList>(GRASPS_TOPIC);
  ROS_INFO("Received %i grasps", grasps->grasps.size());
  
  gpd_ros::GraspConfig optimal_grasp;
  float highscore = FLOAT_MIN;
  for (auto grasp : grasps->grasps) {
    if (grasp.score.data > highscore) {
      optimal_grasp = grasp;
      highscore = grasp.score.data;
    }
  }

  if (highscore == FLOAT_MIN) {
    error("no fitting grasp pose found");
  }

  Eigen::Matrix3f r;
  r(0, 0) = optimal_grasp.approach.x;
  r(1, 0) = optimal_grasp.approach.y;
  r(2, 0) = optimal_grasp.approach.z;
  r(0, 1) = optimal_grasp.binormal.x;
  r(1, 1) = optimal_grasp.binormal.y;
  r(2, 1) = optimal_grasp.binormal.z;
  r(0, 2) = optimal_grasp.axis.x;
  r(1, 2) = optimal_grasp.axis.y;
  r(2, 2) = optimal_grasp.axis.z;

  Eigen::AngleAxisf aa = Eigen::AngleAxisf(M_PI/8, Eigen::Vector3f::UnitZ());

  Eigen::Quaternionf q = Eigen::Quaternionf(Eigen::AngleAxisf(r)) * Eigen::Quaternionf(aa);

  geometry_msgs::Pose target_pose;
  tf::quaternionEigenToMsg(q.cast<double>(), target_pose.orientation);
  target_pose.position = optimal_grasp.position;

  MoveGroupInterface::Plan plan;
  if(!panda.planHandMotion("open", plan)) {
    error("error while planning");
  }
  panda.executeHandMotion(plan);

  if(!panda.planArmMotion(target_pose, plan)) {
    error("error while planning");
  }
  panda.executeArmMotion(plan, 0.2);

  if(!panda.planHandMotion("close", plan)) {
    error("error while planning");
  }
  panda.executeHandMotion(plan, 0.4);
}

static void error(const char* msg)
{
  ROS_ERROR(msg);
  ros::shutdown();
  exit(1);
}