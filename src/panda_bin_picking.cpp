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

static visualization_msgs::MarkerArray createGraspMarkers(const gpd_ros::GraspConfig& grasp);
static visualization_msgs::Marker createFingerMarker(const Eigen::Vector3d& center, const Eigen::Matrix3d& frame, const Eigen::Vector3d& lwh, int id, const std::string& frame_id);
static visualization_msgs::Marker createHandBaseMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id);

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
  ros::Publisher pub_current = n.advertise<sensor_msgs::PointCloud2>(PC_CURRENT_TOPIC, 1);
  ros::Publisher pub_stitched = n.advertise<sensor_msgs::PointCloud2>(PC_STITCHED_TOPIC, 1);

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
  ros::Publisher pub_grasp = n.advertise<visualization_msgs::MarkerArray>("/grasp", 1);

  gpd_ros::GraspConfigList::ConstPtr grasp_configs = ros::topic::waitForMessage<gpd_ros::GraspConfigList>(GRASPS_TOPIC);
  ROS_INFO("Received %i grasps", grasp_configs->grasps.size());

  auto grasps = grasp_configs->grasps;

  std::sort(grasps.begin(), grasps.end(), [](gpd_ros::GraspConfig lhs, gpd_ros::GraspConfig rhs) -> bool {
    return lhs.score.data > rhs.score.data;
  });

  for (const gpd_ros::GraspConfig& grasp : grasps) {

    pub_grasp.publish(createGraspMarkers(grasp));

    Eigen::Matrix3f r;
    r(0, 0) = grasp.approach.x;
    r(1, 0) = grasp.approach.y;
    r(2, 0) = grasp.approach.z;
    r(0, 1) = grasp.binormal.x;
    r(1, 1) = grasp.binormal.y;
    r(2, 1) = grasp.binormal.z;
    r(0, 2) = grasp.axis.x;
    r(1, 2) = grasp.axis.y;
    r(2, 2) = grasp.axis.z;

    // Eigen::AngleAxisf aa = Eigen::AngleAxisf(M_PI/8, Eigen::Vector3f::UnitZ());
    // Eigen::Quaternionf q = Eigen::Quaternionf(Eigen::AngleAxisf(r)) * Eigen::Quaternionf(aa);

    // Eigen::AngleAxisf aa = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitY());
    Eigen::Quaternionf q = Eigen::Quaternionf(r) * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ());

    // Eigen::Quaternionf q(r);

    geometry_msgs::Pose target_pose;
    tf::quaternionEigenToMsg(q.cast<double>(), target_pose.orientation);
    target_pose.position = grasp.position;

    MoveGroupInterface::Plan plan;
    if(!panda.planHandMotion("open", plan)) {
      ROS_WARN("couldnt plan motion for hand");
    } else {
      panda.executeHandMotion(plan);
    }

    if(!panda.planArmMotion(target_pose, plan)) {
      ROS_WARN("couldnt plan motion for arm");
      continue;
    }
    panda.executeArmMotion(plan, 0.2);

    if(!panda.planHandMotion("close", plan)) {
      ROS_WARN("couldnt plan motion for hand");
    } else {
      panda.executeHandMotion(plan, 0.4);
    }

    break;
  }
  
  // gpd_ros::GraspConfig optimal_grasp;
  // float highscore = FLOAT_MIN;
  // for (auto grasp : grasps->grasps) {
  //   if (grasp.score.data > highscore) {
  //     optimal_grasp = grasp;
  //     highscore = grasp.score.data;
  //   }
  // }

  // if (highscore == FLOAT_MIN) {
  //   error("no fitting grasp pose found");
  // }

  // float maxZ = FLOAT_MIN;
  // for (auto grasp : grasps->grasps) {
  //   if (grasp.position.z > maxZ) {
  //     optimal_grasp = grasp;
  //     maxZ = grasp.position.z;
  //   }
  // }

  // if (maxZ == FLOAT_MIN) {
  //   error("no fitting grasp pose found");
  // }
  
  // Eigen::Vector3d x, y, z;
  // tf::vectorMsgToEigen(optimal_grasp.approach, x);
  // tf::vectorMsgToEigen(optimal_grasp.binormal, y);
  // tf::vectorMsgToEigen(optimal_grasp.axis, z);

  // Eigen::Matrix3f r;
  // // r.col(0) = x.cast<float>();
  // // r.col(1) = y.cast<float>();
  // // r.col(2) = z.cast<float>();
  // r(0, 0) = optimal_grasp.approach.x;
  // r(1, 0) = optimal_grasp.approach.y;
  // r(2, 0) = optimal_grasp.approach.z;
  // r(0, 1) = optimal_grasp.binormal.x;
  // r(1, 1) = optimal_grasp.binormal.y;
  // r(2, 1) = optimal_grasp.binormal.z;
  // r(0, 2) = optimal_grasp.axis.x;
  // r(1, 2) = optimal_grasp.axis.y;
  // r(2, 2) = optimal_grasp.axis.z;

  // Eigen::AngleAxisf aa = Eigen::AngleAxisf(M_PI/8, Eigen::Vector3f::UnitZ());

  // Eigen::Quaternionf q = Eigen::Quaternionf(Eigen::AngleAxisf(r)) * Eigen::Quaternionf(aa);

  // geometry_msgs::Pose target_pose;
  // tf::quaternionEigenToMsg(q.cast<double>(), target_pose.orientation);
  // target_pose.position = optimal_grasp.position;

  // MoveGroupInterface::Plan plan;
  // if(!panda.planHandMotion("open", plan)) {
  //   error("error while planning");
  // }
  // panda.executeHandMotion(plan);

  // if(!panda.planArmMotion(target_pose, plan)) {
  //   error("error while planning");
  // }
  // panda.executeArmMotion(plan, 0.2);

  // if(!panda.planHandMotion("close", plan)) {
  //   error("error while planning");
  // }
  // panda.executeHandMotion(plan, 0.4);
}

static visualization_msgs::MarkerArray createGraspMarkers(const gpd_ros::GraspConfig& grasp)
{
  constexpr double hand_depth = 0.06;
  constexpr double hand_height = 0.02;
  constexpr double outer_diameter = 0.12;
  constexpr double finger_width = 0.01;

  constexpr double hw = 0.5 * outer_diameter - 0.5 * finger_width;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker left_finger, right_finger, base, appr;
  Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center, approach_center, base_center, position, approach, binormal, axis;
  Eigen::Matrix3d orientation;

  tf::pointMsgToEigen(grasp.position, position);
  tf::vectorMsgToEigen(grasp.approach, approach);
  tf::vectorMsgToEigen(grasp.binormal, binormal);
  tf::vectorMsgToEigen(grasp.axis, axis);

  orientation.col(0) = approach;
  orientation.col(1) = binormal;
  orientation.col(2) = axis;

  left_bottom = position - hw * binormal;
  right_bottom = position + hw * binormal;
  left_top = left_bottom + hand_depth * approach;
  right_top = right_bottom + hand_depth * approach;
  left_center = left_bottom + 0.5*(left_top - left_bottom);
  right_center = right_bottom + 0.5*(right_top - right_bottom);
  base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*approach;
  approach_center = base_center - 0.04*approach;

  Eigen::Vector3d finger_lwh, approach_lwh;
  finger_lwh << hand_depth, finger_width, hand_height;
  approach_lwh << 0.08, finger_width, hand_height;

  base = createHandBaseMarker(left_bottom, right_bottom, orientation, 0.02, hand_height, 1, "world");
  left_finger = createFingerMarker(left_center, orientation, finger_lwh, 1*3, "world");
  right_finger = createFingerMarker(right_center, orientation, finger_lwh, 1*3+1, "world");
  appr = createFingerMarker(approach_center, orientation, approach_lwh, 1*3+2, "world");

  marker_array.markers.push_back(left_finger);
  marker_array.markers.push_back(right_finger);
  marker_array.markers.push_back(appr);
  marker_array.markers.push_back(base);

  return marker_array;
}

static visualization_msgs::Marker createFingerMarker(const Eigen::Vector3d& center, const Eigen::Matrix3d& frame, const Eigen::Vector3d& lwh, int id, const std::string& frame_id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "finger";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(10);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = lwh(0); // forward direction
  marker.scale.y = lwh(1); // hand closing direction
  marker.scale.z = lwh(2); // hand vertical direction

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.5;

  return marker;
}


static visualization_msgs::Marker createHandBaseMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id)
{
  Eigen::Vector3d center = start + 0.5 * (end - start);

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "hand_base";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(10);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length; // forward direction
  marker.scale.y = (end - start).norm(); // hand closing direction
  marker.scale.z = height; // hand vertical direction

  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  return marker;
}

static void error(const char* msg)
{
  ROS_ERROR(msg);
  ros::shutdown();
  exit(1);
}