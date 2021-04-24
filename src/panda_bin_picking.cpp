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

static visualization_msgs::MarkerArray createGraspMarkers(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation_matrix);
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

  Eigen::Vector3d position, approach, binormal, axis, direction;
  Eigen::Matrix3d rotation_matrix;
  Eigen::Quaterniond rotation_quaternion;
  for (const gpd_ros::GraspConfig& grasp : grasps) {
    tf::pointMsgToEigen(grasp.position, position);
    tf::vectorMsgToEigen(grasp.approach, approach);
    tf::vectorMsgToEigen(grasp.binormal, binormal);
    tf::vectorMsgToEigen(grasp.axis, axis);
  
    rotation_matrix.col(0) = approach;
    rotation_matrix.col(1) = binormal;
    rotation_matrix.col(2) = axis;

    pub_grasp.publish(createGraspMarkers(position, rotation_matrix));

    // rotation_quaternion = rotation_matrix * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ());
    rotation_matrix = rotation_matrix * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ());

    direction = rotation_matrix.col(2);

    geometry_msgs::Pose target_pose;
    tf::pointEigenToMsg(position, target_pose.position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(rotation_matrix), target_pose.orientation); 

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

static visualization_msgs::MarkerArray createGraspMarkers(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation_matrix)
{
  constexpr double hand_depth = 0.06;
  constexpr double hand_height = 0.02;
  constexpr double outer_diameter = 0.12;
  constexpr double finger_width = 0.01;

  constexpr double hw = 0.5*outer_diameter - 0.5*finger_width;

  const Eigen::Vector3d approach = rotation_matrix.col(0);
  const Eigen::Vector3d binormal = rotation_matrix.col(1);
  const Eigen::Vector3d axis = rotation_matrix.col(2);

  const Eigen::Vector3d left_bottom = position - hw * binormal;
  const Eigen::Vector3d right_bottom = position + hw * binormal;
  const Eigen::Vector3d left_top = left_bottom + hand_depth * approach;
  const Eigen::Vector3d right_top = right_bottom + hand_depth * approach;
  const Eigen::Vector3d left_center = left_bottom + 0.5*(left_top - left_bottom);
  const Eigen::Vector3d right_center = right_bottom + 0.5*(right_top - right_bottom);
  const Eigen::Vector3d base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*approach;
  const Eigen::Vector3d approach_center = base_center - 0.04*approach;

  const Eigen::Vector3d finger_lwh(hand_depth, finger_width, hand_height);
  const Eigen::Vector3d approach_lwh(0.08, finger_width, hand_height);

  const visualization_msgs::Marker base = createHandBaseMarker(left_bottom, right_bottom, rotation_matrix, 0.02, hand_height, 1, "world");
  const visualization_msgs::Marker left_finger = createFingerMarker(left_center, rotation_matrix, finger_lwh, 1*3, "world");
  const visualization_msgs::Marker right_finger = createFingerMarker(right_center, rotation_matrix, finger_lwh, 1*3+1, "world");
  const visualization_msgs::Marker appr = createFingerMarker(approach_center, rotation_matrix, approach_lwh, 1*3+2, "world");

  visualization_msgs::MarkerArray marker_array;
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