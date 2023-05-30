#include "visualizer.h"

#include <eigen_conversions/eigen_msg.h>

#define COLOR_RED   Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.5f)
#define COLOR_GREEN Eigen::Vector4f(0.0f, 1.0f, 0.0f, 0.5f)
#define COLOR_BLUE  Eigen::Vector4f(0.0f, 0.0f, 1.0f, 0.5f)

ros::Publisher Visualizer::pub_vis_;
moveit_visual_tools::MoveItVisualToolsPtr Visualizer::visual_tools_;

void Visualizer::init(ros::NodeHandle& node_handle)
{
    // setup visualization publisher
    pub_vis_ = node_handle.advertise<visualization_msgs::MarkerArray>("/panda_bin_picking/visualization", 1);
    
    // setup moveit visual tools
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world");
}

void Visualizer::plotPose(const Pose& pose, const double tcp_offset)
{
    // translate pose by tcp-offset to visualize pose between gripper fingers
    const Eigen::Vector3d direction = pose.rotation.col(2).normalized();
    pub_vis_.publish(createPoseMarkers(pose.position + tcp_offset*direction, pose.rotation));
}

void Visualizer::plotGrasp(const Pose& pose, const HandGeometry& hand_geometry, const double tcp_offset)
{
    // translate pose to match grasp configuration (reverse the translation fron "FrankaPanda.GraspConfigToPose6D()")
    const Eigen::Vector3d direction = pose.rotation.col(2).normalized();
    pub_vis_.publish(createGraspMarkers(pose.position + (tcp_offset-hand_geometry.depth)*direction, pose.rotation*Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitZ()), hand_geometry));
}

void Visualizer::plotTrajectory(const moveit_msgs::RobotTrajectory& traj, const robot_model::JointModelGroup* joint_model_group)
{
    // visualize the trajectory using moveit visual tools
    visual_tools_->publishTrajectoryLine(traj, joint_model_group);
    visual_tools_->trigger();
}

void Visualizer::clear()
{
    // clear marker arrays
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    pub_vis_.publish(marker_array);

    // clear moveit visual tools
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
}

visualization_msgs::MarkerArray Visualizer::createPoseMarkers(const Position& position, const Rotation& rotation)
{
    const visualization_msgs::Marker x = createAxisMarker(position, rotation.col(0), COLOR_RED, 5, "world");
    const visualization_msgs::Marker y = createAxisMarker(position, rotation.col(1), COLOR_GREEN, 6, "world");
    const visualization_msgs::Marker z = createAxisMarker(position, rotation.col(2), COLOR_BLUE, 7, "world");

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(x);
    marker_array.markers.push_back(y);
    marker_array.markers.push_back(z);

    return marker_array;
}

visualization_msgs::Marker Visualizer::createAxisMarker(const Position& origin, const Position& direction, const Eigen::Vector4f& color, int id, const std::string& frame_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "axis";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.points.resize(2);
    tf::pointEigenToMsg(origin, marker.points[0]);
    tf::pointEigenToMsg(origin + direction.normalized()/10, marker.points[1]);

    tf::quaternionEigenToMsg(Eigen::Quaterniond(Eigen::Matrix3d::Identity()), marker.pose.orientation);

    marker.scale.x = 0.01f;
    marker.scale.y = 0.02f;
    marker.scale.z = 0.02f;
    
    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];

    return marker;
}

visualization_msgs::MarkerArray Visualizer::createGraspMarkers(const Position& position, const Rotation& rotation, const HandGeometry& hand_geometry)
{
    const double hw = 0.5*hand_geometry.outer_diameter - 0.5*hand_geometry.finger_width;
    
    const Eigen::Vector3d approach = rotation.col(2);
    const Eigen::Vector3d binormal = -rotation.col(1);
    const Eigen::Vector3d axis = rotation.col(0);

    const Position left_bottom = position - hw * binormal;
    const Position right_bottom = position + hw * binormal;
    const Position left_top = left_bottom + hand_geometry.depth * approach;
    const Position right_top = right_bottom + hand_geometry.depth * approach;
    const Position left_center = left_bottom + 0.5*(left_top - left_bottom);
    const Position right_center = right_bottom + 0.5*(right_top - right_bottom);
    const Position base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*approach;
    const Position approach_center = base_center - 0.04*approach;

    const Eigen::Vector3d finger_lwh(hand_geometry.depth, hand_geometry.finger_width, hand_geometry.height);
    const Eigen::Vector3d approach_lwh(0.08, hand_geometry.finger_width, hand_geometry.height);

    Rotation rotation_mod;
    rotation_mod.col(0) = approach;
    rotation_mod.col(1) = binormal;
    rotation_mod.col(2) = axis;

    const visualization_msgs::Marker base = createHandBaseMarker(left_bottom, right_bottom, rotation_mod, 0.02, hand_geometry.height, 1, "world");
    const visualization_msgs::Marker left_finger = createFingerMarker(left_center, rotation_mod, finger_lwh, 2, "world");
    const visualization_msgs::Marker right_finger = createFingerMarker(right_center, rotation_mod, finger_lwh, 3, "world");
    const visualization_msgs::Marker appr = createFingerMarker(approach_center, rotation_mod, approach_lwh, 4, "world");

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(left_finger);
    marker_array.markers.push_back(right_finger);
    marker_array.markers.push_back(appr);
    marker_array.markers.push_back(base);

    return marker_array;
}
visualization_msgs::Marker Visualizer::createFingerMarker(const Position& center, const Rotation& rotation, const Eigen::Vector3d& lwh, int id, const std::string& frame_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "finger";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    tf::pointEigenToMsg(center, marker.pose.position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(rotation), marker.pose.orientation);

    marker.scale.x = lwh(0);
    marker.scale.y = lwh(1);
    marker.scale.z = lwh(2);
    
    marker.color.r = 0.5;
    marker.color.g = 0.0;
    marker.color.b = 0.9;
    marker.color.a = 0.9;

    return marker;
}
visualization_msgs::Marker Visualizer::createHandBaseMarker(const Position& start, const Position& end, const Rotation& rotation, double length, double height, int id, const std::string& frame_id)
{
    Eigen::Vector3d center = start + 0.5 * (end - start);

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "hand_base";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    tf::pointEigenToMsg(center, marker.pose.position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(rotation), marker.pose.orientation);

    marker.scale.x = length;
    marker.scale.y = (end - start).norm();
    marker.scale.z = height;
 
    marker.color.r = 0.5;
    marker.color.g = 0.0;
    marker.color.b = 0.9;
    marker.color.a = 0.9;

    return marker;
}
