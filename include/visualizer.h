#pragma once

#include "custom_types.h"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Visualizer {
public:
    static void init(ros::NodeHandle& n);

    static void plotPose(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation_matrix);    
    static void plotGrasp(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation_matrix, const HandGeometry& hand_geometry);

    static void clear();
    
private:
    Visualizer() = delete;

    static visualization_msgs::MarkerArray createPoseMarkers(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation_matrix);
    static visualization_msgs::Marker createAxisMarker(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction, const Eigen::Vector4f& color, int id, const std::string& frame_id);

    static visualization_msgs::MarkerArray createGraspMarkers(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation_matrix, const HandGeometry& hand_geometry);
    static visualization_msgs::Marker createFingerMarker(const Eigen::Vector3d& center, const Eigen::Matrix3d& frame, const Eigen::Vector3d& lwh, int id, const std::string& frame_id);
    static visualization_msgs::Marker createHandBaseMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id);

    static ros::Publisher pub_vis_;
};
