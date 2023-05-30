#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gpd_ros/GraspConfig.h>

#define PRINT_METHOD() ROS_INFO("%s::%s", typeid(*this).name()+2, __FUNCTION__)

/**
 * 3D position
 */
typedef Eigen::Vector3d Position;
/**
 *  3D rotation matrix
 */
typedef Eigen::Matrix3d Rotation;

/**
 * 6D pose (position and rotation)
 */
struct Pose {
    Position position;
    Rotation rotation;
};

/**
 * Robot hand geometry description
 */
struct HandGeometry {
    double depth;
    double height;
    double outer_diameter;
    double finger_width;
};

/**
 * List of grasp configurations from grasp pose detection
 */
typedef std::vector<gpd_ros::GraspConfig> GraspConfigList;
/**
 * Joint configuration (list of joint angles)
 */
typedef std::vector<double> JointList;
/**
 * List of 6D-Poses
 */
typedef std::vector<Pose> PoseList;