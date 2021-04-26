#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gpd_ros/GraspConfig.h>

#include <vector>

struct Pose6D {
    Eigen::Vector3d position;
    Eigen::Matrix3d rotation_matrix;
};

struct HandGeometry {
    double depth;
    double height;
    double outer_diameter;
    double finger_width;
};

typedef std::vector<gpd_ros::GraspConfig> GraspConfigList;
typedef std::vector<double> JointList;
typedef std::vector<Pose6D> PoseList;