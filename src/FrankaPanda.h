#pragma once

#include "RealsenseL515.h"
#include "custom_types.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <gpd_ros/GraspConfigList.h>

using moveit::planning_interface::MoveItErrorCode;
using moveit::planning_interface::MoveGroupInterface;

class FrankaPanda {
public:
    FrankaPanda(ros::NodeHandle& n);
    ~FrankaPanda();

    bool detect(graspConfigList& out_grasps);
    bool pick(graspConfigList& grasps);
    
    bool getTransform(const std::string& target_frame, const std::string& source_frame, tf::StampedTransform& out_transform) const;
    
private:
    // Pointcloud
    bool captureAndStitchRealsensePointclouds();
    bool captureRealsensePointcloud();
    sensor_msgs::PointCloud2 realsensePointcloudMessage() const;
    void saveRealsensePointcloud() const;
    
    // Picking
    bool tryPoses(const PoseList& poses);
    bool approachPose(const Pose6D& pose);
    bool moveToPose(const Pose6D& pose);
    bool openHand();
    bool closeHand();   
    Pose6D graspConfigToPose6D(const gpd_ros::GraspConfig& grasp);

    // Planning
    bool planArmMotion(const JointList& target_joints, MoveGroupInterface::Plan& out_plan);
    bool planArmMotion(const Pose6D& target_pose, MoveGroupInterface::Plan& out_plan);
    bool planArmMotion(const std::string& name, MoveGroupInterface::Plan& out_plan);
    bool planHandMotion(const JointList& target_joints, MoveGroupInterface::Plan& out_plan);
    bool planHandMotion(const std::string& name, MoveGroupInterface::Plan& out_plan);

    // Moving
    bool executeArmMotion(const MoveGroupInterface::Plan& plan, const float velocity = 1.0);
    bool executeHandMotion(const MoveGroupInterface::Plan& plan, const float velocity = 1.0);

    MoveGroupInterface move_group_arm_;
    MoveGroupInterface move_group_hand_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;

    RealsenseL515 realsense_;

    ros::Publisher pub_current_; 
    ros::Publisher pub_stitched_;

    static const double TCP_OFFSET;

    static const HandGeometry HAND_GEOMETRY;
    static const JointList OPEN_HAND;
    static const JointList CLOSED_HAND;

    static const std::string WORLD_FRAME;
    static const std::string PC_CURRENT_TOPIC;
    static const std::string PC_STITCHED_TOPIC;
    static const std::string GRASPS_TOPIC;
    static const std::string CLEAR_OCTOMAP_SERVICE;
};
