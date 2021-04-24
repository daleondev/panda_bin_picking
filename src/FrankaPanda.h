#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "RealsenseL515.h"

using moveit::planning_interface::MoveItErrorCode;
using moveit::planning_interface::MoveGroupInterface;
using robot_model_loader::RobotModelLoader;
using robot_model::RobotModelPtr;
using robot_state::RobotStatePtr;

class FrankaPanda {
public:
    FrankaPanda();
    ~FrankaPanda();

    bool planArmMotion(const std::vector<double>& target_joints, MoveGroupInterface::Plan& out_plan);
    bool planArmMotion(const geometry_msgs::Pose& target_pose, MoveGroupInterface::Plan& out_plan);
    bool planHandMotion(const std::string& name, MoveGroupInterface::Plan& out_plan);
    bool planArmHandMotion(const geometry_msgs::Pose& target_pose, MoveGroupInterface::Plan& out_plan);

    bool executeArmMotion(const MoveGroupInterface::Plan& plan, const float velocity = 1.0);
    bool executeHandMotion(const MoveGroupInterface::Plan& plan, const float velocity = 1.0);
    bool executeArmHandMotion(const MoveGroupInterface::Plan& plan, const float velocity = 1.0);

    bool captureRealsensePointcloud();

    sensor_msgs::PointCloud2 getRealsensePointcloud() const;
    bool getTransform(const std::string& target_frame, const std::string& source_frame, tf::StampedTransform& out_transform) const;

    void saveRealsensePointcloud() const;

private:
    MoveGroupInterface move_group_arm_;
    MoveGroupInterface move_group_hand_;
    MoveGroupInterface move_group_arm_hand_;
    RobotModelLoader robot_model_loader_;
    RobotModelPtr robot_model_;
    RobotStatePtr robot_state_;

    RealsenseL515 realsense_;

    const std::string WORLD_FRAME = "/world";
};