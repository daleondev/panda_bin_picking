#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

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

    bool planMotion(const std::vector<double>& target_joints, MoveGroupInterface::Plan& out_plan);
    bool planMotion(const geometry_msgs::Pose& target_pose, MoveGroupInterface::Plan& out_plan);

    bool executeMotion(const MoveGroupInterface::Plan& plan);

    bool captureRealsensePointcloud();

    bool getRealsensePointcloud(sensor_msgs::PointCloud2& out_pc);
    bool getTransform(const std::string& target_frame, const std::string& source_frame, tf::StampedTransform& out_transform);

private:
    MoveGroupInterface move_group_;
    RobotModelLoader robot_model_loader_;
    RobotModelPtr robot_model_;
    RobotStatePtr robot_state_;
    RealsenseL515 realsense_;
    
    const std::string PLANNING_GROUP = "panda_arm";
    const std::string WORLD_FRAME = "/world";
};