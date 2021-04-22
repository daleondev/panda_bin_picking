#include "FrankaPanda.h"

FrankaPanda::FrankaPanda()
: move_group_{ "panda_arm" }, robot_model_loader_{ "robot_description" }
{
    robot_model_ = robot_model_loader_.getModel();
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);

    move_group_.setPlannerId("RRT");
}

FrankaPanda::~FrankaPanda() = default;

bool FrankaPanda::planMotion(const std::vector<double>& target_joints, MoveGroupInterface::Plan& out_plan)
{
    move_group_.setJointValueTarget(target_joints);
    return move_group_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planMotion(const geometry_msgs::Pose& target_pose, MoveGroupInterface::Plan& out_plan)
{
    move_group_.setPoseTarget(target_pose);
    return move_group_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::executeMotion(const MoveGroupInterface::Plan& plan)
{
    return move_group_.execute(plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::realsenseCapture()
{
    tf::StampedTransform transform;
    if (!getTransform("/world", "/camera_depth_optical_frame", transform)) {
        return false;
    }

    realsense_.capture(transform);

    return true;
}

bool FrankaPanda::getRealsensePointcloud(sensor_msgs::PointCloud2& out_pc)
{
    tf::StampedTransform transform;
    if (!getTransform("/camera_depth_optical_frame", "/world", transform)) {
        return false;
    }

    out_pc = realsense_.getPointcloud(transform);

    return true;
}

bool FrankaPanda::getTransform(const std::string& target_frame, const std::string& source_frame, tf::StampedTransform& out_transform)
{
    tf::TransformListener listener;

    try{
        ros::Time now = ros::Time::now();
        listener.waitForTransform(target_frame, source_frame, now, ros::Duration(3.0));
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), out_transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        return false;
    }

    return true;
}