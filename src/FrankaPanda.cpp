#include "FrankaPanda.h"

FrankaPanda::FrankaPanda()
: move_group_arm_{ "panda_arm" }, move_group_hand_{ "hand" }, move_group_arm_hand_{ "panda_arm_hand" }, robot_model_loader_{ "robot_description" }
{
    robot_model_ = robot_model_loader_.getModel();
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);

    move_group_arm_.setPlannerId("RRT");   
    move_group_hand_.setPlannerId("RRT"); 
    move_group_arm_hand_.setPlannerId("RRT"); 
}

FrankaPanda::~FrankaPanda() = default;

bool FrankaPanda::planArmMotion(const std::vector<double>& target_joints, MoveGroupInterface::Plan& out_plan)
{
    move_group_arm_.setJointValueTarget(target_joints);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotion(const geometry_msgs::Pose& target_pose, MoveGroupInterface::Plan& out_plan)
{
    move_group_arm_.setPoseTarget(target_pose);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planHandMotion(const std::string& name, MoveGroupInterface::Plan& out_plan)
{
    move_group_hand_.setNamedTarget(name);
    return move_group_hand_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmHandMotion(const geometry_msgs::Pose& target_pose, MoveGroupInterface::Plan& out_plan)
{
    move_group_arm_hand_.setPoseTarget(target_pose);
    return move_group_arm_hand_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::executeArmMotion(const MoveGroupInterface::Plan& plan, const float velocity)
{
    move_group_arm_.setMaxVelocityScalingFactor(velocity);
    return move_group_arm_.execute(plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::executeHandMotion(const MoveGroupInterface::Plan& plan, const float velocity)
{
    move_group_hand_.setMaxVelocityScalingFactor(velocity);
    return move_group_hand_.execute(plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::executeArmHandMotion(const MoveGroupInterface::Plan& plan, const float velocity)
{
    move_group_arm_hand_.setMaxVelocityScalingFactor(velocity);
    return move_group_arm_hand_.execute(plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::captureRealsensePointcloud()
{
    tf::StampedTransform transform;
    if (!getTransform(WORLD_FRAME, realsense_.getDepthFrame(), transform)) {
        return false;
    }

    realsense_.capture(transform);

    return true;
}

sensor_msgs::PointCloud2 FrankaPanda::getRealsensePointcloud() const
{
    sensor_msgs::PointCloud2 pc = realsense_.getPointcloud();  
    pc.header.frame_id = WORLD_FRAME;
    pc.header.stamp = ros::Time::now();

    return pc;
}

bool FrankaPanda::getTransform(const std::string& target_frame, const std::string& source_frame, tf::StampedTransform& out_transform) const
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

void FrankaPanda::saveRealsensePointcloud() const
{
    realsense_.savePointcloud("pointcloud.pcd");
}