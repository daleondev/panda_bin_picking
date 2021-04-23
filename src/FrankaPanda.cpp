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
    return realsense_.savePointcloud("pointcloud.pcd");
}