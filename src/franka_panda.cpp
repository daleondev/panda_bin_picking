#include "franka_panda.h"
#include "visualizer.h"

#include <std_srvs/Empty.h>

#include <eigen_conversions/eigen_msg.h>

const double FrankaPanda::TCP_OFFSET = 0.1034;

const HandGeometry FrankaPanda::HAND_GEOMETRY   { 0.054, 0.018, 0.09, 0.005 };
const JointList FrankaPanda::OPEN_HAND =        { 0.04, 0.04 };
const JointList FrankaPanda::CLOSED_HAND =      { 0.0, 0.0 };

const std::string FrankaPanda::WORLD_FRAME =            "world";
const std::string FrankaPanda::PC_CURRENT_TOPIC =       "/panda_bin_picking/cloud_current";
const std::string FrankaPanda::PC_STITCHED_TOPIC =      "/panda_bin_picking/cloud_stitched";
const std::string FrankaPanda::GRASPS_TOPIC =           "/detect_grasps/clustered_grasps";
const std::string FrankaPanda::CLEAR_OCTOMAP_SERVICE =  "/clear_octomap";

FrankaPanda::FrankaPanda(ros::NodeHandle& n)
: move_group_arm_{ "panda_arm" }, move_group_hand_{ "hand" }, robot_model_loader_{ "robot_description" }
{
    robot_model_ = robot_model_loader_.getModel();
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);

    pub_current_ = n.advertise<sensor_msgs::PointCloud2>(PC_CURRENT_TOPIC, 1);
    pub_stitched_ = n.advertise<sensor_msgs::PointCloud2>(PC_STITCHED_TOPIC, 1);
}

FrankaPanda::~FrankaPanda() = default;

bool FrankaPanda::detect(GraspConfigList& out_grasps)
{
    if (!captureAndStitchRealsensePointclouds()) {
        ROS_ERROR("Failed to capture and stitch pointclouds");
        return false;
    }
    pub_stitched_.publish(realsensePointcloudMessage());

    gpd_ros::GraspConfigList::ConstPtr grasp_configs = ros::topic::waitForMessage<gpd_ros::GraspConfigList>(GRASPS_TOPIC);
    out_grasps = grasp_configs->grasps;

    return true;
}

bool FrankaPanda::pick(GraspConfigList& grasps)
{
    std::sort(grasps.begin(), grasps.end(), [](gpd_ros::GraspConfig lhs, gpd_ros::GraspConfig rhs) -> bool {
        return lhs.score.data > rhs.score.data;
    });

    PoseList poses;
    poses.reserve(grasps.size());
    for(const gpd_ros::GraspConfig& grasp : grasps) {
        poses.push_back(graspConfigToPose6D(grasp));
    }

    if (!tryPoses(poses)) {
        ROS_ERROR("No grasp was reachable");
        return false;
    }

    return true;
}

bool FrankaPanda::captureAndStitchRealsensePointclouds()
{
    std_srvs::Empty srv;
    ros::service::call(CLEAR_OCTOMAP_SERVICE, srv);

    MoveGroupInterface::Plan plan;

    JointList target_joints = {-3*M_PI_4, 0.0, 0.0, -M_PI_2, M_PI_4, M_PI_2, M_PI_4};
    if(!planArmMotion(target_joints, plan)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(plan);
    captureRealsensePointcloud();
    pub_current_.publish(realsensePointcloudMessage());  

    target_joints = {-M_PI_4, 0.0, 0.0, -M_PI_2, -M_PI_4, M_PI_2, M_PI_4};
    if(!planArmMotion(target_joints, plan)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(plan);
    captureRealsensePointcloud();
    pub_current_.publish(realsensePointcloudMessage());

    // target_joints = {-M_PI_2, 0.0, 0.0, -M_PI_2, 0.0, M_PI_2, M_PI_4};
    target_joints = {-M_PI_2, M_PI_4, 0.0, -M_PI/3.0, 0.0, M_PI_2, M_PI_4};
    if(!planArmMotion(target_joints, plan)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(plan);
    captureRealsensePointcloud();
    pub_current_.publish(realsensePointcloudMessage());

    return true;
}

bool FrankaPanda::tryPoses(const PoseList& poses)
{
    for (const Pose6D& pose : poses) {
        Visualizer::plotGrasp(pose.position + HAND_GEOMETRY.depth * pose.rotation_matrix.col(2).normalized(), pose.rotation_matrix * Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitZ()), HAND_GEOMETRY);

        if (!approachPose(pose)) {
            ROS_WARN("Failed to approach pose");
            continue;
        }

        if (!openHand()) {
            ROS_WARN("Failed to open hand");
            continue;
        }

        if (!moveToPose(pose)) {
            ROS_WARN("Failed to move to pose");
            continue;
        }

        std_srvs::Empty srv;
        ros::service::call(CLEAR_OCTOMAP_SERVICE, srv);
        closeHand();

        return true;
    }

    return false;
}

bool FrankaPanda::approachPose(const Pose6D& pose)
{
    Pose6D target_pose{ pose };

    const Eigen::Vector3d direction = target_pose.rotation_matrix.col(2);
    target_pose.position += -0.04*direction.normalized();

    Visualizer::plotPose(target_pose.position, target_pose.rotation_matrix);

    MoveGroupInterface::Plan plan;
    if (!planArmMotion(target_pose, plan)) {
        return false;
    }
    executeArmMotion(plan);

    return true;
}

bool FrankaPanda::moveToPose(const Pose6D& pose)
{
    const Eigen::Vector3d direction = pose.rotation_matrix.col(2);
    Visualizer::plotPose(pose.position + TCP_OFFSET * direction.normalized(), pose.rotation_matrix);

    MoveGroupInterface::Plan plan;
    if (!planArmMotion(pose, plan)) {
        return false;
    }
    executeArmMotion(plan);

    return true;
}

bool FrankaPanda::openHand()
{
    MoveGroupInterface::Plan plan;
    if (!planHandMotion(OPEN_HAND, plan)) {
        return false;
    }
    executeHandMotion(plan);

    return true;
}

bool FrankaPanda::closeHand()
{
    MoveGroupInterface::Plan plan;
    if (!planHandMotion(CLOSED_HAND, plan)) {
        return false;
    }
    executeHandMotion(plan);

    return true;
}

Pose6D FrankaPanda::graspConfigToPose6D(const gpd_ros::GraspConfig& grasp)
{
    Eigen::Vector3d approach, binormal, axis;
    tf::vectorMsgToEigen(grasp.approach, approach);
    tf::vectorMsgToEigen(grasp.binormal, binormal);
    tf::vectorMsgToEigen(grasp.axis, axis);
  
    Pose6D pose;
    tf::pointMsgToEigen(grasp.position, pose.position);
    pose.rotation_matrix.col(0) = axis;
    pose.rotation_matrix.col(1) = -binormal;
    pose.rotation_matrix.col(2) = approach;

    pose.rotation_matrix = pose.rotation_matrix * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ());

    const Eigen::Vector3d direction = pose.rotation_matrix.col(2);
    pose.position += -TCP_OFFSET * direction.normalized();

    return pose;
}

bool FrankaPanda::planArmMotion(const JointList& target_joints, MoveGroupInterface::Plan& out_plan)
{
    move_group_arm_.setJointValueTarget(target_joints);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotion(const Pose6D& target_pose, MoveGroupInterface::Plan& out_plan)
{
    geometry_msgs::Pose pose;
    tf::pointEigenToMsg(target_pose.position, pose.position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(target_pose.rotation_matrix), pose.orientation);

    move_group_arm_.setPoseTarget(pose);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotion(const std::string& name, MoveGroupInterface::Plan& out_plan)
{
    move_group_arm_.setNamedTarget(name);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planHandMotion(const JointList& target_joints, MoveGroupInterface::Plan& out_plan)
{
    move_group_hand_.setJointValueTarget(target_joints);
    return move_group_hand_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planHandMotion(const std::string& name, MoveGroupInterface::Plan& out_plan)
{
    move_group_hand_.setNamedTarget(name);
    return move_group_hand_.plan(out_plan) == MoveItErrorCode::SUCCESS;
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

bool FrankaPanda::captureRealsensePointcloud()
{
    tf::StampedTransform transform;
    if (!getTransform(WORLD_FRAME, realsense_.getDepthFrame(), transform)) {
        return false;
    }

    realsense_.capture(transform);

    return true;
}

sensor_msgs::PointCloud2 FrankaPanda::realsensePointcloudMessage() const
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