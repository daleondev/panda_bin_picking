#include "franka_panda.h"
#include "visualizer.h"

#include <std_srvs/Empty.h>

#include <eigen_conversions/eigen_msg.h> 
#include <moveit/robot_state/conversions.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>


const HandGeometry FrankaPanda::HAND_GEOMETRY           = { 0.045, 0.018, 0.09, 0.005 };
const JointList FrankaPanda::OPEN_HAND                  = { 0.04, 0.04 };
const JointList FrankaPanda::CLOSED_HAND                = { 0.0, 0.0 }; 

const double FrankaPanda::TCP_OFFSET                    = 0.1034; //0.066;

const std::string FrankaPanda::WORLD_FRAME =            "world";
const std::string FrankaPanda::ARM_GROUP =              "panda_arm";
const std::string FrankaPanda::HAND_GROUP =             "hand";
const std::string FrankaPanda::PC_GPD_TOPIC =           "/panda_bin_picking/cloud_gpd";
const std::string FrankaPanda::PC_OCTOMAP_TOPIC =       "/panda_bin_picking/cloud_octomap";
const std::string FrankaPanda::GRASPS_TOPIC =           "/detect_grasps/clustered_grasps";
const std::string FrankaPanda::CLEAR_OCTOMAP_SERVICE =  "/clear_octomap";

FrankaPanda::FrankaPanda(ros::NodeHandle& node_handle)
: move_group_arm_{ ARM_GROUP }, move_group_hand_{ HAND_GROUP }, robot_model_loader_{ "robot_description" }
{
    // setup robot and planning scene
    robot_model_ = robot_model_loader_.getModel();
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC, planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, true);

    // set default allowed planning time
    move_group_arm_.setPlanningTime(2.0);

    // setup pointcloud-publishers
    pub_gpd_ = node_handle.advertise<sensor_msgs::PointCloud2>(PC_GPD_TOPIC, 1);
    pub_octomap_ = node_handle.advertise<sensor_msgs::PointCloud2>(PC_OCTOMAP_TOPIC, 1);

#ifndef SIMULATION
    // homing gripper
    actionlib::SimpleActionClient<franka_gripper::HomingAction> client("/franka_gripper/homing", true);
    client.waitForServer();
    franka_gripper::HomingGoal goal;
    client.sendGoal(goal);
    if (client.waitForResult(ros::Duration(30.0))) {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Homing finished: %s",state.toString().c_str());
    }
#endif
}

FrankaPanda::~FrankaPanda() = default;

bool FrankaPanda::capture()
{
    PRINT_METHOD();

    // open the gripper at the beginning
    openHand();

    // clear everything pointcloud related
    clearOctomap();
    realsense_.clearPointclouds();

    // move to first capture pose (ptp) and capture pointcloud
    MoveGroupInterface::Plan plan;
    JointList target_joints = {-M_PI_2, 0.0, 0.0, -M_PI_2, 0.0, M_PI_2, -3*M_PI_4};
    if(!planArmMotionPtp(target_joints, plan, 0.5)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(plan);
    captureRealsensePointcloud();

    // move to next capture pose (lin) and capture pointcloud
    moveit_msgs::RobotTrajectory traj;
    geometry_msgs::Pose pose = move_group_arm_.getCurrentPose().pose;
    pose.position.y = pose.position.y + 0.2;
    if(!planArmMotionLin(pose, traj, 0.05)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(traj);
    captureRealsensePointcloud();

    // move to next capture pose (lin) and capture pointcloud
    pose.position.x = pose.position.x + 0.2;
    if(!planArmMotionLin(pose, traj, 0.05)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(traj);
    captureRealsensePointcloud();

    // move to next capture pose (lin) and capture pointcloud
    pose.position.y = pose.position.y - 0.2;
    if(!planArmMotionLin(pose, traj, 0.05)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(traj);
    captureRealsensePointcloud();

    // fuse all captured pointclouds together
    realsense_.fusePointclouds();
    
#ifndef SIMULATION
    realsense_.savePointclouds(); // Debug
#endif

    return true;
}

bool FrankaPanda::detect()
{
    PRINT_METHOD();
    
    // publish the fused pointcloud (for octomap and grasp pose detection)
    const sensor_msgs::PointCloud2 pc = getRealsensePointcloud();
    pub_octomap_.publish(pc);
    pub_gpd_.publish(pc);

    // wait for the detected grasps
    gpd_ros::GraspConfigList::ConstPtr grasp_configs = ros::topic::waitForMessage<gpd_ros::GraspConfigList>(GRASPS_TOPIC, ros::Duration(30.0));
    if (!grasp_configs) {
        ROS_WARN("No Grasp found");
        return false;
    }

    // check if grasps were found
    std::vector<gpd_ros::GraspConfig> grasps = grasp_configs->grasps;
    if (grasps.empty()) {
        ROS_WARN("No Grasp found");
        return false;
    }

    // sort (by score) and convert grasp poses
    std::sort(grasps.begin(), grasps.end(), [](gpd_ros::GraspConfig lhs, gpd_ros::GraspConfig rhs) -> bool {
        return lhs.score.data < rhs.score.data;
    });
    poses_.clear();
    poses_.reserve(grasps.size());
    for(const gpd_ros::GraspConfig& grasp : grasps) {
        poses_.push_back(graspConfigToPose6D(grasp));
    }
    pose_index_ = poses_.size();

#ifdef SIMULATION
    saveGraspPoses6D(); // Debug
#endif

    return true;
}

bool FrankaPanda::planApproach()
{
    PRINT_METHOD();
    
    plan_ = MoveGroupInterface::Plan();

    // loop through all poses (starting at best score)
    while (pose_index_ > 0) {
        pose_index_--;

        // modify grasp pose if necessary
        //  - rotate hand if camera is facing the table
        if (Rotation(poses_[pose_index_].rotation * Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitZ())).col(0)[2] > 0) {
            poses_[pose_index_].rotation = poses_[pose_index_].rotation * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
        }

        // visualize desired grasp pose
        Visualizer::plotGrasp(poses_[pose_index_], HAND_GEOMETRY, TCP_OFFSET);
        
        // calculate target pose for approach
        //  - approach pose is 100mm in front of object
        const Eigen::Vector3d direction = poses_[pose_index_].rotation.col(2).normalized();        
        Pose target_pose{ poses_[pose_index_].position + -0.1*direction, poses_[pose_index_].rotation };

        // visualize target pose
        Visualizer::plotPose(target_pose, TCP_OFFSET);  

        // try planning a path for the target pose 
        if (!planArmMotionPtp(target_pose, plan_, 0.05)) {
            continue;
        }

        // visualize the planned paths trajectory
        Visualizer::plotTrajectory(plan_.trajectory_, robot_model_->getJointModelGroup(ARM_GROUP));
        return true;
    }

    return false;
}

bool FrankaPanda::approach()
{
    PRINT_METHOD();
    
    // open the gripper
    if (!openHand()) {
        return false;
    }

    // execute the approach plan (planned in "planApproach()")
    return executeArmMotion(plan_);
}

bool FrankaPanda::planPick()
{
    PRINT_METHOD();
    
    // clear the octomap
    //  - reason:   if the octomap isnt cleared, the hand will most likely be in collision with it and the planning fails,
    //              but when picking we want the gripper to actually collide with the object
    clearOctomap();

    // visualize gripper pose
    Visualizer::plotGrasp(poses_[pose_index_], HAND_GEOMETRY, TCP_OFFSET);
    Visualizer::plotPose(poses_[pose_index_], TCP_OFFSET);

    // plan trajectory to target pose (lin)
    traj_ = moveit_msgs::RobotTrajectory();
    if (!planArmMotionLin(poses_[pose_index_], traj_, 0.02)) {
        pub_octomap_.publish(getRealsensePointcloud());
        return false;
    }

    // visualize the planned trajectory
    Visualizer::plotTrajectory(traj_, robot_model_->getJointModelGroup(ARM_GROUP));

    return true;
}

bool FrankaPanda::pick()
{
    PRINT_METHOD();
    
    // execute the pick trajectory (planned in "planPick()")
    if(!executeArmMotion(traj_)) {
        pub_octomap_.publish(getRealsensePointcloud());
        return false;
    }

    // close gripper (pick the object)
    if(!closeHand()) {
        pub_octomap_.publish(getRealsensePointcloud());
        return false;
    }
}

bool FrankaPanda::planLift()
{
    PRINT_METHOD();
    
    // calculate target pose (200m above pick pose)
    Pose target_pose{ poses_[pose_index_] };
    target_pose.position.z() += 0.2;

    // visualize target pose
    Visualizer::plotPose(target_pose, TCP_OFFSET);

    // plan trajectory to target pose (lin)
    traj_ = moveit_msgs::RobotTrajectory();
    if(!planArmMotionLin(target_pose, traj_, 0.05)) {
        return false;
    }

    // visualize planned trajectory
    Visualizer::plotTrajectory(traj_, robot_model_->getJointModelGroup(ARM_GROUP));

    return true;
}

bool FrankaPanda::planLiftAlt()
{
    PRINT_METHOD();

    // DEPRECATED
    
    // plan_ = MoveGroupInterface::Plan();

    // Pose target_pose{ poses_[pose_index_] };
    // target_pose.position.z() += 0.2;

    // Visualizer::plotPose(target_pose.position, target_pose.rotation, HAND_GEOMETRY, TCP_OFFSET);

    // if(!planArmMotionPtp(target_pose, plan_, 0.01)) {
    //     return false;
    // }

    // Visualizer::plotTrajectory(plan_.trajectory_, robot_model_->getJointModelGroup(ARM_GROUP));

    // return true;
}

bool FrankaPanda::lift()
{
    PRINT_METHOD();
    
    // execute the lift trajectory (planned in "planLift()")
    pub_octomap_.publish(getRealsensePointcloud());
    return executeArmMotion(traj_);
}

bool FrankaPanda::liftAlt()
{
    PRINT_METHOD();

    // DEPRECATED
   
    // pub_octomap_.publish(getRealsensePointcloud());
    // return executeArmMotion(plan_);
}

bool FrankaPanda::planPlace()
{
    PRINT_METHOD();
    
    // target configuration to place object
    const JointList target_joints = {0, M_PI_4, 0.0, -M_PI/3.0, 0.0, M_PI_2, M_PI_4};

    // plan trajectory to target configuration (ptp)
    if(!planArmMotionPtp(target_joints, plan_, 0.1)) {
        return false;
    }

    // visualize planned path
    Visualizer::plotTrajectory(plan_.trajectory_, robot_model_->getJointModelGroup(ARM_GROUP));

    return true;
}

bool FrankaPanda::place()
{
    PRINT_METHOD();
    
    // execute the place path (planned in "planPlace()")
    if(!executeArmMotion(plan_)) {
        return false;
    }

    // open gripper (release object)
    return openHand();
}

bool FrankaPanda::getTransform(const std::string& target_frame, const std::string& source_frame, tf::StampedTransform& out_transform) const
{
    PRINT_METHOD();
    
    // listen for the desired transformation
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

bool FrankaPanda::captureRealsensePointcloud()
{
    PRINT_METHOD();

#ifndef SIMULATION
    // wait for camera to be steady
    ros::Duration(4.0).sleep();
#endif
    
    // get transformation from depth frame to world frame
    tf::StampedTransform transform;
    if (!getTransform(WORLD_FRAME, realsense_.getDepthFrame(), transform)) {
        return false;
    }

    // capture a pointcloud
    realsense_.capturePointcloud(transform);

    return true;
}

sensor_msgs::PointCloud2 FrankaPanda::getRealsensePointcloud() const
{
    PRINT_METHOD();
    
    // retrieve fused pointlcoud
    sensor_msgs::PointCloud2 pc = realsense_.getPointcloud();  
    pc.header.frame_id = WORLD_FRAME;
    pc.header.stamp = ros::Time::now();

    return pc;
}

void FrankaPanda::clearOctomap() const
{
    PRINT_METHOD();

    // note:    sometimes some methods dont work (not sure why)
    //          so just try all methods (at least one will most likely work)

    // clear octomap through planning scene monitor
    planning_scene_monitor_->clearOctomap();
    
    // clear octomap through service call
    std_srvs::Empty srv;
    ros::service::call(CLEAR_OCTOMAP_SERVICE, srv);

    // clear octomap by publishing empty pointcloud
    sensor_msgs::PointCloud2 pc;
    pub_octomap_.publish(pc);
}

bool FrankaPanda::openHand()
{
    PRINT_METHOD();
    
    // plan and execute hand motion to open gripper
    MoveGroupInterface::Plan plan;
    if (!planHandMotion(OPEN_HAND, plan, 0.03)) {
        return false;
    }
    executeHandMotion(plan);

    return true;
}

bool FrankaPanda::closeHand()
{
    PRINT_METHOD();

#ifdef SIMULATION
    // plan and execute hand motion to close gripper
    MoveGroupInterface::Plan plan;
    if (!planHandMotion(CLOSED_HAND, plan, 0.01)) {
        return false;
    }
    executeHandMotion(plan);
#else
    // setup action client for communication with franka gripper node
    actionlib::SimpleActionClient<franka_gripper::GraspAction> client("/franka_gripper/grasp", true);
    client.waitForServer();

    // create grasp goal
    //  - operation is successful if the distance d between the gripper fingers is: width − ϵ inner < d < width+ ϵ outer
    franka_gripper::GraspGoal goal;
    goal.width = 0.03;
    goal.speed = 0.02;
    goal.force = 6.0;
    goal.epsilon.inner = 0.05;
    goal.epsilon.outer = 0.05;

    // send goal to action server
    client.sendGoal(goal);
    bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

    // wait for action server to finish
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else {
        ROS_ERROR("Action did not finish before the time out.");
        return false;
    }
#endif

    return true;
}

Pose FrankaPanda::graspConfigToPose6D(const gpd_ros::GraspConfig& grasp) const
{
    PRINT_METHOD();

    Pose pose;
    
    // retrieve approach, binormal and axis from grasp configuration
    Eigen::Vector3d approach, binormal, axis;
    tf::vectorMsgToEigen(grasp.approach, approach);
    tf::vectorMsgToEigen(grasp.binormal, binormal);
    tf::vectorMsgToEigen(grasp.axis, axis);

    // convert rotation
    //  - gpd grasps have x-axis pointing towards object
    //  - we need z-axis pointing towards object
    pose.rotation.col(0) = axis;
    pose.rotation.col(1) = -binormal;
    pose.rotation.col(2) = approach;
    pose.rotation = pose.rotation * Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ());

    // adjust position
    //  - gpd grasps have their origin in the "hand-palm"
    //  - we need the position of the endeffektor-link (panda_link8)
    tf::pointMsgToEigen(grasp.position, pose.position);
    const Eigen::Vector3d direction = pose.rotation.col(2).normalized();
    pose.position -= (TCP_OFFSET-HAND_GEOMETRY.depth)*direction;

    return pose;
}

bool FrankaPanda::planArmMotionPtp(const JointList& target_joints, MoveGroupInterface::Plan& out_plan, const float velocity)
{
    PRINT_METHOD();
    
    // set velocity
    move_group_arm_.setMaxVelocityScalingFactor(velocity);

    // plan path
    move_group_arm_.setJointValueTarget(target_joints);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotionPtp(const geometry_msgs::Pose& target_pose, MoveGroupInterface::Plan& out_plan, const float velocity)
{
    PRINT_METHOD();
    
    // set velocity
    move_group_arm_.setMaxVelocityScalingFactor(velocity);

    // plan path
    move_group_arm_.setPoseTarget(target_pose);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotionPtp(const Pose& target_pose, MoveGroupInterface::Plan& out_plan, const float velocity)
{
    PRINT_METHOD();
    
    // set velocity
    move_group_arm_.setMaxVelocityScalingFactor(velocity);

    // convert pose
    geometry_msgs::Pose pose;
    tf::pointEigenToMsg(target_pose.position, pose.position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(target_pose.rotation), pose.orientation);

    // plan path
    move_group_arm_.setPoseTarget(pose);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotionPtp(const std::string& name, MoveGroupInterface::Plan& out_plan, const float velocity)
{
    PRINT_METHOD();
    
    // set velocity
    move_group_arm_.setMaxVelocityScalingFactor(velocity);

    // plan path
    move_group_arm_.setNamedTarget(name);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotionLin(const geometry_msgs::Pose& target_pose, moveit_msgs::RobotTrajectory& out_traj, const float velocity)
{
    PRINT_METHOD();

    // plan trajectory
    const double jump_threshold = 0;
    const double eef_step = 0.01;
    move_group_arm_.setPlanningTime(10.0);
    const double fraction = move_group_arm_.computeCartesianPath({target_pose}, eef_step, jump_threshold, out_traj);
    move_group_arm_.setPlanningTime(2.0);
    ROS_INFO("%.2f%% acheived", fraction * 100.0);

    // set velocity
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    robot_trajectory::RobotTrajectory traj(robot_model_, robot_model_->getJointModelGroup(ARM_GROUP));
    traj.setRobotTrajectoryMsg(*move_group_arm_.getCurrentState(), out_traj);
    iptp.computeTimeStamps(traj, velocity);
    traj.getRobotTrajectoryMsg(out_traj);

    return fraction > 0.99;
}

bool FrankaPanda::planArmMotionLin(const Pose& target_pose, moveit_msgs::RobotTrajectory& out_traj, const float velocity)
{
    PRINT_METHOD();
    
    // convert pose
    std::vector<geometry_msgs::Pose> waypoints(1);
    tf::pointEigenToMsg(target_pose.position, waypoints[0].position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(target_pose.rotation), waypoints[0].orientation);

    // plan trajectory
    const double jump_threshold = 0;
    const double eef_step = 0.01;
    move_group_arm_.setPlanningTime(10.0);
    const double fraction = move_group_arm_.computeCartesianPath(waypoints, eef_step, jump_threshold, out_traj);
    move_group_arm_.setPlanningTime(2.0);
    ROS_INFO("%.2f%% acheived", fraction * 100.0);

    // set velocity
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    robot_trajectory::RobotTrajectory traj(robot_model_, robot_model_->getJointModelGroup(ARM_GROUP));
    traj.setRobotTrajectoryMsg(*move_group_arm_.getCurrentState(), out_traj);
    iptp.computeTimeStamps(traj, velocity);
    traj.getRobotTrajectoryMsg(out_traj);

    return fraction > 0.99;
}

bool FrankaPanda::planHandMotion(const JointList& target_joints, MoveGroupInterface::Plan& out_plan, const float velocity)
{
    PRINT_METHOD();
    
    // set velocity
    move_group_arm_.setMaxVelocityScalingFactor(velocity);

    // plan path
    move_group_hand_.setJointValueTarget(target_joints);
    return move_group_hand_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planHandMotion(const std::string& name, MoveGroupInterface::Plan& out_plan, const float velocity)
{
    PRINT_METHOD();
    
    // set velocity
    move_group_arm_.setMaxVelocityScalingFactor(velocity);

    // plan path
    move_group_hand_.setNamedTarget(name);
    return move_group_hand_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::executeArmMotion(const MoveGroupInterface::Plan& plan)
{
    PRINT_METHOD();
    
    // execute plan
    return move_group_arm_.execute(plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::executeArmMotion(const moveit_msgs::RobotTrajectory& traj)
{    
    PRINT_METHOD();
    
    // execute trajectory
    return move_group_arm_.execute(traj) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::executeHandMotion(const MoveGroupInterface::Plan& plan)
{
    PRINT_METHOD();
    
    // execute plan
    return move_group_hand_.execute(plan) == MoveItErrorCode::SUCCESS;
}

void FrankaPanda::saveGraspPoses6D() const
{
    PRINT_METHOD();

    // print all detected poses (for debugging)
    std::ofstream out("poses.txt");
    out << std::setprecision(16) << std::scientific;
    for (const Pose& pose : poses_) {
        out << pose.position.x() << ' ';
    }
    out << '\n';
    for (const Pose& pose : poses_) {
        out << pose.position.y() << ' ';
    }
    out << '\n';
    for (const Pose& pose : poses_) {
        out << pose.position.z() << ' ';
    }
    out << '\n';
    for (const Pose& pose : poses_) {
        const Eigen::Vector3d rpy = pose.rotation.eulerAngles(2, 1, 0);
        out << rpy[0] << ' ';
    }
    out << '\n';
    for (const Pose& pose : poses_) {
        const Eigen::Vector3d rpy = pose.rotation.eulerAngles(2, 1, 0);
        out << rpy[1] << ' ';
    }
    out << '\n';
    for (const Pose& pose : poses_) {
        const Eigen::Vector3d rpy = pose.rotation.eulerAngles(2, 1, 0);
        out << rpy[2] << ' ';
    }
    out << '\n';
}










    // const auto deg2rad = [](const double deg) -> double {
    //     return deg * M_PI / 180.0;
    // };

    // JointList target_joints = {-3*M_PI_4-deg2rad(10), -deg2rad(30), 0.0, -M_PI_2-deg2rad(20), M_PI_4, M_PI_2-deg2rad(10), -M_PI_4};
    // if(!planArmMotionPtp(target_joints, plan, 0.5)) {
    //     ROS_ERROR("Failed to plan motion for arm");
    //     return false;
    // }
    // executeArmMotion(plan);
    // captureRealsensePointcloud(); 

    // target_joints = {-M_PI_4+deg2rad(10), -deg2rad(30), 0.0, -M_PI_2-deg2rad(20), -M_PI_4, M_PI_2-deg2rad(10), 3*M_PI_4};
    // if(!planArmMotionPtp(target_joints, plan, 0.5)) {
    //     ROS_ERROR("Failed to plan motion for arm");
    //     return false;
    // }
    // executeArmMotion(plan);
    // captureRealsensePointcloud();

    // target_joints = {-M_PI_2, deg2rad(20), 0.0, -M_PI_2+deg2rad(20), 0.0, M_PI_2-deg2rad(20), -3*M_PI_4};
    // if(!planArmMotionPtp(target_joints, plan, 0.5)) {
    //     ROS_ERROR("Failed to plan motion for arm");
    //     return false;
    // }
    // executeArmMotion(plan);
    // captureRealsensePointcloud();