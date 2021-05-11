#include "franka_panda.h"
#include "visualizer.h"

#include <std_srvs/Empty.h>

#include <eigen_conversions/eigen_msg.h> 
#include <moveit/robot_state/conversions.h>

// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>

const HandGeometry FrankaPanda::HAND_GEOMETRY   { 0.046, 0.018, 0.09, 0.005 };
const JointList FrankaPanda::OPEN_HAND =        { 0.04, 0.04 };
const JointList FrankaPanda::CLOSED_HAND =      { 0.0, 0.0 };

const double FrankaPanda::TCP_OFFSET = 0.066; //0.1034

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

    // move_group_arm_.setPlannerId("PTP");

    pub_current_ = n.advertise<sensor_msgs::PointCloud2>(PC_CURRENT_TOPIC, 1);
    pub_stitched_ = n.advertise<sensor_msgs::PointCloud2>(PC_STITCHED_TOPIC, 1);
}

FrankaPanda::~FrankaPanda() = default;

bool FrankaPanda::detect(GraspConfigList& out_grasps)
{
    // Pose6D pose;
    // pose.position = Eigen::Vector3d(0.4627702524794986, 0.1558254417577506, 0.5902303306177155);
    // Eigen::Quaterniond EP;
    // EP.x() = 0.9239580220586624;
    // EP.y() = -0.3824938786685418;
    // EP.z() = 2.719745688542278e-05;
    // EP.w() = 7.426195154867366e-05;
    // pose.rotation_matrix = EP;

    // moveit_msgs::RobotTrajectory traj;
    // if (!planArmMotionLin(pose, traj)) {
    //     return false;
    // }
    // executeArmMotion(traj);

    // return false;

    if (!captureAndStitchRealsensePointclouds()) {
        ROS_ERROR("Failed to capture and stitch pointclouds");
        return false;
    }
    pub_stitched_.publish(realsensePointcloudMessage());

    gpd_ros::GraspConfigList::ConstPtr grasp_configs = ros::topic::waitForMessage<gpd_ros::GraspConfigList>(GRASPS_TOPIC);
    out_grasps = grasp_configs->grasps;

    if (out_grasps.empty()) {
        ROS_ERROR("No Grasp found");
        return false;
    }

    return true;
}

bool FrankaPanda::pickAndPlace(GraspConfigList& grasps)
{
    std::sort(grasps.begin(), grasps.end(), [](gpd_ros::GraspConfig lhs, gpd_ros::GraspConfig rhs) -> bool {
        return lhs.score.data > rhs.score.data;
    });

    PoseList poses;
    poses.reserve(grasps.size());
    for(const gpd_ros::GraspConfig& grasp : grasps) {
        poses.push_back(graspConfigToPose6D(grasp));
    }

    if (!pick(poses)) {
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
    if(!planArmMotionPtp(target_joints, plan)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(plan);
    captureRealsensePointcloud();
    pub_current_.publish(realsensePointcloudMessage());  

    target_joints = {-M_PI_4, 0.0, 0.0, -M_PI_2, -M_PI_4, M_PI_2, M_PI_4};
    if(!planArmMotionPtp(target_joints, plan)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(plan);
    captureRealsensePointcloud();
    pub_current_.publish(realsensePointcloudMessage());

    target_joints = {-M_PI_2, M_PI_4, 0.0, -M_PI/3.0, 0.0, M_PI_2, M_PI_4};
    if(!planArmMotionPtp(target_joints, plan)) {
        ROS_ERROR("Failed to plan motion for arm");
        return false;
    }
    executeArmMotion(plan);
    captureRealsensePointcloud();
    pub_current_.publish(realsensePointcloudMessage());

    return true;
}

bool FrankaPanda::pick(const PoseList& poses)
{
    for (Pose6D pose : poses) {
        Visualizer::plotGrasp(pose.position + TCP_OFFSET * pose.rotation_matrix.col(2).normalized(), pose.rotation_matrix * Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitZ()), HAND_GEOMETRY);     

        ROS_INFO("Press Enter to try moving to the pose");
        std::cin.get();

        if (tryPick(pose)) {
            return true;
        }

        pose.rotation_matrix = pose.rotation_matrix * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());  //Eigen::AngleAxisd(M_PI, pose.rotation_matrix.col(2).normalized());

        if (tryPick(pose)) {
            return true;
        }
    }

    return false;
}

bool FrankaPanda::tryPick(const Pose6D& pose)
{
    if (!approach(pose)) {
        ROS_WARN("Failed to approach");
        return false;
    }

    if (!openHand()) {
        ROS_WARN("Failed to open hand");
        return false;
    }

    // planning_scene::PlanningScene planning_scene(robot_model_);
    // collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    // acm.setEntry("table_link", robot_model_->getEndEffector("hand")->getLinkModelNamesWithCollisionGeometry(), true);

    if (!grasp(pose)) {
        ROS_WARN("Failed to grasp");
        return false;
    }

    std_srvs::Empty srv;
    ros::service::call(CLEAR_OCTOMAP_SERVICE, srv);

    if (!closeHand()) {
        ROS_WARN("Failed to close hand");
        return false;
    }

    // acm.setEntry("table_link", robot_model_->getEndEffector("hand")->getLinkModelNamesWithCollisionGeometry(), false);

    if (!lift(pose)) {
        ROS_WARN("Failed to lift");
        return false;
    }

    return true;
}

bool FrankaPanda::approach(const Pose6D& pose)
{
    Pose6D target_pose{ pose };

    const Eigen::Vector3d direction = target_pose.rotation_matrix.col(2);
    target_pose.position += -0.06*direction.normalized();

    Visualizer::plotPose(target_pose.position, target_pose.rotation_matrix);

    MoveGroupInterface::Plan plan;
    if (!planArmMotionPtp(target_pose, plan)) {
        return false;
    }
    executeArmMotion(plan);

    return true;
}

bool FrankaPanda::grasp(const Pose6D& pose)
{
    Visualizer::plotPose(pose.position, pose.rotation_matrix);

    moveit_msgs::RobotTrajectory traj;
    if (!planArmMotionLin(pose, traj)) {
        return false;
    }
    executeArmMotion(traj);

    return true;
}

bool FrankaPanda::lift(const Pose6D& pose)
{   
    Pose6D target_pose{ pose };

    target_pose.position.z() += 0.5;

    Visualizer::plotPose(target_pose.position, target_pose.rotation_matrix);

    moveit_msgs::RobotTrajectory traj;
    if (!planArmMotionLin(target_pose, traj)) {
        return false;
    }
    executeArmMotion(traj);

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

bool FrankaPanda::planArmMotionPtp(const JointList& target_joints, MoveGroupInterface::Plan& out_plan)
{
    move_group_arm_.setJointValueTarget(target_joints);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotionPtp(const Pose6D& target_pose, MoveGroupInterface::Plan& out_plan)
{
    geometry_msgs::Pose pose;
    tf::pointEigenToMsg(target_pose.position, pose.position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(target_pose.rotation_matrix), pose.orientation);

    move_group_arm_.setPoseTarget(pose);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotionPtp(const std::string& name, MoveGroupInterface::Plan& out_plan)
{
    move_group_arm_.setNamedTarget(name);
    return move_group_arm_.plan(out_plan) == MoveItErrorCode::SUCCESS;
}

bool FrankaPanda::planArmMotionLin(const Pose6D& target_pose, moveit_msgs::RobotTrajectory& out_traj)
{
    std::vector<geometry_msgs::Pose> waypoints(1);
    tf::pointEigenToMsg(target_pose.position, waypoints[0].position);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(target_pose.rotation_matrix), waypoints[0].orientation);

    const double jump_threshold = 0;
    const double eef_step = 0.01;
    const double fraction = move_group_arm_.computeCartesianPath(waypoints, eef_step, jump_threshold, out_traj);

    return fraction != -1.0;
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

bool FrankaPanda::executeArmMotion(const moveit_msgs::RobotTrajectory& traj, const float velocity)
{
    move_group_arm_.setMaxVelocityScalingFactor(velocity);
    return move_group_arm_.execute(traj) == MoveItErrorCode::SUCCESS;
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


// grasp.grasp_pose.header.frame_id = WORLD_FRAME;
// tf::pointEigenToMsg(pose.position, grasp.grasp_pose.pose.position);
// tf::quaternionEigenToMsg(Eigen::Quaterniond(pose.rotation_matrix), grasp.grasp_pose.pose.orientation);

// grasp.pre_grasp_approach.direction.header.frame_id = WORLD_FRAME;
// tf::vectorEigenToMsg(pose.rotation_matrix.col(2).normalized(), grasp.pre_grasp_approach.direction.vector);
// grasp.pre_grasp_approach.min_distance = 0.04;
// grasp.pre_grasp_approach.desired_distance = 0.06;

// grasp.post_grasp_retreat.direction.header.frame_id = WORLD_FRAME;
// tf::vectorEigenToMsg(Eigen::Vector3d::UnitZ(), grasp.post_grasp_retreat.direction.vector);
// grasp.post_grasp_retreat.min_distance = 0.1;
// grasp.post_grasp_retreat.desired_distance = 0.25;

// grasp.pre_grasp_posture.joint_names.resize(2);
// grasp.pre_grasp_posture.joint_names[0] = "panda_finger_joint1";
// grasp.pre_grasp_posture.joint_names[1] = "panda_finger_joint2";

// grasp.pre_grasp_posture.points.resize(1);
// grasp.pre_grasp_posture.points[0].positions.resize(2);
// grasp.pre_grasp_posture.points[0].positions[0] = 0.04;
// grasp.pre_grasp_posture.points[0].positions[1] = 0.04;
// grasp.pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

// grasp.grasp_posture.joint_names.resize(2);
// grasp.grasp_posture.joint_names[0] = "panda_finger_joint1";
// grasp.grasp_posture.joint_names[1] = "panda_finger_joint2";

// grasp.grasp_posture.points.resize(1);
// grasp.grasp_posture.points[0].positions.resize(2);
// grasp.grasp_posture.points[0].positions[0] = 0.00;
// grasp.grasp_posture.points[0].positions[1] = 0.00;
// grasp.grasp_posture.points[0].time_from_start = ros::Duration(0.5);

// grasps.push_back(grasp);