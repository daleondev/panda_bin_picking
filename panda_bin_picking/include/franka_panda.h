#pragma once

#include "realsense_l515.h"
#include "custom_types.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <gpd_ros/GraspConfigList.h>

using moveit::planning_interface::MoveItErrorCode;
using moveit::planning_interface::MoveGroupInterface;

/**
 * Abstraction of the Franka Panda robot with an Intel RealSense LiDAR Camera L515 connected
 * 
 * Allows interaction with the robot (e.g. motion planning, moving, ...)
 */
class FrankaPanda {
public:
    /**
     * Constructor
     * 
     * @param[in] node_handle the ros node handle of the current node
     */
    FrankaPanda(ros::NodeHandle& node_handle);
    /**
     * Destructor
     */
    ~FrankaPanda();

    /**
     * Moves robot to capture poses and capture pointclouds
     * 
     * @return bool success
     */
    bool capture();
    /**
     * Sends pointcloud to grasp pose detection and store the received poses
     * 
     * @return bool success
     */
    bool detect();
    /**
     * Plans a path for approaching the grasp pose
     * 
     * @return bool success
     */
    bool planApproach();
    /**
     * Approaches the grasp pose
     * 
     * @return bool success
     */
    bool approach();
    /**
     * Plans a path for picking the object
     * 
     * @return bool success
     */
    bool planPick();
    /**
     * Picks the object
     * 
     * @return bool success
     */
    bool pick();
    /**
     * Plans a path for lifting the picked object
     * 
     * @return bool success
     */
    bool planLift();
    /**
     * Plans an alternative path for lifting the picked object [DEPRECATED]
     * 
     * @return bool success
     */
    bool planLiftAlt();
    /**
     * Lifts the picked object
     * 
     * @return bool success
     */
    bool lift();
    /**
     * Lifts the picked object alternatively [DEPRECATED]
     * 
     * @return bool success
     */
    bool liftAlt();
    /**
     * Plans a path for placing the picked object
     * 
     * @return bool success
     */
    bool planPlace();
    /**
     * Places the picked object
     * 
     * @return bool success
     */
    bool place();
    
    /**
     * Gets a desired transformation in the robot model
     * 
     * @param[in] target_frame the target frame of the transformation
     * @param[in] source_frame the source frame of the transformation
     * @param[out] out_transform the calculated transformation
     * @return bool success
     */
    bool getTransform(const std::string& target_frame, const std::string& source_frame, tf::StampedTransform& out_transform) const;
    
private:
    /**
     * Captures a pointcloud using the realsense l515 camera
     * 
     * @return bool success
     */   
    bool captureRealsensePointcloud();
    /**
     * Get the fused pointcloud of the captured clouds from the realsense l515 camera
     * 
     * @return PointCloud2 the pointcloud
     */  
    sensor_msgs::PointCloud2 getRealsensePointcloud() const;
    /**
     * Clears the octomap in the planning scene
     */ 
    void clearOctomap() const;

    /**
     * Opens the gripper
     * 
     * @return bool success
     */ 
    bool openHand();
    /**
     * Closes the gripper
     * 
     * @return bool success
     */ 
    bool closeHand();   
    
    /**
     * Converts a grasp config from the grasp pose detection into a 6D-Pose for the robot
     * 
     * @param[in] grasp the grasp config
     * @return Pose the converted 6D-Pose
     */
    Pose graspConfigToPose6D(const gpd_ros::GraspConfig& grasp) const;

    /**
     * Plans a ptp motion for the robot arm
     * 
     * @param[in] target_joints the desired joint configuration
     * @param[out] out_plan the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */
    bool planArmMotionPtp(const JointList& target_joints, MoveGroupInterface::Plan& out_plan, const float velocity = 1.0);
    /**
     * Plans a ptp motion for the robot arm
     * 
     * @param[in] target_pose the desired end effector pose
     * @param[out] out_plan the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */
    bool planArmMotionPtp(const geometry_msgs::Pose& target_pose, MoveGroupInterface::Plan& out_plan, const float velocity = 1.0); 
    /**
     * Plans a ptp motion for the robot arm
     * 
     * @param[in] target_pose the desired end effector pose
     * @param[out] out_plan the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */
    bool planArmMotionPtp(const Pose& target_pose, MoveGroupInterface::Plan& out_plan, const float velocity = 1.0);  
    /**
     * Plans a ptp motion for the robot arm
     * 
     * @param[in] name the desired poses name
     * @param[out] out_plan the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */ 
    bool planArmMotionPtp(const std::string& name, MoveGroupInterface::Plan& out_plan, const float velocity = 1.0);
    /**
     * Plans a lin motion for the robot arm
     * 
     * @param[in] target_joints the desired joint configuration
     * @param[out] out_traj the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */ 
    bool planArmMotionLin(const JointList& target_joints, moveit_msgs::RobotTrajectory& out_traj, const float velocity = 1.0);
    /**
     * Plans a lin motion for the robot arm
     * 
     * @param[in] target_pose the desired end effector pose
     * @param[out] out_traj the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */ 
    bool planArmMotionLin(const geometry_msgs::Pose& target_pose, moveit_msgs::RobotTrajectory& out_traj, const float velocity = 1.0);
    /**
     * Plans a lin motion for the robot arm
     * 
     * @param[in] target_pose the desired end effector pose
     * @param[out] out_traj the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */  
    bool planArmMotionLin(const Pose& target_pose, moveit_msgs::RobotTrajectory& out_traj, const float velocity = 1.0); 
    /**
     * Plans a lin motion for the robot arm
     * 
     * @param[in] name the desired poses name
     * @param[out] out_traj the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */  
    bool planArmMotionLin(const std::string& name, moveit_msgs::RobotTrajectory& out_traj, const float velocity = 1.0);
    /**
     * Plans a motion for the robot gripper
     * 
     * @param[in] target_joints the desired joint configuration
     * @param[out] out_plan the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */ 
    bool planHandMotion(const JointList& target_joints, MoveGroupInterface::Plan& out_plan, const float velocity = 1.0);
    /**
     * Plans a motion for the robot gripper
     * 
     * @param[in] target_joints the desired configurations name
     * @param[out] out_plan the planning result
     * @param[in] velocity the desired velocity scaling of the motion
     * @return bool success
     */ 
    bool planHandMotion(const std::string& name, MoveGroupInterface::Plan& out_plan, const float velocity = 1.0);

    /**
     * Executes a robot arm motion
     * 
     * @param[out] plan the motion plan
     * @return bool success
     */ 
    bool executeArmMotion(const MoveGroupInterface::Plan& plan);
    /**
     * Executes a robot arm motion
     * 
     * @param[out] traj the motion trajectory
     * @return bool success
     */ 
    bool executeArmMotion(const moveit_msgs::RobotTrajectory& traj);
    /**
     * Executes a robot gripper motion
     * 
     * @param[out] plan the motion plan
     * @return bool success
     */ 
    bool executeHandMotion(const MoveGroupInterface::Plan& plan);

    /**
     * Saves the converted grasp poses from the grasp pose detection (for debugging)
     */ 
    void saveGraspPoses6D() const;

    /**
     * Move group for the robot arm
     */ 
    MoveGroupInterface move_group_arm_;
    /**
     * Move group for the robot gripper
     */ 
    MoveGroupInterface move_group_hand_;
    /**
     * Robot model loader
     */ 
    robot_model_loader::RobotModelLoader robot_model_loader_;
    /**
     * Robot model
     */ 
    robot_model::RobotModelPtr robot_model_;
    /**
     * Robot state
     */ 
    robot_state::RobotStatePtr robot_state_;
    /**
     * Planning scene monitor
     */ 
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    /**
     * Intel RealSense LiDAR Camera L515 abstraction
     */ 
    RealsenseL515 realsense_;

    /**
     * Planned path
     */ 
    MoveGroupInterface::Plan plan_;
    /**
     * Planned trajectory
     */ 
    moveit_msgs::RobotTrajectory traj_;
    /**
     * Index of current grasp pose
     */ 
    size_t pose_index_;
    /**
     * Grasp poses from grasp pose detection
     */ 
    PoseList poses_;

    /**
     * Publisher to send pointclouds to grasp pose detection
     */ 
    ros::Publisher pub_gpd_;
    /**
     * Publisher to send pointclouds to octomap
     */ 
    ros::Publisher pub_octomap_;

    /**
     * Offset from robot end effector (panda_link8) to the robot gripper tcp (between the fingertips)
     */ 
    static const double TCP_OFFSET;

    /**
     * Robot gripper geometry
     */ 
    static const HandGeometry HAND_GEOMETRY;
    /**
     * Open gripper configuration
     */ 
    static const JointList OPEN_HAND;
    /**
     * Closed gripper configuration
     */ 
    static const JointList CLOSED_HAND;

    /**
     * World frame name
     */ 
    static const std::string WORLD_FRAME;
    /**
     * Robot arm planning group name
     */ 
    static const std::string ARM_GROUP;
    /**
     * Robot gripper planning group name
     */ 
    static const std::string HAND_GROUP;
    /**
     * Grasp pose detection pointcloud topic (out)
     */ 
    static const std::string PC_GPD_TOPIC;
    /**
     * Octomap pointcloud topic (out)
     */ 
    static const std::string PC_OCTOMAP_TOPIC;
    /**
     * Grasp pose detection grasps topic (in)
     */ 
    static const std::string GRASPS_TOPIC;
    /**
     * Service name for clearing octomap
     */ 
    static const std::string CLEAR_OCTOMAP_SERVICE;  
};
