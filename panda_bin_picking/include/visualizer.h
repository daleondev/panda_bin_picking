#pragma once

#include "custom_types.h"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

/**
 * Visualizer
 * 
 * Allows visualizing grasps, poses and trajectories in Rviz
 */
class Visualizer {
public:
    /**
     * Initializes the visualizer
     * 
     * @param[in] node_handle the ros node handle of the current node
     */
    static void init(ros::NodeHandle& node_handle);

    /**
     * Visualizes a pose in rviz
     * 
     * @param[in] pose the pose to visualize
     * @param[in] tcp_offset the offset from end effector to tcp
     */
    static void plotPose(const Pose& pose, const double tcp_offset);    
    /**
     * Visualizes a grasp in rviz
     * 
     * @param[in] pose the pose to visualize
     * @param[in] hand_geometry the robot gripper geometry
     * @param[in] tcp_offset the offset from end effector to tcp
     */
    static void plotGrasp(const Pose& pose, const HandGeometry& hand_geometry, const double tcp_offset);
    /**
     * Visualizes a trajectory in rviz
     * 
     * @param[in] traj the trajectory to visualize
     * @param[in] joint_model_group the joint model group of the robot
     */
    static void plotTrajectory(const moveit_msgs::RobotTrajectory& traj, const robot_model::JointModelGroup* joint_model_group);

    /**
     * Clears all visualizations from rviz
     */
    static void clear();
    
private:
    /**
     * Deleted Constructor (pure static class)
     */
    Visualizer() = delete;

    /**
     * Creates the Markers for visualizing a pose
     * 
     * @param[in] position the position of the pose
     * @param[in] rotation the rotation of the pose
     */
    static visualization_msgs::MarkerArray createPoseMarkers(const Position& position, const Rotation& rotation);
    /**
     * Creates the Markers for visualizing a poses axis
     * 
     * @param[in] origin the origin of the axis
     * @param[in] direction the direction of the axis
     * @param[in] color the color of the axis
     * @param[in] id the marker id of the axis
     * @param[in] frame_id the frame id of the axis
     */
    static visualization_msgs::Marker createAxisMarker(const Position& origin, const Position& direction, const Eigen::Vector4f& color, int id, const std::string& frame_id);

    /**
     * Creates the Markers for visualizing a grasp
     * 
     * @param[in] position the position of the pose
     * @param[in] rotation the rotation of the pose
     * @param[in] hand_geometry the robot gripper geometry
     */
    static visualization_msgs::MarkerArray createGraspMarkers(const Position& position, const Rotation& rotation, const HandGeometry& hand_geometry);
    /**
     * Creates the Markers for visualizing a grasps finger
     * 
     * @param[in] center the center of the finger
     * @param[in] rotation the rotation of the finger
     * @param[in] lwh the length, width and heigt of the finger
     * @param[in] id the marker id of the finger
     * @param[in] frame_id the frame id of the finger
     */
    static visualization_msgs::Marker createFingerMarker(const Position& center, const Rotation& rotation, const Eigen::Vector3d& lwh, int id, const std::string& frame_id);
    /**
     * Creates the Markers for visualizing a grasps hand base
     * 
     * @param[in] start the start of the hand base
     * @param[in] end the end of the hand base
     * @param[in] rotation the rotation of the hand base
     * @param[in] length the length of the hand base
     * @param[in] height the heigt of the hand base
     * @param[in] id the marker id of the hand base
     * @param[in] frame_id the frame id of the hand base
     */
    static visualization_msgs::Marker createHandBaseMarker(const Position& start, const Position& end, const Rotation& rotation, double length, double height, int id, const std::string& frame_id);

    /**
     * Publisher for visualization markers
     */
    static ros::Publisher pub_vis_;
    /**
     * Moveit visual tools
     */
    static moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};
