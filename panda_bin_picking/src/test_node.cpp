#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <control_msgs/GripperCommandAction.h>

#include <eigen3/Eigen/Geometry>

#define NODE_NAME "test_node"

static void error(const char* msg);
static void grasp(ros::NodeHandle& node_handle);

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  grasp(node_handle);

  ros::shutdown();
  return 0;
}

static void grasp(ros::NodeHandle& node_handle)
{
    // Open
    moveit::planning_interface::MoveGroupInterface move_group("hand");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setJointValueTarget(std::vector<double>{ 0.04, 0.04 });
    move_group.plan(plan);
    move_group.execute(plan);

    ROS_INFO("Press enter to home gripper");
    std::cin.get();

    // Home
    actionlib::SimpleActionClient<franka_gripper::HomingAction> client("/franka_gripper/homing", true);
    client.waitForServer();
    franka_gripper::HomingGoal goal;
    client.sendGoal(goal);

    bool finished_before_timeout = client.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else {
        ROS_ERROR("Action did not finish before the time out.");
    }

    ROS_INFO("Press enter to close gripper");
    std::cin.get();

    // // Close
    // actionlib::SimpleActionClient<control_msgs::GripperCommandAction> client2("/franka_gripper/gripper_action", true);
    // client2.waitForServer();
    // control_msgs::GripperCommandGoal goal2;
    // goal2.command.position = 0.01;
    // goal2.command.max_effort = 5.0;
    // client2.sendGoal(goal2);

    // finished_before_timeout = client.waitForResult(ros::Duration(30.0));

    // if (finished_before_timeout)
    // {
    //     actionlib::SimpleClientGoalState state = client.getState();
    //     ROS_INFO("Action finished: %s",state.toString().c_str());
    // }
    // else {
    //     ROS_ERROR("Action did not finish before the time out.");
    // }

    // Close
    actionlib::SimpleActionClient<franka_gripper::GraspAction> client2("/franka_gripper/grasp", true);
    client2.waitForServer();
    franka_gripper::GraspGoal goal2;
    goal2.width = 0.01;
    goal2.speed = 0.03;
    goal2.force = 3.0;
    goal2.epsilon.inner = 0.5;
    goal2.epsilon.outer = 0.5;
    client2.sendGoal(goal2);

    finished_before_timeout = client.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = client.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else {
        ROS_ERROR("Action did not finish before the time out.");
    }

    ROS_INFO("Press enter to move");
    std::cin.get();

    // move
    moveit::planning_interface::MoveGroupInterface move_group2("panda_arm");
    geometry_msgs::Pose pose = move_group2.getCurrentPose().pose;
    pose.position.z = pose.position.z + 0.03;
    move_group2.setPoseTarget(pose);
    move_group2.plan(plan);
    move_group2.execute(plan);
}

static void error(const char* msg)
{
  ROS_ERROR(msg);
  ros::shutdown();
  exit(1);
}