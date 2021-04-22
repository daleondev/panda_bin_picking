#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char** argv)
{
  // ROS initialisieren
  ros::init(argc, argv, "path_planning_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Planungsgruppe
  static const std::string PLANNING_GROUP = "panda_arm";

  // Move-Group Schnittstelle
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // Robotermodell
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // Roboterstatus
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Zielkonfiguration
  std::vector<double> target_joints = {0, 0, 0,-90 * M_PI / 180, 0, 90 * M_PI / 180, 0};
  move_group.setJointValueTarget(target_joints);

  // Bewegung Planen
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if(move_group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    // Fehler beim Planen -> Knoten beenden
    ros::shutdown();
    return 1;
  }

  // Bewegung Ausführen
  move_group.execute(plan);

  // Knoten beenden
  ros::shutdown();
  return 0;
}