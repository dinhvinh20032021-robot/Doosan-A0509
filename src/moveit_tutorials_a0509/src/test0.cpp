#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_a0509_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Chỉnh tên nhóm cho A0509
  static const std::string PLANNING_GROUP = "arm"; // tên nhóm trong MoveIt cho A0509
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // PlanningSceneInterface
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Lấy thông tin Pose của end-effector
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("tutorial", "Robot ready to take commands for group: %s", PLANNING_GROUP.c_str());

  // Ví dụ Pose đơn giản để di chuyển end-effector
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.3;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.4;

  move_group.setPoseTarget(target_pose);

  // Lập kế hoạch
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    ROS_INFO_NAMED("tutorial", "Plan found, executing...");
    move_group.move();
  }
  else
  {
    ROS_WARN_NAMED("tutorial", "No valid motion plan found!");
  }

  ros::shutdown();
  return 0;
}