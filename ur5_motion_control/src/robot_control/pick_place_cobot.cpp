#include <ros/ros.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// tau = 1 rotation in radiants
const double tau = 2 * M_PI;

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface&
                            planning_scene_interface) {
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(5);

  // Add the wall
  collision_objects[0].id = "wall";
  collision_objects[0].header.frame_id = "base_link";
  // Define primitive dimension, position of the table 1
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type =
      collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.05;
  collision_objects[0].primitives[0].dimensions[1] = 0.5;
  collision_objects[0].primitives[0].dimensions[2] = 0.3;
  // pose of wall
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = -0.35;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.15;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  // Add tabe 1 to the scene
  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the ground
  collision_objects[1].id = "ground";
  collision_objects[1].header.frame_id = "base_link";
  // Define primitive dimension
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type =
      collision_objects[0].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 1;
  collision_objects[1].primitives[0].dimensions[1] = 1;
  collision_objects[1].primitives[0].dimensions[2] = 0.2;
  // pose
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = -0.11;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  // Add tabe 1 to the scene
  collision_objects[1].operation = collision_objects[1].ADD;

  // Add the box1
  collision_objects[2].id = "box1";
  collision_objects[2].header.frame_id = "base_link";
  // Define primitive dimension, position of the box1
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type =
      collision_objects[0].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.3;
  collision_objects[2].primitives[0].dimensions[1] = 0.3;
  collision_objects[2].primitives[0].dimensions[2] = 0.3;
  // pose of box1
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.0;
  collision_objects[2].primitive_poses[0].position.y = 0.65;
  collision_objects[2].primitive_poses[0].position.z = 0.15;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  // Add box1 to the scene
  collision_objects[2].operation = collision_objects[2].ADD;

  // Add the box2
  collision_objects[3].id = "box2";
  collision_objects[3].header.frame_id = "base_link";
  // Define primitive dimension, position of the box2
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type =
      collision_objects[0].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.3;
  collision_objects[3].primitives[0].dimensions[1] = 0.3;
  collision_objects[3].primitives[0].dimensions[2] = 0.20;
  // pose of box2
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = -0.5;
  collision_objects[3].primitive_poses[0].position.y = 0.0;
  collision_objects[3].primitive_poses[0].position.z = 0.1;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;
  // Add box2 to the scene
  collision_objects[3].operation = collision_objects[3].ADD;

  //   // Add the rod
  //   collision_objects[4].id = "rod";
  //   collision_objects[4].header.frame_id = "base_link";
  //   // Define primitive dimension, position of the rod
  //   collision_objects[4].primitives.resize(1);
  //   collision_objects[4].primitives[0].type =
  //       collision_objects[0].primitives[0].CYLINDER;
  //   collision_objects[4].primitives[0].dimensions.resize(3);
  //   collision_objects[4].primitives[0].dimensions[0] = 0.1;
  //   collision_objects[4].primitives[0].dimensions[1] = 0.02;
  //   collision_objects[4].primitives[0].dimensions[2] = 0.02;
  //   // pose of rod
  //   collision_objects[4].primitive_poses.resize(1);
  //   collision_objects[4].primitive_poses[0].position.x = 0.0;
  //   collision_objects[4].primitive_poses[0].position.y = 0.55;
  //   collision_objects[4].primitive_poses[0].position.z = 0.35;
  //   collision_objects[4].primitive_poses[0].orientation.w = 1.0;
  //   // Add rod to the scene
  //   collision_objects[4].operation = collision_objects[4].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cobot_pick_and_place");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  /////////////////////////////////////////////////////////////////////////////////
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  move_group.setPlanningTime(20.0);
  move_group.setNumPlanningAttempts(1000);

  addCollisionObject(planning_scene_interface);

  //////////////////////////
  // Add attaching object //
  //////////////////////////
  moveit_msgs::CollisionObject cube;
  cube.id = "grasping_object";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.02;
  primitive.dimensions[1] = 0.02;
  primitive.dimensions[2] = 0.1;

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.0;
  pose.position.y = 0.55;
  pose.position.z = 0.351;

  cube.primitives.push_back(primitive);
  cube.primitive_poses.push_back(pose);
  cube.operation = cube.ADD;
  cube.header.frame_id = "base_link";
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(cube);

  planning_scene_interface.addCollisionObjects(collision_objects);
  sleep(4);

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("shoulder_pan_joint");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.trigger();

  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Reference frame: %s",
                 move_group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("tutorial", "End effector link: %s",
                 move_group.getEndEffectorLink().c_str());

  // Start the demo
  visual_tools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the demo");

  ///////////////////////////////////////////////////
  // Description and creating object in  workspace //
  ///////////////////////////////////////////////////

  ////////////
  // Plan 1 //
  ////////////
  tf2::Quaternion quaternion;
  geometry_msgs::Pose target_pose1;

  quaternion.setRPY(0 * M_PI / 180, 180 * M_PI / 180, 0 * M_PI / 180);
  target_pose1.orientation = tf2::toMsg(quaternion);
  target_pose1.position.x = 0.0;
  target_pose1.position.y = 0.55;
  target_pose1.position.z = 0.45;
  move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s",
                 success ? "" : "FAILED");

  move_group.move();

  ///////////////
  // Attaching //
  ///////////////
  ROS_INFO("Attaching object to the robot");
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "tool0";
  attached_object.object = cube;
  planning_scene_interface.applyAttachedCollisionObject(attached_object);
  sleep(2);

  ////////////
  // Plan 2 //
  ////////////

  geometry_msgs::Pose target_pose2;
  quaternion.setRPY(-180 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180);

  target_pose2.orientation = tf2::toMsg(quaternion);
  target_pose2.position.x = -0.60;
  target_pose2.position.y = 0.0;
  target_pose2.position.z = 0.36;
  move_group.setPoseTarget(target_pose2);

  success = (move_group.plan(my_plan) ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s",
                 success ? "" : "FAILED");

  move_group.move();

  ///////////////
  // Detaching //
  ///////////////
  ROS_INFO("Detaching object to the robot");
  cube.operation = cube.REMOVE;
  attached_object.link_name = "tool0";
  attached_object.object = cube;
  planning_scene_interface.applyAttachedCollisionObject(attached_object);
  sleep(2);

  ////////////
  // Plan 3 //
  ////////////

  geometry_msgs::Pose target_pose3;
  quaternion.setRPY(-180 * M_PI / 180, 0 * M_PI / 180, 0 * M_PI / 180);

  target_pose3.orientation = tf2::toMsg(quaternion);
  target_pose3.position.x = 0.3;
  target_pose3.position.y = 0.2;
  target_pose3.position.z = 0.4;
  move_group.setPoseTarget(target_pose3);

  success = (move_group.plan(my_plan) ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s",
                 success ? "" : "FAILED");

  move_group.move();

  ros::waitForShutdown();
  return 0;
}