/* Code for a move_group subtask part of the project ""New task generation for humanoid robots based on case and user communication". This code is based in the code https://github.com/ros-planning/moveit_pr2/blob/indigo-devel/pr2_moveit_tutorials/pick_place/src/pick_place_tutorial.cpp by Ioan Sucan.
 *
 * Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)
 *
 * The objective of this code is de performing of pick and place actions with the robot TEO.
 *
 */


#include <ros/ros.h>

//MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
//#include <teo_moveit/custom_environment5.h>

//Load the robot description
static const std::string ROBOT_DESCRIPTION="robot_description";

void pick(moveit::planning_interface::MoveGroup &group)
{
  geometry_msgs::PoseStamped p;
  /*** MESSAGE INFO **
  # A Pose with reference coordinate frame and timestamp
  Header header
  Pose pose
  **/
  //First we move the moveit group to the place where is the object we want to pick.
  p.header.frame_id = "base_link";
  //The position of the manipulator relative to base_link for the grasp
  //p.pose.position.x = 0.32;
  p.pose.position.x = 0.41449;
  //p.pose.position.y = -0.7;
  p.pose.position.y = -0.45821;
  p.pose.position.z = 0.50566;
  p.pose.orientation.x = 2.3899e-05;
  p.pose.orientation.y = 4.042e-05;
  p.pose.orientation.z = -0.00013261;
  p.pose.orientation.w = 1;

  // Link confugurations for the said pose
  // r_wrist_pitch_link: 0.41449; -0.45821; 0.50566; orientation: 2.3899e-05; 4.042e-05; -0.00013261; 1
  // r_wrist_yaw_link: 0.41449; -0.45821; 0.50566; orientation: -3.0244e-05; -0.39368; -0.00013131; 0.91925
  // r_elbow_pitch_link: 0.26829; -0.45818; 0.64504; orientation: 0.074875; -0.38649; -0.17503; 0.90243

  /*co.primitive_poses[0].position.x = 0.5;
  //co.primitive_poses[0].position.y = -0.7;
  co.primitive_poses[0].position.y = -0.3;
  co.primitive_poses[0].position.z = 0.5;
  */

  //Get the EEF id
  std::string end_effector_link;
  end_effector_link = group.getEndEffectorLink();
  std::cout << "the endeffector link is ===> " << end_effector_link<< std::endl;

  /* //Test code for random positions

  geometry_msgs::PoseStamped target_pose1;
  target_pose1= group.getRandomPose();
  printf("the random pose is %f %f %f /n", target_pose1.pose.position.x, target_pose1.pose.position.y, target_pose1.pose.position.z);
  group.setPoseTarget(target_pose1, end_effector_link);
  group.setGoalTolerance(0.4);
  //group.setPlannerId("ESTkConfigDefault");


  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(5.0);
  */

  group.setGoalTolerance(0.01);
  group.setPoseTarget(p, end_effector_link);

  bool success;
  int i=0;

  while (success == false){
    success = group.move();
    i++;
  }

  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(20.0);

  group.attachObject("part");
  ROS_INFO("End of attaching");

  //std::cout<<"FIN DEL SINGLE PLAN";
}

void place(moveit::planning_interface::MoveGroup &group)
{
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
  p.pose.position.x = 0.42231;
  p.pose.position.y = -0.24126;
  p.pose.position.z = 0.50166;
  p.pose.orientation.x = 0.0047819;
  p.pose.orientation.y = -0.0024211;
  p.pose.orientation.z = -0.0027795;
  p.pose.orientation.w = 0.99998;

  //place position
  // 0.42231; -0.24126; 0.50166; 0.0047819; -0.0024211; -0.0027795; 0.99998;

  //Get the EEF id
  std::string end_effector_link;
  end_effector_link = group.getEndEffectorLink();
  //std::cout << "the endeffector is :::::> " << end_effector_link<< "/n";

  group.setGoalTolerance(0.01);
  group.setPoseTarget(p, end_effector_link);

  bool success;
  int i=0;

  while (success == false){
    success = group.move();
    i++;
  }

  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(5.0);

  //Once we have moved the arm near the object the next step is to grab it. Note that since our EEF doesnt have any DOF,
  //we cant perform the grasp action. For this reason in this step we only do a simple attach action of the object to the link.
  //pub_aco.publish(aco);

  group.detachObject("part");
  ROS_INFO("End of detaching");

  //std::cout<<"FIN DETACHING PLAN";
}

void reset(moveit::planning_interface::MoveGroup &group)
{
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
  p= group.getCurrentPose();
  p.pose.position.x = -0.019606;
  p.pose.position.y = -0.31503;
  p.pose.position.z = -0.031482;
  p.pose.orientation.x = 0.038979;
  p.pose.orientation.y = 0.00071756;
  p.pose.orientation.z = -0.026593;
  p.pose.orientation.w = 0.99889;

  //reset position
  // -0.019606; -0.31503; -0.031482// 0.038979; 0.00071756; -0.026593; 0.99889

  //Get the EEF id
  std::string end_effector_link;
  end_effector_link = group.getEndEffectorLink();
  //std::cout << "the endeffector is :::::> " << end_effector_link<< "/n";

  group.setPoseTarget(p, end_effector_link);
  group.setGoalTolerance(0.01);

  bool success;
  int i=0;

  while (success == false && i<10){
    success = group.move();
    i++;
  }

  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(10.0);

  //Once we have moved the arm near the object the next step is to grab it. Note that since our EEF doesnt have any DOF,
  //we cant perform the grasp action. For this reason in this step we only do a simple attach action of the object to the link.
  //pub_aco.publish(aco);

  //group.detachObject("part");
  ROS_INFO("FIN");

  //std::cout<<"FIN DETACHING PLAN";
}
int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::WallDuration(1.0).sleep();

  //We specify the moveit group we want to work with
  moveit::planning_interface::MoveGroup group("right_arm");
  group.setPlanningTime(20.0);

  //Define new variable for the collision objects (co).
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_link";

  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitive_poses.resize(1);

  /*
  // remove pole
  co.id = "pole";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  //co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.x = 0.6;
  //co.primitive_poses[0].position.y = -0.4;
  co.primitive_poses[0].position.y = -0.1;
  co.primitive_poses[0].position.z = 0.85;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);
  */

  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.45;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
  //co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.x = 0.60;
  //co.primitive_poses[0].position.y = -0.2;
  co.primitive_poses[0].position.y = -0.1;
  co.primitive_poses[0].position.z = 0.07;
  pub_co.publish(co);


  //add part (pick&place collision obj)
  co.id = "part";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;
  //co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
  //co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  //co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.15;

  //co.primitive_poses[0].position.x = 0.6;
  co.primitive_poses[0].position.x = 0.42;
  //co.primitive_poses[0].position.y = -0.7;
  co.primitive_poses[0].position.y = -0.4;
  co.primitive_poses[0].position.z = 0.20;
  pub_co.publish(co);


  //add part (attached object)
  moveit_msgs::AttachedCollisionObject aco;
  aco.link_name = "r_wrist_pitch_link";
  aco.object = co;

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  pick(group);

  ros::WallDuration(1.0).sleep();

  place(group);

  ros::WallDuration(1.0).sleep();

  reset(group);

  ros::waitForShutdown();
  return 0;
}
