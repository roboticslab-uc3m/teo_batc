#include <ros/ros.h>

//MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

//Load the robot description
static const std::string ROBOT_DESCRIPTION="robot_description";

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

  //pick(group);

  ros::WallDuration(1.0).sleep();

  //place(group);

  ros::waitForShutdown();
  return 0;
}
