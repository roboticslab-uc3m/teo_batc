/* Code part of the project "New task generation for humanoid robots based on case and user communication"
 *
 * Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)
 *
 * This node is the central node which is in charge of the control of the high level task "drink coffe"
 *
 */

#include <ros/ros.h>
//MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include "teo_moveit/move_group_srv.h"
#include "teo_moveit/pick_srv.h"
#include "teo_moveit/place_srv.h"
#include <cstdlib>
#include "worlds/coffee_table_world.h"


int main(int argc, char **argv)
{
  ros::init (argc, argv, "drink_coffee_control");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Define the nodes we will want to use
  ros::NodeHandle nh;
  //collision object pub
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  //Define de srv we will want to use
  ros::ServiceClient move_group_client = nh.serviceClient<teo_moveit::move_group_srv>("move_group_srv");
  ros::ServiceClient pick_client = nh.serviceClient<teo_moveit::pick_srv>("pick_srv");
  ros::ServiceClient place_client = nh.serviceClient<teo_moveit::place_srv>("place_srv");
  //Define a new srv variable
  teo_moveit::move_group_srv mg_srv;
  teo_moveit::pick_srv pick_srv;
  teo_moveit::place_srv place_srv;


  ros::WallDuration(1.0).sleep();

  //init the environment
  moveit_msgs::CollisionObject co;
  init_world(pub_co, co);

  //Define new variable for the collision objects (co).
  //moveit_msgs::CollisionObject co;
  //co.header.stamp = ros::Time::now();
  //co.header.frame_id = "base_link";

  /*
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitive_poses.resize(1);

  // Start the initialization of the environment
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

  //Cylinder object (cup)
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  co.primitive_poses.resize(1);

  //add cylinder
  co.id = "cup";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  //Shape of the cup
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.15;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.075;

  //Position cup
  co.primitive_poses[0].position.x = 0.44;
  co.primitive_poses[0].position.y = -0.2;
  co.primitive_poses[0].position.z = 0.20;

  //Publish cup
  pub_co.publish(co);

  //attached object cup
  moveit_msgs::AttachedCollisionObject aco;
  aco.link_name = "r_wrist_pitch_link";
  aco.object = co;
  */

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  //Creation of a variable "p" for the position
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
  mg_srv.request.pos_tolerance=0.01;
  mg_srv.request.ang_tolerance=0.1;

  //The position of the manipulator relative to base_link for the cup pick
  //Pos:0.41508; -0.44088; 0.37607
  //Orientation: 0.32493; 0.00081521; -0.00032667; 0.94574
  p.pose.position.x = 0.41508;
  p.pose.position.y = -0.44088;
  p.pose.position.z = 0.37607;
  p.pose.orientation.x = 0.32493;
  p.pose.orientation.y = 0.00081521;
  p.pose.orientation.z = -0.00032667;
  p.pose.orientation.w = 0.94574;

  // Link confugurations for the said pose
  // r_wrist_pitch_link: 0.41449; -0.45821; 0.50566; orientation: 2.3899e-05; 4.042e-05; -0.00013261; 1
  // r_wrist_yaw_link: 0.41449; -0.45821; 0.50566; orientation: -3.0244e-05; -0.39368; -0.00013131; 0.91925
  // r_elbow_pitch_link: 0.26829; -0.45818; 0.64504; orientation: 0.074875; -0.38649; -0.17503; 0.90243

  //we call the service of move_Group
  mg_srv.request.group_name="right_arm";
  mg_srv.request.p=p;
  mg_srv.request.pos_tolerance=0.01;
  mg_srv.request.ang_tolerance=0.1;

  //We call the service, and make sure it worked
  if (move_group_client.call(mg_srv))
  {
    ROS_INFO("It should have move \(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call service (o-o^)");
    return 1;
  }

  //We call the service to pick the object
  pick_srv.request.group_name="right_arm";
  pick_srv.request.pick_object=co;

  //We call the service, and make sure it worked
  if (pick_client.call(pick_srv))
  {
    ROS_INFO("It should have pick the object \(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call service (o-o^)");
    return 1;
  }


  //Move cup to mouth; Param important for this task mouth position.
  //pos: 0.30532; -0.27516; 0.69567
  //ori: 0.32091; 0.0044592; 0.0030924; 0.94709
  p.pose.position.x = 0.30532;
  p.pose.position.y = -0.27516;
  p.pose.position.z = 0.69567;
  p.pose.orientation.x = 0.32091;
  p.pose.orientation.y = 0.0044592;
  p.pose.orientation.z = 0.0030924;
  p.pose.orientation.w = 0.94709;

  mg_srv.request.group_name="right_arm";
  mg_srv.request.p=p;
  mg_srv.request.pos_tolerance=0.01;
  mg_srv.request.ang_tolerance=0.1;

  //We call the service, and make sure it worked
  if (move_group_client.call(mg_srv))
  {
    ROS_INFO("It should have move \(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call service (o-o^)");
    return 1;
  }

  //Drink coffe; Param important for this task mouth position. Note here only the orientation changes, the system has to learn from this
  //pos: 0.29832; -0.27483; 0.6943
  //ori: 0.29961; -0.10914; 0.34385; 0.88323
  p.pose.position.x = 0.30532;
  p.pose.position.y = -0.27516;
  p.pose.position.z = 0.69567;
  p.pose.orientation.x = 0.29961;
  p.pose.orientation.y = -0.10914;
  p.pose.orientation.z = 0.34385;
  p.pose.orientation.w = 0.88323;

  mg_srv.request.group_name="right_arm";
  mg_srv.request.p=p;
  mg_srv.request.pos_tolerance=0.01;
  mg_srv.request.ang_tolerance=0.1;

  //We call the service, and make sure it worked
  if (move_group_client.call(mg_srv))
  {
    ROS_INFO("It should have move \(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call service (o-o^)");
    return 1;
  }

  //Place cup
  //pos: 0.45495; -0.24189; 0.37658
  //ori: 0.33127; 0.02771; 0.076958; 0.93998
  p.pose.position.x = 0.45495;
  p.pose.position.y = -0.24189;
  p.pose.position.z = 0.37658;
  p.pose.orientation.x = 0.33127;
  p.pose.orientation.y = 0.02771;
  p.pose.orientation.z = 0.076958;
  p.pose.orientation.w = 0.93998;

  mg_srv.request.group_name="right_arm";
  mg_srv.request.p=p;
  mg_srv.request.pos_tolerance=0.01;
  mg_srv.request.ang_tolerance=0.1;

  //We call the service, and make sure it worked
  if (move_group_client.call(mg_srv))
  {
    ROS_INFO("It should have move \(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call service (o-o^)");
    return 1;
  }

  //We call the service to pick the object
  place_srv.request.group_name="right_arm";
  place_srv.request.place_object=co;

  //We call the service, and make sure it worked
  if (place_client.call(place_srv))
  {
    ROS_INFO("It should have pick the object \(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call service (o-o^)");
    return 1;
  }

  /*
  //Dettach object
  group.detachObject("cup");
  ROS_INFO("End of detaching");
  */

  //Reset position
  p.pose.position.x = -0.019606;
  p.pose.position.y = -0.31503;
  p.pose.position.z = -0.031482;
  p.pose.orientation.x = 0.038979;
  p.pose.orientation.y = 0.00071756;
  p.pose.orientation.z = -0.026593;
  p.pose.orientation.w = 0.99889;

  mg_srv.request.group_name="right_arm";
  mg_srv.request.p=p;
  mg_srv.request.pos_tolerance=0.01;
  mg_srv.request.ang_tolerance=0.1;

  //We call the service, and make sure it worked
  if (move_group_client.call(mg_srv))
  {
    ROS_INFO("It should have move \\(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call service (o-o^)");
    return 1;
  }

  //move_group(group,p);


  //ros::WallDuration(1.0).sleep();


  //place(group);

  ros::WallDuration(1.0).sleep();

  //reset(group);

  ros::waitForShutdown();
  return 0;
}
