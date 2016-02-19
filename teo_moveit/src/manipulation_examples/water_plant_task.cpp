/* Code part of the project "New task generation for humanoid robots based on case and user communication"
 *
 * Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)
 *
 * This node is the central node which is in charge of the control of the high level task "water plant"
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
#include "worlds/water_plant_world.h"


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

  //init the environment, and return object to manipulate
  //TODO: BE ABLE TO INIT THE ENVIRONMENT AND PASS A CO OBJECT TO MAIN FROM A HEADER FILE
  moveit_msgs::CollisionObject co;
  init_world(pub_co, co);

  /************************************************************ Environment setup******************************************************************************
  //if (ros::isInitialize()==false)
    //	ROS_ERROR("ROS HAS NOT BEEN INITIALIZED BEFORE THE CALL OF THIS FUNCTION (o-o^)");

    //ros::NodeHandle nh;
    //collision object pub
    //ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

    //Define new variable for the table
    moveit_msgs::CollisionObject co;

    co.header.stamp = ros::Time::now();
    co.header.frame_id = "base_link";

    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitive_poses.resize(1);

    //Start the initialization of the environment
    //remove table
    co.id = "table";
    //First we remove the old table
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    //add table
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.45;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
    //co.primitive_poses[0].position.x = 0.7;
    co.primitive_poses[0].position.x = 0.60;
    //co.primitive_poses[0].position.y = -0.2;
    co.primitive_poses[0].position.y = -0.1;
    co.primitive_poses[0].position.z = 0.07;

    // add table
    co.operation = moveit_msgs::CollisionObject::ADD;
    pub_co.publish(co);


    //****************************************************************************************************

    //Define new variable for the water_can
    //moveit_msgs::CollisionObject co;

    co.header.stamp = ros::Time::now();
    co.header.frame_id = "base_link";

    //Cylinder object
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
    co.primitive_poses.resize(1);

    //add cylinder
    co.id = "water_can";
    //First we remove
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);


    //Shape of the water can
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.26;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.12;

    //Position cup
    co.primitive_poses[0].position.x = 0.54;
    co.primitive_poses[0].position.y = -0.5;
    co.primitive_poses[0].position.z = 0.26;

    //Then we add
    co.operation = moveit_msgs::CollisionObject::ADD;
    pub_co.publish(co);

    //**********************************************************************************************************************************

    //Define new variable for the Pot
    //moveit_msgs::CollisionObject co;

    co.header.stamp = ros::Time::now();
    co.header.frame_id = "base_link";

    //add cylinder (pot)
    co.id = "plant_pot";
    //First we remove
    co.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co.publish(co);

    //Shape of the water can
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.80;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.16;

    //Position cup
    co.primitive_poses[0].position.x = 0.1;
    co.primitive_poses[0].position.y = -0.65;
    co.primitive_poses[0].position.z = -0.46;

    //Then we add
    co.operation = moveit_msgs::CollisionObject::ADD;
    pub_co.publish(co);

    //attached object cup
    //moveit_msgs::AttachedCollisionObject aco;
    //aco.link_name = "r_wrist_pitch_link";
    //aco.object = co;

    ros::WallDuration(1.0).sleep();
  /************************************************************ Environment setup END******************************************************************************/

  //Define new variable for the collision objects (co).
  //moveit_msgs::CollisionObject co;
  //co.header.stamp = ros::Time::now();
  //co.header.frame_id = "base_link";

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  //Creation of a variable "p" for the position
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";

  //The position of the manipulator relative to base_link for the water can pick is
  //Pos: 0.19392; -0.53167; 0.22629
  //Orientation: -0.47883; -0.5658; 0.43283; 0.51307
  p.pose.position.x = 0.19392;
  p.pose.position.y = -0.53167;
  p.pose.position.z = 0.22629;
  p.pose.orientation.x = -0.47883;
  p.pose.orientation.y = -0.5658;
  p.pose.orientation.z = 0.43283;
  p.pose.orientation.w = 0.51307;

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


  //Move water_can to pot position; Param important for this task pot position.
  //pos: 0.13308; -0.58622; 0.2414
  //ori: -0.71151; -0.10409; -0.11445; 0.68543
  p.pose.position.x =0.13308;
  p.pose.position.y = -0.58622;
  p.pose.position.z = 0.2414;
  p.pose.orientation.x = -0.71151;
  p.pose.orientation.y = -0.10409;
  p.pose.orientation.z = -0.11445;
  p.pose.orientation.w = 0.68543;

  mg_srv.request.group_name="right_arm";
  mg_srv.request.p=p;
  mg_srv.request.pos_tolerance=0.15;
  mg_srv.request.ang_tolerance=0.01;

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

  //water plant: Param important for this task pot position. Note here only the orientation changes, the system has to learn from this
  //pos: 0.072208; -0.68184; 0.27764
  //ori: -0.54253; -0.12736; -0.094337; 0.82495
  //modified y.orentation rto water the plan and .w to have the unity quaternion
  p.pose.position.x = 0.072208;
  p.pose.position.y = -0.68184;
  p.pose.position.z = 0.27764;
  p.pose.orientation.x = -0.54253;
  p.pose.orientation.y = -0.12736;
  p.pose.orientation.z = -0.094337;
  p.pose.orientation.w = 0.82495;

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

  //w8 to water the plant
  ros::WallDuration(5.0).sleep();

  //Place water can. Init pose
  p.pose.position.x = 0.19392;
  p.pose.position.y = -0.53167;
  p.pose.position.z = 0.20629;
  p.pose.orientation.x = -0.47883;
  p.pose.orientation.y = -0.5658;
  p.pose.orientation.z = 0.43283;
  p.pose.orientation.w = 0.51307;

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
    ROS_INFO("It should have place the object \(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call service (o-o^)");
    return 1;
  }

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

  //place(group );

  ros::WallDuration(1.0).sleep();

  //reset(group);

  ros::waitForShutdown();
  return 0;
}
