/* Code part of the project "New task generation using old cases and a description of the task"
 *
 * Author: Raúl Fernández Fernández (raulfernandezbis@gmail.com)
 *
 * This a test header file to try to init the worl using arrays of collision object msgs.
 *
 */

#ifndef TEO_COFFEE_TABLE_WORLD
#define TEO_COFFEE_TABLE_WORLD

void init_world(ros::Publisher pub_co,  teo_moveit::co_array& world_co)
{

  int i=0;
  
  world_co->co_array[i].header.stamp = ros::Time::now();
  world_co->co_array[i].header.frame_id = "base_link";

  world_co->co_array[i].primitives.resize(1);
  world_co->co_array[i].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  world_co->co_array[i].primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  world_co->co_array[i].primitive_poses.resize(1);

  // Start the initialization of the environment
  // remove table
  world_co->co_array[i].id = "table";
  world_co->co_array[i].operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(world_co->co_array[i]);

  // add table
  i++;
  world_co->co_array[i].operation = moveit_msgs::CollisionObject::ADD;
  world_co->co_array[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.45;
  world_co->co_array[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
  world_co->co_array[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.1;
  //co.primitive_poses[0].position.x = 0.7;
  world_co->co_array[i].primitive_poses[0].position.x = 0.60;
  //co.primitive_poses[0].position.y = -0.2;
  world_co->co_array[i].primitive_poses[0].position.y = -0.1;
  world_co->co_array[i].primitive_poses[0].position.z = 0.07;
  pub_co.publish(co);

  //Cylinder object (cup)
  world_co->co_array[i].primitives.resize(1);
  world_co->co_array[i].primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  world_co->co_array[i].primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  world_co->co_array[i].primitive_poses.resize(1);

  //add cylinder
  world_co->co_array[i].id = "cup";
  world_co->co_array[i].operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(world_co->co_array[i]);

  //Shape of the cup
  world_co->co_array[i].operation = moveit_msgs::CollisionObject::ADD;
  world_co->co_array[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.15;
  world_co->co_array[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.075;

  //Position cup
  world_co->co_array[i].primitive_poses[0].position.x = 0.44;
  world_co->co_array[i].primitive_poses[0].position.y = -0.2;
  world_co->co_array[i].primitive_poses[0].position.z = 0.20;

  //Publish cup
  pub_co.publish(co);

  //attached object cup
  moveit_msgs::AttachedCollisionObject aco;
  aco.link_name = "r_wrist_pitch_link";
  aco.object = co;
  
  ros::WallDuration(1.0).sleep();
}

#endif
