/* Code part of the project "New task generation using old cases and a description of the task"
 *
 * Author: Raúl Fernández Fernández (raulfernandezbis@gmail.com)
 *
 * This a header file for the initialization of a custom environment consisted in a table and a coffee cup.
 *
 */

#ifndef TEO_COFFEE_TABLE_WORLD
#define TEO_COFFEE_TABLE_WORLD

void init_world(ros::Publisher pub_co,  moveit_msgs::CollisionObject& co)
{

  
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_link";

  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
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
  co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
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
  
  ros::WallDuration(1.0).sleep();
}

#endif
