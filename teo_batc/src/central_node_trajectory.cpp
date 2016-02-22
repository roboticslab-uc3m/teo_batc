/* This code part of the project "New task generation for humanoid robots based on case and user communication"
 *
 * Author: Raúl Fernández Fernández (raulfernandezbis@gmail.com)
 *
 * The goal of this node is to be the central node that is in charge to init and control the system.
 *
 */

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include "teo_batc/trajectory_gen_srv.h"
#include "worlds/worlds.h"




int main(int argc, char **argv)
{
  ros::init(argc, argv, "central_node");
  ros::NodeHandle nh;

  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  //Define the srv we will want to use
  ros::ServiceClient trajectory_client = nh.serviceClient<teo_batc::trajectory_gen_srv>("trajectory_gen");

  //Define srv variables
  teo_batc::trajectory_gen_srv trajectory_srv;
  
  ROS_INFO("Waiting for Trajectory Service");
  ros::service::waitForService("trajectory_gen",-1);
  ROS_INFO("Trajectory service on");

  trajectory_srv.request.i.push_back(5);

  //The first thing we do is to do the communication form
  if (trajectory_client.call(trajectory_srv))
  {
    ROS_INFO("It should have move (0u0*)");
  }
  else
  {
    ROS_ERROR("Failed to call service (o-o^)");
    return 1;
  }

  ROS_INFO("All processes should have finished (0u0*)");
  ros::waitForShutdown();

  return 0;
}
