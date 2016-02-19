/* Code part of the project "New task generation for humanoid robots based on case and user communication"
 *
 * Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)
 *
 * The goal of this node is to be the central node that is in charge to init the system, receives the info from the communication also right now will be in charge
 * of the generation of the base solution to send to the task generation node
 *
 */

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include "teo_moveit/communication_srv.h"
#include "teo_moveit/param.h"
#include "teo_moveit/param_array.h"
#include <teo_moveit/base_solution_srv.h>
#include <teo_moveit/teo_task_template_srv.h>
#include "worlds/worlds.h"




int main(int argc, char **argv)
{
  ros::init(argc, argv, "central_node");
  ros::NodeHandle nh;

  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  //Define the srv we will want to use
  ros::ServiceClient base_solution_client = nh.serviceClient<teo_moveit::base_solution_srv>("base_solution");
  ros::ServiceClient teo_task_template_client = nh.serviceClient<teo_moveit::teo_task_template_srv>("teo_task_template");

  //Define srv variables
  teo_moveit::base_solution_srv base_solution;
  teo_moveit::teo_task_template_srv teo_task_template;

  //The first thing we do is to do the communication form
  //Call service


  if (base_solution_client.call(base_solution))
  {
    ROS_INFO("It should have move \(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call base_solution service (o-o^)");
    return 1;
  }

  teo_task_template.request.param_array=base_solution.response.param_array;

  //init the environment, we can modify init_world to init different environments from a parameter.
  moveit_msgs::CollisionObject co;
  init_world(pub_co, co, base_solution.response.param_array[0].world);

  ros::WallDuration(10.0).sleep();

  //Now we have a base_solution (param_array), so we will generate a new solution task.
  if (teo_task_template_client.call(teo_task_template))
  {
    ROS_INFO("It should have move \(0u0)/");
  }
  else
  {
    ROS_ERROR("Failed to call teo_task_template service (o-o^)");
    return 1;
  }

  ROS_INFO("All processes should have finished \(0u0)/");
  //We call the service base_solution

  //We call the service to generate a new task.


  ros::waitForShutdown();

  return 0;
}
