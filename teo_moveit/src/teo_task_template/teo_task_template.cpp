/* Code for a move_group subtask part of the project "New task generation using old cases and a description of the task"
 *
 * Author: Raúl Fernández Fernández
 *
 * The main goal of this code is to be a template for the automatic task generation of new tasks. The way this code works
 * is quite simple it just calls some service depending of some params that the ROS node send to it.
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
#include "std_msgs/String.h"
#include "worlds/worlds.h"
#include "teo_moveit/param.h"
#include "teo_moveit/param_array.h"
#include <teo_moveit/teo_task_template_srv.h>

bool teo_task_template(teo_moveit::teo_task_template_srv::Request  &req, teo_moveit::teo_task_template_srv::Response &res)
{
  ros::NodeHandle nh;
  //Define world
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  //Define de srv we will want to use
  //task_services(){
  ros::ServiceClient move_group_client = nh.serviceClient<teo_moveit::move_group_srv>("move_group_srv");
  ros::ServiceClient pick_client = nh.serviceClient<teo_moveit::pick_srv>("pick_srv");
  ros::ServiceClient place_client = nh.serviceClient<teo_moveit::place_srv>("place_srv");
  //srv variables
  teo_moveit::move_group_srv mg_srv;
  teo_moveit::pick_srv pick_srv;
  teo_moveit::place_srv place_srv;

  //init the environment, we can modify init_world in worlds.h to init different environments from a parameter.
  moveit_msgs::CollisionObject co;
  init_world(pub_co, co, req.param_array[0].world);

  ros::WallDuration(10.0).sleep();

  //We define a loop with the number of servers we want to execute
  int i=0;
  ROS_INFO("Yeah received something thing!!");
  while(i<req.param_array.size()){

      //A "0" in an array means we want to do a move group task
      if(req.param_array[i].task==0){
      ROS_INFO("Yeah i know its a movegroup TASK!");
      //we call services for move_group
      mg_srv.request.group_name=req.param_array[i].group_name;
      mg_srv.request.p=req.param_array[i].p;
      //mg_srv.request.comm.pos_tolerance=0.01;
      //mg_srv.request.comm.ang_tolerance=0.1;
      mg_srv.request.pos_tolerance= req.param_array[i].pos_tolerance;
      mg_srv.request.ang_tolerance= req.param_array[i].ang_tolerance;
      //We call the service, and make sure it worked
      if (move_group_client.call(mg_srv))
        {
            ROS_INFO("It should have move \(0u0)/");
        }
            else
        {
            ROS_ERROR("Failed to call service (o-o^)");
            break;
        }
      }

      //A "1" in an array means we want to do a pick task
      if(req.param_array[i].task==1){
        //We call the service to pick the object
        pick_srv.request.group_name=req.param_array[i].group_name;
        pick_srv.request.pick_object=co;

        //We call the service, and make sure it worked
        if (pick_client.call(pick_srv))
        {
          ROS_INFO("It should have pick the object \(0u0)/");
        }
        else
        {
          ROS_ERROR("Failed to call service (o-o^)");
          break;
        }
      }

      //A "2" in an array means we want to do a place task
      if(req.param_array[i].task==2){
        //We call the service to pick the object
        place_srv.request.group_name=req.param_array[i].group_name;
        place_srv.request.place_object=co;

        //We call the service, and make sure it worked
        if (place_client.call(place_srv))
        {
          ROS_INFO("It should have pick the object \(0u0)/");
        }
        else
        {
          ROS_ERROR("Failed to call service (o-o^)");
          break;
        }
      }

    i++;
    ros::WallDuration(10.0).sleep();
  }
  return true;

}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "teo_task_template");

  //Define the nodes we will want to use
  ros::NodeHandle nh;
  //collision object pub
  //ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  //CARE FROM THIS POINT THERE COULD BE SOME PSEUDOCODE SYNTAXIS. ALFA PROGRAMMING. ALWAYS YOURS RAUL

  //Once we have finnished the communication and we have generated a base solution, this base solution is send here
  //to generate a new task. Hence what we receive here is the structure new task+params+atomic tasks), an array msg of msgs.
  //TODO: Subscribe to communication and make that all of this params can be modify but the dynamic_reconfigure package or other node.
  //right now im thinking that maybe this node is only the subscriber that will be other node (adaptation or communication one)
  //the one in charge to get that info from communication or adaptation.
  ros::ServiceServer service = nh.advertiseService("teo_task_template", teo_task_template);

  ros::spin();
  return 0;
}
