/* Code for a move_group subtask part of the project "New task generation for humanoid robots based on case and user communication"
 *
 * Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)
 *
 * This node creates a service with the objective to move the "group_action" requested to the "pose" given, using moveit! for this porpouse
 *
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "teo_moveit/move_group_srv.h"

bool move_group_action(teo_moveit::move_group_srv::Request &req, teo_moveit::move_group_srv::Response &res)
{
  //We specify the moveit group we want to work with
  moveit::planning_interface::MoveGroup group(req.group_name);

  //Get the EEF id
  group.setPlanningTime(20.0);
  std::string end_effector_link;
  end_effector_link = group.getEndEffectorLink();
  std::cout << "the endeffector link is ===> " << end_effector_link<< std::endl;

  group.setGoalTolerance(0.01);
  group.setPoseTarget(req.p, end_effector_link);

  bool success;
  int i=0;

  while (success == false && i<10){
    success = group.move();
    i++;
  }

  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(10.0);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_srv");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("move_group_srv", move_group_action);
  ROS_INFO("Ready to move, i think i could have even already move myself");
  ros::spin();

  return 0;
}
