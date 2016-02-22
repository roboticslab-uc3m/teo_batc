/* Code part of the project "New task generation for humanoid robots based on case and user communication"
 *
 * Author: Raúl Fernández Fernández(raulfernandezbis@gmail.com)
 *
 * This node is the node where all the atomic subtasks are saved. This is like the atomic memory of the robot. Every atomic task
 * means a service here that can be called from other nodes.
 *
 */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "teo_moveit/move_group_srv.h"
#include "teo_moveit/pick_srv.h"
#include "teo_moveit/place_srv.h"

class move_group_control {
public:
    ros::NodeHandle nh_;
    std::string planning_group_name,PLANNER_ID;
    double POS_TOLARENCE, ANG_TOLARENCE, PLANING_TIME;
    // interface with MoveIt
    boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;
    move_group_control() :
        nh_("~"),
        planning_group_name("right_arm"),
        POS_TOLARENCE(0.01),
        ANG_TOLARENCE(0.1),
        PLANING_TIME(20.0)
        //PLANNER_ID("RRTConnectkConfigDefault")
    {
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name));
    move_group_->setPlanningTime(PLANING_TIME);
    move_group_->setGoalOrientationTolerance(ANG_TOLARENCE);
    move_group_->setGoalPositionTolerance(POS_TOLARENCE);
    //move_group_->setPlannerId(PLANNER_ID);
    move_group_->setStartStateToCurrentState();
    sleep(5);
   }
   bool move_group_action(teo_moveit::move_group_srv::Request &req, teo_moveit::move_group_srv::Response &res);
   bool pick_action(teo_moveit::pick_srv::Request &req, teo_moveit::pick_srv::Response &res);
   bool place_action(teo_moveit::place_srv::Request &req, teo_moveit::place_srv::Response &res);
};

bool move_group_control::move_group_action(teo_moveit::move_group_srv::Request &req, teo_moveit::move_group_srv::Response &res)
{
  //We specify the moveit group we want to work with, using the class created for move_group
  if (planning_group_name != req.group_name){
    planning_group_name= req.group_name;
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name));
  }
  //We specify that we want to plan from the current position
  move_group_->setStartStateToCurrentState();
  move_group_->setGoalOrientationTolerance(req.ang_tolerance);
  move_group_->setGoalPositionTolerance(req.pos_tolerance);

  //moveit::planning_interface::MoveGroup group(req.group_name);

  //Get the EEF id
  //group.setPlanningTime(20.0);
  std::string end_effector_link;
  end_effector_link = move_group_->getEndEffectorLink();
  std::cout << "the endeffector link is ===> " << end_effector_link<< std::endl;

  //group.setGoalTolerance(0.01);
  move_group_->setPoseTarget(req.p, end_effector_link);

  bool success;
  int i=0;

  while (success == false && i<10){
  success = move_group_->move();
    i++;
  }

  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");

  return true;
}

bool move_group_control::pick_action(teo_moveit::pick_srv::Request &req, teo_moveit::pick_srv::Response &res)
{
  //We specify the moveit group we want to work with
  //moveit::planning_interface::MoveGroup group(req.group_name);
  //We specify the moveit group we want to work with, using the class created for move_group
  if (planning_group_name != req.group_name){
    planning_group_name= req.group_name;
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name));
  }

  move_group_->attachObject(req.pick_object.id);
  ROS_INFO("End of attaching");

  return true;
}

bool move_group_control::place_action(teo_moveit::place_srv::Request &req, teo_moveit::place_srv::Response &res)
{
  //We specify the moveit group we want to work with
  //moveit::planning_interface::MoveGroup group(req.group_name);
  //We specify the moveit group we want to work with, using the class created for move_group
  if (planning_group_name != req.group_name){
    planning_group_name= req.group_name;
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name));
  }

  move_group_->detachObject(req.place_object.id);
  ROS_INFO("End of detaching");

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "atomic_task_memory");
  ros::NodeHandle nh;

  move_group_control move_group;
  ros::ServiceServer move_group_service = nh.advertiseService("move_group_srv", &move_group_control::move_group_action, &move_group);
  ros::ServiceServer pick_service = nh.advertiseService("pick_srv", &move_group_control::pick_action, &move_group);
  ros::ServiceServer place_service = nh.advertiseService("place_srv", &move_group_control::place_action, &move_group);
  //ROS_INFO("I think i have just remember a few things that can be usefull, Im ready senpai");
  ROS_INFO("I think i have just remember a few things that can be usefull, Im ready");

  //moveit needs to have a async spinner with multiple threads for some of its fuctions so:
  ros::AsyncSpinner spinner(4); // Use 4 threads, at most 4 task can be executed paralell.
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
