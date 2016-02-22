/* Code for a move_group subtask part of the project "New task generation using old cases and a description of the task"
 *
 * Author: Raúl Fernández Fernández (raulfernandezbis@gmail.com)
 *
 * This is just a publisher tester.
 *
 */

#include <ros/ros.h>
//MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include "teo_moveit/param.h"
#include "teo_moveit/param_array.h"

int main(int argc, char **argv)
{
  ros::init (argc, argv, "publisher_test");

  //Define the nodes we will want to use
  ros::NodeHandle nh;
  ros::Publisher pub_test = nh.advertise<teo_moveit::param_array>("test_topic", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
       while (ros::ok())
       {
          teo_moveit::param data;
          teo_moveit::param_array msg;

          //we define the first sub task
          data.task=0; //Tarea 0, move_group
          data.group_name="right_arm";
          data.p.header.frame_id = "base_link";
          data.p.pose.position.x = 0.41508;
          data.p.pose.position.y = -0.44088;
          data.p.pose.position.z = 0.37607;
          data.p.pose.orientation.x = 0.32493;
          data.p.pose.orientation.y = 0.00081521;
          data.p.pose.orientation.z = -0.00032667;
          data.p.pose.orientation.w = 0.94574;
          data.pos_tolerance=0.01;
          data.ang_tolerance=0.1;
          msg.param_array.push_back(data);

          pub_test.publish(msg);

          ros::spinOnce();

             loop_rate.sleep();
             ++count;
      }

      return 0;

}
