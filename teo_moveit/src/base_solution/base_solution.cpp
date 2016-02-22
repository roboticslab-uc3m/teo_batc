/* Code for a move_group subtask part of the project "New task generation using old cases and a description of the task"
 *
 * Author: Raúl Fernández Fernández (raulfernandezbis@gmail.com)
 *
 * The goal of this node is to generate the base solution for the CBR system, this service will use the comm info and the BDD info
 * to generate a base solution. NOTE: Its not the same as the adaptation node, though the adapt. node, also generate sol.
 * SERVICE: base_solution; I: ---; O: param_array; Depends: teo_moveit/communication node.
 *
 */
#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <string>
#include <teo_moveit/communication_srv.h>
#include <teo_moveit/base_solution_srv.h>
#include <teo_moveit/param.h>



bool base_solution(teo_moveit::base_solution_srv::Request  &req, teo_moveit::base_solution_srv::Response &res){
    //Definde nh
    ros::NodeHandle nh;

    //Define de srv we will want to use to get the info from the communication
    ros::ServiceClient communication_client = nh.serviceClient<teo_moveit::communication_srv>("communication");
    //Define a new srv variable
    teo_moveit::communication_srv communication_srv;

    //PART1: BEGIN THE COMMUNICATION TO GET ALL THE NEEDED INFO

    //The first thing we do is to do the communication form
    if (communication_client.call(communication_srv))
    {
      ROS_INFO("It should have move (0u0*)");
    }
    else
    {
      ROS_ERROR("Failed to call service (o-o^)");
      return 1;
    }


    //first we define some predefined positions
    //TODO: Define some predefined position, like "close_object"... to load when the communication calls for it. If its possible do this in an .h file.
    geometry_msgs::PoseStamped p_taza;
    p_taza.header.frame_id = "base_link";
    p_taza.pose.position.x = 0.41508;
    p_taza.pose.position.y = -0.44088;
    p_taza.pose.position.z = 0.37607;
    p_taza.pose.orientation.x = 0.32493;
    p_taza.pose.orientation.y = 0.00081521;
    p_taza.pose.orientation.z = -0.00032667;
    p_taza.pose.orientation.w = 0.94574;

    geometry_msgs::PoseStamped p_boca;
    p_boca.header.frame_id = "base_link";
    p_boca.pose.position.x = 0.30532;
    p_boca.pose.position.y = -0.27516;
    p_boca.pose.position.z = 0.69567;
    p_boca.pose.orientation.x = 0.32091;
    p_boca.pose.orientation.y = 0.0044592;
    p_boca.pose.orientation.z = 0.0030924;
    p_boca.pose.orientation.w = 0.94709;


    //TODO: Be able to init the world in every of the atomic task.
    int i=0;
    teo_moveit::param data;
    //teo_moveit::param_array msg;
    data.world=communication_srv.response.entorno[i];
    while (i<(communication_srv.response.atomic_task.size()-1)){

        //we define the first sub task
        data.task=communication_srv.response.atomic_task[i]; //Tarea 0, move_group
        data.group_name=communication_srv.response.move_group[i];
        if(communication_srv.response.pose[i]=="close_cup")
            data.p=p_taza;
        if(communication_srv.response.pose[i]=="close_mouth")
            data.p=p_boca;
        data.pos_tolerance=0.01;
        data.ang_tolerance=0.1;
        res.param_array.push_back(data);

        ++i;
  }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "base_solution_generator");
    ros::NodeHandle nh;
    //Advertise the service we want to use.
    ros::ServiceServer service = nh.advertiseService("base_solution", base_solution);

    ros::spin();

    return 0;

}
