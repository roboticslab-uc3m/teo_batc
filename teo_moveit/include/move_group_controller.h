/* Code for a move_group subtask part of the project "New task generation using old cases and a description of the task"
 *
 * Author: Raúl Fernández Fernández (raulfernandezbis@gmail.com)
 *
 * This .h file creates a class with a move_group object that can be used between nodes.
 *
 */

//Math
#include <math.h>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;


class move_group_controller {
public:

	ros::NodeHandle nh_;
	std::string planning_group_name;
	double POS_TOLARENCE, ANG_TOLARENCE, PLANING_TIME;
	// interface with MoveIt
	boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

	move_group_controller();
	virtual ~move_group_controller();
        
        //We define the fuctions (node) that will use this object.
	bool move_group_action(teo_moveit::move_group_srv::Request &req, teo_moveit::move_group_srv::Response &res);

	bool pick_action(teo_moveit::pick_srv::Request &req, teo_moveit::pick_srv::Response &res);

	//geometry_msgs::Pose moveToInitialPos();
/*
private:
	struct posioint_correction{
		double x,y,z;
	};

	string eef, base_reference;
	bool collisionBoxesPresent;
	geometry_msgs::Pose current_pos, target_pos;
	moveit::planning_interface::MoveGroup::Plan move_plan;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	std::vector<std::string> collisionCB_id;

	tf::TransformListener chessBoard_listener;
	tf::StampedTransform transform_cameraToGrid;

	bool greaterPositonDifference(tf::StampedTransform cbPose, double diffX, double diffY, double diffZ);
	std::vector<moveit_msgs::CollisionObject> moveCollisionObjChessboard();

	bool moveToPos(geometry_msgs::Pose target);
	geometry_msgs::Pose rotateBase(double degree);
	posioint_correction getTargetPoseFromChessBoard(
			geometry_msgs::Pose &current_pose, tf::StampedTransform chessB_pose);
	std::vector<moveit_msgs::CollisionObject> creatCollisionObjChessboard(
			tf::StampedTransform cameraGrid, bool cheat);
	geometry_msgs::Pose orientateArmToCB(geometry_msgs::Pose CB_pose, double yawError);
*/

};

