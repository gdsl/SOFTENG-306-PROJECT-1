/*
 * PickerRobot.cpp
 *
 *  Created on: 6/08/2015
 *      Author: user
 */

#include "PickerRobot.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "PickerRobot.h"
#include "Entity.h"

//PickerRobot::PickerRobot(){
//}

//PickerRobot::~PickerRobot(){
//}
//PickerRobot pickerRobot;


int main(int argc, char **argv)
{
	//pickerRobot=PickerRobot();
	//pickerRobot.Test();
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "PickerRobot");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

	/*
	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);*/

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok())
	{


		loop_rate.sleep();
		++count;
	}

	return 0;

}
