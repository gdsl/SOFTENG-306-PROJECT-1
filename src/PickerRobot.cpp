#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include "Robot.h"
#include "PickerRobot.h"
PickerRobot::PickerRobot():Robot(){

}

PickerRobot::~PickerRobot(){
}
PickerRobot pickerRobot;

void callBackStageOdm(const nav_msgs::Odometry msg){
	pickerRobot.stageOdom_callback(msg);
}

void PickerRobot::movement(){
	if (std::abs(pickerRobot.theta-0.5)<0.1){
		if(std::abs(pickerRobot.y+10.0)>0.1){
			pickerRobot.moveForward(1);
		}else{
			pickerRobot.moveForward(0);
		}
	}
}

int main(int argc, char **argv)
{
	pickerRobot=PickerRobot();
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "PickerRobot");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	pickerRobot.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

	//subscribe to listen to messages coming from stage
	pickerRobot.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("odom",1000, callBackStageOdm);
	//ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok())
	{
		pickerRobot.faceSouth(1);
		pickerRobot.movement();
		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}

	return 0;

}
