#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
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
	//robot oscillates
	pickerRobot.moveForward(1,1);
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
	ros::Publisher pub=n.advertise<std_msgs::String>("status",1000);
	//subscribe to listen to messages coming from stage
	pickerRobot.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	//ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);

	// refer to advertise msg type
	std_msgs::String status_msg;

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok())
	{
		pickerRobot.faceSouth(0.4);
		pickerRobot.movement();
		ros::spinOnce();
		loop_rate.sleep();
		status_msg.data = "test";

		pub.publish(status_msg);
		++count;
	}

	return 0;

}
