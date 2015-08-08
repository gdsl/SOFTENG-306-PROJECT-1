#include "CarrierRobot.h"
#include <ros/console.h>
#include "std_msgs/String.h"

CarrierRobot::CarrierRobot() {
	// TODO Auto-generated constructor stub

}

CarrierRobot::~CarrierRobot() {
	// TODO Auto-generated destructor stub
}

CarrierRobot carrierRobot;

/*
 * Wrapper method for the callBackStageOdm method
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	carrierRobot.stageOdom_callback(msg);
}

void mypubCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("sub echoing pub: %s",msg->data.c_str());
	carrierRobot.faceSouth(0.4);
}


int main(int argc, char **argv)
{
	carrierRobot=CarrierRobot();
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "CarrierRobot");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;


	ros::Rate loop_rate(10);
	// tell master you want to sub to topic
	carrierRobot.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
		//subscribe to listen to messages coming from stage
	carrierRobot.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	ros::Subscriber mysub_object = n.subscribe("/robot_0/status",1000,mypubCallback);
	//a count of howmany messages we have sent
	int count = 0;


	while (ros::ok())
	{

		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}

	return 0;

}
