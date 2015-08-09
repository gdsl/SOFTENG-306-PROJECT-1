#include "ros/ros.h"
#include <sstream>
#include "Robot.h"
#include "se306project/robot_status.h"
#include "se306project/carrier_status.h"
#include "PickerRobot.h"

PickerRobot::PickerRobot():Robot(){

}

PickerRobot::~PickerRobot(){
}
PickerRobot pickerRobot;
std::string status="Moving";
double distance=1;

void callBackStageOdm(const nav_msgs::Odometry msg){
	pickerRobot.stageOdom_callback(msg);
}

void recieveCarrierRobotStatus(const se306project::carrier_status::ConstPtr& msg){
	if (msg->status.compare("Arrived")==0){
		if (distance==1){
			distance=5;
		}else if (distance ==5){
			distance=1;
		}
		pickerRobot.setDesireLocation(false);
		status="Moving";
	}
}

void PickerRobot::movement(){
	//robot oscillates
	if(status.compare("Full")!=0){
		if (pickerRobot.getDesireLocation()){
			status="Full";
		}else{
			pickerRobot.moveForward(distance,1);
		}
	}
	//pickerRobot.moveForward(distance,1);

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
	pickerRobot.ro
	ros::Publisher pub=n.advertise<se306project::robot_status>("status",1000);
	//subscribe to listen to messages coming from stage
	pickerRobot.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	//ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);
	ros::Subscriber mysub_object = n.subscribe<se306project::carrier_status>("/robot_2/status",1000,recieveCarrierRobotStatus);

	// initalise robot status message
	se306project::robot_status status_msg;

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok())
	{
		status_msg.my_counter = count++;
		status_msg.status=status;
		status_msg.pos_x=pickerRobot.getX();
		status_msg.pos_y=pickerRobot.getY();
		status_msg.pos_theta=pickerRobot.getAng();
		pub.publish(status_msg);

		ros::spinOnce();
		pickerRobot.faceSouth(0.4);
		pickerRobot.movement();
		loop_rate.sleep();
		//assigne to status message

		++count;
	}

	return 0;

}
