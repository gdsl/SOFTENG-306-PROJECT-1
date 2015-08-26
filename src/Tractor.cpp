#include <ros/console.h>
#include "std_msgs/String.h"
#include "Tractor.h"
#include <sstream>

/*
 * Default constructor for carrier Robot
 */
Tractor::Tractor() {

}

/*
 * Constructor for carrier Robot with status
 */
Tractor::Tractor(double x,double y,double theta,double linearVel, double angularVel) : Entity(x, y, theta, linearVel, angularVel,0) {
}

/*
 * Default destructor for carrier Robot
 */
Tractor::~Tractor() {

}

// Create carrier robot and status
Tractor tractor;

/*
 * Wrapper method for the callBackStageOdm method
 */
void callBackStageOdm(const nav_msgs::Odometry msg) {
	tractor.stageOdom_callback(msg);
}

void receiveTractorControl(const std_msgs::String::ConstPtr& msg) {
//	ROS_INFO("tractor echoing tractor control: %s",msg->data.c_str());
	if (strcmp(msg->data.c_str(), "left") == 0) {
		tractor.setLin(0);
		tractor.setAng(3);
	} else if (strcmp(msg->data.c_str(), "right") == 0) {
		tractor.setLin(0);
		tractor.setAng(-3);
	} else if (strcmp(msg->data.c_str(), "up") == 0) {
		tractor.setLin(10);
		tractor.setAng(0);
	} else if (strcmp(msg->data.c_str(), "down") == 0) {
		tractor.setLin(-10);
		tractor.setAng(0);
	} else {
		tractor.setLin(0);
		tractor.setAng(0);
	}
}

int main(int argc, char **argv) {
	// You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "Tractor");
	std::string xString = argv[1];
	// Initialize the Carrier robot with the correct position, velocity and state parameters.
	tractor=Tractor(0,0,0,0,0);

	// NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	std::ostringstream oss;
	oss << "robot_" << xString.c_str() << "/cmd_vel";
	tractor.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>(oss.str().c_str(),1000);

	// Subscribe to listen to messages coming from stage for odometry (this is base pose so it is
	// relative to the absolute frame of the farm.
	tractor.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);


	std::string buffer;
	while(std::cin) {
		std::getline(std::cin, buffer);

		if (strcmp(buffer.c_str(), "left") == 0) {
			// ROS_WARN(buffer.c_str());
			tractor.setLin(0);
			tractor.setAng(0.55);
		} else if (strcmp(buffer.c_str(), "right") == 0) {
			tractor.setLin(0);
			tractor.setAng(-0.5);
		} else if (strcmp(buffer.c_str(), "up") == 0) {
			tractor.setLin(1);
			tractor.setAng(0);
		} else if (strcmp(buffer.c_str(), "down") == 0) {
			tractor.setLin(-1);
			tractor.setAng(0);
		} else {
			tractor.setLin(0);
			tractor.setAng(0);
		}
		tractor.updateOdometry();
		ros::spinOnce();
	}

	// ROS infinite loop
	while (ros::ok())
	{
		ros::spinOnce();
		tractor.updateOdometry();
		loop_rate.sleep();
	}
	return 0;
}

