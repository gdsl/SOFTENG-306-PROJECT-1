#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <stdlib.h>
#include "Beacon.h"

Beacon::Beacon() : Entity() {

}

Beacon::Beacon(double x,double y,double theta,double linearVel, double angularVel) : Entity(x, y, theta, linearVel, angularVel,0) {

}

Beacon::~Beacon() {

}

Beacon beacon;

void stage_callback(nav_msgs::Odometry msg) {
	beacon.stageOdom_callback(msg);
}

int main(int argc, char **argv) {    
	// Initialise ros
	ros::init(argc,argv,"beacon");

	// Convert input parameters for beacon initialization from String to respective types
	std::string xString = argv[1];
	std::string yString = argv[2];
	double xPos = atof(xString.c_str());
	double yPos = atof(yString.c_str());

	// Initialize the beacon with the correct position and velocity parameters
	beacon=Beacon(xPos,yPos,0,0,0);

	// Create ros handler for this node
	ros::NodeHandle n;
	beacon.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::Publisher robotNode_location_pub = n.advertise<nav_msgs::Odometry>(argv[3],1000);
	beacon.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
	ros::Rate loop_rate(10);
	nav_msgs::Odometry tempMessage;

	while (ros::ok()) {
		tempMessage.pose.pose.position.x = beacon.getX();
		tempMessage.pose.pose.position.y = beacon.getY();
		robotNode_location_pub.publish(tempMessage);
		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}
