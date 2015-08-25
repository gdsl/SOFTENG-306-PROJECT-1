#include "ros/ros.h"
#include <sstream>
#include <stdlib.h>
#include "se306project/robot_status.h"
#include "Constants.h"
#include <vector>
#include <string>
#include "Entity.h"
#include "EntityMock.h"

EntityMock::EntityMock():Entity(){
}

EntityMock::EntityMock(double x,double y,double theta,double linearVel, double angularVel)
:Entity( x, y, theta, linearVel,  angularVel){
}

// destructor
EntityMock::~EntityMock() {}

//EntityMock instance
EntityMock entityMock;


/*
 * Wrapper method for the callBackStageOdm method (in Entity)
 */
void callBackStageOdm(const nav_msgs::Odometry msg) {
	entityMock.stageOdom_callback(msg);
}

/**
 * Call back method for laser work out the avoidance logic for picker robot
 */
void callBackLaserScan(const sensor_msgs::LaserScan msg) {
}


int main(int argc, char **argv)
{
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "EntityMock");

	//initialize the EntityMock robot with the correct position, velocity and state parameters.
	entityMock = EntityMock(0,0,0,0,0);

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//robot advertise it node for its velocity message to be published.
	entityMock.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	//robot advertise it node for its status message to be published.
	ros::Publisher pub=n.advertise<se306project::robot_status>("status",1000);

	//subscribe to listen to messages coming from stage about is position relative to absolute frame
	entityMock.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	//subscribe to obstacle detection
	entityMock.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000,callBackLaserScan);

	// initalise robot status message
	se306project::robot_status status_msg;
	unsigned int num_readings = 100;
	double laser_frequency = 40;
	double ranges[num_readings];
	double intensities[num_readings];

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok())
	{
		//assign to status message
		status_msg.my_counter = count++;//add counter to message to broadcast
		status_msg.status=entityMock.getStatus();//add status to message to broadcast
		//status_msg.status ="Full";
		status_msg.pos_x=entityMock.getX(); //add x to message to broadcast
		status_msg.pos_y=entityMock.getY();//add y to message to broadcast
		status_msg.pos_theta=entityMock.getTheta(); //add angle to message to broadcast
		status_msg.obstacle = entityMock.getObstacleStatus();
		pub.publish(status_msg);//publish the message for other node

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
