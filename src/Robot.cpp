#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "Robot.h"

/**
 * Default constructor of Robot. Calls the other constructor with default values.
 */

Robot::Robot():Robot(0, 0, 0.0, 0.0, 0.0){}

/**
 * Initialises Robot object with given parameters.
 */

Robot::Robot(int x, int y, double theta, double linearVelocity, double angularVelocity)
{
	// initialise variables
	this->x = x;
	this->y = y;
	this->theta = theta;
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
}

/**
 * Update the position of the robot
 */

void Robot::setPose(int x, int y, double theta)
{
	this->x = x;
	this->y = y;
	this->theta = theta;
}

/**
 * Update the velocity of robot
 */

void Robot::setVelocity(double linearVelocity, double angularVelocity)
{
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
}

/**
 * This is the call back function to process odometry messages coming from Stage.
 */

void Robot::stageOdom_callback(nav_msgs::Odometry msg)
{
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;

	// ROS logging api
	ROS_INFO("Current x position is: %f", x);
	ROS_INFO("Current y position is: %f", y);
}

/**
 * Message to stage of robot's odometry
 */

void Robot::updateOdometry()
{
	robotNode_cmdvel.linear.x = linearVelocity;
	robotNode_cmdvel.angular.z = angularVelocity;

	// publish message
	robotNode_stage_pub.publish(robotNode_cmdvel);
}

// movement towards a point
void Robot::moveTo(geometry_msgs::Point point){}
