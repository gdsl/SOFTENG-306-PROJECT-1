#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "Entity.h"

/**
 * Default constructor of Entity. Calls the other constructor with default values.
 */

Entity::Entity():Entity(0,0,0.0,0.0,0.0){

}

/**
 * Default destructor
 */
Entity::~Entity(){

}

/**
 * Initialises Entity object with given parameters.
 */

Entity::Entity(int x, int y, double theta, double linearVelocity, double angularVelocity)
{
	// initialise variables
	this->x = x;
	this->y = y;
	this->theta = theta;
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
	destination_x=x;
	destination_y=y;
	destination_theta=theta;
	destination_linearVelocity=linearVelocity;
	destination_angularVelocity=angularVelocity;
}

/**
 * Update the position of the Entity
 */

void Entity::setPose(int x, int y, double theta)
{
	this->x = x;
	this->y = y;
	this->theta = theta;
}

/**
 * Update the velocity of Entity
 */

void Entity::setVelocity(double linearVelocity, double angularVelocity)
{
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
}

/**
 * This is the call back function to process odometry messages coming from Stage.
 */

void Entity::stageOdom_callback(nav_msgs::Odometry msg)
{
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;

	// ROS logging api
	//ROS_INFO("Current x position is: %f", x);
	//ROS_INFO("Current y position is: %f", y);
}

void Entity::StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number

}

/**
 * Message to stage of Entity's odometry
 */

void Entity::updateOdometry()
{
	robotNode_cmdvel.linear.x = linearVelocity;
	robotNode_cmdvel.angular.z = angularVelocity;

	// publish message
	robotNode_stage_pub.publish(robotNode_cmdvel);
}

/**
 * Message to move the robot forward in the direction it is facing
 * Note unit is in meters
 */
void Entity::moveForward(int distanceToMove){}

/**
 * Message to rotate the robot.
 */
void Entity::rotate(double angleToRotate){

}

/**
 * Message to rotate the robot such that it faces North
 */
void Entity::faceNorth(){}

/**
 * Message to rotate the robot such that it faces South
 */
void Entity::faceSouth(){}

/**
 * Message to rotate the robot such that it faces East
 */
void Entity::faceEast(){}

/**
 * Message to rotate the robot such that it faces West
 */
void Entity::faceWest(){}
