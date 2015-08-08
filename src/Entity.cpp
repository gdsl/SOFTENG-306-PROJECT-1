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
	robotNode_cmdvel.linear.y = 0;
	// publish message
	robotNode_stage_pub.publish(robotNode_cmdvel);
}

/**
 * Message to move the robot forward in the direction it is facing
 * Note unit is in meters
 * input:	double vel: the velocity of the robot moving forward
 */
void Entity::moveForward(double vel){}

/**
 * Message to rotate the robot.
 * Input:	double angleToRotate: the angle robot will rotate to relative to absoulte frame.
 *			double angleSpeed: how fast we want the robot to rotate. Note its speed so always +ve
 */
void Entity::rotate(double angleToRotateTo, double angleSpeed){
	if (angleToRotateTo!=theta){
		if (theta>angleToRotateTo){
			angularVelocity=-angleSpeed;
			updateOdometry();
		}else{
			angularVelocity=angleSpeed;
			updateOdometry();
		}
	}else{
		angularVelocity=0;
		linearVelocity=0;
		updateOdometry();
	}
}

/**
 * Message to rotate the robot such that it faces North
 */
void Entity::faceNorth(double angleSpeed){
	rotate(0, angleSpeed);
}

/**
 * Message to rotate the robot such that it faces South
 */
void Entity::faceSouth(double angleSpeed){}

/**
 * Message to rotate the robot such that it faces East
 */
void Entity::faceEast(double angleSpeed){}

/**
 * Message to rotate the robot such that it faces West
 */
void Entity::faceWest(double angleSpeed){}
