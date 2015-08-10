#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <sstream>
#include "Entity.h"
#include <cmath>
#include <tf/tf.h>

/**
 * Default constructor of Entity. Calls the other constructor with default values.
 */

Entity::Entity():Entity(0,0,0,0.01,0.01){

}

/**
 * Default destructor
 */
Entity::~Entity(){

}

/**
 * Constructor with parameters
 */
Entity::Entity(double x, double y, double theta, double linearVelocity, double angularVelocity){
	// initialise variables
	this->x=x;
	this->y=y;
	this->theta = theta;
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
	desireLocation=false;
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
	
    tf::Pose pose;
    tf::poseMsgToTF(msg.pose.pose,pose);

    theta = tf::getYaw(pose.getRotation());    
	// ROS logging api
	//ROS_INFO("Current x position is: %f", x);
	//ROS_INFO("Current y position is: %f", y);
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
 * Message to move the entity forward in the direction it is facing
 * Note unit is in meters
 * input:	double vel: the velocity of the entity moving forward
 *			double distance: the amount of distance to move
 */
void Entity::moveForward(double distance, double vel){
	if (!desireLocation){
		if(x-distance>0.01){
			linearVelocity=-vel;
		}else if(distance-x>0.01){
			linearVelocity=vel;
		} else{
			desireLocation=true;
			linearVelocity=0;
		}
		angularVelocity=0;
		updateOdometry(); //update the information to stage
	}
}

/**
 * Message to rotate the entity.
 * Input:	double angleToRotate: In radians, the angle entity will rotate to relative to absoulte frame.
 *			double angleSpeed: how fast we want the entity to rotate. Note its speed so always +ve
 */
void Entity::rotate(double angleToRotateTo, double angleSpeed){
	//Check if angleToRotateTo and the current angle is similar. If not rotate.
	if (std::abs(angleToRotateTo-theta)>0.0001){
		if (std::abs(angleToRotateTo-theta)<(0.01)){//slow down speed when very near
				//ROS_INFO(""+(angleToRotateTo-theta));
				angularVelocity=0.001*angleSpeed;
				updateOdometry();
		}else if (std::abs(angleToRotateTo-theta)<(0.1)){//slow down speed when near
			//ROS_INFO(""+(angleToRotateTo-theta));
			angularVelocity=0.1*angleSpeed;
			updateOdometry();
		}else{
			angularVelocity=angleSpeed;
		}
		updateOdometry();
	}else{
		//if angle similar stop rotating
		angularVelocity=0;
		linearVelocity=0;
		updateOdometry(); //update the information to stage
	}
}

/**
 * Message to rotate the entity such that it faces North
 */
void Entity::faceNorth(double angleSpeed){
	rotate(-M_PI/2, angleSpeed);
}

/**
 * Message to rotate the entity such that it faces South
 */
void Entity::faceSouth(double angleSpeed){
	rotate(M_PI/2, angleSpeed);
}

/**
 * Message to rotate the entity such that it faces East
 */
void Entity::faceEast(double angleSpeed){
	rotate(M_PI, angleSpeed);
}

/**
 * Message to rotate the entity such that it faces West
 */
void Entity::faceWest(double angleSpeed){
	rotate(0,angleSpeed);
}

/**
 * Getter method for x position of entity
 */
double Entity::getX() {
    return x;
}

/**
 * Getter method for y position of entity
 */
double Entity::getY() {
    return y;
}

/**
 * Getter method for angle of entity
 */
double Entity::getTheta() {
    return theta;
}

/**
 * Getter method for linear velocity of entity
 */
double Entity::getLin() {
    return linearVelocity;
}

/**
 * Getter method for angular velocity of entity
 */
double Entity::getAng() {
    return angularVelocity;
}

/**
 * Getter method for desire location of entity
 */
bool Entity::getDesireLocation() {
    return desireLocation;
}

/**
 * setter method for desire location of entity
 */
void Entity::setDesireLocation(bool desireLocation){
	this->desireLocation=desireLocation;
}
