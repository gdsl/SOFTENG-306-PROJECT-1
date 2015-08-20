#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <sstream>
#include "Entity.h"
#include <cmath>
#include <tf/tf.h>
#include <vector>
#include <string>
#include "Movement.h"
#include <sensor_msgs/LaserScan.h>

/**
 * Default constructor of Entity. Calls the other constructor with default values.
 */

Entity::Entity():Entity(0,0,0,0,0){

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
	//the distance of the nearest obstacle
	this->minDistance=30.0;
	//set the default obstacle angle as a value larger than 180
	this->obstacleAngle=270;
	this->criticalIntensity=0;
	this->numOfScan=0; //variable for the scan number alternat from 0 to 1
	this->previousScanDistance=0; //variable for the critical scan distance previous
	this->previousScanIntensity=0; //variable for the critical scan intensity previous
	this->previousScanNumber=0; //variable for the numberth of the previous scan critical object
	this->previousScanNumberMin=0; //variable for the min numberth of the previous scan critcial object can be scan
	this->previousScanNumberMax=0; //variable for the max numberth of the previous scan critcial object can be scan
}

Movement currentMovement;//create field for current movement of the node.
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
	ROS_INFO("Current x position is: %f", x);
	ROS_INFO("Current y position is: %f", y);
}

void Entity::stageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample numbe
	//range vector means distance measure corresponds to the a set of angles

	// reset values
	bool found=false;
	minDistance = 10;
	obstacleAngle = 270;
	criticalIntensity=0; //the critical intensity of the surrounding specific subclass should implement avoidance plan
	int l=msg.ranges.size(); // sizeof(msg.ranges[0]);
	for (int i=45; i<l-45; i++){ //only process the object in +45 to -45 degree of the nodes laser
		if (msg.ranges[i]< minDistance) {//work out the minimum distance object
			minDistance = msg.ranges[i];
			obstacleAngle= (i/l) * msg.angle_increment + msg.angle_min;
		}
		if (msg.ranges[i]<1){//work most fatal intensity
			if(numOfScan==0){
				if(msg.intensities[i]>previousScanIntensity){
					previousScanIntensity=msg.intensities[i];//record first scan intensity
					previousScanDistance=msg.ranges[i];//record first scan range
					previousScanNumber=i;
				}
			}
		}
	}
	if(numOfScan==1){//check if there is perpendicular movement
		if(previousScanDistance<msg.ranges[previousScanNumber]&&previousScanIntensity==msg.intensities[previousScanNumber]){//obstacle got closer
			criticalIntensity=previousScanIntensity;
		}else{
			int currentMax=previousScanNumber;
			int currentMin=previousScanNumber;
			for(int i=previousScanNumber;i<l-45;i++){//work out max number of scan critical object still can be observed
				if(previousScanIntensity!=msg.intensities[i]&&!found){
					currentMax=i-1;
					found=true;
				}
			}
			found=false;
			for(int i=previousScanNumber;i>44;i--){//work out min number of scan critical object still can be observed
				if(previousScanIntensity!=msg.intensities[i]&&!found){
					currentMin=i+1;
					found=true;
				}
			}
			if (currentMax!=previousScanNumberMax||currentMin!=previousScanNumberMin){//it is moving horizontally or rotating
				criticalIntensity=4;//halt movement
			}
		}
		numOfScan=0;
	}else{
		for(int i=previousScanNumber;i<l-45;i++){//work out max number of scan critical object still can be observed
			if(previousScanIntensity!=msg.intensities[i]&&!found){
				previousScanNumberMax=i-1;
				found=true;
			}
		}
		found=false;
		for(int i=previousScanNumber;i>44;i--){//work out min number of scan critical object still can be observed
			if(previousScanIntensity!=msg.intensities[i]&&!found){
				previousScanNumberMin=i+1;
				found=true;
			}
		}
		criticalIntensity=4;//halt movement
		numOfScan+=1;
	}
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
 * Message to get node to start going through the movement queue
 */
void Entity::move(){
	if(avoidanceQueue.size()>0){
		desireLocation=false;
		currentMovement=avoidanceQueue.front();
		if(currentMovement.getType().compare("forward_x")==0){
			//call move forward for x direction
			moveForward(currentMovement.getPos(),currentMovement.getVel(),"x",1);
		}else if (currentMovement.getType().compare("forward_y")==0){
			//call move forward for y direction
			moveForward(currentMovement.getPos(),currentMovement.getVel(),"y",1);
		}else{
			//call rotate
			rotate(currentMovement.getPos(),currentMovement.getVel(),1);
		}
	}else if(movementQueue.size()>0){
		desireLocation=false;
		currentMovement=movementQueue.front();
		if(currentMovement.getType().compare("forward_x")==0){
			//call move forward for x direction
			moveForward(currentMovement.getPos(),currentMovement.getVel(),"x",2);
		}else if (currentMovement.getType().compare("forward_y")==0){
			//call move forward for y direction
			moveForward(currentMovement.getPos(),currentMovement.getVel(),"y",2);
		}else{
			//call rotate
			rotate(currentMovement.getPos(),currentMovement.getVel(),2);
		}
	}else{
		desireLocation=true;
	}
}

/**
 * Method to remove movements from movement queue
 */
void Entity::movementComplete(){
	//convert to position
	movementQueue.erase(movementQueue.begin());//remove movement from queue
	desireLocation=true;
}

/**
 * Method to remove movements to from avoidance queue
 */
void Entity::avoidanceComplete(){
	//convert to position
	avoidanceQueue.erase(avoidanceQueue.begin());
	desireLocation=true;
}

/**
 * Method to get the size of the vector of the movement queue
 */
int Entity::getMovementQueueSize() {
	return movementQueue.size();
}

/**
 * Method to get the size of the vector of the avoidance queue
 */
int Entity::getAvoidanceQueueSize() {
	return avoidanceQueue.size();
}
/**
 * Method to add movements to movement queue
 * Distance: the value relative to the absolute frame of reference
 * eg if +5(north) in y then distance is 5 if -5(south) in y then distance is -5
 */
void Entity::addMovement(std::string type, double distance,double velocity){
	//convert to position
	double pos=0;
	if (type.compare("rotation")!=0){
		bool useCurrent=true; //boolean to check if current location should be use
		if (movementQueue.size()>0){//check if queue have initial values
			bool found=false;
			int foundIndex=movementQueue.size();
			ROS_INFO("queue size: %d", foundIndex);
			int index=foundIndex-1;
			ROS_INFO("index: %d", index);
			while(index>=0){
				if(movementQueue.at(index).getType().compare(type)==0){
					found=true;
					foundIndex=index;
				}
				index-=1;
			}
			if (found){//if found same type use that as reference for position
				useCurrent=false;
				pos=distance+movementQueue.at(foundIndex).getPos();
			}
		}
		if (useCurrent){//when no other forward movement to reference use current location
			if((type.compare("forward_x"))==0){
				pos=x+distance;
			}else if ((type.compare("forward_y"))==0){
				pos=y+distance;
			}
		}
	}else{
		pos=distance;
	}
	//ROS_INFO("pos: %f", pos);
	Movement m=Movement(type,pos,velocity);
	movementQueue.push_back(m);
}

/**
 * Method to add movements to front of movement queue
 * Distance: the value relative to the absolute frame of reference
 * eg if +5(north) in y then distance is 5 if -5(south) in y then distance is -5
 * queueNum of queue 1 for avoidance 2 for movements
 */
void Entity::addMovementFront(std::string type, double distance,double velocity, int queueNum){
	//convert to position
	double pos=0;
	if (type.compare("rotation")!=0){
		bool useCurrent=true; //boolean to check if current location should be use
		if((type.compare("forward_x"))==0){
			pos=x+distance;
		}else if ((type.compare("forward_y"))==0){
			pos=y+distance;
		}
	}else{
		pos=distance;
	}
	//ROS_INFO("pos: %f", pos);
	Movement m=Movement(type,pos,velocity);
	if(queueNum==2){
		movementQueue.insert(movementQueue.begin(),m);
	}else{
		avoidanceQueue.insert(avoidanceQueue.begin(),m);
	}
}

/**
 * Message to move the entity forward in the direction it is facing
 * Note unit is in meters
 * input:	double vel: the velocity of the entity moving forward
 *			double pos: the absolute position to move to
 *			int queueNum: which queue is dispatch from 1 for avoidance 2 for movement
 */
void Entity::moveForward(double pos, double vel, std::string direction,int queueNum){
	double position=0;
	if (direction.compare("x")==0){
		position=x;
	}else{
		position=y;
	}
	if (!desireLocation){//TODO slow down
		ROS_INFO("mfpos: %f", pos);
		if (std::abs(position-pos)>=0.1){
			if(std::abs(position-pos)<=0.2){
				linearVelocity=0.1;
			}else if(std::abs(position-pos)<=1){
				linearVelocity=1;
			}else{
				linearVelocity=vel;
			}
			if (position>pos){//make sure the robot can slight go backwards to adjust to right position
				if(directionFacing==EAST){ //if facing east then velocity should be negative since overshoot
					linearVelocity=-linearVelocity;
				}else if(directionFacing==NORTH){ //if facing North then velocity should be negative since overshoot
					linearVelocity=-linearVelocity;
				}
				//	linearVelocity=-linearVelocity;
			}else if (pos>position){//now in the -ve direction to our frame of reference
				if(directionFacing==WEST){ //if facing west then velocity should be negative since overshoot
					linearVelocity=-linearVelocity;
				}else if(directionFacing==SOUTH){ //if facing south then velocity should be negative since overshoot
					linearVelocity=-linearVelocity;
				}
			}
		}else{
			if (queueNum==2){
				movementComplete();//call method complete to remove complete movement from queue
			}else{
				avoidanceComplete();//call method to remove from avoidance queue
			}
			linearVelocity=0;
		}
	}else{
		desireLocation=true;
		linearVelocity=0;
	}
	angularVelocity=0;
	updateOdometry(); //update the information to stage
}

/**
 * Message to rotate the entity.
 * Input:	double angleToRotate: In radians, the angle entity will rotate to relative to absoulte frame.
 *			double angleSpeed: how fast we want the entity to rotate. Note its speed so always +ve
 *			int queueNum: which queue is dispatch from 1 for avoidance 2 for movement
 */
void Entity::rotate(double angleToRotateTo, double angleSpeed, int queueNum){
	//Check if angleToRotateTo and the current angle is similar. If not rotate.
	if (std::abs(angleToRotateTo-theta)>0.0001){
		if (std::abs(angleToRotateTo-theta)<(0.002)){//slow down speed when very near
			//ROS_INFO(""+(angleToRotateTo-theta));
			angularVelocity=0.001;
			updateOdometry();
		}else if (std::abs(angleToRotateTo-theta)<(0.05)){//slow down speed when very near
			//ROS_INFO(""+(angleToRotateTo-theta));
			angularVelocity=0.01;
			updateOdometry();
		}else if (std::abs(angleToRotateTo-theta)<(0.3)){//slow down speed when near
			//ROS_INFO(""+(angleToRotateTo-theta));
			angularVelocity=0.1;
			updateOdometry();
		}else{
			angularVelocity=angleSpeed;
		}
		if (angleToRotateTo<theta){//if angle to rotate to is less than theta rotate CW
			angularVelocity=-angularVelocity;
		}else if (angleToRotateTo==M_PI){
			//if its 180 degrees (this can be +ve or -ve so need to make sure fastest turn  implemented
			if (theta<0){// if -ve theta then CW is fastest
				angularVelocity=-angularVelocity;
			}
		}
		updateOdometry();
	}else{
		//set the direction the robot is now facing
		if(theta<0.1 && theta>-0.1){ //if facing east then velocity should be negative since overshoot
			directionFacing=EAST;
		}else if(theta<M_PI/2+0.1 && theta>M_PI/2-0.1){ //if facing North then velocity should be negative since overshoot
			directionFacing=NORTH;
		}else if(theta<-M_PI+0.1 || theta>M_PI-0.1){ //if facing west then velocity should be negative since overshoot
			directionFacing=WEST;
		}else if(theta<-M_PI/2+0.1 && theta>((-M_PI/2)-0.1)){ //if facing south then velocity should be negative since overshoot
			directionFacing=SOUTH;
		}
		if (queueNum==2){
			movementComplete();//call method complete to remove complete movement from queue
		}else{
			avoidanceComplete();//call method to remove from avoidance queue
		}
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
	addMovement("rotation",M_PI/2, angleSpeed);
}

/**
 * Message to rotate the entity such that it faces South
 */
void Entity::faceSouth(double angleSpeed){
	addMovement("rotation",-M_PI/2, angleSpeed);
}

/**
 * Message to rotate the entity such that it faces East
 */
void Entity::faceEast(double angleSpeed){
	addMovement("rotation",0, angleSpeed);
}

/**
 * Message to rotate the entity such that it faces West
 */
void Entity::faceWest(double angleSpeed){
	addMovement("rotation",M_PI, angleSpeed);
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
 * Getter method for min distance of obstacle from entity
 */

double Entity::getMinDistance() {
	return minDistance;
}

/**
 * Getter method for getting obstacle angle from entity to minDistance obstacle
 */

double Entity::getObstacleAngle() {
	return obstacleAngle;
}

/**
 * Getter method for getting the critical intensity of the entity
 * so the highest intensity of laser callback for close object
 */
int Entity::getCriticalIntensity() {
	return criticalIntensity;
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


/**
 * Getter method for status of the entity
 */
std::string Entity::getStatus() {
	return status;
}

/**
 * Getter method for the direction the entity is facing
 */
Entity::Direction Entity::getDirectionFacing() {
	return directionFacing;
}

/**
 * setter method for status of the entity
 */
void Entity::setStatus(std::string status){
	this->status=status;
}

