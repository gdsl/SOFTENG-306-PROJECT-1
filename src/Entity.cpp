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
#include "Constants.h"
#include <sensor_msgs/LaserScan.h>

/**
 * Default constructor of Entity. Calls the other constructor with default values.
 */

Entity::Entity():Entity(0,0,0,0,0) {

}

/**
 * Default destructor
 */
Entity::~Entity() {

}

/**
 * Constructor with parameters
 */
Entity::Entity(double x, double y, double theta, double linearVelocity, double angularVelocity) {
	// Initialise variables
	this->x=x;
	this->y=y;
	this->theta = theta;
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
	desireLocation=false;
	// The distance of the nearest obstacle
	this->minDistance=30.0;
	// Set the default obstacle angle as a value larger than 180
	this-> obstacleStatus = "No obstacles";
	this->obstacleAngle=270;
	this->criticalIntensity=0;
	this->numOfScan=0; 		
	this->previousScanDistance=0; 	// Variable for the critical scan distance previous
	this->previousScanIntensity=0; 	// Variable for the critical scan intensity previous
	this->previousScanNumber=0; 	// Variable for the numberth of the previous scan critical objects
	this->previousScanNumberMin=0; 	// Variable for the min number of the previous scan critical object can be scan
	this->previousScanNumberMax=0; 	// Variable for the max number of the previous scan critical object can be scan
	this->avoidanceCase=NONE;
	this->previousAvoidanceCase=NONE;
	this->z=0;
	this->zVelocity=0;
}

// Field for current movement of the node
Movement currentMovement;

/**
 * Update the position of the Entity
 */
void Entity::setPose(int x, int y, double theta) {
	this->x = x;
	this->y = y;
	this->theta = theta;
}

/**
 * Update the velocity of Entity
 */

void Entity::setVelocity(double linearVelocity, double angularVelocity) {
	this->linearVelocity = linearVelocity;
	this->angularVelocity = angularVelocity;
}

/**
 * This is the call back function to process odometry messages coming from Stage
 */

void Entity::stageOdom_callback(nav_msgs::Odometry msg) {
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
	z = msg.pose.pose.position.z;


	tf::Pose pose;
	tf::poseMsgToTF(msg.pose.pose,pose);

	theta = tf::getYaw(pose.getRotation());
	// ROS logging api
	ROS_INFO("Current x position is: %f", x);
	ROS_INFO("Current y position is: %f", y);
}

/**
 * This is the callback function to process laser scan messages
 * You can access the range data from msg.ranges[i]. i = sample number
 * Range vector means distance measure corresponds to the a set of angles
 */
void Entity::stageLaser_callback(sensor_msgs::LaserScan msg) {
	// Reset values
	bool found=false;
	avoidanceCase=NONE;
	minDistance = 10;
	obstacleAngle = 270;
	// The critical intensity of the surrounding specific subclass should implement avoidance plan
	criticalIntensity=0; 
	int l=msg.ranges.size(); 
	// Varaible for current intensity
	double currentIntensity=0;
	// Only process the object in +49 to -49 degree of the nodes laser
	if(numOfScan==0){//reset previous values
		previousScanIntensity=0;
		previousScanDistance=0;
		previousScanNumber=0;
	}
	for (int i=0; i<l; i++){ 
		// Work out the minimum distance object
		if (msg.ranges[i]< minDistance) {
			if(msg.intensities[i]>1||msg.ranges[i]<0.5){//if its not tree or tree very close
				minDistance = msg.ranges[i];
				if(msg.ranges[i]<1.1&&msg.intensities[i]>currentIntensity) {
					// Record the most critical intensity within 1.1
					currentIntensity=msg.intensities[i];
				}
				obstacleAngle= (i/l) * msg.angle_increment + msg.angle_min;
			}
		}
		if (numOfScan==0) {// Work most fatal intensity
			if(msg.ranges[i]<1.1&&msg.intensities[i]>previousScanIntensity) {
				// Record first scan intensity
				previousScanIntensity=msg.intensities[i];
				// Record first scan range
				previousScanDistance=msg.ranges[i];
				previousScanNumber=i;
			}
		}
	}

	if (minDistance<1.1&&currentIntensity>=1) {
		// Check if there is perpendicular movement
		if (currentIntensity==1.0) {
			// The object in way is a weed
			avoidanceCase=TREE;
		}else if (currentIntensity==WEED_INTENSITY) {
			// The object in way is a weed
			avoidanceCase=WEED;
		} else if (currentIntensity>=LIVING_MIN_INTENSITY){
			// The object is a living object that is not weed
			avoidanceCase=LIVING_OBJ;
		}
		if (numOfScan==1) {
			
				// Obstacle got closer
			if (previousScanDistance<msg.ranges[previousScanNumber]&&previousScanIntensity==msg.intensities[previousScanNumber]) {
				// The object is face on
				avoidanceCase=FACE_ON;
			} else {
				int currentMax=previousScanNumber;
				int currentMin=previousScanNumber;
				// Work out max number of scan critical object still can be observed
				for(int i=previousScanNumber;i<l-41;i++) {
					if(previousScanIntensity!=msg.intensities[i]&&!found) {
						currentMax=i-1;
						found=true;
					}
				}
				found=false;
				// Work out min number of scan critical object still can be observed
				for(int i=previousScanNumber;i>41;i--) {
					if(previousScanIntensity!=msg.intensities[i]&&!found) {
						currentMin=i+1;
						found=true;
					}
				}
				// It is moving horizontally or rotating
				//TODO
				if (currentMax!=previousScanNumberMax||currentMin!=previousScanNumberMin) {
					// Avoidance case is perpendicular
					avoidanceCase=PERPENDICULAR;
				} else {
					// If x or y distance didn't change then entity must be stationary
					avoidanceCase=STATIONARY;
				}
				avoidanceCase=STATIONARY;
			}
			numOfScan=0;
			previousAvoidanceCase=avoidanceCase;
		} else {
				// Work out max number of scan critical object still can be observed
				for(int i=previousScanNumber;i<l-41;i++) {
					if(previousScanIntensity!=msg.intensities[i]&&!found) {
						previousScanNumberMax=i-1;
						found=true;
					}
				}
				found=false;
				// Work out min number of scan critical object still can be observed
				for(int i=previousScanNumber;i>41;i--) {
					if(previousScanIntensity!=msg.intensities[i]&&!found) {
						previousScanNumberMin=i+1;
						found=true;
					}
				}
			// set avoidance case to previous
			avoidanceCase=previousAvoidanceCase;
			numOfScan+=1;
		}
	}
	criticalIntensity=currentIntensity;
}

/**
 * Message to stage of Entity's odometry
 */
void Entity::updateOdometry() {
	robotNode_cmdvel.linear.x = linearVelocity;
	robotNode_cmdvel.angular.z = angularVelocity;
	robotNode_cmdvel.angular.x = -zVelocity;
	// Publish message
	robotNode_stage_pub.publish(robotNode_cmdvel);
}

/**
 * Message to get node to start going through the movement queue
 */
void Entity::move() {
	if (avoidanceQueue.size()>0) {
		desireLocation=false;
		currentMovement=avoidanceQueue.front();
		if (currentMovement.getType().compare("forward_x")==0) {
			// Call move forward for x direction
			moveForward(currentMovement.getPos(),currentMovement.getVel(),"x",1);
		} else if (currentMovement.getType().compare("forward_y")==0) {
			// Call move forward for y direction
			moveForward(currentMovement.getPos(),currentMovement.getVel(),"y",1);
		}else {
			// Call rotate
			rotate(currentMovement.getPos(),currentMovement.getVel(),1);
		}
	} else if (movementQueue.size()>0) {
		desireLocation=false;
		currentMovement=movementQueue.front();
		if (currentMovement.getType().compare("forward_x")==0) {
			// Call move forward for x direction
			moveForward(currentMovement.getPos(),currentMovement.getVel(),"x",2);
		} else if (currentMovement.getType().compare("forward_y")==0) {
			// Call move forward for y direction
			moveForward(currentMovement.getPos(),currentMovement.getVel(),"y",2);
		} else if (currentMovement.getType().compare("forward_z")==0){ 
			moveZ(currentMovement.getPos(),currentMovement.getVel(),2);
		}else {
			// Call rotate
			rotate(currentMovement.getPos(),currentMovement.getVel(),2);
		}
	} else {
		desireLocation=true;
	}
}

/**
 * Method to remove movements from movement queue
 */
void Entity::movementComplete() {
	// Remove movement from queue
	movementQueue.erase(movementQueue.begin());
	desireLocation=true;
}

/**
 * Method to remove movements to from avoidance queue
 */
void Entity::avoidanceComplete() {
	//convert to position
	avoidanceQueue.erase(avoidanceQueue.begin());
	desireLocation=true;
}

/**
 * Method that will handle avoidance of obstacle by moving slight around the obstacle
 * Input:
 * 		Entity entity: the entity to avoid the obstacle
 * 		double x: the magnitude of amount x to move
 * 		double y: the magnitude of amount y to move
 */
void Entity::avoidObstacle(Entity entity, double x,double y){
	if(entity.getDirectionFacing()== NORTH){
		entity.addMovementFront("rotation",M_PI/2,1,1);
		entity.addMovementFront("forward_x",x,1,1);
		entity.addMovementFront("rotation",0, 1,1);
		entity.addMovementFront("forward_y",y,1,1);
		entity.addMovementFront("rotation",M_PI/2,1,1);
		entity.addMovementFront("forward_x",-x,1,1);
		entity.addMovementFront("rotation",M_PI,1,1);
		entity.addMovementFront("forward_x",0,0,1);//this is at front of front
		//pickerRobot.move();
	}else if(entity.getDirectionFacing()== SOUTH){
		entity.addMovementFront("rotation",-M_PI/2,1,1);
		entity.addMovementFront("forward_x",x,1,1);
		entity.addMovementFront("rotation",0, 1,1);
		entity.addMovementFront("forward_y",-y,1,1);
		entity.addMovementFront("rotation",-M_PI/2,1,1);
		entity.addMovementFront("forward_x",-x,1,1);
		entity.addMovementFront("rotation",M_PI,1,1);
		entity.addMovementFront("forward_x",0,0,1);//this is at front of front
	}else if(entity.getDirectionFacing()== EAST){
		entity.addMovementFront("rotation",0, 1,1);
		entity.addMovementFront("forward_y",y,1,1);
		entity.addMovementFront("rotation",M_PI/2, 1,1);
		entity.addMovementFront("forward_x",x,0,1);
		entity.addMovementFront("rotation",0, 1,1);
		entity.addMovementFront("forward_y",-y,1,1);
		entity.addMovementFront("rotation",-M_PI/2, 1,1);
		entity.addMovementFront("forward_x",0,0,1);//this is at front of front
	}else if(entity.getDirectionFacing()== WEST){
		entity.addMovementFront("rotation",M_PI, 1,1);
		entity.addMovementFront("forward_y",y,1,1);
		entity.addMovementFront("rotation",M_PI/2, 1,1);
		entity.addMovementFront("forward_x",-x,0,1);
		entity.addMovementFront("rotation",M_PI, 1,1);
		entity.addMovementFront("forward_y",-y,1,1);
		entity.addMovementFront("rotation",-M_PI/2, 1,1);
		entity.addMovementFront("forward_x",0,0,1);//this is at front of front
	}
}

/**
 * Method to empty movement queue
 */
void Entity::flushMovementQueue() {
	movementQueue.clear();
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
 * Method to get the avoidance cases
 */
Entity::AvoidanceCase Entity::getAvoidanceCase() {
	return avoidanceCase;
}

/**
 * Method to add movements to movement queue
 * Distance: the value relative to the absolute frame of reference
 * eg if +5(north) in y then distance is 5 if -5(south) in y then distance is -5
 */
void Entity::addMovement(std::string type, double distance,double velocity) {
	//convert to position
	double pos=0;
	if (type.compare("rotation")!=0) {
		// Boolean to check if current location should be used
		bool useCurrent=true; 
		// Check if queue has initial values
		if (movementQueue.size()>0) {
			bool found=false;
			int foundIndex=movementQueue.size();
			ROS_INFO("queue size: %d", foundIndex);
			int index=foundIndex-1;
			ROS_INFO("index: %d", index);
			while(index>=0){
				if(movementQueue.at(index).getType().compare(type)==0) {
					found=true; 
					foundIndex=index;
				}
				index-=1;
			}
			// If found same type use that as reference for position
			if (found){
				useCurrent=false;
				pos=distance+movementQueue.at(foundIndex).getPos();
			}
		}
		// When no other forward movement to reference use current location
		if (useCurrent) {
			if ((type.compare("forward_x"))==0) {
				pos=x+distance;
			} else if ((type.compare("forward_y"))==0) {
				pos=y+distance;
			}
		}
	} else {
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
void Entity::addMovementFront(std::string type, double distance,double velocity, int queueNum) {
	//convert to position
	double pos=0;
	if (type.compare("rotation")!=0){
		if ((type.compare("forward_x"))==0) {
			pos=x+distance;
		}else if ((type.compare("forward_y"))==0) {
			pos=y+distance;
		}else if((type.compare("forward_z"))==0){
			pos=distance;
		}
	} else {
		pos=distance;
	}
	//ROS_INFO("pos: %f", pos);
	Movement m=Movement(type,pos,velocity);
	if (queueNum==2) {
		movementQueue.insert(movementQueue.begin(),m);
	} else {
		avoidanceQueue.insert(avoidanceQueue.begin(),m);
	}
}

/**
 * Message to move the entity forward in the direction it is facing
 * Note unit is in meters
 * input:	double vel: the velocity of the entity moving forward
 *		double pos: the absolute position to move to
 *		int queueNum: which queue is dispatch from 1 for avoidance 2 for movement
 */
void Entity::moveForward(double pos, double vel, std::string direction,int queueNum) {
	double position=0;
	if (direction.compare("x")==0){
		position=x;
	}else if(direction.compare("y")==0){
		position=y;
	}
	if (!desireLocation){//TODO slow down
		ROS_INFO("mfpos: %f", pos);
		if (std::abs(position-pos)>=0.01&&vel!=0){
			if(std::abs(position-pos)<=0.2&&vel>0.1){
				linearVelocity=0.1;
			}else if(std::abs(position-pos)<=1&&vel>1){
				linearVelocity=1;
			}else{
				linearVelocity=vel;
			}
			// Make sure the robot can slightly go backwards to adjust to right position
			if (position>pos&&position!=z){
				// If facing east then velocity should be negative since overshoot
				if(directionFacing==EAST){ 
					linearVelocity=-linearVelocity;
					// If facing North then velocity should be negative since overshoot
				}else if(directionFacing==NORTH){ 
					linearVelocity=-linearVelocity;
				}
				// Now in the -ve direction to our frame of reference
			}else if (pos>position&&position!=z){
				// If facing west then velocity should be negative since overshoot
				if(directionFacing==WEST){ 
					linearVelocity=-linearVelocity;
					// If facing south then velocity should be negative since overshoot
				}else if(directionFacing==SOUTH){ 
					linearVelocity=-linearVelocity;
				}
			}
		}else{
			if (queueNum==2){
				// Call method complete to remove complete movement from queue
				movementComplete();
			}else{
				// Call method to remove from avoidance queue
				avoidanceComplete();
			}
			linearVelocity=0;
		}
	}else{
		desireLocation=true;
		if (queueNum==2){
			// Call method complete to remove complete movement from queue
			movementComplete();
		}else{
			// Call method to remove from avoidance queue
			avoidanceComplete();
		}
		linearVelocity=0;
	}
	angularVelocity=0;
	// Update the information to stage
	updateOdometry();
}

/**
 * Message to move the entity in the z
 * Note unit is in meters
 * input:	double vel: the velocity of the entity moving forward
 *		double pos: the absolute position to move to
 *		int queueNum: which queue is dispatch from 1 for avoidance 2 for movement
 */
void Entity::moveZ(double pos, double vel,int queueNum) {
	double position=z;

	/*if (!desireLocation){//TODO slow down
		ROS_INFO("mfpos: %f", pos);
		if (std::abs(position-pos)>=0.01){
			if(std::abs(position-pos)<=0.2&&vel>0.1){
				zVelocity=-0.1;
			}else if(std::abs(position-pos)<=1&&vel>1){
				zVelocity=-1;
			}else{
				zVelocity=vel;
			}
		}else{
			if (queueNum==2){
				// Call method complete to remove complete movement from queue
				movementComplete();
			}else{
				// Call method to remove from avoidance queue
				avoidanceComplete();
			}
			zVelocity=0;
		}
	}else{
		desireLocation=true;
		zVelocity=0;
	}*/
	zVelocity=3;
	linearVelocity=0;
	angularVelocity=0;
	// Update the information to stage
	updateOdometry();


}
/**
 * Message to rotate the entity.
 * Input:	double angleToRotate: In radians, the angle entity will rotate to relative to absoulte frame.
 *		double angleSpeed: how fast we want the entity to rotate. Note its speed so always +ve
 *		int queueNum: which queue is dispatch from 1 for avoidance 2 for movement
 */
void Entity::rotate(double angleToRotateTo, double angleSpeed, int queueNum) {
	// Check if angleToRotateTo and the current angle is similar. If not rotate.
	if (std::abs(angleToRotateTo-theta)>0.0001){
		// Slow down speed when very near
		if (std::abs(angleToRotateTo-theta)<(0.002)){
			// ROS_INFO(""+(angleToRotateTo-theta));
			angularVelocity=0.001;
			updateOdometry();
			// Slow down speed when very near
		} else if (std::abs(angleToRotateTo-theta)<(0.05)){
			// ROS_INFO(""+(angleToRotateTo-theta));
			angularVelocity=0.01;
			updateOdometry();
			// Slow down speed when near
		} else if (std::abs(angleToRotateTo-theta)<(0.3)){
			// ROS_INFO(""+(angleToRotateTo-theta));
			angularVelocity=0.1;
			updateOdometry();
		} else{
			angularVelocity=angleSpeed;
		}
		if (angleToRotateTo<theta){//if angle to rotate to is less than theta rotate CW
			angularVelocity=-angularVelocity;
		} else if (angleToRotateTo==M_PI){
			// If it's 180 degrees (this can be +ve or -ve so need to make sure fastest turn  implemented)
			// If -ve theta then CW is fastest
			if (theta<0){
				angularVelocity=-angularVelocity;
			}
		}
		updateOdometry();
	} else{
		// Set the direction the robot is now facing
		// If facing East then velocity should be negative since overshoot
		if(theta<0.1 && theta>-0.1){
			directionFacing=EAST;
			// If facing North then velocity should be negative since overshoot
		} else if(theta<M_PI/2+0.1 && theta>M_PI/2-0.1){
			directionFacing=NORTH;
			// If facing West then velocity should be negative since overshoot
		} else if(theta<-M_PI+0.1 || theta>M_PI-0.1){ 
			directionFacing=WEST;
			// If facing South then velocity should be negative since overshoot
		} else if(theta<-M_PI/2+0.1 && theta>((-M_PI/2)-0.1)){ 
			directionFacing=SOUTH;
		}
		if (queueNum==2){
			// Call method complete to remove complete movement from queue
			movementComplete();
		} else{
			// Call method to remove from avoidance queue
			avoidanceComplete();
		}
		// If angle similar stop rotating
		angularVelocity=0;
		linearVelocity=0;
		// Update the information to stage
		updateOdometry();
	}
}

/**
 * Method to determine the status of the entity
 */
void Entity::determineStatus() {
	// Logic to determine current status of Entity - Walking/Idle/Turning
	// Convert radians to degrees
	double angle = roundf(theta * 57.2957795 * 100) / 100;
	// Check if entity is moving (and therefore 'walking')
	if (linearVelocity > 0.01) {
		status = "Walking";
	}
	// Check if entity is facing North/East/South/West AND not moving (and therefore 'idle')
	else if ((angle == -360) || (angle == -270) || (angle == -180) || (angle == -90) || (angle == 0) || (angle == 90) || (angle == 180) || (angle == 270) || (angle == 360) && (linearVelocity == 0)) {
		status = "Idle";
	}
	// Check if entity is 'turning'
	else {
		status = "Turning";
	}
}

/**
 * Message to rotate the entity such that it faces North
 */
void Entity::faceNorth(double angleSpeed) {
	addMovement("rotation",M_PI/2, angleSpeed);
}

/**
 * Message to rotate the entity such that it faces South
 */
void Entity::faceSouth(double angleSpeed) {
	addMovement("rotation",-M_PI/2, angleSpeed);
}

/**
 * Message to rotate the entity such that it faces East
 */
void Entity::faceEast(double angleSpeed) {
	addMovement("rotation",0, angleSpeed);
}

/**
 * Message to rotate the entity such that it faces West
 */
void Entity::faceWest(double angleSpeed) {
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
 * Setter method for linear velocity of entity
 */
void Entity::setLin(double lv) {
	linearVelocity = lv;
}

/**
 * Setter method for angular velocity of entity
 */
void Entity::setAng(double av) {
	angularVelocity = av;
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
 * Getter method for desired location of entity
 */
bool Entity::getDesireLocation() {
	return desireLocation;
}

/**
 * Setter method for desired location of entity
 */
void Entity::setDesireLocation(bool desireLocation) {
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
 * Setter method for status of the entity
 */
void Entity::setStatus(std::string status) {
	this->status=status;
}

/**
 * Getter method for the obstacle detection status
 */
std::string Entity::getObstacleStatus(){
	return obstacleStatus;
}

/**
 * setter method for the obstacle detection status
 */
void Entity::setObstacleStatus(std::string obstacleStatus){
	this->obstacleStatus=obstacleStatus;
}

