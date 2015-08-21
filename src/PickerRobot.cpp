#include "ros/ros.h"
#include <sstream>
#include <stdlib.h>
#include "Robot.h"
#include "se306project/robot_status.h"
#include "se306project/carrier_status.h"
#include "PickerRobot.h"
#include "Constants.h"
#include <vector>

PickerRobot::PickerRobot():Robot(){

}

PickerRobot::PickerRobot(std::string status){
	this->setStatus(status);
}

PickerRobot::PickerRobot(double x,double y,double theta,double linearVel, double angularVel,std::string status)
	:Robot( x, y, theta, linearVel,  angularVel){
	this->setStatus(status);
    this->setState(DISPATCH);
}

PickerRobot::~PickerRobot(){
}
PickerRobot pickerRobot;
//std::string status="Moving";
std::string previousStatus = "Moving";
std::string obstacleStatus = "No obstacles";
double distance=1;
//destination of next beacon
double destX = 0;
double destY = 0;
bool atDestX = false, atDestY = false;
//subscriber to subscribe to the destination beacon
ros::Subscriber beacon_sub;
//the start and finish for this PickerRobot's fruit picking path
int startBeacon = 0, finishBeacon = 0;
//current beacon
int currentBeacon = 0;
//boolean to signal if the Picker has subscribed to its next beacon
bool hasNewBeacon();

/**
 * Getter method for the bin capacity of the picker robot
 */
int PickerRobot::getBinCapacity(){
	return bin_capacity;
}

/**
 * Setter method for the bin capacity of the picker robot
 */
void PickerRobot::setBinCapacity(int bin_capacity){
	this->bin_capacity=bin_capacity;
}

/*
 * Wrapper method for the callBackStageOdm method (in Entity)
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	pickerRobot.stageOdom_callback(msg);
}

void callBackLaserScan(const sensor_msgs::LaserScan msg) {
	pickerRobot.stageLaser_callback(msg);
	int pickrange=2;
	if (pickerRobot.getStatus().compare("Moving")==0){
		//TODO if(msg.ranges[0]<=pickrange&&msg.intensities[0]==1){
		if(msg.ranges[0]<=pickrange&&msg.ranges[7]>=pickrange){
			pickerRobot.setBinCapacity(pickerRobot.getBinCapacity()+2);
		}
	}
	if (pickerRobot.getMinDistance() < 1) {
		obstacleStatus = "Obstacle nearby";
	} else {
		obstacleStatus = "No obstacles";
	}
}
/*
 * Method that process the carrier robot message received.
 * This method is called when message is received.
 */
void recieveCarrierRobotStatus(const se306project::carrier_status::ConstPtr& msg){
	if (msg->status.compare("Transporting")==0&&pickerRobot.getStatus().compare("Full")==0){
		if (distance==1){
			distance=5;
		}else if (distance ==5){
			distance=1;
		}
		pickerRobot.movement();
		pickerRobot.setDesireLocation(false);
		pickerRobot.setStatus("Moving");
	}
}

/**
 * Method for the carrier robot's states transition and implementation
 */
void PickerRobot::stateLogic(ros::NodeHandle n){
//	if(pickerRobot.getBinCapacity()>=BIN_CAPACITY){
//		pickerRobot.setStatus("Full");
//		pickerRobot.addMovementFront("forward_x",0,0,1);
//		pickerRobot.move();
//		//TODO halt movement
//	}
//	if (pickerRobot.getStatus().compare("Moving")==0){
//		pickerRobot.move();
//		if(pickerRobot.getMovementQueueSize()<1){
//			pickerRobot.setStatus("Full");
//		}
//	}
    if (pickerRobot.getMovementQueueSize() == 0) {
        //if the current state is dispatch then the Picker Robot should move to the starting position of its picking path.
        //this is the first beacon along its path.
        if (pickerRobot.getState() == DISPATCH) {
            //set the current beacon to be the starting beacon
            currentBeacon = startBeacon;
            //subscribe to the new beacon
            subscribeNextBeacon(n);
            pickerRobot.movement();
            pickerRobot.setState(PICKING);

        } else if (pickerRobot.getState() == PICKING) {
            //check if the Picker Robot is moving East or West along a row of kiwi fruit and adjust next beacon number accordingly
            //if Picker is starting on the left of a row, make it go to the right.
            if ((currentBeacon % 2) == 1) {currentBeacon++;}
            //if Picker is starting on the right of a row, make it go to the left.
            else {currentBeacon--;}
            subscribeNextBeacon(n);
            pickerRobot.movement();
            //change state so when movements are complete, Picker should be at end of row and this state change will cause it to
            //move down to the next row
            pickerRobot.setState(GO_TO_NEXT_BEACON);

        } else if (pickerRobot.getState() == GO_TO_NEXT_BEACON) {
            //check if the Picker has reached the final beacon in its fruit picking path
            if (currentBeacon == finishBeacon) {
                //if it has then change the state to finished
                pickerRobot.setState(FINISHED);
            //otherwise move the picker down to the next row which is just the current beacon plus 2
            } else {
                currentBeacon = currentBeacon + 2;
                subscribeNextBeacon(n);
                pickerRobot.movement();
                //change state to PICKING so the next time this code is executed, it will tell the Picker to start
                //picking the row it is at
                pickerRobot.setState(PICKING);
            }

        } else if (pickerRobot.getState() == FULL_BIN) {

        } else if (pickerRobot.getState() == FINISHED) {

        }
    }
}
/*
 * Method for the logic of PickerRobot running its movement queue.
 */
void PickerRobot::movement(){
    //temporary variable used in calculation for distance to move
    double distanceToMove = 0;
    double currentX = pickerRobot.getX();
    double currentY = pickerRobot.getY();
    //if the Picker has received the destination of the next beacon
    //add the horizontal movement to the movement queue
    //if the robot is not at its destination
    if (destX != 0 && destY != 0) {
        if (pickerRobot.getMovementQueueSize() == 0) {
            if (!atDestX) {            
                //check if the Robot needs to go West
                if (currentX > destX) {
                    //calculate the distance to move backwards along X axis
                    distanceToMove = -(currentX - destX);
                    //make sure the Robot is facing West, if not, turn it West.
                    if (pickerRobot.getDirectionFacing() != WEST) {pickerRobot.faceWest(1);}                
                //otherwise it means the Robot needs to go East
                } else if (currentX < destX) {
                    distanceToMove = destX - currentX;
                    //make sure the Robot is facing West, if not, turn it West.
                    if (pickerRobot.getDirectionFacing() != EAST) {pickerRobot.faceEast(1);}
                }
                pickerRobot.addMovement("forward_x", distanceToMove, 1);
            } else {
                //now add the vertical movement to the movement queue
                if (!atDestY) {
                    //check if the Robot needs to go South
                    if (currentY > destY) {
                        //calculate the distance to move backwards along Y axis
                        distanceToMove = -(currentY - destY);
                        //make sure the Robot is facing South, if not, turn it South.
                        if (pickerRobot.getDirectionFacing() != SOUTH) {
                            pickerRobot.faceSouth(1);                    
                        }                
                    //otherwise it means the Robot needs to go North
                    } else if (currentY < destY) {
                        distanceToMove = destY - currentY;
                        //make sure the Robot is facing North, if not, turn it North.
                        if (pickerRobot.getDirectionFacing() != NORTH) {pickerRobot.faceNorth(1);}
                    }
                    pickerRobot.addMovement("forward_y", distanceToMove, 1);
                }            
            }
        }
    }
    
}

/*
 * Method that is called whenever a message is received from a beacon.
 * It will use the message to determine the Picker's next destination.
 */
void beaconCallback(const nav_msgs::Odometry msg) {
    destX = msg.pose.pose.position.x;
    destY = msg.pose.pose.position.y;
    
    if (std::abs(destX - pickerRobot.getX()) < 0.01) {
        atDestX = true;
        //ROS_INFO("AT BEACON X POSITION");
    }
    else {atDestX = false;}
    
    if (std::abs(destY - pickerRobot.getY())<0.01) {
        atDestY = true;
        //ROS_INFO("AT BEACON Y POSITION");
    }
    else {atDestY = false;}
    
    //debugging purposes
    //ROS_INFO("Next beacon x position is: %f", destX);
	//ROS_INFO("Next beacon y position is: %f", destY);
}


/*
 * Resubscribe the beacon subscriber to the next beacon after converting the beacon number to a string
 * Assumes the beacon number has already been updated to contain the next destination beacon.
 */
void subscribeNextBeacon(ros::NodeHandle n) {
    if (atDestX && atDestY) {
        std::string currentBeaconS;
        std::stringstream out;
        out << currentBeacon;
        currentBeaconS = out.str();
        beacon_sub = n.subscribe<nav_msgs::Odometry>("/beacon" + currentBeaconS + "/", 1000, beaconCallback);
        atDestX = false;
        atDestY = false;
    }
    
    
}

int main(int argc, char **argv)
{
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "PickerRobot");
	
    // convert input parameters for Robot initialization from String to respective types
    std::string xString = argv[1];
    std::string yString = argv[2];
    double xPos = atof(xString.c_str());
    double yPos = atof(yString.c_str());
    ROS_INFO("x start: %f", xPos);
    ROS_INFO("y start: %f", yPos);
    
    //initialize the Picker robot with the correct position, velocity and state parameters.
	pickerRobot=PickerRobot(xPos,yPos,M_PI/2,0,0,"Moving");
	//pickerRobot=PickerRobot(-42,24,M_PI/2,0,0,"Moving");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//picker robot advertise it node for its velocity message to be published.
	pickerRobot.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	//picker robot advertise it node for its status message to be published.
	ros::Publisher pub=n.advertise<se306project::robot_status>("status",1000);

	//subscribe to listen to messages coming from stage about is position relative to absolute frame
	pickerRobot.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	//subscribe to obstacle detection
	pickerRobot.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000,callBackLaserScan);
	//subscribe to carrier robot's status message
	ros::Subscriber mysub_object = n.subscribe<se306project::carrier_status>("/robot_1/status",1000,recieveCarrierRobotStatus);
    
    // assign beacon subscriber to the first beacon for this Picker robot's path.
    beacon_sub = n.subscribe<nav_msgs::Odometry>("/beacon1/", 1000, beaconCallback);

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
		status_msg.status=pickerRobot.getStatus();//add status to message to broadcast
		status_msg.pos_x=pickerRobot.getX(); //add x to message to broadcast
		status_msg.pos_y=pickerRobot.getY();//add y to message to broadcast
		status_msg.pos_theta=pickerRobot.getTheta(); //add angle to message to broadcast
		status_msg.obstacle = obstacleStatus;
		pub.publish(status_msg);//publish the message for other node
        
        //subscribeNextBeacon(n);
        //pickerRobot.movement();
        //pickerRobot.move();
        pickerRobot.stateLogic();
		//TODO debug
//		if(count==7){			
//			pickerRobot.move();
//		}
//		if(count>7){
//			pickerRobot.stateLogic();
//		}
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
