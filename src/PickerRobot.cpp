#include "ros/ros.h"
#include <sstream>
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

PickerRobot::~PickerRobot(){
}
PickerRobot pickerRobot;
//std::string status="Moving";
std::string previousStatus = "Moving";
std::string obstacleStatus = "No obstacles";
double distance=1;
//destination of next beacon
double destX;
double destY;

/*
 * Wrapper method for the callBackStageOdm method (in Entity)
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	pickerRobot.stageOdom_callback(msg);
}

void callBackLaserScan(const sensor_msgs::LaserScan msg) {
	pickerRobot.stageLaser_callback(msg);

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
void PickerRobot::stateLogic(){
	if (pickerRobot.getStatus().compare("Moving")==0){
		if(pickerRobot.movementQueue.size()<1){
			pickerRobot.setStatus("Full");
		}
	}
}
/*
 * Method for the logic of PickerRobot running its movement queue.
 */
void PickerRobot::movement(){
	//If status is not full the picker robot will keep moving
	/*if(status.compare("Full")!=0){
		if (pickerRobot.getDesireLocation()){
			//if picker robot is at desire location set status to full
			status="Full";
		}else{
			//if picker robot is not at desire location keep moving
			//pickerRobot.addMovement("forward_x",1,1);
		}

	}*/
	//pickerRobot.moveForward(distance,1);
//ALPHA MOVEMENT COMMENTED BELOW    
	if (distance==1){
		pickerRobot.faceEast(1);
		pickerRobot.addMovement("forward_x",37.5,1);
		pickerRobot.faceSouth(1);
		pickerRobot.addMovement("forward_y",-3.35,1);
		pickerRobot.faceWest(1);
		pickerRobot.addMovement("forward_x",-37.5,1);
	}else if (distance ==5){
		pickerRobot.faceEast(1);
		pickerRobot.addMovement("forward_x",37.5,1);
		pickerRobot.faceNorth(1);
		pickerRobot.addMovement("forward_y",3.35,1);
		pickerRobot.faceWest(1);
		pickerRobot.addMovement("forward_x",-37.5,1);
	}
//    //temporary variable used in calculation for distance to move
//    double distanceToMove = 0;
//    //if there is a destination
//    if (destX != null) {
//        //if the robot is not at its destination
//        if (x != destX) {
//            //check if we need to go backwards
//            if (x > destX) {
//                
//                pickerRobot.addMovement("forward_x",
//    }
}

/*
 * Method that is called whenever a message is received from a beacon.
 * It will use the message to determine the Picker's next destination.
 */
void beaconCallback(const nav_msgs::Odometry msg) {
    destX = msg.pose.pose.position.x;
    destY = msg.pose.pose.position.y;
    
    //debugging purposes
    ROS_INFO("Beacon_1 x position is: %f", x);
	ROS_INFO("Beacon_1 y position is: %f", y);
}

int main(int argc, char **argv)
{
	pickerRobot=PickerRobot("Moving");
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "PickerRobot");

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
    
    // create subscribers for all beacons on world
    ros:: Subscriber beacon1_sub = n.subscribe<nav_msgs::Odometry>("/beacon1/", 1000, beaconCallback);
    //ros:: Subscriber beacon2_sub = n.subscribe<nav_msgs::Odometry>("/beacon2/", 1000, beaconCallback);
        
    // add them all to the beacon queue so the PickerRobot can process them one at a time
    pickerRobot.beaconQueue.push_back(beacon1_sub);
    //pickerRobot.beaconQueue.push_back(beacon2_sub);

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

		pickerRobot.move();//robot move
		//TODO debug
		if(count==7){
			pickerRobot.movement();
		}
		if(count>7){
			pickerRobot.stateLogic();
		}
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
