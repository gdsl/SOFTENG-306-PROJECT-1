#include "CarrierRobot.h"
#include <ros/console.h>
#include "se306project/carrier_status.h"
#include "se306project/robot_status.h"
#include "se306project/weed_status.h"
#include <utility>
#include <vector>
#include <iostream>
#include <string>

/*
 * Default constructor for carrier Robot
 */
CarrierRobot::CarrierRobot() {}

/*
 * Constructor for carrier Robot with status
 */
CarrierRobot::CarrierRobot(double x,double y,double theta,double linearVel, double angularVel,std::string status)
:Robot( x, y, theta, linearVel,  angularVel){
	this->setStatus(status);
	this->setState(IDLE);
	carrierInFront = true;
	initialMovement = false;
	yDistanceTravel = 0;
	xDistanceTravel = 0;
}

/*
 * Default destructor for carrier Robot
 */
CarrierRobot::~CarrierRobot() {}

// getter function
bool CarrierRobot::isInitialMovement() {
	return this->initialMovement;
}

bool CarrierRobot::isCarrierInFront() {
	return this->carrierInFront;
}

double CarrierRobot::getYDistanceTravel() {
	return this->yDistanceTravel;
}

double CarrierRobot::getXDistanceTravel() {
	return this->xDistanceTravel;
}

//setter function
void CarrierRobot::setYDistanceTravel(double y) {
	this->yDistanceTravel = y;
}

void CarrierRobot::setXDistanceTravel(double x) {
	this->xDistanceTravel = x;
}

void CarrierRobot::setCarrierInFront(bool front) {
	this->carrierInFront = front;
}

void CarrierRobot::setInitialMovement(bool initial) {
	this->initialMovement = initial;
}



//create carrier robot and status
CarrierRobot carrierRobot;
//std::string status="Idle";
std::string previousStatus = "Idle";
std::vector<std::pair<double,double> > seenPointList;
double targetX = 0;
double targetY = 0;

//string splitting function, taken from: http://stackoverflow.com/questions/236129/split-a-string-in-c
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}
	return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}

/*
 * Wrapper method for the callBackStageOdm method
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	carrierRobot.stageOdom_callback(msg);
}

void callBackLaserScan(const sensor_msgs::LaserScan msg) {
	carrierRobot.stageLaser_callback(msg);//call super class callback which is where the laser scanning logic is designed

	if (carrierRobot.getAvoidanceCase()!=Entity::NONE&&carrierRobot.getAvoidanceCase()!=Entity::TREE) {//check if there is need to avoid obstacle
		if(carrierRobot.getState()!=Robot::IDLE){//check if robot is idle or not
			carrierRobot.setObstacleStatus("Obstacle nearby");
			if(carrierRobot.getAvoidanceCase()==Entity::WEED){// if its weed stop
				carrierRobot.addMovementFront("forward_x",0,0,1);//add empty movement to front of avoidance to stop
				carrierRobot.setObstacleStatus("Weed! Help!");
			}else if(carrierRobot.getAvoidanceCase()==Entity::LIVING_OBJ){//if its human or animal stop
				carrierRobot.addMovementFront("forward_x",0,0,1);//add empty movement to front of avoidance to stop
				carrierRobot.setObstacleStatus("Living Object");
			}else if(carrierRobot.getAvoidanceCase()==Entity::HALT){//if its halt stop
				carrierRobot.addMovementFront("forward_x",0,0,1);//add empty movement to front of avoidance to stop
				carrierRobot.setObstacleStatus("Obstacle nearby. Halt");
			}else if(carrierRobot.getAvoidanceCase()==Entity::STATIONARY&& carrierRobot.getCriticalIntensity()>1){//if its stationary robot
				//if the carrier robot is infront and carrier is queue then halt
				if(carrierRobot.getState()==carrierRobot.QUEUE&& carrierRobot.getCriticalIntensity()==3){//if its carrier
					carrierRobot.addMovementFront("forward_x",0,0,1);//this is at front of queue
				}else if(carrierRobot.getCriticalIntensity()!=2|| carrierRobot.getMinDistance()<0.4){//if not picker robot or if its too close
					if(carrierRobot.getDirectionFacing()== carrierRobot.NORTH){
						carrierRobot.addMovementFront("rotation",M_PI/2,1,1);
						carrierRobot.addMovementFront("forward_x",3,1,1);
						carrierRobot.addMovementFront("rotation",0, 1,1);
						carrierRobot.addMovementFront("forward_y",3,1,1);
						carrierRobot.addMovementFront("rotation",M_PI/2,1,1);
						carrierRobot.addMovementFront("forward_x",-3,1,1);
						carrierRobot.addMovementFront("rotation",M_PI,1,1);
						carrierRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
						//carrierRobot.move();
					}else if(carrierRobot.getDirectionFacing()== carrierRobot.SOUTH){
						carrierRobot.addMovementFront("rotation",-M_PI/2,1,1);
						carrierRobot.addMovementFront("forward_x",3,1,1);
						carrierRobot.addMovementFront("rotation",0, 1,1);
						carrierRobot.addMovementFront("forward_y",-3,1,1);
						carrierRobot.addMovementFront("rotation",-M_PI/2,1,1);
						carrierRobot.addMovementFront("forward_x",-3,1,1);
						carrierRobot.addMovementFront("rotation",M_PI,1,1);
						carrierRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
						//carrierRobot.move();
					}else if(carrierRobot.getDirectionFacing()== carrierRobot.EAST){
						carrierRobot.addMovementFront("rotation",0, 1,1);
						carrierRobot.addMovementFront("forward_y",3,1,1);
						carrierRobot.addMovementFront("rotation",M_PI/2, 1,1);
						carrierRobot.addMovementFront("forward_x",3,0,1);
						carrierRobot.addMovementFront("rotation",0, 1,1);
						carrierRobot.addMovementFront("forward_y",-3,1,1);
						carrierRobot.addMovementFront("rotation",-M_PI/2, 1,1);
						carrierRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
						//carrierRobot.move();
					}else if(carrierRobot.getDirectionFacing()== carrierRobot.WEST){
						carrierRobot.addMovementFront("rotation",M_PI, 1,1);
						carrierRobot.addMovementFront("forward_y",3,1,1);
						carrierRobot.addMovementFront("rotation",M_PI/2, 1,1);
						carrierRobot.addMovementFront("forward_x",-3,0,1);
						carrierRobot.addMovementFront("rotation",M_PI, 1,1);
						carrierRobot.addMovementFront("forward_y",-3,1,1);
						carrierRobot.addMovementFront("rotation",-M_PI/2, 1,1);
						carrierRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
						//carrierRobot.move();
					}
				}
			}else if(carrierRobot.getAvoidanceCase()==Entity::PERPENDICULAR){
				if(carrierRobot.getDirectionFacing()== carrierRobot.NORTH||carrierRobot.getDirectionFacing()== carrierRobot.SOUTH){
					//if robot moving in the y direction give way
					carrierRobot.addMovementFront("forward_x",0,0,1);
				}
			}else if(carrierRobot.getAvoidanceCase()==Entity::FACE_ON){
				if(carrierRobot.getAvoidanceQueueSize()<=0){
					if(carrierRobot.getDirectionFacing()== carrierRobot.NORTH&&carrierRobot.getObstacleStatus().compare("Obstacle nearby")!=0){
						carrierRobot.addMovementFront("rotation",M_PI/2,1,1);
						carrierRobot.addMovementFront("forward_x",3,1,1);
						carrierRobot.addMovementFront("rotation",0, 1,1);
						carrierRobot.addMovementFront("forward_y",3,1,1);
						carrierRobot.addMovementFront("rotation",M_PI/2,1,1);
						carrierRobot.addMovementFront("forward_x",-3,1,1);
						carrierRobot.addMovementFront("rotation",M_PI,1,1);
						carrierRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
						//carrierRobot.move();
					}else if(carrierRobot.getDirectionFacing()== carrierRobot.EAST&&carrierRobot.getObstacleStatus().compare("Obstacle nearby")!=0){
						carrierRobot.addMovementFront("rotation",0, 1,1);
						carrierRobot.addMovementFront("forward_y",3,1,1);
						carrierRobot.addMovementFront("rotation",M_PI/2, 1,1);
						carrierRobot.addMovementFront("forward_x",3,0,1);
						carrierRobot.addMovementFront("rotation",0, 1,1);
						carrierRobot.addMovementFront("forward_y",-3,1,1);
						carrierRobot.addMovementFront("rotation",-M_PI/2, 1,1);
						carrierRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
						//carrierRobot.move();
					}
				}else{
					//halt movement if already have avoidance logic
					carrierRobot.addMovementFront("forward_x",0,0,1);
					//carrierRobot.move();
				}
			}
			//get carrier to move
			carrierRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
			carrierRobot.move();
		}
	} else {
		carrierRobot.setObstacleStatus("No obstacles");
	}
}

/*
 * Method that process the picker robot message received.
 * This method is called when message is received.
 */
void recievePickerRobotStatus(const se306project::robot_status::ConstPtr& msg)
{
	//Check the status of Carrier robot so see how it should act
	//when status is arrived it means that the carrier robot has arrived at picker
	if (carrierRobot.getStatus().compare("Arrived")==0){
		//Change status to transporting as the carrier robot is now taking away the full bin
		carrierRobot.setState(Robot::TRANSPORTING);
		if (carrierRobot.getMovementQueueSize() == 0) {
			carrierRobot.faceWest(1);
			carrierRobot.addMovement("forward_x", -1*carrierRobot.getXDistanceTravel(),1);

			if (carrierRobot.getY() > 0) {
				carrierRobot.faceSouth(1);
			} else {
				carrierRobot.faceNorth(1);
			}
			carrierRobot.addMovement("forward_y", 0-carrierRobot.getY(),1);

			carrierRobot.faceWest(1);
			carrierRobot.addMovement("forward_x", -10,1);
		}
	}else if(carrierRobot.getStatus().compare("Idle")==0){
		//when the carrier robot is idle and the picker robot is full the carrier robot move to it.
		if ((msg->status).compare("Full") == 0){

			//check the list of points that carrier robot seen
			bool seen = false;
			std::pair<double,double> currentPoint;
			currentPoint.first = -5;
			currentPoint.second = 8.15;

			for (std::vector<std::pair<double,double> >::iterator it = seenPointList.begin(); it != seenPointList.end(); ++it) {
				if (*it == currentPoint) {
					seen = true;
					break;
				}
			}

			//if the carrier robot have not seen the point or other carrier is not moving to the point
			//dispatch the current carrier to this point
			if (!seen) {
				carrierRobot.setState(Robot::MOVING);
				if (carrierRobot.getMovementQueueSize() <= 1) {
					targetX = msg->pos_x;
					targetY = msg->pos_y;
					carrierRobot.faceEast(1);
					carrierRobot.addMovement("forward_x",10,1);
					double pickerY = msg->pos_y;
					//double pickerY = 8.15;
					if (pickerY >= carrierRobot.getY() ){
						carrierRobot.faceNorth(1);
					} else {
						carrierRobot.faceSouth(1);
					}
					//yDistanceTravel = pickerY - carrierRobot.getY();
					carrierRobot.setYDistanceTravel(pickerY - carrierRobot.getY());
					carrierRobot.addMovement("forward_y",carrierRobot.getYDistanceTravel(),1);

					double pickerX = msg->pos_x;
					//double pickerX = -5;
					carrierRobot.faceEast(1);
					//xDistanceTravel = pickerX - carrierRobot.getX() -10;
					double distanceBetween = 3.65;
					carrierRobot.setXDistanceTravel(pickerX - carrierRobot.getX() -10 - distanceBetween);
					carrierRobot.addMovement("forward_x",carrierRobot.getXDistanceTravel(),1);
				}
			}
		}

	}
}

/**
 *Method for carrier robot status callback
 */
void receiveCarrierRobotStatus(const se306project::robot_status::ConstPtr& msg)
{
	//checking the other carrier target location while waiting to be dispatch
	if (carrierRobot.getState() == Robot::IDLE) {
		std::string status = msg->status;
		//only want the carrier robot that is moving toward the picker robots
		if (status.find("Moving") !=  std::string::npos) {
			std::vector<std::string> stringList = split(status,' ');

			double x;
			double y;
			std::stringstream convertX(stringList[1]);
			std::stringstream convertY(stringList[2]);

			if (!(convertX >> x)) x = 0;
			if (!(convertY >> y)) y = 0;

			//check if the point is not in seen list already then add it in.
			std::pair<double,double> currentPoint;
			currentPoint.first = x;
			currentPoint.second = y;
			bool seen = false;
			for (std::vector<std::pair<double,double> >::iterator it = seenPointList.begin(); it != seenPointList.end(); ++it) {
				if (*it == currentPoint) {
					seen = true;
					break;
				}
			}

			if (!seen) seenPointList.push_back(currentPoint);
		}
	}
}

/**
 * Method for the carrier robot's states transition and implementation
 */
void CarrierRobot::stateLogic(){
	if (carrierRobot.getState() == IDLE) {
		carrierRobot.setStatus("Idle");
	} else if (carrierRobot.getState() == MOVING) {
		std::string status="Moving ";
		std::stringstream convert;
		std::stringstream convert2;
		convert << targetX;
		convert2 << targetY;
		status=status+convert.str()+" "+convert2.str();
		carrierRobot.setStatus(status);
		ROS_INFO("x start: %s", status.c_str());
		if (carrierRobot.getMovementQueueSize() == 0 ) {
			carrierRobot.setState(Robot::ARRIVED);
		}
	} else if (carrierRobot.getState() == ARRIVED) {
		carrierRobot.setStatus("Arrived");
	} else if (carrierRobot.getState() == TRANSPORTING) {
		carrierRobot.setStatus("Transporting");

		if (carrierRobot.getMovementQueueSize() == 0 ) {
			//move up the queue line to the dispatch area.
			carrierRobot.setState(Robot::QUEUE);
			carrierRobot.faceNorth(1);
			double distance = 24 - carrierRobot.getY();
			carrierRobot.addMovement("forward_y", distance, 1);
		}
	} else if (carrierRobot.getState() == QUEUE) {
		carrierRobot.setStatus("Queueing");

		if (carrierRobot.getMovementQueueSize() == 0 && carrierRobot.getY() >= 23.99 ) {
			//if the robot is in the dispatch area, switch idle state and ready to dispatch
			seenPointList.clear();
			carrierRobot.setState(Robot::IDLE);
		}
	}
	carrierRobot.move();

}

int main(int argc, char **argv)
{
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "CarrierRobot");

	// convert input parameters for Robot initialization from String to respective types
	std::string xString = argv[1];
	std::string yString = argv[2];
	double xPos = atof(xString.c_str());
	double yPos = atof(yString.c_str());
	ROS_INFO("x start: %f", xPos);
	ROS_INFO("y start: %f", yPos);

	//initialize the Carrier robot with the correct position, velocity and state parameters.
	carrierRobot=CarrierRobot(xPos,yPos,M_PI/2,0,0,"Idle");
	carrierRobot.setState(Robot::TRANSPORTING);

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;


	ros::Rate loop_rate(10);
	//Broadcast the node's velocity information for other and stage to subscribe to.
	carrierRobot.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	//Broadcast the node's status information for other to subscribe to.
	ros::Publisher pub=n.advertise<se306project::carrier_status>("status",1000);

	//subscribe to listen to messages coming from stage for odometry (this is base pose so it is
	//relative to the absolute frame of the farm.
	carrierRobot.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	//relative to the obstacle information
	carrierRobot.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, callBackLaserScan);

	// publish weed obstacle
	carrierRobot.weed_obstacle_pub = n.advertise<se306project::weed_status>("weed", 1000);

	//subscribe to the status of picker
	//getting picker robot starting number and ending number;
	std::string start(argv[3]);
	std::string end(argv[4]);
	int s = atoi(start.c_str());
	int e = atoi(end.c_str());
	int size = e-s+1;
	std::string topicName;

	//subscribing all the picker robot
	ros::Subscriber *array = new ros::Subscriber[size];
	int index = 0;
	for (int i = s; i<=e; i++) {
		std::stringstream convert;
		convert << i;
		topicName = "/robot_" + convert.str() + "/status";
		array[index] = n.subscribe<se306project::robot_status>(topicName,1000,recievePickerRobotStatus);
		index++;
	}

	//subscribe to other carrier
	std::string carrierStart(argv[5]);
	std::string carrierEnd(argv[6]);
	int a = atoi(carrierStart.c_str());
	int b = atoi(carrierEnd.c_str());
	size = b-a+1;

	//subscribing all the carrier robot

	ros::Subscriber *carrierArray = new ros::Subscriber[size];
	index = 0;
	for (int i = a; i<=b; i++) {
		std::stringstream convert;
		convert << i;
		topicName = "/robot_" + convert.str() + "/status";
		carrierArray[index] = n.subscribe<se306project::robot_status>(topicName,1000,receiveCarrierRobotStatus);
		index++;
	}

	//a count of how many messages we have sent
	int count = 0;
	//carrier status message initialisation
	se306project::carrier_status status_msg;

	//ROS loop
	while (ros::ok())
	{		
		status_msg.my_counter=count;		//add counter to message
		status_msg.status=carrierRobot.getStatus();		//add status to message
		status_msg.pos_x=carrierRobot.getX(); //add x to message to broadcast
		status_msg.pos_y=carrierRobot.getY();//add y to message to broadcast
		status_msg.pos_theta=carrierRobot.getTheta(); //add angle to message to broadcast
		status_msg.obstacle = carrierRobot.getObstacleStatus();
		pub.publish(status_msg);	//publish message
		carrierRobot.stateLogic();
		//carrierRobot.move();
		ros::spinOnce();
		loop_rate.sleep();
		++count; // increase counter
	}

	delete[] array;
	delete[] carrierArray;
	return 0;
}
