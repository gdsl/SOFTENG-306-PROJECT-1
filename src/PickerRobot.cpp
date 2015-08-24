#include "ros/ros.h"
#include <sstream>
#include <stdlib.h>
#include "Robot.h"
#include "se306project/robot_status.h"
#include "se306project/carrier_status.h"
#include "PickerRobot.h"
#include "Constants.h"
#include <vector>
#include <string>

PickerRobot::PickerRobot():Robot(){
    this->bin_capacity=0;
    this->pick_range=pick_range;
}

PickerRobot::PickerRobot(std::string status){
	this->setStatus(status);
    this->bin_capacity=0;
    this->pick_range=pick_range;
}

PickerRobot::PickerRobot(double x,double y,double theta,double linearVel, double angularVel,std::string status,double pick_range)
	:Robot( x, y, theta, linearVel,  angularVel){
	this->setStatus(status);
    this->setState(DISPATCH);
    this->bin_capacity=0;
    this->pick_range=pick_range;

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
//position of previous beacon
double oldDestX = 0;
double oldDestY = 0;
bool atDestX = false, atDestY = false;
//subscriber to subscribe to the destination beacon
ros::Subscriber beacon_sub;
//the start and finish for this PickerRobot's fruit picking path
int startBeacon = 0, finishBeacon = 0;
//current beacon
int currentBeacon = 0;
//boolean to signal if the Picker has subscribed to its next beacon
bool hasNewBeacon, targetBeaconSet = false;

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

/**
 * Getter method for the pick range of the picker robot
 */
double PickerRobot::getPickRange(){
	return pick_range;
}

/**
 * Setter method for the pick range of the picker robot
 */
void PickerRobot::setPickRange(double pick_range){
	this->pick_range=pick_range;
}

/*
 * Wrapper method for the callBackStageOdm method (in Entity)
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	pickerRobot.stageOdom_callback(msg);
}

void callBackLaserScan(const sensor_msgs::LaserScan msg) {
	pickerRobot.stageLaser_callback(msg);
	if (pickerRobot.getAvoidanceCase()!=Entity::NONE) {//check if there is need to avoid obstacle
		if(pickerRobot.getState()!=Robot::IDLE){//check if robot is idle or not
			obstacleStatus = "Obstacle nearby";
			if(pickerRobot.getAvoidanceCase()==Entity::WEED){// if its weed stop
				pickerRobot.addMovementFront("forward_x",0,0,1);//add empty movement to front of avoidance to stop
				obstacleStatus = "Weed! Help!";
			}else if(pickerRobot.getAvoidanceCase()==Entity::LIVING_OBJ){//if its human or animal stop
				pickerRobot.addMovementFront("forward_x",0,0,1);//add empty movement to front of avoidance to stop
			}else if(pickerRobot.getAvoidanceCase()==Entity::HALT){//if its halt stop
				pickerRobot.addMovementFront("forward_x",0,0,1);//add empty movement to front of avoidance to stop
			}else if(pickerRobot.getAvoidanceCase()==Entity::STATIONARY&& pickerRobot.getCriticalIntensity()>1){//if its stationary robot
					if(pickerRobot.getDirectionFacing()== pickerRobot.NORTH){
						pickerRobot.addMovementFront("rotation",M_PI/2,1,1);
						pickerRobot.addMovementFront("forward_x",3,1,1);
						pickerRobot.addMovementFront("rotation",0, 1,1);
						pickerRobot.addMovementFront("forward_y",3,1,1);
						pickerRobot.addMovementFront("rotation",M_PI/2,1,1);
						pickerRobot.addMovementFront("forward_x",-3,1,1);
						pickerRobot.addMovementFront("rotation",M_PI,1,1);
						pickerRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
						//pickerRobot.move();
					}else if(pickerRobot.getDirectionFacing()== pickerRobot.SOUTH){
						pickerRobot.addMovementFront("rotation",-M_PI/2,1,1);
						pickerRobot.addMovementFront("forward_x",3,1,1);
						pickerRobot.addMovementFront("rotation",0, 1,1);
						pickerRobot.addMovementFront("forward_y",-3,1,1);
						pickerRobot.addMovementFront("rotation",-M_PI/2,1,1);
						pickerRobot.addMovementFront("forward_x",-3,1,1);
						pickerRobot.addMovementFront("rotation",M_PI,1,1);
						pickerRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
					}else if(pickerRobot.getDirectionFacing()== pickerRobot.EAST){
						pickerRobot.addMovementFront("rotation",0, 1,1);
						pickerRobot.addMovementFront("forward_y",3,1,1);
						pickerRobot.addMovementFront("rotation",M_PI/2, 1,1);
						pickerRobot.addMovementFront("forward_x",3,0,1);
						pickerRobot.addMovementFront("rotation",0, 1,1);
						pickerRobot.addMovementFront("forward_y",-3,1,1);
						pickerRobot.addMovementFront("rotation",-M_PI/2, 1,1);
						pickerRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
					}else if(pickerRobot.getDirectionFacing()== pickerRobot.WEST){
						pickerRobot.addMovementFront("rotation",M_PI, 1,1);
						pickerRobot.addMovementFront("forward_y",3,1,1);
						pickerRobot.addMovementFront("rotation",M_PI/2, 1,1);
						pickerRobot.addMovementFront("forward_x",-3,0,1);
						pickerRobot.addMovementFront("rotation",M_PI, 1,1);
						pickerRobot.addMovementFront("forward_y",-3,1,1);
						pickerRobot.addMovementFront("rotation",-M_PI/2, 1,1);
						pickerRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
					}
			}else if(pickerRobot.getAvoidanceCase()==Entity::PERPENDICULAR){
				if(pickerRobot.getDirectionFacing()== pickerRobot.NORTH||pickerRobot.getDirectionFacing()== pickerRobot.SOUTH){
					//if robot moving in the y direction give way
					pickerRobot.addMovementFront("forward_x",0,0,1);
				}
			}else if(pickerRobot.getAvoidanceCase()==Entity::FACE_ON){
				if(pickerRobot.getAvoidanceQueueSize()<=0){
					if(pickerRobot.getDirectionFacing()== pickerRobot.NORTH&&obstacleStatus.compare("Obstacle nearby")!=0){
						pickerRobot.addMovementFront("rotation",M_PI/2,1,1);
						pickerRobot.addMovementFront("forward_x",3,1,1);
						pickerRobot.addMovementFront("rotation",0, 1,1);
						pickerRobot.addMovementFront("forward_y",3,1,1);
						pickerRobot.addMovementFront("rotation",M_PI/2,1,1);
						pickerRobot.addMovementFront("forward_x",-3,1,1);
						pickerRobot.addMovementFront("rotation",M_PI,1,1);
						pickerRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
					}else if(pickerRobot.getDirectionFacing()== pickerRobot.EAST&&obstacleStatus.compare("Obstacle nearby")!=0){
						pickerRobot.addMovementFront("rotation",0, 1,1);
						pickerRobot.addMovementFront("forward_y",3,1,1);
						pickerRobot.addMovementFront("rotation",M_PI/2, 1,1);
						pickerRobot.addMovementFront("forward_x",3,0,1);
						pickerRobot.addMovementFront("rotation",0, 1,1);
						pickerRobot.addMovementFront("forward_y",-3,1,1);
						pickerRobot.addMovementFront("rotation",-M_PI/2, 1,1);
						pickerRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
					}
				}else{
					//halt movement if already have avoidance logic
					pickerRobot.addMovementFront("forward_x",0,0,1);
				}
			}
			//get carrier to move
			pickerRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
			pickerRobot.move();
		}
	} else {
		obstacleStatus = "No obstacles"; //only pick when obstacle detected
		if (pickerRobot.getStatus().compare("Picking")==0){
			ROS_INFO("Laser %d",pickerRobot.getBinCapacity());
			ROS_INFO("pick range %f",pickerRobot.getPickRange());
			if(msg.ranges[0]<=pickerRobot.getPickRange()&&msg.intensities[0]==1&&msg.ranges[5]>=pickerRobot.getPickRange()){
				pickerRobot.setBinCapacity(pickerRobot.getBinCapacity()+1);
				if(pickerRobot.getBinCapacity()>=BIN_CAPACITY){
					pickerRobot.setState(Robot::FULL_BIN);
				}
			}
		}
	}
}
/*
 * Method that process the carrier robot message received.
 * This method is called when message is received.
 */
void recieveCarrierRobotStatus(const se306project::carrier_status::ConstPtr& msg){
	if ((msg->status.compare("Arrived")==0)&&pickerRobot.getStatus().compare("Full")==0){
		pickerRobot.setStatus("Picking");
		pickerRobot.setBinCapacity(0);
		pickerRobot.setState(Robot::PICKING);
	}
}

/**
 * Method for the picker robot's states transition and implementation
 */
void PickerRobot::stateLogic(ros::NodeHandle n){
//	if(pickerRobot.getBinCapacity()>=BIN_CAPACITY){
//		pickerRobot.setStatus("Full");
//		pickerRobot.addMovementFront("forward_x",0,0,1);
//		pickerRobot.move();
//
//	}
//	if (pickerRobot.getStatus().compare("Moving")==0){
//		pickerRobot.move();
//		if(pickerRobot.getMovementQueueSize()<1){
//			pickerRobot.setStatus("Full");
//		}
//	}
    
    //if the Picker robot is currently executing movements, it means that it has not yet reached its next target beacon
    //only if the next beacon has been reached, should a state update occur.
    if(pickerRobot.getState() == FULL_BIN) {
    	pickerRobot.setStatus("Full");
    	pickerRobot.addMovementFront("forward_x",0,0,1);//halt when full
    }else if (pickerRobot.getMovementQueueSize()==0){
        
        //reset this boolean to indicate that the Picker has not yet received directions from its next target beacon
        //hasNewBeacon = false;
        
        //if the current state is dispatch then the Picker Robot should move to the starting position of its picking path.
        //this is the first beacon along its path.
        if (pickerRobot.getState() == DISPATCH) {
            ROS_INFO("IM HERE");
            //set the current beacon to be the starting beacon
            currentBeacon = startBeacon;
            //if the Picker has received directions from next beacon, proceed to next state as it involves the next beacon
            if (hasNewBeacon) {
                pickerRobot.movement();
                //pickerRobot.setState(PICKING);
                //pickerRobot.setStatus("Moving");
                pickerRobot.setState(GO_TO_NEXT_BEACON);
                pickerRobot.setStatus("To next beacon");
            }

        } else if (pickerRobot.getState() == GO_TO_NEXT_BEACON) {
            //this check is required if this "if block" is accessed twice before the subscribed beacon starts sending messages
            //if the messages do not start arriving in time the currentBeacon would be continuously updated.
            if (!targetBeaconSet) {
                //check if the Picker Robot is moving East or West along a row of kiwi fruit and adjust next beacon number accordingly
                //if Picker is starting on the left of a row, make it go to the right.
                if ((currentBeacon % 2) == 1) {currentBeacon++;}
                //if Picker is starting on the right of a row, make it go to the left.
                else {currentBeacon--;}
                targetBeaconSet = true;
                subscribeNextBeacon(n);           
            }
            //change state so when movements are complete, Picker should be at end of row and this state change will cause it to
            //move down to the next row
            if (hasNewBeacon) {
                pickerRobot.movement();
                //as messages have arrived and movements have been added to queue, this can be set to false for the next time this State is set
                pickerRobot.setState(PICKING);
                pickerRobot.setStatus("Picking");
                targetBeaconSet = false;
            }


        } else if (pickerRobot.getState() == PICKING) {
            //check if the Picker has reached the final beacon in its fruit picking path
            if (currentBeacon == finishBeacon) {
                //if it has then change the state to finished
                pickerRobot.setState(FINISHED);
            //otherwise move the picker down to the next row which is just the current beacon plus 2
            } else {
                //this check is required if this "if block" is accessed twice before the subscribed beacon starts sending messages
                //if the messages do not start arriving in time the currentBeacon would be continuously updated.
                if (!targetBeaconSet) {
                    currentBeacon = currentBeacon + 2;
                    targetBeaconSet = true;
                    subscribeNextBeacon(n);       
                }
                //change state to PICKING so the next time this code is executed, it will tell the Picker to start
                //picking the row it is at
                if (hasNewBeacon) {
                    pickerRobot.movement();
                    //as messages have arrived and movements have been added to queue, this can be set to false for the next time this State is set
                    pickerRobot.setState(GO_TO_NEXT_BEACON);
					pickerRobot.setStatus("To next beacon");
                    targetBeaconSet = false;
                }
            }

        } /*else if (pickerRobot.getState() == FULL_BIN) {
        	pickerRobot.setStatus("Full");
        	pickerRobot.addMovementFront("forward_x",0,0,1);//halt when full
        }*/ else if (pickerRobot.getState() == FINISHED) {

        }
        //ROS_INFO("current beacon = %d", currentBeacon);
    }
    pickerRobot.move();
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
            }
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
        //once the required movements to go to the next beacon have been pushed to the movement queue 
        //set these values so in the beaconCallback method it can tell that the Robot has not yet received 
        //positions from the NEXT beacon on its path.
        oldDestX = destX;
        oldDestY = destY;
    }
    
}

/*
 * Method that is called whenever a message is received from a beacon.
 * It will use the message to determine the Picker's next destination.
 */
void beaconCallback(const nav_msgs::Odometry msg) {
    destX = msg.pose.pose.position.x;
    destY = msg.pose.pose.position.y;
    
    //check if the current beacon is the new target, if so, signal through the boolean hasNewBeacon
    if (destX != oldDestX || destY != oldDestY) {
        hasNewBeacon = true;            
    } else {
        hasNewBeacon = false;   
    }
    
    //check if the destination has been reached
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
    ROS_INFO("Next beacon x position is: %f", destX);
	ROS_INFO("Next beacon y position is: %f", destY);
}


/*
 * Resubscribe the beacon subscriber to the next beacon after converting the beacon number to a string
 * Assumes the beacon number has already been updated to contain the next destination beacon.
 */
void PickerRobot::subscribeNextBeacon(ros::NodeHandle n) {
    ROS_INFO("I MADE IT HERE TOO");
    std::string currentBeaconS;
    std::stringstream out;
    out << currentBeacon;
    currentBeaconS = out.str();
    beacon_sub = n.subscribe<nav_msgs::Odometry>("/beacon" + currentBeaconS + "/", 1000, beaconCallback);
    //beacon_sub.shutdown();
    //beacon_sub = n.subscribe<nav_msgs::Odometry>("/beacon1/", 1000, beaconCallback);

    atDestX = false;
    atDestY = false;
}

int main(int argc, char **argv)
{
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "PickerRobot");
	
    // convert input parameters for Robot initialization from String to respective types
    std::string xString = argv[1];
    std::string yString = argv[2];    
    std::string pickRangeString = argv[5];
    double xPos = atof(xString.c_str());
    double yPos = atof(yString.c_str());
    double pickRange = (atof(pickRangeString.c_str()))/2+0.1;
    ROS_INFO("x start: %f", xPos);
    ROS_INFO("y start: %f", yPos);
    ROS_INFO("pick range %f",pickRange);
    //assign start and finish beacons from input parameters of launch file
    std::string startString = argv[3];
    std::string finishString = argv[4];
    std::stringstream ss(startString);
    ss >> startBeacon;
    std::stringstream ss2(finishString);
    ss2 >> finishBeacon;
    currentBeacon = startBeacon;
    
    //initialize the Picker robot with the correct position, velocity and state parameters.
	pickerRobot=PickerRobot(xPos,yPos,M_PI/2,0,0,"To next beacon",pickRange);

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
	ros::Subscriber mysub_object = n.subscribe<se306project::carrier_status>("/robot_25/status",1000,recieveCarrierRobotStatus);
    
    // assign beacon subscriber to the first beacon for this Picker robot's path.
    pickerRobot.subscribeNextBeacon(n);
    //beacon_sub.shutdown();

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
        //status_msg.status ="Full";
		status_msg.pos_x=pickerRobot.getX(); //add x to message to broadcast
		status_msg.pos_y=pickerRobot.getY();//add y to message to broadcast
		status_msg.pos_theta=pickerRobot.getTheta(); //add angle to message to broadcast
		status_msg.obstacle = obstacleStatus;
		pub.publish(status_msg);//publish the message for other node
        
        pickerRobot.stateLogic(n);
        
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	return 0;
}
