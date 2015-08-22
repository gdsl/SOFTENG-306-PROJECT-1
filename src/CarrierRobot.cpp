#include "CarrierRobot.h"
#include <ros/console.h>
#include "se306project/carrier_status.h"
#include "se306project/robot_status.h"
#include <utility>
#include <vector>

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
std::string obstacleStatus = "No obstacles";
std::vector<std::pair<double,double> > seenPointList;
/*
 * Wrapper method for the callBackStageOdm method
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	carrierRobot.stageOdom_callback(msg);
}

void callBackLaserScan(const sensor_msgs::LaserScan msg) {
	carrierRobot.stageLaser_callback(msg);

    //detecting carrier in front
    if (carrierRobot.isInitialMovement()) {
        int l=msg.intensities.size();
        carrierRobot.setCarrierInFront(false);
        for (int i = 45; i<l-45; i++) {
            if (msg.intensities[i] == 3) {
                carrierRobot.setCarrierInFront(true);
                break;
            } 
        }
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
			// carrier robot will approach picker but will leave a space to avoid colliding
            if (carrierRobot.isInitialMovement()) {
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


                if ( !(carrierRobot.isCarrierInFront()) && !seen) {
                    carrierRobot.setState(Robot::MOVING);    
                    if (carrierRobot.getMovementQueueSize() <= 1) {
                        
                        carrierRobot.faceEast(1);
                        carrierRobot.addMovement("forward_x",10,1);
                       // double pickerY = msg->pos_y;
                        double pickerY = 8.15;
                        if (pickerY >= carrierRobot.getY() ){
                           carrierRobot.faceNorth(1);
                        } else {
                           carrierRobot.faceSouth(1);
                        }
                        //yDistanceTravel = pickerY - carrierRobot.getY();
                        carrierRobot.setYDistanceTravel(pickerY - carrierRobot.getY());
                        carrierRobot.addMovement("forward_y",carrierRobot.getYDistanceTravel(),1);
                        
                        //double pickerX = msg->pos_x;
                        double pickerX = -5;
                        carrierRobot.faceEast(1);
                        //xDistanceTravel = pickerX - carrierRobot.getX() -10;
                        carrierRobot.setXDistanceTravel(pickerX - carrierRobot.getX() -10);
                        carrierRobot.addMovement("forward_x",carrierRobot.getXDistanceTravel(),1);                                       
                    }
                } else if ( !seen && carrierRobot.isCarrierInFront() ) {
                    seenPointList.push_back(currentPoint);  
                }
            }
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
        carrierRobot.setStatus("Moving");   

        if (carrierRobot.getMovementQueueSize() == 0 ) {
            carrierRobot.setState(Robot::ARRIVED);
        }
    } else if (carrierRobot.getState() == ARRIVED) {
        carrierRobot.setStatus("Arrived");
    } else if (carrierRobot.getState() == TRANSPORTING) {
        carrierRobot.setStatus("Transporting"); 

        if (carrierRobot.getMovementQueueSize() == 0 ) {
            carrierRobot.setState(Robot::QUEUE);
            carrierRobot.faceNorth(1);
            double distance = 24 - carrierRobot.getY();
            carrierRobot.addMovement("forward_y", distance, 1);
        }
    } else if (carrierRobot.getState() == QUEUE) {
        carrierRobot.setStatus("Queueing");
    
         if (carrierRobot.getMovementQueueSize() == 0 && carrierRobot.getY() >= 23.99 ) {
            carrierRobot.setState(Robot::IDLE);
        }
    } 
    
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
    carrierRobot.setState(Robot::IDLE);
    
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

	//a count of how many messages we have sent
	int count = 0;
	//carrier status message initialisation
	se306project::carrier_status status_msg;
    carrierRobot.addMovement("forward_y", 0.25, 1);
    
	//ROS loop
	while (ros::ok())
	{
		ros::spinOnce();
		status_msg.my_counter=count;		//add counter to message
		status_msg.status=carrierRobot.getStatus();		//add status to message
		status_msg.pos_x=carrierRobot.getX(); //add x to message to broadcast
		status_msg.pos_y=carrierRobot.getY();//add y to message to broadcast
		status_msg.pos_theta=carrierRobot.getTheta(); //add angle to message to broadcast
		status_msg.obstacle = obstacleStatus;
		pub.publish(status_msg);	//publish message
		carrierRobot.stateLogic();
        carrierRobot.move();
        if (carrierRobot.getMovementQueueSize() == 0) carrierRobot.setInitialMovement(true);
		loop_rate.sleep();
		++count; // increase counter
	}

    delete[] array;
	return 0;
}
