#include "CarrierRobot.h"
#include <ros/console.h>
#include "se306project/carrier_status.h"
#include "se306project/robot_status.h"
#include <utility>
#include <vector>

/*
 * Default constructor for carrier Robot
 */
CarrierRobot::CarrierRobot() {
	// TODO Auto-generated constructor stub
}

/*
 * Constructor for carrier Robot with status
 */
CarrierRobot::CarrierRobot(double x,double y,double theta,double linearVel, double angularVel,std::string status)
	:Robot( x, y, theta, linearVel,  angularVel){
	this->setStatus(status);
    this->setState(IDLE);
}

/*
 * Default destructor for carrier Robot
 */
CarrierRobot::~CarrierRobot() {
	// TODO Auto-generated destructor stub
}

//create carrier robot and status
CarrierRobot carrierRobot;
//std::string status="Idle";
std::string previousStatus = "Idle";
std::string obstacleStatus = "No obstacles";
bool carrierInFront = true;
bool initialMovement = false;
double yDistanceTravel = 0;
double xDistanceTravel = 0;
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
    if (initialMovement) {
        int l=msg.intensities.size();
        carrierInFront = false;
        for (int i = 45; i<l-45; i++) {
            if (msg.intensities[i] == 3) {
                carrierInFront = true;
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
            carrierRobot.addMovement("forward_x", -1*xDistanceTravel,1);
            if (yDistanceTravel >= 0 ) {
                carrierRobot.faceSouth(1);
            }
            else {
                carrierRobot.faceNorth(1);
            }
            carrierRobot.addMovement("forward_y", -1*yDistanceTravel,1);
            carrierRobot.faceWest(1);
            carrierRobot.addMovement("forward_x", -10,1);
        }
	}else if(carrierRobot.getStatus().compare("Idle")==0){
		//when the carrier robot is idle and the picker robot is full the carrier robot move to it.
		if ((msg->status).compare("Full") == 0){
			// carrier robot will approach picker but will leave a space to avoid colliding
            if (initialMovement) {
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


                if ( !carrierInFront && !seen) {
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
                        yDistanceTravel = pickerY - carrierRobot.getY();
                        carrierRobot.addMovement("forward_y",yDistanceTravel,1);
                        
                        //double pickerX = msg->pos_x;
                        double pickerX = -5;
                        carrierRobot.faceEast(1);
                        xDistanceTravel = pickerX - carrierRobot.getX() -10;
                        carrierRobot.addMovement("forward_x",xDistanceTravel,1);                                       
                    }
                } else if (!seen && carrierInFront) {
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
            carrierRobot.setState(Robot::RETURN);
            carrierRobot.faceNorth(1);
        }
    } else if (carrierRobot.getState() == RETURN) {
        carrierRobot.setStatus("Returning");
    
         if (carrierRobot.getMovementQueueSize() == 0 ) {
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
	carrierRobot=CarrierRobot(xPos,yPos,0,0,0,"Idle");
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
    //ros::Subscriber mysub_object = n.subscribe<se306project::robot_status>("/robot_0/status",1000,recievePickerRobotStatus);
    std::string number(argv[3]);
    std::string topicName = "/robot_" + number + "/status";
    ros::Subscriber mysub_object = n.subscribe<se306project::robot_status>(topicName,1000,recievePickerRobotStatus);

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
        if (carrierRobot.getMovementQueueSize() == 0) initialMovement = true; 
		loop_rate.sleep();
		++count; // increase counter
	}
	return 0;
}

