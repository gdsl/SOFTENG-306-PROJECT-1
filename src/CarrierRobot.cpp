#include "CarrierRobot.h"
#include <ros/console.h>
#include "se306project/carrier_status.h"
#include "se306project/robot_status.h"

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
/*
 * Wrapper method for the callBackStageOdm method
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	carrierRobot.stageOdom_callback(msg);
}

void callBackLaserScan(const sensor_msgs::LaserScan msg) {
	carrierRobot.stageLaser_callback(msg);

	if (carrierRobot.getMinDistance() < 1) {

		if(carrierRobot.getCriticalIntensity()>=4){//if its human or dog stop
			carrierRobot.addMovementFront("forward_x",0,0,1);
			//carrierRobot.move();
		}else{
			if(carrierRobot.getAvoidanceQueueSize()<=0){
				if(carrierRobot.getDirectionFacing()== carrierRobot.NORTH&&obstacleStatus.compare("Obstacle nearby")!=0){
					carrierRobot.addMovementFront("rotation",M_PI/2,1,1);
					carrierRobot.addMovementFront("forward_x",3,1,1);
					carrierRobot.addMovementFront("rotation",0, 1,1);
					carrierRobot.addMovementFront("forward_y",3,1,1);
					carrierRobot.addMovementFront("rotation",M_PI/2,1,1);
					carrierRobot.addMovementFront("forward_x",-3,1,1);
					carrierRobot.addMovementFront("rotation",M_PI,1,1);
					carrierRobot.addMovementFront("forward_x",0,0,1);//this is at front of front
					//carrierRobot.move();
				}
				if(carrierRobot.getDirectionFacing()== carrierRobot.EAST&&obstacleStatus.compare("Obstacle nearby")!=0){
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
				carrierRobot.move();
			}
		}
		//halt movement if already have avoidance logic
		obstacleStatus = "Obstacle nearby";
	} else {
		obstacleStatus = "No obstacles";
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
		carrierRobot.setStatus("Transporting");
		carrierRobot.faceWest(1);
		carrierRobot.addMovement("forward_x",-34.5, 1);
		carrierRobot.faceNorth(1);
		carrierRobot.addMovement("forward_y",std::abs(15-carrierRobot.getY()),1);
		//carrierRobot.setDesireLocation(false);//refresh that it can recieve more desire location
	}else if(carrierRobot.getStatus().compare("Idle")==0){
		//when the carrier robot is idle and the picker robot is full the carrier robot move to it.
		if ((msg->status).compare("Full")==0){
			// carrier robot will approach picker but will leave a space to avoid colliding
			carrierRobot.faceSouth(1);
			carrierRobot.addMovement("forward_y",-std::abs(double((msg->pos_y)-carrierRobot.getY())),1);
			carrierRobot.faceEast(1);
			carrierRobot.addMovement("forward_x",double((msg->pos_x)-carrierRobot.getX()-3), 1);
			carrierRobot.setStatus("Moving");
		}
	}else if (carrierRobot.getStatus().compare("Obstacle nearby") == 0) {

	}
}
/**
 * Method for the carrier robot's states transition and implementation
 */
void CarrierRobot::stateLogic(){
	if(carrierRobot.getStatus().compare("Transporting")==0){
		//if the carrier is in transporting state move
		//if the carrier is transporting it will move to bin drop off area (the driveway)
		if(carrierRobot.getMovementQueueSize()<1){
			carrierRobot.setStatus("Idle"); //when carrier robot complete transporting full bin to driveway it
			//become Idle again (free)
			carrierRobot.setDesireLocation(false);//refresh that it can recieve more desire location
		}
		carrierRobot.move();
	}else if(carrierRobot.getStatus().compare("Moving")==0){
		//check if the robot has anymore movement in queue if not set state to arrive
		if(carrierRobot.getMovementQueueSize()<1){
			//carrierRobot.setStatus("Arrived");
		}
		//if the carrier is in moving state, move
		//if (obstacleStatus.compare("Obstacle nearby")!=0){
		carrierRobot.move();
		//}
	}else if (obstacleStatus.compare("Obstacle nearby")==0){
		carrierRobot.setStatus("Moving");
		carrierRobot.move();
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
    //subscribe to the status of picker robot
	ros::Subscriber mysub_object = n.subscribe<se306project::robot_status>("/robot_0/status",1000,recievePickerRobotStatus);


	//a count of how many messages we have sent
	int count = 0;
	//carrier status message initialisation
	se306project::carrier_status status_msg;

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
		loop_rate.sleep();
		++count; // increase counter
	}
	return 0;
}

