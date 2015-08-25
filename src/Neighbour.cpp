#include "Neighbour.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "se306project/robot_status.h"
#include <math.h>

/**
 * Default constructor for Neighbour
 */
Neighbour::Neighbour():Neighbour(0,0,0,0,0){

}

/**
 * Default constructor for Neighbour
 */
Neighbour::Neighbour(double x, double y):Neighbour(x,y,0,0,0){
       
}

/**
 * Call super class constructor
 */
Neighbour::Neighbour(double x, double y, double theta, double linearVelocity, double angularVelocity) : Person(x, y) {
	

}

Neighbour neighbour;
// Default cat behaviour = walking
std::string status="Idle";
bool queueFull = false;

// Keeps track of current position that cat is facing
double radians;
double angle;
bool initial = true;
/**
 * Represents FSM for Neighbour. Given an action, update the current status
 */
/*void Neighbour::next(std::string action) {
	std::string currentStatus = getStatus();

	//if (currentStatus =="WALKING")ccc{
	//if (
	//}
}*/


void Neighbour::stageOdom_callback(const nav_msgs::Odometry msg) {
	Person::stageOdom_callback(msg);
}

int main(int argc, char **argv) {
	// Initialise ros
	ros::init(argc,argv,"Neighbour");
	// Create ros handler for this node
	ros::NodeHandle n;
	std::string xPosArg = argv[1];
	std::string yPosString = argv[2];
	double yPos = atof(yPosString.c_str());
	double xPos = atof(xPosArg.c_str());
	neighbour=Neighbour(xPos,yPos);

	neighbour.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	neighbour.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 1000, &Neighbour::stageOdom_callback, &neighbour);
	//Neighbour.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, &Neighbour::stageLaser_callback, &Neighbour);
	neighbour.Neighbour_status_pub = n.advertise<se306project::robot_status>("status",1000);


	ros::Rate loop_rate(10);

	se306project::robot_status status_msg;
	// ROS infinite loop
	while (ros::ok()) {
                // Message to stage
		//neighbour.faceWest(1);
                //neighbour.addMovement("forward_x", -35, 1);
                //neighbour.move();
                //neighbour.setStatus("Moving to a robot");

                if (neighbour.getMovementQueueSize() == 0){
                      if(neighbour.getX()>39){
                   neighbour.faceWest(1);
                   neighbour.addMovement("forward_x", -35, 1);
                        }
                else if (neighbour.getX()<8){
                   neighbour.faceNorth(1);
                   neighbour.faceSouth(1);
                   neighbour.faceEast(1);
                   neighbour.addMovement("forward_x", 35, 1);
            }
		
}
		neighbour.move();
		// Publish neighbour status
		status_msg.pos_x = neighbour.getX();
		status_msg.pos_y = neighbour.getY();
		status_msg.pos_theta = neighbour.getTheta();
		status_msg.status = neighbour.getStatus();
		//publish message
		neighbour.Neighbour_status_pub.publish(status_msg);
                ros::spinOnce();
		loop_rate.sleep();
                neighbour.determineStatus();
                if (( neighbour.getStatus()=="Turning")&&(neighbour.getX()<8)){
                    neighbour.setStatus("Observing");
}


               
}
}
