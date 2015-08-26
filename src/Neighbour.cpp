#include "Neighbour.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "se306project/human_status.h"
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
// Default cat behaviour = Idle
std::string status="Idle";


void Neighbour::stageOdom_callback(const nav_msgs::Odometry msg) {
	Person::stageOdom_callback(msg);
}

/**
 * Getter method for the origin x  of the picker robot
 */
double Neighbour::getOriginX() {
	return originXPos;
}

/**
 * Setter method for the origin x of the picker robot
 */
void Neighbour::setOriginX(double originXPos) {
	this->originXPos=originXPos;
}

/**
 * Getter method for the origin y  of the picker robot
 */
double Neighbour::getOriginY() {
	return originYPos;
}

/**
 * Setter method for the origin x of the picker robot
 */
void Neighbour::setOriginY(double originYPos) {
	this->originYPos=originYPos;
}
/**
 * Call back method for laser work out the avoidance logic for neighbour
 */
void callBackLaserScan(const sensor_msgs::LaserScan msg) {
	neighbour.stageLaser_callback(msg);//call supercalss laser call back for detection case work out

	if (neighbour.getAvoidanceCase()!=Entity::NONE&&!neighbour.isRotating()) {//check if there is need to avoid obstacle
		if (neighbour.getCriticalIntensity()==2&&neighbour.getAvoidanceQueueSize()==0&&neighbour.getObstacleStatus().compare("Obstacle nearby")!=0){ //if sense picker robot run away
			neighbour.setObstacleStatus("Obstacle nearby");			
			neighbour.addMovementFront("forward_x",0,0,1);//halt current movement
			neighbour.move();
			neighbour.flushMovementQueue();
			neighbour.addMovementFront("rotation",0,1,1);
			neighbour.addMovement("forward_x", neighbour.getOriginX()-neighbour.getX(), 1);
			neighbour.setStatus("Moving back");
		}else if(neighbour.getCriticalIntensity()!=2&&neighbour.getAvoidanceQueueSize()==0&&neighbour.getObstacleStatus().compare("Obstacle nearby")!=0){
			neighbour.setObstacleStatus("Obstacle nearby");
			neighbour.avoidObstacle(2.5,0.5);//call avoid obstacle method in entity to avoid obstacle
		}else if (neighbour.getMinDistance()<0.7&&neighbour.getCriticalIntensity()>=1&&neighbour.getAvoidanceQueueSize()>0){
			neighbour.addMovementFront("forward_x",0,0,1);//halt movement if already have obstacle
		}
		neighbour.move();
	}else{
		neighbour.setObstacleStatus("No obstacles");
	}
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
	neighbour.setOriginY(yPos);
	neighbour.setOriginX(xPos);
	neighbour.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	neighbour.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 1000, &Neighbour::stageOdom_callback, &neighbour);
	neighbour.Neighbour_status_pub = n.advertise<se306project::human_status>("status",1000);
	//subscribe to laser
	neighbour.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000,callBackLaserScan);

	ros::Rate loop_rate(10);

        //Add initial movement
	se306project::human_status status_msg;
	neighbour.setStatus("Finding a robot");
	neighbour.faceWest(1);
	neighbour.addMovement("forward_x", -35, 1);
	// ROS infinite loop
	while (ros::ok()) {

                ros::spinOnce();
		loop_rate.sleep();

                //determine the logic of neighbour movement 
		if (neighbour.getMovementQueueSize() == 0){
			if(neighbour.getStatus().compare("Finding a robot")==0){
				neighbour.setStatus("Moving back");
				neighbour.faceEast(1);
                neighbour.addMovement("forward_x", neighbour.getOriginX()-neighbour.getX(), 1);
			}else{
				neighbour.setStatus("Finding a robot");
				neighbour.faceWest(1);
				neighbour.addMovement("forward_x", -35, 1);
			}
		}
		if(neighbour.getObstacleStatus().compare("Obstacle nearby")!=0){
			neighbour.move();
		}
		// Publish neighbour status
		status_msg.pos_x = neighbour.getX();
		status_msg.pos_y = neighbour.getY();
		status_msg.pos_theta = neighbour.getTheta();
		status_msg.status = neighbour.getStatus();
		neighbour.Neighbour_status_pub.publish(status_msg);             
	}
}
