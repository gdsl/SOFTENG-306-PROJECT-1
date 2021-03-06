#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Animal.h"
#include "Cat.h"
#include "se306project/animal_status.h"
#include <stdlib.h>
#include <time.h>

Cat::Cat() : Animal() {

}

Cat::Cat(double x, double y) : Animal(x,y) {

}

Cat::~Cat() {

}

Cat cat;

// Default cat behaviour = walking
std::string status="Walking";
bool queueFull = false;

// Keeps track of current position that cat is facing
double radians;
double angle;
bool initial = true;

void stage_callback(nav_msgs::Odometry msg) {
	cat.stageOdom_callback(msg);
}

/**
 * Call back method for laser work out the avoidance logic for Cat
 */
void callBackLaserScan(const sensor_msgs::LaserScan msg) {
	cat.stageLaser_callback(msg);//call supercalss laser call back for detection case work out

	if (cat.getAvoidanceCase()!=Entity::NONE&&!cat.isRotating()) {//check if there is need to avoid obstacle
		if(cat.getCriticalIntensity()!=2&&cat.getAvoidanceQueueSize()==0&&cat.getObstacleStatus().compare("Obstacle nearby")!=0){
			cat.setObstacleStatus("Obstacle nearby");
			cat.avoidObstacle(2.5,0.5);//call avoid obstacle method in entity to avoid obstacle
		}else if (cat.getMinDistance()<0.7&&cat.getCriticalIntensity()>=1&&cat.getAvoidanceQueueSize()>0){
			cat.addMovementFront("forward_x",0,0,1);//halt movement if already have obstacle
		}
		cat.move();
	}else{
		cat.setObstacleStatus("No obstacles");
	}
}

int main(int argc, char **argv) 
{
	// Initialize the cat robot with the correct position position

	ros::init(argc,argv,"Animal");
	std::string xPosArg = argv[1];
	std::string spacingArg = argv[2];
	double spacing = atof(spacingArg.c_str());
	double xPos = atof(xPosArg.c_str());
	cat = Cat(xPos, 21.500);

	// Create ros handler for this node
	ros::NodeHandle n;

	cat.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

	cat.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
	ros::Rate loop_rate(10); 

	//subscribe to laser for cats
	cat.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000,callBackLaserScan);

	// Broadcast the node's status information for other to subscribe to.
	ros::Publisher pub=n.advertise<se306project::animal_status>("status",1000);
	se306project::animal_status status_msg;
	// 0 rotatiing to the side, 1 moving horizontally, 2 rotating to bnorth/south, 3 moving vertical, 4 stop
	while (ros::ok())
	{
		// Message to stage 
		if(cat.getObstacleStatus().compare("Obstacle nearby")!=0){
			cat.move();
		}
		// Give cat a small initial movement to fill in its GUI status
		if (initial) {
			cat.addMovement("forward_x",0.1,1);
		}
		initial = false;

		// Generate random number of seconds for cat to sleep between 50-100s
		double num = (rand() % 100) / 2;
		double time = 50 + num;

		if (cat.getMovementQueueSize() == 0) {
			cat.addMovement("forward_x", 0.1, (0.1 / time));
			cat.faceEast(1);
			cat.addMovement("forward_x",5,1);
			cat.faceWest(1);
			cat.addMovement("forward_x",-5,1);
		}

		// Add Cat variables to status message to be broadcast
		status_msg.status=status;
		status_msg.pos_x=cat.getX();
		status_msg.pos_y=cat.getY();
		status_msg.pos_theta=cat.getAng();
		status_msg.obstacle=cat.getObstacleStatus();
		// Publish status message
		pub.publish(status_msg);
		ros::spinOnce();
		loop_rate.sleep();
		// Determine status of cat
		cat.determineStatus();
		//if ( cat.getLin() == 0.01 ) {
		//	status = "Sleeping";
		//}
	}
	return 0;
}
