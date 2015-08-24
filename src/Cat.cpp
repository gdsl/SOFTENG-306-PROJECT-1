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

int main(int argc, char **argv) 
{
	std::string xPosArg = argv[1];
    	std::string spacingArg = argv[2];    
	double spacing = atof(spacingArg.c_str());
	double xPos = atof(xPosArg.c_str());

	// Initialize the cat robot with the correct position position
	cat = Cat(xPos, 21.500);   

	ros::init(argc,argv,"Animal");

	// Create ros handler for this node
	ros::NodeHandle n;
    
	cat.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

	cat.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
	ros::Rate loop_rate(10); 
	
	// Broadcast the node's status information for other to subscribe to.
	ros::Publisher pub=n.advertise<se306project::animal_status>("status",1000);
	se306project::animal_status status_msg;
	// 0 rotatiing to the side, 1 moving horizontally, 2 rotating to bnorth/south, 3 moving vertical, 4 stop
	while (ros::ok())
	{
		// Message to stage 
		cat.move();
		
		// Give cat a small initial movement to fill in its GUI status
		if (initial) {
			cat.addMovement("forward_x",-0.1,1);
		}
		initial = false;

		// Generate random number of seconds for cat to sleep between 20-70s
		int num = (rand() % 100) / 2;
		int time = 20 + num;

		if (cat.getMovementQueueSize() == 0) {
			sleep(time);
			cat.faceEast(1);
			cat.addMovement("forward_x",-5,1);
			cat.faceWest(1);
			cat.addMovement("forward_x",5,1);
		}

		if (cat.getMovementQueueSize() == 1) {
			status = "Sleeping";
		}
	        
    		/*if (cat.getMovementQueueSize() == 0) {
			cat.faceWest(1);
			cat.addMovement("forward_x",-5,1);
			cat.faceSouth(1);
            		cat.addMovement("forward_y", -1.25 , 1);  
		    	cat.faceEast(1);
			cat.addMovement("forward_x",5,1);
			cat.faceNorth(1);
			cat.addMovement("forward_y",1.25,1);
		}*/

		// Add Cat variables to status message to be broadcast
		status_msg.status=status;
		status_msg.pos_x=cat.getX();
		status_msg.pos_y=cat.getY();
		status_msg.pos_theta=cat.getAng();
		// Publish status message
		pub.publish(status_msg);
		ros::spinOnce();
	    
		loop_rate.sleep();

		/*// Logic to determine current status of Cat - Walking/Sleeping/Turning
		// Convert radians to degrees
		radians = Cat.getTheta();
		angle = roundf(radians * 57.2957795 * 100) / 100;

		// Check if cat is moving (and therefore 'walking')
		if (Cat.getLin() < -0.01) {
			status = "Walking";
		}
		// Check if cat is facing North/East/South/West AND not moving (and therefore 'sleeping')
		else if ((angle == -360) || (angle == -270) || (angle == -180) || (angle == -90) || (angle == 0) || (angle == 90) || (angle == 180) || (angle == 270) || (angle == 360) && (Cat.getLin() == 0)) {
			status = "Sleeping";
		}
		else {
			status = "Turning";
		}*/
                cat.determineStatus();
	}
	return 0;
}
