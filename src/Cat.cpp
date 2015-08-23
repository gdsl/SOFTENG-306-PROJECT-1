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

Cat Cat(-20.000, 21.500);
// Default cat behaviour = walking
std::string status="Walking";
bool queueFull = false;

// Keeps track of current position that cat is facing
double radians;
double angle;
bool initial = true;

void stage_callback(nav_msgs::Odometry msg) {
    Cat.stageOdom_callback(msg);
}

int main(int argc, char **argv) 
{
    
	ros::init(argc,argv,"Animal");

	// Create ros handler for this node
	ros::NodeHandle n;
    
	Cat.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

	Cat.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
	ros::Rate loop_rate(10); 
	
	// Broadcast the node's status information for other to subscribe to.
	ros::Publisher pub=n.advertise<se306project::animal_status>("status",1000);
	se306project::animal_status status_msg;
	// 0 rotatiing to the side, 1 moving horizontally, 2 rotating to bnorth/south, 3 moving vertical, 4 stop
	while (ros::ok())
	{
		// Message to stage 
		Cat.move();
		
		// Give cat a small initial movement to fill in its GUI status
		if (initial) {
			Cat.addMovement("forward_x",-0.1,1);
		}
		initial = false;

		// Generate random number of seconds for cat to sleep between 30-50s
		int num = (rand() % 100) / 5;
		int time = 30 + num;

		if (Cat.getMovementQueueSize() == 0) {
			sleep(5);
			Cat.faceEast(1);
			Cat.addMovement("forward_x",-5,1);
			Cat.faceWest(1);
			Cat.addMovement("forward_x",5,1);
		}
	        
    		/*if (Cat.getMovementQueueSize() == 0) {
			Cat.faceWest(1);
			Cat.addMovement("forward_x",-5,1);
			Cat.faceSouth(1);
            		Cat.addMovement("forward_y", -1.25 , 1);  
		    	Cat.faceEast(1);
			Cat.addMovement("forward_x",5,1);
			Cat.faceNorth(1);
			Cat.addMovement("forward_y",1.25,1);
		}*/

		// Add Cat variables to status message to be broadcast
		status_msg.status=status;
		status_msg.pos_x=Cat.getX();
		status_msg.pos_y=Cat.getY();
		status_msg.pos_theta=Cat.getAng();
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
                Cat.determineStatus();
	}
	return 0;
}
