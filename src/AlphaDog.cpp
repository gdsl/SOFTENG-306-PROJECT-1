#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Animal.h"
#include "AlphaDog.h"
#include "se306project/animal_status.h"
#include <stdlib.h>
#include <time.h>
 
AlphaDog::AlphaDog() : Animal() {
    
}

AlphaDog::AlphaDog(double x, double y) : Animal(x,y) {

}

AlphaDog::~AlphaDog() {

}

AlphaDog alphaDog(-3.75,17.5);
// Default dog behaviour = walking
std::string status="Moonwalking";
bool queueFull = false;

// Keeps track of current position that dog is facing
double radians;
double angle;

void stage_callback(nav_msgs::Odometry msg) {
    alphaDog.stageOdom_callback(msg);
}

int main(int argc, char **argv) {
	// Initialise ros    
	ros::init(argc,argv,"Animal");

	// Create ros handler for this node
	ros::NodeHandle n;
    
	alphaDog.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	alphaDog.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
	ros::Rate loop_rate(10); 
	
	// Broadcast the node's status information for other to subscribe to.
	ros::Publisher pub=n.advertise<se306project::animal_status>("status",1000);
	se306project::animal_status status_msg;
	// 0 rotatiing to the side, 1, moving horizontally, 2 rotating to bnorth/south, 3 moving vertical, 4 stop
	while (ros::ok()) {
		// Message to stage 
		alphaDog.move();
        
    		if (alphaDog.getMovementQueueSize() == 0) {
			alphaDog.faceWest(1);
			alphaDog.addMovement("forward_x",-5,1);
			alphaDog.faceSouth(1);
            		alphaDog.addMovement("forward_y", -1.25 , 1);  
		    	alphaDog.faceEast(1);
		    	alphaDog.addMovement("forward_x",5,1);
		    	alphaDog.faceNorth(1);
		   	alphaDog.addMovement("forward_y",1.25,1);
	    	}

		// Add Dog variables to status message to be broadcast
		status_msg.status=status;
		status_msg.pos_x=alphaDog.getX();
		status_msg.pos_y=alphaDog.getY();
		status_msg.pos_theta=alphaDog.getAng();
		// Publish status message
		pub.publish(status_msg);
		ros::spinOnce();
        	loop_rate.sleep();

		/*// Logic to determine current status of Dog - Walking/Idle/Turning
		// Convert radians to degrees
		radians = alphaDog.getTheta();
		angle = roundf(radians * 57.2957795 * 100) / 100;

		// Check if dog is moving (and therefore 'walking')
		if (alphaDog.getLin() > 0.01) {
			status = "Moonwalking";
		}
		// Check if dog is facing North/East/South/West AND not moving (and therefore 'idle')
		else if ((angle == -360) || (angle == -270) || (angle == -180) || (angle == -90) || (angle == 0) || (angle == 90) || (angle == 180) || (angle == 270) || (angle == 360) && (alphaDog.getLin() == 0)) {
			status = "Idle";
		}
		else {
			status = "Turning";
		}*/
                alphaDog.determineStatus();
	}
	return 0;
}
