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

AlphaDog alphaDog(0,0);
// Default dog behaviour = walking
std::string status="Walking";
bool antiClockwise = false;

// Keeps track of current position that dog is facing
double radians;
double angle;

void stage_callback(nav_msgs::Odometry msg) {
    alphaDog.stageOdom_callback(msg);
}

int main(int argc, char **argv) {
	// Initialise ros    
	ros::init(argc,argv,"Animal");
    
    // convert input parameters for person initialization from String to respective types
    std::string xString = argv[1];
    std::string yString = argv[2];
    std::string rowString = argv[3];
    std::string spacingString = argv[4];
    double rowWidth = atof(rowString.c_str());
    double trunkSpacing = atof(spacingString.c_str());
    double xPos = atof(xString.c_str());
    double yPos = atof(yString.c_str());
    double left = xPos - trunkSpacing;
    double right = xPos;
    double top = yPos;
    double bottom = yPos - rowWidth;
    alphaDog = AlphaDog(xPos,yPos);

	// Create ros handler for this node
	ros::NodeHandle n;
    
	alphaDog.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	alphaDog.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
	ros::Rate loop_rate(10); 
	
	// Broadcast the node's status information for other to subscribe to.
	ros::Publisher pub=n.advertise<se306project::animal_status>("status",1000);
	se306project::animal_status status_msg;
	// 0 down section, 1 up section, 2 left section, 3 right section
    AlphaDog::State state = AlphaDog::TOP;
	while (ros::ok()) {
		// Message to stage 
		alphaDog.move();
        
        if (alphaDog.getMovementQueueSize() == 0) {
            antiClockwise = !antiClockwise;
            if (state == AlphaDog::TOP) {
                if (antiClockwise) {
                    alphaDog.faceWest(1);
                    alphaDog.addMovement("forward_x", left-alphaDog.getX(), 1);
                    state = AlphaDog::LEFT;
                } else {
                    alphaDog.faceEast(1);           
                    alphaDog.addMovement("forward_x", right-alphaDog.getX(), 1);
                    state = AlphaDog::RIGHT;
                }
            } else if (state == AlphaDog::LEFT)  {
                if (antiClockwise) {
                    alphaDog.faceSouth(1);
                    alphaDog.addMovement("forward_y", bottom-alphaDog.getY(),1);
                    state = AlphaDog::BOTTOM;
                } else {
                    alphaDog.faceNorth(1);           
                    alphaDog.addMovement("forward_y", top-alphaDog.getY(), 1);
                    state = AlphaDog::TOP;
                }
            } else if (state == AlphaDog::BOTTOM) {
                if (antiClockwise) {
                    alphaDog.faceEast(1);           
                    alphaDog.addMovement("forward_x", right-alphaDog.getX(), 1);
                    state = AlphaDog::RIGHT;
                } else {
                    alphaDog.faceWest(1);
                    alphaDog.addMovement("forward_x", left-alphaDog.getX(), 1);
                    state = AlphaDog::LEFT;
                }
            } else if (state == AlphaDog::RIGHT) {
                if (antiClockwise) {
                    alphaDog.faceNorth(1);           
                    alphaDog.addMovement("forward_y", top-alphaDog.getY(), 1);
                    state = AlphaDog::TOP;
                } else {
                    alphaDog.faceSouth(1);
                    alphaDog.addMovement("forward_y", bottom-alphaDog.getY(),1);
                    state = AlphaDog::BOTTOM;
                }
            }
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
