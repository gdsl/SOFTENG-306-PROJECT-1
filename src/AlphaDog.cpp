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
    this->antiClockwise = false;
}

AlphaDog::~AlphaDog() {

}

void AlphaDog::switchDirection() {
    this->antiClockwise = !(this->antiClockwise);
}

bool AlphaDog::isAntiClockwise() {
    return this-> antiClockwise;
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

/**
 * Call back method for laser work out the avoidance logic for dog robot
 */
void callBackLaserScan(const sensor_msgs::LaserScan msg) {
	alphaDog.stageLaser_callback(msg);//call supercalss laser call back
	if (alphaDog.getAvoidanceCase()!=Entity::NONE) {//check if there is need to avoid obstacle
		alphaDog.setObstacleStatus("Obstacles nearby");
		//flush its movement
		alphaDog.flushMovementQueue();
		alphaDog.addMovementFront("forward_x",0,0,1);//this is at front of front
		alphaDog.move();

	} else {
		alphaDog.setObstacleStatus("No obstacles");
	}
}

int main(int argc, char **argv) {
	// Initialise ros    
	ros::init(argc,argv,"Animal");
    
    // convert input parameters for person initialization from String to respective types
    std::string xString = argv[1];
    std::string yString = argv[2];
    std::string rowString = argv[4];
    std::string spacingString = argv[5];
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
	alphaDog.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, callBackLaserScan);

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
           if (state == AlphaDog::TOP) {
                if (alphaDog.isAntiClockwise()) {
                    alphaDog.faceWest(1);
                    alphaDog.addMovement("forward_x", left-alphaDog.getX(), 1);
                    state = AlphaDog::LEFT;
                } else {
                    alphaDog.faceEast(1);           
                    alphaDog.addMovement("forward_x", right-alphaDog.getX(), 1);
                    state = AlphaDog::RIGHT;
                }
            } else if (state == AlphaDog::LEFT)  {
                if (alphaDog.isAntiClockwise()) {
                    alphaDog.faceSouth(1);
                    alphaDog.addMovement("forward_y", bottom-alphaDog.getY(),1);
                    state = AlphaDog::BOTTOM;
                } else {
                    alphaDog.faceNorth(1);           
                    alphaDog.addMovement("forward_y", top-alphaDog.getY(), 1);
                    state = AlphaDog::TOP;
                }
            } else if (state == AlphaDog::BOTTOM) {
                if (alphaDog.isAntiClockwise()) {
                    alphaDog.faceEast(1);           
                    alphaDog.addMovement("forward_x", right-alphaDog.getX(), 1);
                    state = AlphaDog::RIGHT;
                } else {
                    alphaDog.faceWest(1);
                    alphaDog.addMovement("forward_x", left-alphaDog.getX(), 1);
                    state = AlphaDog::LEFT;
                }
            } else if (state == AlphaDog::RIGHT) {
                if (alphaDog.isAntiClockwise()) {
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
                alphaDog.determineStatus();
	}
	return 0;
}
