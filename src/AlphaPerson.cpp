#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Person.h"
#include "AlphaPerson.h"
#include "se306project/robot_status.h"
#include "se306project/human_status.h"
#include <math.h>
 
AlphaPerson::AlphaPerson() : Person() {

}
AlphaPerson::~AlphaPerson() {

}

AlphaPerson alphaPerson;
// Default human behaviour = walking
std::string status="Walking";

// Keeps track of current position that human is facing
double radians;
double angle;

void stage_callback(nav_msgs::Odometry msg) {
    alphaPerson.stageOdom_callback(msg);
    //alphaPerson.setPose(x,y,0);

}

int main(int argc, char **argv) 
{
    
    
    //initialise ros    
    ros::init(argc,argv,"AlphaPerson");

    //create ros handler for this node
    ros::NodeHandle n;
    
    alphaPerson.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::Publisher pub = n.advertise<se306project::human_status>("status",100);

    alphaPerson.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
    srand(time(NULL));
    ros::Rate loop_rate(10); 
    bool targetReach = true;
    double targetX = 1;
    double targetY = 1;
    bool once = true;
    int state = 0;
	int count = 0;
	se306project::human_status status_msg;

    while (ros::ok())
    { 
     alphaPerson.move();
     if (alphaPerson.getMovementQueueSize() == 0 && state == 0) {

            alphaPerson.faceEast(1);
            if (once ) {
                alphaPerson.addMovement("forward_x", 17.5, 1);
                once = false;
            }
            else {
                alphaPerson.addMovement("forward_x", 15, 1);
            }
            alphaPerson.faceSouth(1);
            alphaPerson.faceEast(1);
            alphaPerson.addMovement("forward_x", 15, 1);
            alphaPerson.faceSouth(1);
            state = 1;
            
	} else if (alphaPerson.getMovementQueueSize() == 0 && state == 1) {
        //returning         
            alphaPerson.faceWest(1);
            alphaPerson.addMovement("forward_x", -30, 1 );
            alphaPerson.faceSouth(1);
            state = 0;
	} 
    
	//assign to status message
	status_msg.my_counter = count++;//add counter to message to broadcast
	status_msg.status=status;//add status to message to broadcast
	status_msg.pos_x=alphaPerson.getX(); //add x to message to broadcast
	status_msg.pos_y=alphaPerson.getY();//add y to message to broadcast
	status_msg.pos_theta=alphaPerson.getTheta(); //add angle to message to broadcast
	pub.publish(status_msg);//publish the message for other node
        
        ros::spinOnce();
        loop_rate.sleep();

	// ******** MOVE THIS FUNCTION TO ENTITY - REFACTOR **************
	// Logic to determine current status of Human - Walking/Idle/Turning
	// Convert radians to degrees
	radians = alphaPerson.getTheta();
	angle = roundf(radians * 57.2957795 * 100) / 100;
	// Check if human is moving (and therefore 'walking')
	if (alphaPerson.getLin() > 0.01) {
		status = "Walking";
	}
	// Check if human is facing North/East/South/West AND not moving (and therefore 'idle')
	else if ((angle == -360) || (angle == -270) || (angle == -180) || (angle == -90) || (angle == 0) || (angle == 90) || (angle == 180) || (angle == 270) || (angle == 360) && (alphaPerson.getLin() == 0)) {
		status = "Idle";
	}
	else {
		status = "Turning";
	}
    }
    return 0;
}
