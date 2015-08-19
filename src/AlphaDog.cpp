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
AlphaDog::~AlphaDog() {

}

AlphaDog alphaDog;
// Default dog behaviour = walking
std::string status="Moonwalking";
bool queueFull = false;

// Keeps track of current position that dog is facing
double radians;
double angle;

void stage_callback(nav_msgs::Odometry msg) {
    alphaDog.stageOdom_callback(msg);
    //alphaDog.setPose(x,y,0);

}

int main(int argc, char **argv) 
{
    
    
    //initialise ros    
    ros::init(argc,argv,"Animal");

    //create ros handler for this node
    ros::NodeHandle n;
    
    alphaDog.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  

    alphaDog.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
    ros::Rate loop_rate(10); 
    bool onceX = true;
    bool onceY = true;
	
    //Broadcast the node's status information for other to subscribe to.
    ros::Publisher pub=n.advertise<se306project::animal_status>("status",1000);
    se306project::animal_status status_msg;
    //0 rotatiing to the side, 1, moving horizontally, 2 rotating to bnorth/south, 3 moving vertical, 4 stop
    while (ros::ok())
    {
        //message to stage 
        alphaDog.move();
        
    	if (alphaDog.getMovementQueueSize() == 0) {
		    alphaDog.faceWest(1);
            if (onceX ) {
			    alphaDog.addMovement("forward_x",-8.25,1);
                onceX = false;
            } else {
                alphaDog.addMovement("forward_x",-5,1);
            }
			alphaDog.faceSouth(1);
            if (onceY) {
			    alphaDog.addMovement("forward_y",16.25,1);
                onceY = false;
            } else {
                alphaDog.addMovement("forward_y", -1.25 , 1);
            } 
		alphaDog.faceEast(1);
		alphaDog.addMovement("forward_x",5,1);
		alphaDog.faceNorth(1);
		alphaDog.addMovement("forward_y",1.25,1);
	}
	//status_msg.my_counter=count;		//add counter to message
	status_msg.status=status;		//add status to message
	status_msg.pos_x=alphaDog.getX(); //add x to message to broadcast
	status_msg.pos_y=alphaDog.getY();//add y to message to broadcast
	status_msg.pos_theta=alphaDog.getAng(); //add angle to message to broadcast
	//status_msg.obstacle = obstacleStatus;
	pub.publish(status_msg);	//publish message
	ros::spinOnce();
    
        loop_rate.sleep();

	// ******** MOVE THIS FUNCTION TO ENTITY - REFACTOR **************
	// Logic to determine current status of Human - Walking/Idle/Turning
	// Convert radians to degrees
	radians = alphaDog.getTheta();
	angle = roundf(radians * 57.2957795 * 100) / 100;
	// Check if human is moving (and therefore 'walking')
	if (alphaDog.getLin() > 0.01) {
		status = "Moonwalking";
	}
	// Check if human is facing North/East/South/West AND not moving (and therefore 'idle')
	else if ((angle == -360) || (angle == -270) || (angle == -180) || (angle == -90) || (angle == 0) || (angle == 90) || (angle == 180) || (angle == 270) || (angle == 360) && (alphaDog.getLin() == 0)) {
		status = "Idle";
	}
	else {
		status = "Turning";
	}
    }
    return 0;
}
