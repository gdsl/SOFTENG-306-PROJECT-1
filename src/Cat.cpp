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

Cat Cat(-3.75,17.5);
// Default dog behaviour = walking
std::string status="Moonwalking";
bool queueFull = false;

// Keeps track of current position that dog is facing
double radians;
double angle;

void stage_callback(nav_msgs::Odometry msg) {
    Cat.stageOdom_callback(msg);
    //Cat.setPose(x,y,0);

}

int main(int argc, char **argv) 
{
    
    //initialise ros    
    ros::init(argc,argv,"Animal");

    //create ros handler for this node
    ros::NodeHandle n;
    
    Cat.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  

    Cat.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
    ros::Rate loop_rate(10); 
	
    //Broadcast the node's status information for other to subscribe to.
    ros::Publisher pub=n.advertise<se306project::animal_status>("status",1000);
    se306project::animal_status status_msg;
    //0 rotatiing to the side, 1, moving horizontally, 2 rotating to bnorth/south, 3 moving vertical, 4 stop
    while (ros::ok())
    {
        //message to stage 
        Cat.move();
        
    	if (Cat.getMovementQueueSize() == 0) {
		    Cat.faceWest(1);
            Cat.addMovement("forward_x",-5,1);
			Cat.faceSouth(1);
            Cat.addMovement("forward_y", -1.25 , 1);  
		    Cat.faceEast(1);
		    Cat.addMovement("forward_x",5,1);
		    Cat.faceNorth(1);
		    Cat.addMovement("forward_y",1.25,1);
	    }
	//status_msg.my_counter=count;		//add counter to message
	status_msg.status=status;		//add status to message
	status_msg.pos_x=Cat.getX(); //add x to message to broadcast
	status_msg.pos_y=Cat.getY();//add y to message to broadcast
	status_msg.pos_theta=Cat.getAng(); //add angle to message to broadcast
	//status_msg.obstacle = obstacleStatus;
	pub.publish(status_msg);	//publish message
	ros::spinOnce();
    
        loop_rate.sleep();

	// ******** MOVE THIS FUNCTION TO ENTITY - REFACTOR **************
	// Logic to determine current status of Human - Walking/Idle/Turning
	// Convert radians to degrees
	radians = Cat.getTheta();
	angle = roundf(radians * 57.2957795 * 100) / 100;
	// Check if human is moving (and therefore 'walking')
	if (Cat.getLin() > 0.01) {
		status = "Moonwalking";
	}
	// Check if human is facing North/East/South/West AND not moving (and therefore 'idle')
	else if ((angle == -360) || (angle == -270) || (angle == -180) || (angle == -90) || (angle == 0) || (angle == 90) || (angle == 180) || (angle == 270) || (angle == 360) && (Cat.getLin() == 0)) {
		status = "Idle";
	}
	else {
		status = "Turning";
	}
    }
    return 0;
}
