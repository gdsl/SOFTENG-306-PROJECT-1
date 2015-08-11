#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Person.h"
#include "AlphaPerson.h"
#include "se306project/robot_status.h"
 
AlphaPerson::AlphaPerson() : Person() {

}
AlphaPerson::~AlphaPerson() {

}

AlphaPerson alphaPerson;
// Default human behaviour = walking
std::string status="Walking";

void stage_callback(nav_msgs::Odometry msg) {
    alphaPerson.stageOdom_callback(msg);
    //alphaPerson.setPose(x,y,0);

}

/*
 * Method that processes the human message received.
 * This method is called when message is received.
 */
/*void recieveHumanStatus(const se306project::human_status::ConstPtr& msg){
	// Check if human is turning
	if (msg->pos_theta != 0) {
		status = "Turning";
	}
	else {
		status = "Walking";
	}
}*/

int main(int argc, char **argv) 
{
    
    
    //initialise ros    
    ros::init(argc,argv,"AlphaPerson");

    //create ros handler for this node
    ros::NodeHandle n;
    
    alphaPerson.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::Publisher pub = n.advertise<se306project::robot_status>("status_topic",100);

    alphaPerson.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
    srand(time(NULL));
    ros::Rate loop_rate(10); 
    bool targetReach = true;
    double targetX = 1;
    double targetY = 1;



    while (ros::ok())
    {

        loop_rate.sleep();

     alphaPerson.move();
        
     
        alphaPerson.faceWest(1);
        alphaPerson.addMovement("forward_x",-5,1);
        alphaPerson.faceSouth(1);
        alphaPerson.addMovement("forward_y",-5,1);
        alphaPerson.faceEast(1);
        alphaPerson.addMovement("forward_x",5,1);
        alphaPerson.faceNorth(1);
        alphaPerson.addMovement("forward_y",5,1);

        ros::spinOnce();	
	//	se306project::robot_status status_msg;
		//status_msg.status="Hello World";		//add status to message
	//	pub.publish(status_msg);	//publish message
		
    


    }
   
    return 0;
}
