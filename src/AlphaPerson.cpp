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
	ros::Publisher pub = n.advertise<se306project::robot_status>("status_topic",100);

    alphaPerson.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("odom",1000,stage_callback);
    srand(time(NULL));
    ros::Rate loop_rate(10); 
    bool targetReach = true;
    double targetX = 1;
    double targetY = 1;



    while (ros::ok())
    {

        loop_rate.sleep();

        //message to stage
        alphaPerson.setVelocity(0,0.2);
        alphaPerson.updateOdometry();

//ros::spinOnce();	

		se306project::robot_status status_msg;
		status_msg.status="Hello World";		//add status to message
		pub.publish(status_msg);	//publish message
		
    


    }
   
    return 0;
}
