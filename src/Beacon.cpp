#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Beacon.h"
 
Beacon::Beacon() : Entity() {
    
}

Beacon::~Beacon() {

}

Beacon beacon;

void stage_callback(nav_msgs::Odometry msg) {
    beacon.stageOdom_callback(msg);
}

int main(int argc, char **argv) 
{
    
    
    //initialise ros    
    ros::init(argc,argv,"beacon");

    //create ros handler for this node
    ros::NodeHandle n;
    
    beacon.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    beacon.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);

    ros::Rate loop_rate(10); 
	
    while (ros::ok())
    {
        
	    ros::spinOnce();
        loop_rate.sleep();
        return 0;
    }
}
