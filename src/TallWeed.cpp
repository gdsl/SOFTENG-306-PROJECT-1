#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "TallWeed.h"
 
TallWeed::TallWeed() : Entity() {
    
}

TallWeed::~TallWeed() {

}



TallWeed tallWeed;
ros:: Subscriber worker_sub;

void workerCallback(const nav_msgs::Odometry msg) {
    double destX = msg.pose.pose.position.x;
    double destY = msg.pose.pose.position.y;
    ROS_INFO("Worker x position is: %f", destX);
	ROS_INFO("Worker y position is: %f", destY);
}

void stage_callback(nav_msgs::Odometry msg) {
    tallWeed.stageOdom_callback(msg);
}

int main(int argc, char **argv) 
{
    
    
    //initialise ros    
    ros::init(argc,argv,"tallWeed");

    //create ros handler for this node
    ros::NodeHandle n;
    
    tallWeed.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    ros::Publisher robotNode_location_pub = n.advertise<nav_msgs::Odometry>(argv[1],1000);
    tallWeed.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
    //subscribe to worker
     worker_sub = n.subscribe<nav_msgs::Odometry>(argv[2], 1000, workerCallback);
     //test if argv[2] is really the status of the person
	ROS_INFO("argv[2] is: %s", argv[2]);

    ros::Rate loop_rate(10); 
	nav_msgs::Odometry tempMessage; 
    while (ros::ok())
    {
        tempMessage.pose.pose.position.x = tallWeed.getX();
        tempMessage.pose.pose.position.y = tallWeed.getY();  
        robotNode_location_pub.publish(tempMessage);
    
	    ros::spinOnce();
        loop_rate.sleep();
        
    }
    return 0;
}
