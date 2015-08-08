#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Entity.h"
#include "Animal.h"
 
Animal::Animal() : Entity() {

}
Animal::~Animal() {

}

Animal alphaDog;

void stage_callback(nav_msgs::Odometry msg) {
    alphaDog.stageOdom_callback(msg);
    int x = msg.pose.pose.position.x;
	int y = msg.pose.pose.position.y;
    alphaDog.setPose(x,y,0);

}


int main(int argc, char **argv) 
{
    
    
    //initialise ros    
    ros::init(argc,argv,"Animal");

    //create ros handler for this node
    ros::NodeHandle n;
    
    alphaDog.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  

    alphaDog.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("odom",1000,stage_callback);
    
    ros::Rate loop_rate(10); 

    while (ros::ok())
    {
        //message to stage
        alphaDog.setVelocity(0.2,0.2);
        alphaDog.updateOdometry();
        

        ros::spinOnce();
    
        loop_rate.sleep();

    }
   
    return 0;
}
