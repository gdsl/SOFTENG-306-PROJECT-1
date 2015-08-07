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
geometry_msgs::Twist RobotNode_cmdvel;

void stage_callback(nav_msgs::Odometry msg) {
    alphaDog.stageOdom_callback(msg);
}


int main(int argc, char **argv) 
{
    
    
    //initialise ros    
    ros::init(argc,argv,"Animal");

    //create ros handler for this node
    ros::NodeHandle n;
    
  //  alphaDog.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    //    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);


//    alphaDog.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("alphaDog/odom",1000,stage_callback);
    
    ros::Rate loop_rate(10); 

    while (ros::ok())
    {
        //message to stage
  //      alphaDog.setVelocity(0.2,0.2);
        //alphaDog.updateOdometry();
        
        RobotNode_cmdvel.linear.x = 1000000000;
        RobotNode_cmdvel.linear.y = 0;
        RobotNode_cmdvel.angular.z = 1000000000;
       // alphaDog.robotNode_stage_pub.publish(RobotNode_cmdvel);
        RobotNode_stage_pub.publish(RobotNode_cmdvel);
       // ros::spinOnce();
    
        loop_rate.sleep();

    }
   
    return 0;
}
