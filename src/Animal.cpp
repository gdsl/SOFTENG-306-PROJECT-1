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
    //alphaDog.setPose(x,y,0);

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
        //alphaDog.setVelocity(0,0.2);

        double targetX = 20;
        double targetY = 20;

        double diffX = alphaDog.x - targetX;
        double diffY = alphaDog.y - targetY;

        if (abs(diffX) > 0.5 ) {
            if (diffX > 0) {
                alphaDog.turnWest(2.0);
                
            } else {
                alphaDog.turnWest(2.0);
            }
            if (alphaDog.angularVelocity == 0 ) {
                alphaDog.moveForward(2.0);
            }

        ) else if (abs(diffY) > 0.5) {
            if (diffY > 0) {
                alphaDog.turnSouth(2.0);
                
            } else {
                alphaDog.turnNorth(2.0);
            }
            if (alphaDog.angularVelocity == 0 ) {
                alphaDog.moveForward(2.0);
            }
        } else {
            alphaDog.setVelocity(0,0);
        }
        
        alphaDog.updateOdometry();
        

        ros::spinOnce();
    
        loop_rate.sleep();

    }
   
    return 0;
}
