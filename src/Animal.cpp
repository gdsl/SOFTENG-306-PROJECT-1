#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Entity.h"
#include "Animal.h"
#include <stdlib.h>
#include <time.h>
 
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
    srand(time(NULL));
    ros::Rate loop_rate(10); 
    bool targetReach = true;
    int range = 10;
    double targetX =( rand() % (2*range+1)) + (-range);
    double targetY =( rand() % (2*range+1) )+ (-range);
    double errors = 0.25;



    while (ros::ok())
    {
        //message to stage
        //alphaDog.setVelocity(0,0.2);
        
       double dist = sqrt((pow((alphaDog.getX()-targetX),2) + pow((alphaDog.getY()-targetY),2)));
        if (dist < errors+0.25) targetReach = true;
        
        if ( alphaDog.getLin() == 0 && alphaDog.getAng() == 0 ) {
            targetReach = true;
        }
        if (targetReach) {
            targetX =( rand() % (2*range+1)) + (-range);
            targetY =( rand() % (2*range+1) )+ (-range);
            targetReach = false;
        }
    
       // debugMsg.linear.x = targetX;
        //debugMsg.linear.y = targetY;
        //debug_pub.publish(debugMsg);

        double diffX = alphaDog.getX() - targetX;
        double diffY = alphaDog.getY() - targetY;
        
        if (abs(diffX) > errors ) {
            if (diffX > 0) {
                alphaDog.faceWest(2.0);
                if (alphaDog.getAng() == 0 ) {
                    alphaDog.addMovement("forward_x",diffX,-2.0);
                } 
                
            } else {
                alphaDog.faceEast(2.0);
                if (alphaDog.getAng() == 0 ) {
                    alphaDog.addMovement("forward_x",diffX,2.0);
                } 
            }

            


        } else if (abs(diffY) > errors) {

            if (diffY > 0) {
                alphaDog.faceSouth(2.0);
                if (alphaDog.getAng() == 0 ) {
                    alphaDog.addMovement("forward_y",diffY,-2.0);
                }
                
            } else {
                alphaDog.faceNorth(2.0);
                if (alphaDog.getAng() == 0 ) {
                    alphaDog.addMovement("forward_y",diffY,2.0);
                }
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
