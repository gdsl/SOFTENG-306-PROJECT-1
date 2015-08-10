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
    int range = 5;
    //double targetX =( rand() % (2*range+1)) + (-range);
    //double targetY =( rand() % (2*range+1) )+ (-range);
    double targetX = -3;
    double targetY = -3;
    double errors = 0.25;
    double state = 0;
    int horizontalSign = 1;
    int verticalSign = 1;
   
    //0 rotatiing to the side, 1, moving horizontally, 2 rotating to bnorth/south, 3 moving vertical, 4 stop
    while (ros::ok())
    {
        //message to stage 
        double diffX = alphaDog.getX() - targetX;
        double diffY = alphaDog.getY() - targetY;
        
        alphaDog.setVelocity(0,0);
        if (state == 0 ) {
               if (diffX > 0 ) {
                    alphaDog.faceWest(2.0);
                    horizontalSign = 1;
               } else {
                    alphaDog.faceEast(2.0);
                    horizontalSign = -1;
               }
                
               if (alphaDog.getAng() == 0) {
                    state = 1;
                }
        } else if (state == 1 ) {
            //alphaDog.setDesireLocation(false);
            //alphaDog.moveForward(targetX, horizontalSign*0.5);   
            alphaDog.addMovement("forward_x",targetX,horizontalSign*0.5);
            if (alphaDog.getLin() == 0) {
                state = 2;
            }
        } else if (state == 2) {
            if (diffY > 0 ) {
                    alphaDog.faceSouth(2.0);
                    verticalSign = 1;
               } else {
                    alphaDog.faceNorth(2.0);
                    verticalSign = -1;
               }
                
               if (alphaDog.getAng() == 0) {
                    state = 3;
                }
        } else if (state == 3) {
            //alphaDog.setDesireLocation(false);
            //alphaDog.moveForward(targetY,2.0);   
            alphaDog.addMovement("foward_y",targetY,verticalSign*0.5);
            if (alphaDog.getLin() == 0) {
                state =4;
            }
            alphaDog.updateOdometry();
        } else if (state == 4) {
            targetX =( rand() % (2*range+1)) + (-range);
            targetY =( rand() % (2*range+1) )+ (-range);
            state = 0;  
            alphaDog.setVelocity(0,0); 
            alphaDog.updateOdometry();
        } 

        ros::spinOnce();
    
        loop_rate.sleep();

    }
   
    return 0;
}
