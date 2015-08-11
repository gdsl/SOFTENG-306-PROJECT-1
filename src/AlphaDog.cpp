#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Animal.h"
#include "AlphaDog.h"
#include <stdlib.h>
#include <time.h>
 
AlphaDog::AlphaDog() : Animal() {

}
AlphaDog::~AlphaDog() {

}

AlphaDog alphaDog;

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
  

    alphaDog.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_callback);
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
         alphaDog.move();
        
     
        alphaDog.faceWest(1);
        alphaDog.addMovement("forward_x",-5,1);
        alphaDog.faceSouth(1);
        alphaDog.addMovement("forward_y",-5,1);
        alphaDog.faceEast(1);
        alphaDog.addMovement("forward_x",5,1);
        alphaDog.faceNorth(1);
        alphaDog.addMovement("forward_y",5,1);
    
        ros::spinOnce();
    
        loop_rate.sleep();

    }
   
    return 0;
}
