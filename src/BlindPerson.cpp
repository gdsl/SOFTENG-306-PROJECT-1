#include "BlindPerson.h"
#include "Person.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "se306project/blindperson_status.h"
#include <math.h>

BlindPerson blindPerson;

BlindPerson::BlindPerson(){
}

BlindPerson::BlindPerson(double x, double y): Person(x,y) {
}

BlindPerson::~BlindPerson(){
}

void BlindPerson::followDog(){
}

void stage_positionCallback(nav_msgs::Odometry msg) {
    blindPerson.stageOdom_callback(msg);
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"BlindPerson");
	ros::NodeHandle n;
    
    //blindPerson.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	//ros::Publisher pub = n.advertise<se306project::human_status>("status",1000);
    
    //blindPerson.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_positionCallback);
    
    //se306project::blindperson_status status_msg;
    
    ros::Rate loop_rate(10);
    
    while(ros::ok) {        
        blindPerson.addMovement("forward_x",0.75,1);
        blindPerson.move();
    }
    
    return 0;
}
