#include "ros/ros.h"
#include "Robot.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

Robot::Robot(){
}

Robot::~Robot(){
}

//message from stage
Robot::stageOdom_callback(nav_msgs::Odometry msg){
	double x=msg.pose.pose.position.x;
	double y=msg.pose.pose.position.y;
}

//message to stage of robot's odometry
Robot::updateOdometry(){
	//not moving
	robotNode_cmdvel.linear.x=0;
	robotNode_cmdvel.angular.z = 0;
	
	robotNode_stage_pub.publish(robotNode_cmdvel);
}

//movement towards a point
Robot::moveTo(geometry_msgs::Point point){
	
}
