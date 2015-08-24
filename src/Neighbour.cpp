#include "Neighbour.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "se306project/robot_status.h"
#include <math.h>

/**
 * Default constructor for Neighbour
 */
Neighbour::Neighbour():Neighbour(0,0,0,0,0){}

/**
 * Call super class constructor
 */
Neighbour::Neighbour(double x, double y, double theta, double linearVelocity, double angularVelocity)
: Person(x, y)
{
	setStatus("WALKING");
}

/*
 * Update nearest robot
 
void Neighbour::updateNearestRobot(const nav_msgs::Odometry msg)
{
	// Find nearest robot_status
	double robotDistance = sqrt(pow(targetX-getX(),2.0)+pow(targetY-getY(),2.0));
	double msgDistance = sqrt(pow(msg.pose.pose.position.x-getX(),2.0)+pow(msg.pose.pose.position.y-getY(),2.0));

	if (msgDistance<robotDistance) {
		targetX = msg.pose.pose.position.x;
		targetY = msg.pose.pose.position.y;
	}
}*/

/**
 * Represents FSM for Neighbour. Given an action, update the current status
 */
void Neighbour::next(std::string action)
{
	std::string currentStatus = neighbour.getStatus();
        
        //if (currentStatus =="WALKING"){
               //if (
//}
}

/*void stage_laserCallback(sensor_msgs::LaserScan msg) {
  
    neighbour.stageLaser_callback(msg);
        int l=msg.intensities.size();
        double minDist = 1000;
        double robotAngle; 
        //std::vector<double> v;
        //std::map<double,double> Xmap;
        //std::map<double,double> Ymap;
        for (int i = 0; i<l; i++) {
            if (((msg.intensities[i] == 2)||(msg.intensities[i]==3))&&(msg.ranges[i]<minDist)) { 
                        minDistance = msg.ranges[i];
			robotAngle= (i/l) * msg.angle_increment + msg.angle_min;
            }
        }
}*/

/*int Neighbour::getTargetX()
{
	return targetX;
}

int Neighbour::getTargetY()
{
	return targetY;
}*/

void Neighbour::stageOdom_callback(const nav_msgs::Odometry msg)
{
	Person::stageOdom_callback(msg);
}

int main(int argc, char **argv)
{
	//initialise ros
	ros::init(argc,argv,"Neighbour");
	//create ros handler for this node
	ros::NodeHandle n;

	Neighbour Neighbour;
	Neighbour.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	Neighbour.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 1000, &Neighbour::stageOdom_callback, &Neighbour);
	Neighbour.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, &Neighbour::stageLaser_callback, &Neighbour);
	Neighbour.Neighbour_status_pub = n.advertise<se306project::robot_status>("status",1000);

	/*//subscribing all the picker robot
        ros::Subscriber *array = new ros::Subscriber[size];
        int index = 0;
        for (int i = s; i<=e; i++) {
             std::stringstream convert;
             convert << i;
             topicName = "/robot_" + convert.str() + "/status";
             array[index] = n.subscribe<se306project::robot_status>(topicName,1000,recievePickerRobotStatus);
             index++;
        }

         //subscribing all the carrier robot
         ros::Subscriber *carrierArray = new ros::Subscriber[size];
         index = 0;
         for (int i = a; i<=b; i++) {
              std::stringstream convert;
              convert << i;
              topicName = "/robot_" + convert.str() + "/status";
              carrierArray[index] = n.subscribe<se306project::robot_status>(topicName,1000,receiveCarrierRobotStatus);
              index++;
         } */ 

	ros::Rate loop_rate(10);

	se306project::robot_status status_msg;
	//ROS loop
	while (ros::ok())
	{
		ros::spinOnce();
		// publish Neighbour status
		status_msg.pos_x = Neighbour.getX();
		status_msg.pos_y = Neighbour.getY();
		status_msg.pos_theta = Neighbour.getTheta();
		status_msg.status = Neighbour.getStatus();
		Neighbour.Neighbour_status_pub.publish(status_msg);	//publish message

		loop_rate.sleep();
	}
}
