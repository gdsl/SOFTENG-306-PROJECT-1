#include "GardenWorker.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

/**
 * Default constructor for GardenWorker
 */
GardenWorker::GardenWorker():Person()
{
	targetX = 0;
	targetY = 0;
	weedCounter = 0;
	setStatus("Idle");
}

/**
 * Call super class constructor
 */
GardenWorker::GardenWorker(double x, double y, double theta, double linearVelocity, double angularVelocity)
: Person(x, y)
{
	targetX = 0;
	targetY = 0;
	weedCounter = 0;
	setStatus("Idle");
}

/**
 * Update the number of weed pulled by garden worker
 */
void GardenWorker::increment()
{
	weedCounter++;
}

/**
 * Update nearest
 */
void GardenWorker::updateNearestWeed(const nav_msgs::Odometry msg)
{
	// Find nearest weed_status
	double weedDistance = sqrt(pow(targetX-getX(),2.0)+pow(targetY-getY(),2.0));
	double msgDistance = sqrt(pow(msg.pose.pose.position.x-getX(),2.0)+pow(msg.pose.pose.position.y-getY(),2.0));

	if (msgDistance<weedDistance) {
		targetX = msg.pose.pose.position.x;
		targetY = msg.pose.pose.position.y;
	}
}

/**
 * Represents FSM for GardenWorker. Given an action, update the current status
 */
void GardenWorker::next(std::string action)
{
	std::string currentStatus = getStatus();
	if (currentStatus.compare("Idle")==0) {
		if (action.compare("Move")==0) {
			setStatus("Moving");
		} else if (action.compare("Pull")==0) {
			setStatus("Pull Weed");
		}
	} else if (currentStatus.compare("Moving")==0) {
		if (action.compare("Pull")==0) {
			setStatus("Pull Weed");
		} else if (action.compare("Stop")==0) {
			setStatus("Idle");
		}
	} else if (currentStatus.compare("Pull Weed")==0) {
		if (action.compare("Finish")==0) {
			setStatus("Done");
		}
	} else if (currentStatus.compare("Done")==0) {
		setStatus("Idle");
	}
}

void GardenWorker::stageLaser_callback(const sensor_msgs::LaserScan msg)
{
	// invoke parent stagelaser
	Person::stageLaser_callback(msg);
	ROS_ERROR("TESTING");
}

uint GardenWorker::getWeedCounter()
{
	return weedCounter;
}

int GardenWorker::getTargetX()
{
	return targetX;
}

int GardenWorker::getTargetY()
{
	return targetY;
}

void GardenWorker::stageOdom_callback(const nav_msgs::Odometry msg)
{
	Person::stageOdom_callback(msg);
}

int main(int argc, char **argv)
{
	//initialise ros
	ros::init(argc,argv,"GardenWorker");
	//create ros handler for this node
	ros::NodeHandle n;

	GardenWorker gardenWorker(0,0,0,0,0);
	gardenWorker.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	gardenWorker.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 1000, &GardenWorker::stageOdom_callback, &gardenWorker);
	gardenWorker.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, &GardenWorker::stageLaser_callback, &gardenWorker);
	gardenWorker.tallweed_pose_sub = n.subscribe<nav_msgs::Odometry>("weed",1000, &GardenWorker::updateNearestWeed, &gardenWorker);
	gardenWorker.gardenworker_status_pub = n.advertise<se306project::gardenworker_status>("gardenworker",1000);

	ros::Rate loop_rate(10);

	se306project::gardenworker_status status_msg;
	//ROS loop
	while (ros::ok())
	{
		ros::spinOnce();
		status_msg.pos_x = gardenWorker.getX();
		status_msg.pos_y = gardenWorker.getY();
		status_msg.pos_theta = gardenWorker.getTheta();
		status_msg.weed_counter = gardenWorker.getWeedCounter();
		status_msg.status = gardenWorker.getStatus();
		gardenWorker.gardenworker_status_pub.publish(status_msg);	//publish message
		loop_rate.sleep();
	}
}
