#include "GardenWorker.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

/**
 * Default constructor for GardenWorker
 */
GardenWorker::GardenWorker():GardenWorker(0,0,0,0,0){}

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
	//Person::stageLaser_callback(msg);
	//ROS_ERROR("TESTING");
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
	//Person::stageOdom_callback(msg);
}

int main(int argc, char **argv)
{
	//initialise ros
	ros::init(argc,argv,"GardenWorker");
	//create ros handler for this node
	ros::NodeHandle n;

	// argv[1] = topic name
	// argv[2] = tallweed node start index
	// argv[3] = tallweed node end index

	GardenWorker gardenWorker;
	gardenWorker.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	gardenWorker.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 1000, &GardenWorker::stageOdom_callback, &gardenWorker);
	gardenWorker.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, &GardenWorker::stageLaser_callback, &gardenWorker);
	gardenWorker.gardenworker_status_pub = n.advertise<se306project::gardenworker_status>(argv[1],1000);

	// subscribe to every tallweed
	std::string start(argv[2]);
	std::string end(argv[3]);
	int s = atoi(start.c_str());
	int e = atoi(end.c_str());

	// -1 means no beacons, do not subscribe to weed else subscribe
	if (!(s == -1 && e == -1)) {
		int size = e-s+1;
		std::stringstream topicName;

		gardenWorker.tallweed_pose_sub = new ros::Subscriber[size];

		int index = 0;
		for (int i = s; i < e+1; i++) {
			//reset stringstream
			topicName.str(std::string());
			// give in topicname
			topicName << "/robot_" << i << "/base_pose_ground_truth";
			gardenWorker.tallweed_pose_sub[index] = n.subscribe<nav_msgs::Odometry>(topicName.str(),1000,&GardenWorker::updateNearestWeed, &gardenWorker);
			index++;
		}

	}

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
