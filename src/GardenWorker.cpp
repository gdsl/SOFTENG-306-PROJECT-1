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
GardenWorker::GardenWorker(double x, double y, double theta, double linearVelocity, double angularVelocity) : Person(x, y) {
	initialX = x;
	initialY = y;
	targetX = 0;
	targetY = 0;
	closestToWeed = true;
	messagesReceived = 0;
	communicationPartners = 0;
	setStatus("Idle");
}

void GardenWorker::weedRemovalRequest(const se306project::weed_status msg) {
	std::string currentStatus = getStatus();
	if (currentStatus.compare("Idle")==0) {
		targetX = msg.pos_x;
		targetY = msg.pos_y;
		closestToWeed = true;
		next("Communicate");
	}
}

void GardenWorker::weedRemovalDelegation(const se306project::robot_status msg) {
	std::string currentStatus = getStatus();

	if (currentStatus.compare("Communicating")==0 && msg.status.compare("Communicating")==0) {
		int distance = sqrt(pow(targetX-getX(),2.0)+pow(targetY-getY(),2.0));
		int otherDistance = sqrt(pow(targetX-msg.pos_x,2.0)+pow(targetY-msg.pos_y,2.0));

		if (otherDistance < distance) {
			closestToWeed = false;
		}

		messagesReceived++;
	}

	if (messagesReceived % communicationPartners == 0) {
		if (closestToWeed) {
			next("Move");
		} else {
			next("Stop");
		}
	}

}

/**
 * Represents FSM for GardenWorker - given an action, update the current status
 */
void GardenWorker::next(std::string action) {
	std::string currentStatus = getStatus();
	if (currentStatus.compare("Idle")==0) {
		if (action.compare("Communicate")==0) {
			setStatus("Communicating");
		}
	} else if (currentStatus.compare("Communicating")) {
		if (action.compare("Move")==0) {
			setStatus("Moving");
		} else if (action.compare("Stop")==0) {
			setStatus("Idle");
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

void GardenWorker::stageLaser_callback(const sensor_msgs::LaserScan msg) {
	// 	invoke parent stagelaser
	//	Person::stageLaser_callback(msg);
	//	std::string currentStatus = getStatus();
	//	if (currentStatus.compare("Idle")==0) {
	//		if (getAvoidanceCase()!=Entity::NONE) {//check if there is need to avoid obstacle
	//			ROS_ERROR("LOL");
	//		} else {
	//			// check which orientation
	//			if (getDirectionFacing()== Entity::NORTH) {
	//				if (targetY > getY()) {
	//					// do nothing
	//				} else {
	//
	//				}
	//			} else if (getDirectionFacing() == Entity::EAST) {
	//
	//			} else if (getDirectionFacing() == Entity::SOUTH) {
	//
	//			} else {
	//
	//			}
	//			ROS_ERROR("Current x %f", getX());
	//			ROS_ERROR("Current y %f", getY());
	//			ROS_ERROR("Target x %d", targetX);
	//			ROS_ERROR("Target y %d", targetY);
	//		}
	//		next("Move");
	//	}
}

int GardenWorker::getTargetX() {
	return targetX;
}

int GardenWorker::getTargetY() {
	return targetY;
}

void GardenWorker::setCommunicationPartners(int communicationPartners) {
	this->communicationPartners = communicationPartners;
}

void GardenWorker::stageOdom_callback(const nav_msgs::Odometry msg) {
	Person::stageOdom_callback(msg);
}

int main(int argc, char **argv) {
	// Initialise ros
	ros::init(argc,argv,"GardenWorker");
	// Create ros handler for this node
	ros::NodeHandle n;

	// argv[1] = robot node start index
	// argv[2] = robot node end index
	// argv[3] = gardenworker node start index
	// argv[4] = gardenworker node end index
	// argv[5] = my node index

	GardenWorker gardenWorker;
	gardenWorker.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	gardenWorker.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 1000, &GardenWorker::stageOdom_callback, &gardenWorker);
	gardenWorker.baseScan_Sub = n.subscribe<sensor_msgs::LaserScan>("base_scan", 1000, &GardenWorker::stageLaser_callback, &gardenWorker);
	gardenWorker.gardenworker_status_pub = n.advertise<se306project::robot_status>("status",1000);

	// Subscribe to every robot
	std::string start(argv[1]);
	std::string end(argv[2]);
	int s = atoi(start.c_str());
	int e = atoi(end.c_str());

	std::stringstream topicName;
	int index = 0;

	// -1 means no beacons, do not subscribe to weed else subscribe
	if (!(s == -1 && e == -1)) {
		int size = e-s+1;

		gardenWorker.tallweed_pose_sub = new ros::Subscriber[size];

		for (int i = s; i < e+1; i++) {
			//reset stringstream
			topicName.str(std::string());
			// give in topicname
			topicName << "/robot_" << i << "/weed";
			gardenWorker.tallweed_pose_sub[index] = n.subscribe<se306project::weed_status>(topicName.str(),1000,&GardenWorker::weedRemovalRequest, &gardenWorker);
			index++;
		}
	}

	// subscribe to every other gardenworker except yourself
	std::string gw_start(argv[3]);
	std::string gw_end(argv[4]);
	std::string gw_index(argv[5]);
	int gw_s = atoi(gw_start.c_str());
	int gw_e = atoi(gw_end.c_str());
	int gw_i = atoi(gw_index.c_str());

	int gw_size = gw_e - gw_s + 1;
	gardenWorker.gardenworker_status_sub = new ros::Subscriber[gw_size];

	// set number of communication partners
	gardenWorker.setCommunicationPartners(gw_e - gw_s);

	// reset topicName and index
	topicName.str(std::string());
	index = 0;

	// subscribe to every gardenworker except yourself
	for (int i = gw_s; i < gw_e+1; i++) {
		if (i != gw_i) {
			topicName.str(std::string());
			topicName << "/robot_" << i << "/status";
			gardenWorker.gardenworker_status_sub[index] = n.subscribe<se306project::robot_status>(topicName.str(),1000,&GardenWorker::weedRemovalDelegation, &gardenWorker);
			index++;
		}
	}

	ros::Rate loop_rate(10);
	se306project::robot_status status_msg;
	// ROS infinite loop
	while (ros::ok())
	{
		ros::spinOnce();
		// Publish garden worker status
		status_msg.pos_x = gardenWorker.getX();
		status_msg.pos_y = gardenWorker.getY();
		status_msg.pos_theta = gardenWorker.getTheta();
		status_msg.status = gardenWorker.getStatus();
		// Publish message
		gardenWorker.gardenworker_status_pub.publish(status_msg);
		loop_rate.sleep();
	}
}
