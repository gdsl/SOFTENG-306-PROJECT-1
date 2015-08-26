#include "GardenWorker.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <time.h>

/**
 * Default constructor for GardenWorker
 */
GardenWorker::GardenWorker():GardenWorker(0,0,0,0,0){}

/**
 * Call super class constructor
 */
GardenWorker::GardenWorker(double x, double y, double theta, double linearVelocity, double angularVelocity) : Person(x, y) {
	// give in nan so that initialX and initialY can be changed in callback
	initialX = std::numeric_limits<double>::quiet_NaN();
	initialY = std::numeric_limits<double>::quiet_NaN();
	targetX = 0;
	targetY = 0;
	closestToWeed = true;
	messagesReceived = 0;
	communicationPartners = 0;
	setStatus("Idle");
	setObstacleStatus("No obstacles");
}

void GardenWorker::weedRemovalRequest(const se306project::weed_status msg) {
	se306project::gardenworker_status pubmsg;
	std::string currentStatus = getStatus();

	if (currentStatus.compare("Idle")==0) {
		targetX = msg.pos_x;
		targetY = msg.pos_y;
		closestToWeed = true;

		if (communicationPartners == 0) {
			next("Move");
		} else {
			next("Communicate");
		}
	}

	int distance = sqrt(pow(targetX-getX(),2.0)+pow(targetY-getY(),2.0));
	pubmsg.distance = distance;
	pubmsg.status = getStatus();

	gardenworker_weedinfo_pub.publish(pubmsg);
}

void GardenWorker::weedRemovalDelegation(const se306project::gardenworker_status msg) {
	std::string currentStatus = getStatus();

	if (currentStatus.compare("Communicating")==0 && msg.status.compare("Communicating")==0) {
		int distance = sqrt(pow(targetX-getX(),2.0)+pow(targetY-getY(),2.0));
		int otherDistance = msg.distance;

		if (otherDistance < distance) {
			closestToWeed = false;
		}
	}

	messagesReceived++;

	if (messagesReceived % communicationPartners == 0) {
		if (currentStatus.compare("Communicating")==0 && closestToWeed) {
			next("Move");
		} else if (currentStatus.compare("Moving")!=0){
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
		} else if (action.compare("Move") == 0) {
			ROS_ERROR("Change");
			setStatus("Moving");
		}
	} else if (currentStatus.compare("Communicating")==0) {
		if (action.compare("Move")==0) {
			setStatus("Moving");
		} else if (action.compare("Stop")==0) {
			setStatus("Idle");
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
		if (action.compare("Move") == 0) {
			setStatus("Moving");
		} else if (action.compare("Stop") == 0) {
			setStatus("Idle");
		}
	}
}

void GardenWorker::stageLaser_callback(const sensor_msgs::LaserScan msg) {
	// 	invoke parent stagelaser
	Person::stageLaser_callback(msg);
	std::string currentStatus = getStatus();
	//ROS_ERROR("Current Status: %s", currentStatus.c_str());
	if (currentStatus.compare("Idle") == 0) {
		pulled = false;
		flushMovementQueue();
	} else if (currentStatus.compare("Moving") == 0) {

		//		if (getAvoidanceCase()!=Entity::NONE) {//check if there is need to avoid obstacle
		//			ROS_ERROR("LOL");
		//			return;
		//		} else {
		//			setObstacleStatus("No obstacles"); //only pick when obstacle detected
		//		}

		//		if (getAvoidanceCase() == Entity::WEED) {
		//			ROS_ERROR("WEED");
		//			//addMovementFront("forward_x",0,0,1);
		//		}

		// check if an obstacle detected. if the obstacle is tallweed, change to pull weed

		double errorMargin = 3;

		if (pulled && abs(getX() - initialX) <= errorMargin && abs(getY() - initialY) <= errorMargin) {
			if (getDirectionFacing() == Entity::NORTH || getDirectionFacing() == Entity::SOUTH) {
				addMovementFront("forward_y",0,0,1);
			} else {
				addMovementFront("forward_x",0,0,1);
			}

			next("Stop");
			return;
		}

		if (!pulled && getAvoidanceCase() == Entity::WEED && abs(getX() - targetX) <= errorMargin && abs(getY() - targetY) <= errorMargin) {
			// start timer
			time(&c_start);
			next("Pull");
			return;
		}

		if (getMovementQueueSize() == 0) {
			double distanceToMove = 0;
			Entity::Direction direction = getDirectionFacing();

			if (getX() != targetX) {
				//check if the Robot needs to go West
				if (getX() > targetX) {
					//calculate the distance to move backwards along X axis
					distanceToMove = -(getX() - targetX);
					//make sure the Robot is facing West, if not, turn it West.
					if (direction != WEST) {
						//ROS_ERROR("CAL1L");
						faceWest(1);
						direction = WEST;
					}
					//otherwise it means the Robot needs to go East
				} else if (getX() < targetX) {
					distanceToMove = targetX - getX();
					//make sure the Robot is facing West, if not, turn it West.
					if (direction != EAST) {
						//ROS_ERROR("EAST");
						faceEast(1);
						direction = EAST;
					}
				}

				addMovement("forward_x", distanceToMove, 1);
			}
			//now add the vertical movement to the movement queue
			if (getY() != targetY) {
				//check if the Robot needs to go South
				if (getY() > targetY) {
					//calculate the distance to move backwards along Y axis
					distanceToMove = -(getY() - targetY);
					//make sure the Robot is facing South, if not, turn it South.
					if (direction != SOUTH) {
						faceSouth(1);
						direction = SOUTH;
					}
					//otherwise it means the Robot needs to go North
				} else if (getY() < targetY) {
					distanceToMove = targetY - getY();
					//make sure the Robot is facing North, if not, turn it North.
					if (direction != NORTH) {
						faceNorth(1);
						direction = NORTH;
					}
				}

				addMovement("forward_y", distanceToMove, 1);
			}
		}

	} else if (currentStatus.compare("Pull Weed") == 0) {

		if (getDirectionFacing() == Entity::NORTH || getDirectionFacing() == Entity::SOUTH) {
			addMovementFront("forward_y",0,0,1);
		} else {
			addMovementFront("forward_x",0,0,1);
		}

		// change to done after certain time
		time(&c_end);

		double diff_t = difftime(c_end, c_start);

		if (diff_t >= 3) {
			next("Finish");
		}

	} else if (currentStatus.compare("Done") == 0) {
		// return back to original

		targetX = initialX;
		targetY = initialY;
		pulled = true;
		flushMovementQueue();
		next("Move");
	}
	move();
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

	// give in initial x, y coordinates
	if (isnan(initialX) && isnan(initialY)) {
		initialX = msg.pose.pose.position.x;
		initialY = msg.pose.pose.position.y;
	}

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
	gardenWorker.gardenworker_weedinfo_pub = n.advertise<se306project::gardenworker_status>("weedinfo",1000);

	// Subscribe to every robot
	std::string start(argv[1]);
	std::string end(argv[2]);
	int s = atoi(start.c_str());
	int e = atoi(end.c_str());

	std::stringstream topicName;
	int index = 0;

	// -1 means no beacons, do not subscribe to robotworkers else subscribe
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
			topicName << "/robot_" << i << "/weedinfo";
			gardenWorker.gardenworker_status_sub[index] = n.subscribe<se306project::gardenworker_status>(topicName.str(),1000,&GardenWorker::weedRemovalDelegation, &gardenWorker);
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
		status_msg.obstacle = gardenWorker.getObstacleStatus();
		// Publish message
		gardenWorker.gardenworker_status_pub.publish(status_msg);
		loop_rate.sleep();
	}
}
