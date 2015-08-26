#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "se306project/robot_status.h"
#include <sstream>
#include "TallWeed.h"

TallWeed::TallWeed() : Entity() {

}

TallWeed::~TallWeed() {

}

/**
 * Move the weed underground
 */
void TallWeed::update_position() {
	// recalculate weed position

	int targetX = 50;
	int targetY = 20;

	flushMovementQueue();
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
}

void TallWeed::workerCallback(const se306project::robot_status msg) {
	std::string status = msg.status;
	double destX = msg.pos_x;
	double destY = msg.pos_y;

	if (status.compare("Done") == 0) {
		double distance = sqrt(pow(destX-getX(),2.0)+pow(destY-getY(),2.0));

		if (distance <= NEARBYDISTANCE) {
			update_position();
		}

	}
}

void TallWeed::stageOdom_callback(nav_msgs::Odometry msg) {
	Entity::stageOdom_callback(msg);

	if (getAvoidanceCase()!=Entity::NONE&&!isRotating()) {//check if there is need to avoid obstacle
		if(getCriticalIntensity()!=9&&getCriticalIntensity()!=2&&getAvoidanceQueueSize()==0&&getObstacleStatus().compare("Obstacle nearby")!=0){
			setObstacleStatus("Obstacle nearby");
			avoidObstacle(3,0.5);//call avoid obstacle method in entity to avoid obstacle
		}else if (getMinDistance()<0.7&&getCriticalIntensity()>1&&getAvoidanceQueueSize()>0){
			addMovementFront("forward_x",0,0,1);//halt movement if already have obstacle
		}
	}else{
		setObstacleStatus("No obstacles");
	}
	move();
}

int main(int argc, char **argv) {
	// Initialise ros
	ros::init(argc,argv,"tallWeed");
	// Create ros handler for this node
	ros::NodeHandle n;
	TallWeed tallWeed;
	tallWeed.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	tallWeed.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,&TallWeed::stageOdom_callback,&tallWeed);

	// Subscribe to worker
	std::string start(argv[4]);
	std::string end(argv[5]);
	int s = atoi(start.c_str());
	int e = atoi(end.c_str());

	if (!(s == -1 && e == -1)) {
		int size = e-s+1;
		std::stringstream topicName;

		tallWeed.workerSubscribers = new ros::Subscriber[size];
		int index = 0;
		for (int i = s; i < e+1; i++) {
			topicName.str(std::string());
			topicName << "/robot_" << i << "/status";
			tallWeed.workerSubscribers[index] = n.subscribe<se306project::robot_status>(topicName.str(),1000,&TallWeed::workerCallback,&tallWeed);
			index++;
		}
	}
	int count=0;
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		if (count==50){
			tallWeed.addMovementFront("forward_z",-5,-1,2);
			//tallWeed.move();
		}
		tallWeed.move();

		loop_rate.sleep();
		count=count+1;
	}
	return 0;
}
