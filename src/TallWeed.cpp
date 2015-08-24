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

void TallWeed::update_position() {
	// Generate new random position
	int x = rand() % 82 - 36;
	int y = rand() % 52 - 26;
	ROS_ERROR("MADE IT");
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

    ros::Rate loop_rate(10); 
    while (ros::ok())
    {
	    ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
