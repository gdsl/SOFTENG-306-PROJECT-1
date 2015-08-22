#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "se306project/gardenworker_status.h"
#include <sstream>
#include "TallWeed.h"
 
TallWeed::TallWeed() : Entity() {}

TallWeed::~TallWeed() {

}

void TallWeed::workerCallback(const nav_msgs::Odometry msg) {
    double destX = msg.pose.pose.position.x;
    double destY = msg.pose.pose.position.y;
    ROS_INFO("Worker x position is: %f", destX);
	ROS_INFO("Worker y position is: %f", destY);
}

void TallWeed::stageOdom_callback(nav_msgs::Odometry msg) {
    Entity::stageOdom_callback(msg);
}

int main(int argc, char **argv) 
{
    //initialise ros    
    ros::init(argc,argv,"tallWeed");

    //create ros handler for this node
    ros::NodeHandle n;
    
    TallWeed tallWeed;

    tallWeed.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
    tallWeed.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,&TallWeed::stageOdom_callback,&tallWeed);

    //subscribe to worker
    std::string start(argv[2]);
    std::string end(argv[3]);
    int s = atoi(start.c_str());
    int e = atoi(end.c_str());
    int size = e-s+1;
    std::string topicName;

    tallWeed.workerSubscribers = new ros::Subscriber[size];

    int index = 0;
    for (int i = s; s < e+1; i++) {
		topicName = "/robot_" + i + "/base_pose_ground_truth";
		tallWeed.workerSubscribers[index] = n.subscribe<nav_msgs::Odometry>(topicName,1000,&TallWeed::workerCallback,&tallWeed);
		index++;
    }

    //test if argv[2] is really the status of the person
	//ROS_INFO("argv[2] is: %s", argv[2]);

    ros::Rate loop_rate(10); 
	//nav_msgs::Odometry tempMessage;
    while (ros::ok())
    {
	    ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
