#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "Person.h"
#include "BlindPerson.h"
#include "se306project/robot_status.h"
#include "se306project/human_status.h"
#include "se306project/animal_status.h"
#include <math.h>

BlindPerson::BlindPerson() : Person() {

}

BlindPerson::BlindPerson(double x, double y): Person(x,y) {
}

BlindPerson::~BlindPerson() {

}

BlindPerson blindPerson(-30.00,21.15);

void stage_positionCallback(nav_msgs::Odometry msg) {
	blindPerson.stageOdom_callback(msg);
}
 
double targetX;
double targetY;

void dog_positionCallback(const se306project::animal_status::ConstPtr& msg) {
	targetX = msg->pos_x;
    targetY = msg->pos_y;
}


int main(int argc, char **argv) {


	// Initialise ros
	ros::init(argc,argv,"BlindPerson");

	// Convert input parameters for person initialization from String to respective types
	std::string xString = argv[1];
	std::string yString = argv[2];
    std::string dogString = argv[4];
	double xPos = atof(xString.c_str());
	double yPos = atof(yString.c_str());
	blindPerson = BlindPerson(xPos,yPos);

	// Create ros handler for this node
	ros::NodeHandle n;
	blindPerson.robotNode_stage_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	ros::Publisher pub = n.advertise<se306project::human_status>("status",1000);
	blindPerson.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000,stage_positionCallback);

	std::string topicName = "/robot_" +dogString + "/status";
	ros::Subscriber sub = n.subscribe<se306project::animal_status>(topicName,1000, dog_positionCallback);
	
	ros::Rate loop_rate(10);
	se306project::human_status status_msg;
	blindPerson.setStatus("Following dog");

	while (ros::ok()) {
        //blindPerson.move();
		//blindPerson.faceNorth(1);

        float angleToDog = atan2(blindPerson.getY() - targetY, blindPerson.getX() - targetX);
        float dist = sqrt((blindPerson.getX() - targetX)*(blindPerson.getX() - targetX) + (blindPerson.getY() - targetY)*(blindPerson.getY() - targetY));
        float difference = -1 * (angleToDog - blindPerson.getTheta());
        if (difference < -1*M_PI) { difference += 2*M_PI; }
        else if (difference > M_PI) { difference -= 2*M_PI; }

      //  if (difference > 0) { difference = M_PI - difference; }
      //  else { difference = difference + M_PI; }
      //  ROS_FATAL("theta is %f", blindPerson.getTheta());
        if (dist < 1) {
            dist = 0;
            difference = 0;
        }
		blindPerson.setLin(dist);
		blindPerson.setAng(difference);
        blindPerson.updateOdometry();

        status_msg.pos_x = blindPerson.getX();
		status_msg.pos_y = blindPerson.getY();
		status_msg.pos_theta = blindPerson.getTheta();
		status_msg.status = blindPerson.getStatus();
		pub.publish(status_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}


