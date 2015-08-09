#include "CarrierRobot.h"
#include <ros/console.h>
#include "se306project/carrier_status.h"
#include "se306project/robot_status.h"

CarrierRobot::CarrierRobot() {
	// TODO Auto-generated constructor stub

}

CarrierRobot::~CarrierRobot() {
	// TODO Auto-generated destructor stub
}

CarrierRobot carrierRobot;
std::string status="Idle";
/*
 * Wrapper method for the callBackStageOdm method
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	carrierRobot.stageOdom_callback(msg);
}

void recievePickerRobotStatus(const se306project::robot_status::ConstPtr& msg)
{
	//ROS_INFO("sub echoing pub: %d",msg->my_counter);
	//ROS_INFO("sub echoing pub: %s",msg->status.c_str());
	//ROS_INFO("sub echoing pub: %f",msg->pos_x);
	if ((msg->status).compare("Full")==0){
		ROS_INFO("Tests");
		if (!carrierRobot.getDesireLocation()){
			carrierRobot.moveForward(double(msg->pos_x),1);
		}else{
			status="Arrived";
		}
	}else if ((msg->status).compare("Moving")==0){
		status="Idle";
		carrierRobot.setDesireLocation(false);
	}
}

int main(int argc, char **argv)
{
	carrierRobot=CarrierRobot();
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "CarrierRobot");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;


	ros::Rate loop_rate(10);
	// tell master you want to sub to topic
	carrierRobot.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
		//subscribe to listen to messages coming from stage
	carrierRobot.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	ros::Subscriber mysub_object = n.subscribe<se306project::robot_status>("/robot_0/status",1000,recievePickerRobotStatus);
	ros::Publisher pub=n.advertise<se306project::carrier_status>("status",1000);


	//a count of howmany messages we have sent
	int count = 0;
	se306project::carrier_status status_msg;

	while (ros::ok())
	{
		status_msg.my_counter=count;
		status_msg.status=status;
		pub.publish(status_msg);
		ros::spinOnce();
		loop_rate.sleep();

		++count;
	}

	return 0;

}
