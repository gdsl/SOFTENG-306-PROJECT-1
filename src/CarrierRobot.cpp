#include "CarrierRobot.h"
#include <ros/console.h>
#include "se306project/carrier_status.h"
#include "se306project/robot_status.h"

/*
 * Default constructor for carrier Robot
 */
CarrierRobot::CarrierRobot() {
	// TODO Auto-generated constructor stub

}

/*
 * Default destructor for carrier Robot
 */
CarrierRobot::~CarrierRobot() {
	// TODO Auto-generated destructor stub
}

//create carrier robot and status
CarrierRobot carrierRobot;
std::string status="Idle";

/*
 * Wrapper method for the callBackStageOdm method
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	carrierRobot.stageOdom_callback(msg);
}

/*
 * Method that process the picker robot message received.
 * This method is called when message is received.
 */
void recievePickerRobotStatus(const se306project::robot_status::ConstPtr& msg)
{
	//ROS_INFO("sub echoing pub: %d",msg->my_counter);
	//ROS_INFO("sub echoing pub: %s",msg->status.c_str());
	//ROS_INFO("sub echoing pub: %f",msg->pos_x);

	//Check the status of Carrier robot so see how it should act
	//when status is arrived it means that the carrier robot has arrived at picker
	if (status.compare("Arrived")==0){
		//Change status to transporting as the carrier robot is now taking away the full bin
		status="Transporting";
		carrierRobot.setDesireLocation(false);//refresh that it can recieve more desire location
	}else if(status.compare("Idle")==0){
		//when the carrier robot is idle and the picker robot is full the carrier robot move to it.
		if ((msg->status).compare("Full")==0){
			if (!carrierRobot.getDesireLocation()){
				// carrier robot will approach picker but will leave a space to avoid colliding
				carrierRobot.moveForward(double((msg->pos_x)+3), 1);
			}else{
				//if carrier robot is at picker robot change status to Arrived
				status="Arrived";
			}
		}
	}else if(status.compare("Transporting")==0){
		//if the carrier is transporting it will move to bin drop off area (the driveway)
		if (!carrierRobot.getDesireLocation()){
			//carrier will move to right 10meters to imaginery dump place
			carrierRobot.moveForward(10, 1);
		}else{
			status="Idle"; //when carrier robot complete transporting full bin to driveway it
			//become Idle again (free)
			carrierRobot.setDesireLocation(false);//refresh that it can recieve more desire location
		}

	}


}

int main(int argc, char **argv)
{
	//initialise carrierRobot so method can be invoke on it
	carrierRobot=CarrierRobot();

	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "CarrierRobot");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;


	ros::Rate loop_rate(10);
	//Broadcast the node's velocity information for other and stage to subscribe to.
	carrierRobot.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	//Broadcast the node's status information for other to subscribe to.
	ros::Publisher pub=n.advertise<se306project::carrier_status>("status",1000);

	//subscribe to listen to messages coming from stage for odometry (this is base pose so it is
	//relative to the absolute frame of the farm.
	carrierRobot.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	//subscribe to the status of picker robot
	ros::Subscriber mysub_object = n.subscribe<se306project::robot_status>("/robot_0/status",1000,recievePickerRobotStatus);


	//a count of how many messages we have sent
	int count = 0;
	//carrier status message initialisation
	se306project::carrier_status status_msg;

	//ROS loop
	while (ros::ok())
	{
		status_msg.my_counter=count;		//add counter to message
		status_msg.status=status;		//add status to message
		pub.publish(status_msg);	//publish message
		ros::spinOnce();
		loop_rate.sleep();

		++count; // increase counter
	}

	return 0;

}
