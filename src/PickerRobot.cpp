#include "ros/ros.h"
#include <sstream>
#include "Robot.h"
#include "se306project/robot_status.h"
#include "se306project/carrier_status.h"
#include "PickerRobot.h"

PickerRobot::PickerRobot():Robot(){

}

PickerRobot::~PickerRobot(){
}
PickerRobot pickerRobot;
std::string status="Moving";
double distance=1;

/*
 * Wrapper method for the callBackStageOdm method (in Entity)
 */
void callBackStageOdm(const nav_msgs::Odometry msg){
	pickerRobot.stageOdom_callback(msg);
}

/*
 * Method that process the carrier robot message received.
 * This method is called when message is received.
 */
void recieveCarrierRobotStatus(const se306project::carrier_status::ConstPtr& msg){
	if (msg->status.compare("Transporting")==0&&status.compare("Full")==0){
		if (distance==1){
			distance=5;
		}else if (distance ==5){
			distance=1;
		}
		pickerRobot.movement();
		pickerRobot.setDesireLocation(false);
		status="Moving";
	}
}

/*
 * Method for the logic of PickerRobot running its movement queue.
 */
void PickerRobot::movement(){
	//If status is not full the picker robot will keep moving
	/*if(status.compare("Full")!=0){
		if (pickerRobot.getDesireLocation()){
			//if picker robot is at desire location set status to full
			status="Full";
		}else{
			//if picker robot is not at desire location keep moving
			//pickerRobot.addMovement("forward_x",1,1);
		}

	}*/
	//pickerRobot.moveForward(distance,1);
	if (distance==1){
		pickerRobot.faceEast(1);
		pickerRobot.addMovement("forward_x",37.5,1);
		pickerRobot.faceSouth(1);
		pickerRobot.addMovement("forward_y",-4.35,1);
		pickerRobot.faceWest(1);
		pickerRobot.addMovement("forward_x",-37.5,1);
	}else if (distance ==5){
		pickerRobot.faceEast(1);
		pickerRobot.addMovement("forward_x",37.5,1);
		pickerRobot.faceNorth(1);
		pickerRobot.addMovement("forward_y",4.35,1);
		pickerRobot.faceWest(1);
		pickerRobot.addMovement("forward_x",-37.5,1);
	}

}

int main(int argc, char **argv)
{
	pickerRobot=PickerRobot();
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "PickerRobot");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//picker robot advertise it node for its velocity message to be published.
	pickerRobot.robotNode_stage_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
	//picker robot advertise it node for its status message to be published.
	ros::Publisher pub=n.advertise<se306project::robot_status>("status",1000);

	//subscribe to listen to messages coming from stage about is position relative to absolute frame
	pickerRobot.stageOdo_Sub = n.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1000, callBackStageOdm);
	//subscribe to carrier robot's status message
	ros::Subscriber mysub_object = n.subscribe<se306project::carrier_status>("/robot_1/status",1000,recieveCarrierRobotStatus);

	// initalise robot status message
	se306project::robot_status status_msg;

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	while (ros::ok())
	{
		//assign to status message
		status_msg.my_counter = count++;//add counter to message to broadcast
		status_msg.status=status;//add status to message to broadcast
		status_msg.pos_x=pickerRobot.getX(); //add x to message to broadcast
		status_msg.pos_y=pickerRobot.getY();//add y to message to broadcast
		status_msg.pos_theta=pickerRobot.getAng(); //add angle to message to broadcast
		pub.publish(status_msg);//publish the message for other node

		pickerRobot.move();//robot move
		//TODO debug
		if(count==7){
			pickerRobot.movement();
		}
		if(count>7){
			if(pickerRobot.movementQueue.size()<1){
				if (status.compare("Moving")==0){
					status="Full";
				}
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}

	return 0;

}
